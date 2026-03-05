/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 * Copyright 2021, HENSOLDT Cyber
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <autoconf.h>
#include <elfloader/gen_config.h>

#include <drivers.h>
#include <drivers/uart.h>
#include <printf.h>
#include <types.h>
#include <abort.h>
#include <strops.h>
#include <cpuid.h>

#include <binaries/efi/efi.h>
#include <elfloader.h>

/* 0xd00dfeed in big endian */
#define DTB_MAGIC (0xedfe0dd0)

/* Maximum alignment we need to preserve when relocating (64K) */
#define MAX_ALIGN_BITS (14)

#ifdef CONFIG_IMAGE_EFI
ALIGN(BIT(PAGE_BITS)) VISIBLE
char core_stack_alloc[CONFIG_MAX_NUM_NODES][BIT(PAGE_BITS)];
#endif

struct image_info kernel_info;
struct image_info user_info;
void const *dtb;
size_t dtb_size;

extern void finish_relocation(int offset, void *_dynamic, unsigned int total_offset);
void continue_boot(int was_relocated);

/*
 * Flush a range of memory from all cache levels to Point of Coherence.
 * On T234, dc cisw (clean by set/way) only reaches the CPU caches,
 * not the system-level cache (SLC). dc cvac goes all the way to PoC,
 * ensuring data reaches DRAM.
 */
static void flush_to_poc(void *start, size_t size)
{
    uintptr_t addr = (uintptr_t)start & ~63UL;  /* align to cache line */
    uintptr_t end = (uintptr_t)start + size;
    while (addr < end) {
        __asm__ volatile("dc cvac, %0" :: "r"(addr) : "memory");
        addr += 64;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

#ifdef CONFIG_IMAGE_EFI
/*
 * Send one BPMP IPC message and wait for response.
 * BPMP IPC shmem (0x40070000) is mapped in UEFI page tables.
 * Must be called before ExitBootServices.
 */
static int bpmp_send_phys(uint32_t mrq, const uint32_t *payload, int nwords)
{
    volatile uint32_t *tx = (volatile uint32_t *)(uintptr_t)0x40070000;
    volatile uint32_t *rx = (volatile uint32_t *)(uintptr_t)0x40071000;
    volatile uint32_t *db = (volatile uint32_t *)(uintptr_t)0x3c90300;

    uint32_t tc = tx[0];
    uint32_t rx_exp = rx[0] + 1;

    tx[32] = mrq;
    tx[33] = 2;  /* flags = MSG_RING */
    for (int i = 0; i < nwords; i++)
        tx[34 + i] = payload[i];

    __asm__ volatile("dsb sy" ::: "memory");
    tx[0] = tc + 1;
    __asm__ volatile("dsb sy" ::: "memory");
    *db = 1;
    __asm__ volatile("dsb sy" ::: "memory");

    int timeout = 2000000;
    while (rx[0] != rx_exp && --timeout > 0);
    if (timeout > 0) {
        rx[16] = rx_exp;
        __asm__ volatile("dsb sy" ::: "memory");
    }
    return timeout > 0;
}

/* Enable UARTC clock via BPMP: CLK_ENABLE + SET_RATE + RESET_DEASSERT */
static void bpmp_enable_uartc(void)
{
    uint32_t p[4];

    /* CLK_ENABLE: MRQ_CLK=22, CMD=7, clk=157 (UARTC) */
    p[0] = (7 << 24) | 157;
    p[1] = 0;
    bpmp_send_phys(22, p, 2);

    /* CLK_SET_RATE: 1843200 Hz = 115200 * 16 */
    p[0] = (2 << 24) | 157;
    p[1] = 0;
    p[2] = 1843200;
    p[3] = 0;
    bpmp_send_phys(22, p, 4);

    /* RESET_DEASSERT: MRQ_RESET=20, CMD=2, rst=102 (UARTC) */
    p[0] = 2;
    p[1] = 102;
    bpmp_send_phys(20, p, 2);
}
#endif

/*
 * Make sure the ELF loader is below the kernel's first virtual address
 * so that when we enable the MMU we can keep executing.
 */
extern char _DYNAMIC[];
void relocate_below_kernel(void)
{
    /*
     * These are the ELF loader's physical addresses,
     * since we are either running with MMU off or
     * identity-mapped.
     */
    uintptr_t UNUSED start = (uintptr_t)_text;
    uintptr_t end = (uintptr_t)_end;

    if (end <= kernel_info.virt_region_start) {
        /*
         * If the ELF loader is already below the kernel,
         * skip relocation.
         */
        continue_boot(0);
        return;
    }

#ifdef CONFIG_IMAGE_EFI
    /*
     * Note: we make the (potentially incorrect) assumption
     * that there is enough physical RAM below the kernel's first vaddr
     * to fit the ELF loader.
     * FIXME: do we need to make sure we don't accidentally wipe out the DTB too?
     */
    uintptr_t size = end - start;

    /*
     * we ROUND_UP size in this calculation so that all aligned things
     * (interrupt vectors, stack, etc.) end up in similarly aligned locations.
     * The strictes alignment requirement we have is the 64K-aligned AArch32
     * page tables, so we use that to calculate the new base of the elfloader.
     */
    uintptr_t new_base = kernel_info.virt_region_start - (ROUND_UP(size, MAX_ALIGN_BITS));
    uint32_t offset = start - new_base;
    printf("relocating from %p-%p to %p-%p... size=0x%x (padded size = 0x%x)\n", start, end, new_base, new_base + size,
           size, ROUND_UP(size, MAX_ALIGN_BITS));

    memmove((void *)new_base, (void *)start, size);

    /* call into assembly to do the finishing touches */
    finish_relocation(offset, _DYNAMIC, new_base);
#else
    printf("ERROR: The ELF loader does not support relocating itself. You"
           " probably need to move the kernel window higher, or the load"
           " address lower.\n");
    abort();
#endif
}

/*
 * Entry point.
 *
 * Unpack images, setup the MMU, jump to the kernel.
 */
void main(UNUSED void *arg)
{
    void *bootloader_dtb = NULL;

    /* initialize platform to a state where we can print to a UART */
    if (initialise_devices()) {
        printf("ERROR: Did not successfully return from initialise_devices()\n");
        abort();
    }

    platform_init();

    /* Print welcome message. */
    printf("\n[seL4 orin-nano v115-no-keepalive]\n");
    printf("ELF-loader started on ");
    print_cpuid();
    {
        uint32_t cel;
        uint64_t sctlr;
        __asm__ volatile("mrs %0, CurrentEL" : "=r"(cel));
        __asm__ volatile("mrs %0, sctlr_el2" : "=r"(sctlr));
        printf("  CurrentEL=%u (EL%u) sctlr_el2=0x%lx (MMU %s)\n",
               cel, cel >> 2, sctlr, (sctlr & 1) ? "ON" : "OFF");
    }
    printf("  paddr=[%p..%p]\n", _text, (uintptr_t)_end - 1);

#if defined(CONFIG_IMAGE_UIMAGE)

    /* U-Boot passes a DTB. Ancient bootloaders may pass atags. When booting via
     * bootelf argc is NULL.
     */
    if (arg && (DTB_MAGIC == *(uint32_t *)arg)) {
        bootloader_dtb = arg;
    }

#elif defined(CONFIG_IMAGE_EFI)

    bootloader_dtb = efi_get_fdt();
    printf("  dtb from UEFI=%p\n", bootloader_dtb);

    /* Enable UARTC clock via BPMP before ExitBootServices.
     * BPMP IPC shmem (0x40070000) is mapped in UEFI page tables.
     * UARTC MMIO (0xc280000) is NOT mapped — don't touch it here. */
    bpmp_enable_uartc();

    printf("Exiting UEFI boot services...\n");

    if (efi_exit_boot_services() != EFI_SUCCESS) {
        printf("ERROR: Unable to exit UEFI boot services!\n");
        abort();
    }

    /* UART MMIO (0xc280000) is not mapped in UEFI page tables.
     * Output is silently dropped until continue_boot() enables UART. */

#endif

    if (bootloader_dtb) {
        printf("  dtb=%p\n", bootloader_dtb);
    } else {
        printf("No DTB passed in from boot loader.\n");
    }

    printf("  elfloader=[%p..%p]\n", _text, (uintptr_t)_end - 1);

    /* Unpack ELF images into memory. */
    unsigned int num_apps = 0;
    int ret = load_images(&kernel_info, &user_info, 1, &num_apps,
                          bootloader_dtb, &dtb, &dtb_size);
    if (0 != ret) {
        printf("ERROR: image loading failed\n");
        abort();
    }

    if (num_apps != 1) {
        printf("ERROR: expected to load just 1 app, actually loaded %u apps\n",
               num_apps);
        abort();
    }

    /*
     * We don't really know where we've been loaded.
     * It's possible that EFI loaded us in a place
     * that will become part of the 'kernel window'
     * once we switch to the boot page tables.
     * Make sure this is not the case.
     */
    relocate_below_kernel();
    printf("ERROR: Relocation failed, aborting!\n");
    abort();
}


void continue_boot(int was_relocated)
{
    printf("continue_boot(%d)\n", was_relocated);
    printf("  CB step 1\n");

    if (was_relocated) {
        printf("ELF loader relocated, continuing boot...\n");
    }

    printf("  CB step 2\n");

    /*
     * If we were relocated, we need to re-initialise the
     * driver model so all its pointers are set up properly.
     */
    if (was_relocated) {
        if (initialise_devices()) {
            printf("ERROR: Did not successfully return from initialise_devices()\n");
            abort();
        }
    }

    printf("  CB step 3\n");

#if (defined(CONFIG_ARCH_ARM_V7A) || defined(CONFIG_ARCH_ARM_V8A)) && !defined(CONFIG_ARM_HYPERVISOR_SUPPORT)
    if (is_hyp_mode()) {
        extern void leave_hyp(void);
        leave_hyp();
    }
#endif
    printf("  CB step 4\n");

    /* Setup MMU. */
    printf("  hyp=%d\n", is_hyp_mode());
    if (is_hyp_mode()) {
#ifdef CONFIG_ARCH_AARCH64
        extern void disable_caches_hyp();

        /* Build boot page tables BEFORE disabling caches so we can
         * flush them to PoC (DRAM) with dc cvac while caches are ON.
         * On T234, dc cisw only reaches CPU caches, not the system-level
         * cache (SLC). dc cvac reaches PoC past the SLC. */
        printf("  init_hyp_boot_vspace...\n");
        init_hyp_boot_vspace(&kernel_info);
        printf("  vspace done\n");

        /* Flush everything to PoC (DRAM) using dc cvac (caches ON).
         * This ensures data passes through all caches including T234's SLC
         * and reaches DRAM before we disable caches.
         * Must flush: boot page tables, kernel image, user image, DTB,
         * and elfloader text/data/BSS (includes stack). */
        printf("  flushing to PoC...\n");
        {
            extern uint64_t _boot_pgd_down[];
            extern uint64_t _boot_pud_down[];
            extern uint64_t _boot_pud_up[];
            extern uint64_t _boot_pmd_up[];
            flush_to_poc(_boot_pgd_down, 4096);
            flush_to_poc(_boot_pud_down, 4096);
            flush_to_poc(_boot_pud_up, 4096);
            flush_to_poc(_boot_pmd_up, 4096);
        }
        flush_to_poc((void *)(uintptr_t)kernel_info.phys_region_start,
                     kernel_info.phys_region_end - kernel_info.phys_region_start);
        flush_to_poc((void *)(uintptr_t)user_info.phys_region_start,
                     user_info.phys_region_end - user_info.phys_region_start);
        if (dtb) {
            flush_to_poc((void *)dtb, dtb_size);
        }
        /* Flush elfloader itself last (includes stack, globals, BSS).
         * The flush_to_poc calls above dirtied stack frames — this final
         * flush cleans them all to DRAM. */
        flush_to_poc(_text, (uintptr_t)_end - (uintptr_t)_text);

        disable_caches_hyp();

        /* === Caches OFF (MMU still ON with UEFI page tables) ===
         * UARTC MMIO (0xc280000) is NOT mapped in UEFI page tables.
         * No elfloader UART output from here — kernel uses UART_PPTR. */
#endif
    } else {
        init_boot_vspace(&kernel_info);
    }

#if CONFIG_MAX_NUM_NODES > 1
    printf("  smp_boot...\n");
    smp_boot();
    printf("  smp done\n");
#endif /* CONFIG_MAX_NUM_NODES */

    if (is_hyp_mode()) {
        arm_enable_hyp_mmu();
    } else {
        arm_enable_mmu();
    }
    /* Enter kernel — UART output starts from kernel via UART_PPTR. */
    ((init_arm_kernel_t)kernel_info.virt_entry)(user_info.phys_region_start,
                                                user_info.phys_region_end,
                                                user_info.phys_virt_offset,
                                                user_info.virt_entry,
                                                (word_t)dtb,
                                                dtb_size);

    /* We should never get here. */
    abort();
}
