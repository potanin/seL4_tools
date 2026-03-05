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

#ifdef CONFIG_IMAGE_EFI
/* Actual UARTA clock rate set by BPMP (0 = unknown) */
static uint32_t uarta_clk_rate = 0;
#endif

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

/* Set to 1 once UART pinmux + clock + divisor are configured (MMU off).
 * plat_console_putchar() in efi_init.c checks this before writing. */
volatile int uart_mmio_ready = 0;

#ifdef CONFIG_IMAGE_EFI
/*
 * Send one BPMP IPC message and wait for response.
 * Returns 1 on success, 0 on timeout.
 * tx/rx are uint32_t pointers to IVC shmem bases.
 */
static int bpmp_send(volatile uint32_t *tx, volatile uint32_t *rx,
                     volatile uint32_t *doorbell,
                     uint32_t mrq, const uint32_t *payload, int payload_words)
{
    uint32_t tc = tx[0];
    uint32_t rx_exp = rx[0] + 1;

    /* Write frame at offset 0x80 (index 32) */
    tx[32] = mrq;        /* MRQ code */
    tx[33] = 2;          /* flags = MSG_RING (BIT(1)) */
    for (int i = 0; i < payload_words; i++)
        tx[34 + i] = payload[i];

    __asm__ volatile("dsb sy" ::: "memory");
    tx[0] = tc + 1;      /* increment w_count */
    __asm__ volatile("dsb sy" ::: "memory");
    *doorbell = 1;        /* ring doorbell */
    __asm__ volatile("dsb sy" ::: "memory");

    /* Poll for response */
    int timeout = 2000000;
    while (rx[0] != rx_exp && --timeout > 0);
    if (timeout > 0) {
        rx[16] = rx_exp;  /* acknowledge: set r_count */
        __asm__ volatile("dsb sy" ::: "memory");
    }
    return timeout > 0;
}

/*
 * Enable UARTA clock and deassert reset via BPMP IPC.
 * Can be called with or without MMU — just needs physical access to:
 *   TX shmem @ 0x40070000, RX shmem @ 0x40071000, DB3 @ 0x3c90300
 */
static uint32_t bpmp_enable_uarta(volatile uint32_t *tx, volatile uint32_t *rx,
                                  volatile uint32_t *db)
{
    uint32_t payload[4];
    uint32_t rate = 0;

    /* CLK_ENABLE (MRQ_CLK=22, CMD=7, clk=155) */
    payload[0] = (7 << 24) | 155;
    payload[1] = 0;
    bpmp_send(tx, rx, db, 22, payload, 2);

    /* CLK_SET_RATE 1843200 Hz (MRQ_CLK=22, CMD=2, clk=155)
     * Tegra194+ uses support_clk_src_div: set clock = baud * 16,
     * then use divisor=1. Linux serial-tegra does this. */
    payload[0] = (2 << 24) | 155;
    payload[1] = 0;
    payload[2] = 1843200;  /* 115200 * 16 */
    payload[3] = 0;
    if (bpmp_send(tx, rx, db, 22, payload, 4)) {
        rate = rx[34];  /* returned rate */
    }

    /* RESET_DEASSERT (MRQ_RESET=20, CMD=2, rst=100) */
    payload[0] = 2;
    payload[1] = 100;
    bpmp_send(tx, rx, db, 20, payload, 2);

    return rate;
}
#endif /* CONFIG_IMAGE_EFI */

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
    printf("\n[seL4 orin-nano v112]\n");
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

    /* === BPMP IPC: enable UARTA clock before ExitBootServices === */
    {
        volatile uint32_t *tx = (volatile uint32_t *)(uintptr_t)0x40070000;
        volatile uint32_t *rx = (volatile uint32_t *)(uintptr_t)0x40071000;
        volatile uint32_t *db3 = (volatile uint32_t *)(uintptr_t)0x3c90300;

        uarta_clk_rate = bpmp_enable_uarta(tx, rx, db3);
        printf("  BPMP: clk_rate=%u\n", uarta_clk_rate);

        /* CLK_GET_RATE to verify (MRQ_CLK=22, CMD=5, clk=155) */
        {
            uint32_t payload[2];
            payload[0] = (5 << 24) | 155;  /* CMD=5 (GET_RATE), clk=155 */
            payload[1] = 0;
            if (bpmp_send(tx, rx, db3, 22, payload, 2)) {
                printf("  CLK_GET_RATE=%u\n", rx[34]);
            } else {
                printf("  CLK_GET_RATE: timeout\n");
            }
        }

        /* Read UART registers to see UEFI's current config */
        {
            volatile uint32_t *uart = (volatile uint32_t *)0x3100000UL;
            uint32_t lcr = uart[3];
            /* Set DLAB to read divisor */
            uart[3] = lcr | 0x80;
            uint32_t dll = uart[0];
            uint32_t dlm = uart[1];
            uart[3] = lcr;  /* restore LCR */
            uint32_t lsr = uart[5];
            printf("  UART: LCR=0x%x DLL=%u DLM=%u LSR=0x%x\n",
                   lcr, dll, dlm, lsr);

            /* Now set divisor=1 to match the new 1843200 Hz clock.
             * This prevents garbled output after ExitBootServices:
             * baud = 1843200 / (16 * 1) = 115200. */
            uart[1] = 0;        /* IER = 0 */
            uart[3] = 0x83;     /* LCR = DLAB | 8N1 */
            uart[0] = 1;        /* DLL = 1 */
            uart[1] = 0;        /* DLM = 0 */
            uart[3] = 0x03;     /* LCR = 8N1 */
            uart[2] = 0x07;     /* FCR = enable + reset FIFOs */
        }
    }

    printf("Exiting UEFI boot services...\n");

    if (efi_exit_boot_services() != EFI_SUCCESS) {
        printf("ERROR: Unable to exit UEFI boot services!\n");
        abort();
    }

    /* UART MMIO is not accessible until MMU is off (after disable_caches_hyp).
     * plat_console_putchar checks uart_mmio_ready before writing. */

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
#ifdef CONFIG_IMAGE_EFI
        extern void efi_conout_disable(void);
#endif

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

        /* === Caches OFF (MMU still ON with UEFI page tables) === */
#ifdef CONFIG_IMAGE_EFI
        /* Re-write critical volatile state directly (caches off → DRAM). */
        efi_conout_disable();  /* set efi_conout_active=0 in RAM */
        uart_mmio_ready = 1;   /* enable UART path in plat_console_putchar */

        /* Re-init UART with divisor=1 (clock = 1843200 from BPMP). */
        {
            volatile uint32_t *uart = (volatile uint32_t *)(uintptr_t)0x3100000;
            uart[1] = 0;
            uart[3] = 0x83;  /* LCR = DLAB | 8N1 */
            uart[0] = 1;     /* DLL = 1 */
            uart[1] = 0;     /* DLM = 0 */
            uart[3] = 0x03;  /* LCR = 8N1 */
            uart[2] = 0x07;  /* FCR = enable + reset FIFOs */
            uart[4] = 0x00;  /* MCR = 0 */
            { volatile int j; for (j = 0; j < 100000; j++); }
        }
#endif

        printf("v53 dcache ok\n");

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
        printf("Enabling hypervisor MMU and jumping to entry point...\n");
        printf("  kernel ventry=%p phys=%p\n",
               kernel_info.virt_entry, kernel_info.phys_region_start);
        printf("  user phys=[%p..%p] ventry=%p\n",
               user_info.phys_region_start, user_info.phys_region_end,
               user_info.virt_entry);
        printf("  dtb=%p size=%u\n", dtb, (unsigned)dtb_size);
        arm_enable_hyp_mmu();
        /* Raw UART write — printf may not work after page table switch */
        {
            volatile uint32_t *uart = (volatile uint32_t *)(uintptr_t)0x3100000;
            volatile int j;
            uart[0] = 'M'; for (j = 0; j < 100000; j++) {}
            uart[0] = 'M'; for (j = 0; j < 100000; j++) {}
            uart[0] = 'U'; for (j = 0; j < 100000; j++) {}
            uart[0] = '\r'; for (j = 0; j < 100000; j++) {}
            uart[0] = '\n'; for (j = 0; j < 100000; j++) {}
        }
    } else {
        printf("Enabling MMU and jumping to entry point...\n\n");
        arm_enable_mmu();
    }
    /* Enter kernel. The UART is no longer accessible here. */
    ((init_arm_kernel_t)kernel_info.virt_entry)(user_info.phys_region_start,
                                                user_info.phys_region_end,
                                                user_info.phys_virt_offset,
                                                user_info.virt_entry,
                                                (word_t)dtb,
                                                dtb_size);

    /* We should never get here. */
    abort();
}
