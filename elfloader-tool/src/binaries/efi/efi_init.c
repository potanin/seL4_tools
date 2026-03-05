/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <binaries/efi/efi.h>
#include <elfloader_common.h>

void *__application_handle = NULL;             // current efi application handler
efi_system_table_t *__efi_system_table = NULL; // current efi system table

extern void _start(void);
/* Minimal UEFI console output for early debug (UCS-2 strings) */
typedef struct {
    void *Reset;
    unsigned long (*OutputString)(void *this, uint16_t *string);
} efi_simple_text_out_t;

static int efi_conout_active = 1;

static void efi_putchar(unsigned int c)
{
    if (!efi_conout_active || !__efi_system_table || !__efi_system_table->con_out) {
        return;
    }
    efi_simple_text_out_t *con_out = (efi_simple_text_out_t *)__efi_system_table->con_out;
    if (con_out->OutputString) {
        uint16_t str[3];
        int i = 0;
        if (c == '\n') {
            str[i++] = '\r';
        }
        str[i++] = (uint16_t)c;
        str[i] = 0;
        con_out->OutputString(con_out, str);
    }
}

void efi_conout_disable(void)
{
    efi_conout_active = 0;
}

/*
 * Override the WEAK plat_console_putchar.
 * Before ExitBootServices: output to UEFI ConOut (HDMI).
 * After ExitBootServices: output is dropped (no-op).
 * UART output is handled directly in continue_boot() after MMU is off.
 */
extern volatile int uart_mmio_ready;

/*
 * Output mode flag:
 * 0 = ConOut (HDMI, before ExitBootServices)
 * 1 = UART MMIO (after ExitBootServices)
 *
 * Set to 1 by efi_exit_boot_services() BEFORE dcache operations.
 * This is safe because the variable is written while caches are still in
 * a consistent state. After dcache cisw on T234, this variable may be stale,
 * but it would only be stale to its ORIGINAL value (0) or its SET value (1).
 * If stale as 0, we'd try ConOut which won't output (conout disabled) —
 * harmless. We avoid the crash by not calling into UEFI at all once output
 * is in UART mode.
 */
static volatile int use_uart = 0;

void plat_console_switch_to_uart(void)
{
    use_uart = 1;
}

int plat_console_putchar(unsigned int c);
int plat_console_putchar(unsigned int c)
{
    if (!use_uart) {
        /* Before ExitBootServices: ConOut (HDMI) */
        efi_putchar(c);
        return 0;
    }

    /* After ExitBootServices: always UART */
    volatile uint32_t *uart = (volatile uint32_t *)0x3100000UL;
    if (c == '\n') {
        int t = 50000;
        while (!(uart[5] & 0x20) && --t > 0) { }
        uart[0] = '\r';
    }
    {
        int t = 50000;
        while (!(uart[5] & 0x20) && --t > 0) { }
        uart[0] = (uint32_t)c;
    }
    return 0;
}

unsigned int efi_main(uintptr_t application_handle, uintptr_t efi_system_table)
{
    clear_bss();
    __application_handle = (void *)application_handle;
    __efi_system_table = (efi_system_table_t *)efi_system_table;
    _start();
    return 0;
}

void *efi_get_fdt(void)
{
    efi_guid_t fdt_guid = make_efi_guid(0xb1b621d5, 0xf19c, 0x41a5,  0x83, 0x0b, 0xd9, 0x15, 0x2c, 0x69, 0xaa, 0xe0);
    efi_config_table_t *tables = (efi_config_table_t *)__efi_system_table->tables;

    for (uint32_t i = 0; i < __efi_system_table->nr_tables; i++) {
        if (!efi_guideq(fdt_guid, tables[i].guid))
            continue;

        return (void *)tables[i].table;
    }

    return NULL;
}

/* Before starting the kernel we should notify the UEFI firmware about it
 * otherwise the internal watchdog may reboot us after 5 min.
 *
 * This means boot time services are not available anymore. We should store
 * system information e.g. current memory map and pass them to kernel.
 */
unsigned long efi_exit_boot_services(void)
{
    unsigned long status;
    efi_memory_desc_t *memory_map;
    unsigned long map_size;
    unsigned long desc_size, key;
    uint32_t desc_version;

    efi_boot_services_t *bts = get_efi_boot_services();

    /*
     * As the number of existing memeory segments are unknown,
     * we need to resort to a trial and error to guess that.
     * We start from 32 and increase it by one until get a valid value.
     */
    map_size = sizeof(*memory_map) * 32;

again:
    status = bts->allocate_pool(EFI_LOADER_DATA, map_size, (void **)&memory_map);

    if (status != EFI_SUCCESS)
        return status;

    status = bts->get_memory_map(&map_size, memory_map, &key, &desc_size, &desc_version);
    if (status == EFI_BUFFER_TOO_SMALL) {
        bts->free_pool(memory_map);

        map_size += sizeof(*memory_map);
        goto again;
    }

    if (status != EFI_SUCCESS){
        bts->free_pool(memory_map);
        return status;
    }

    /* Switch output to UART before exiting boot services.
     * This must happen while caches are still consistent (before any dcache ops). */
    efi_conout_disable();
    plat_console_switch_to_uart();

    status = bts->exit_boot_services(__application_handle, key);
    return status;
}
