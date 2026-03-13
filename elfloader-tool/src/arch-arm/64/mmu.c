/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <autoconf.h>
#include <elfloader/gen_config.h>
#include <types.h>
#include <elfloader.h>
#include <mode/structures.h>
#include <printf.h>
#include <abort.h>

/*
* Create the 1:1 elfloader mapping to jump into the kernel after enabling the MMU.
*/
static void __attribute__((unused)) init_downpages(void)
{
    word_t i;
    vaddr_t start_vaddr = (vaddr_t)_text & ~MASK(ARM_2MB_BLOCK_BITS);
    vaddr_t end_vaddr = (vaddr_t)_end;

    _boot_pgd_down[0] = ((uintptr_t)_boot_pud_down) | BIT(1) | BIT(0); /* its a page table */

    /* We only map in 1 GiB, so check that the loader doesn't cross 1GiB boundary. */
    if (GET_PUD_INDEX(start_vaddr) != GET_PUD_INDEX(end_vaddr)) {
        printf("We only map 1GiB, but elfloader paddr range covers multiple GiB.\n");
        abort();
    }

    _boot_pud_down[GET_PUD_INDEX(start_vaddr)] = ((uintptr_t)_boot_pmd_down) | BIT(1) | BIT(0);

    for (i = GET_PMD_INDEX(start_vaddr); i <= GET_PMD_INDEX(end_vaddr); i++) {
        _boot_pmd_down[i] = (uintptr_t) start_vaddr
                            | BIT(10)  /* access flag */
                            | (4 << 2) /* MT_NORMAL memory */
                            | BIT(0);  /* 2M block */
        start_vaddr += BIT(ARM_2MB_BLOCK_BITS);
    }
}

/*
 * Map the full 4 GiB PA range using 1 GiB PUD block entries with MT_NORMAL.
 *
 * On SoCs with firmware-protected memory regions (e.g. NVIDIA T234), the
 * upstream init_downpages() approach of mapping only the elfloader's own
 * 2 MiB PMD blocks leaves the rest of the PA space unmapped. This is safe
 * on most platforms, but on T234 the Cortex-A78AE speculatively reads from
 * Normal-mapped addresses. If non-DRAM regions are left unmapped, speculative
 * fetches to those addresses fault immediately (translation fault at EL2).
 *
 * Mapping the full range as MT_NORMAL avoids translation faults from
 * speculative accesses. The kernel's SDRAM-only physical window and the
 * 2 MiB low-address reservation (in boot.c) prevent any actual RAS errors
 * from firmware-protected carve-outs once the kernel takes over.
 *
 * This is equivalent to the upstream approach before ee7435b, but uses
 * MT_NORMAL (index 4) instead of DEVICE_nGnRnE (index 0).
 */
static void init_downpages_full(void)
{
    word_t i;

    _boot_pgd_down[0] = ((uintptr_t)_boot_pud_down) | BIT(1) | BIT(0); /* its a page table */

    for (i = 0; i < BIT(PUD_BITS); i++) {
        _boot_pud_down[i] = (i << ARM_1GB_BLOCK_BITS)
                            | BIT(10) /* access flag */
                            | (4 << 2) /* MT_NORMAL memory */
                            | BIT(0); /* 1G block */
    }
}

/*
* Create a "boot" page table, which contains a 1:1 mapping below
* the kernel's first vaddr, and a virtual-to-physical mapping above the
* kernel's first vaddr.
*/
void init_boot_vspace(struct image_info *kernel_info)
{
    word_t i;
    vaddr_t first_vaddr = kernel_info->virt_region_start;
    vaddr_t last_vaddr = kernel_info->virt_region_end;
    paddr_t first_paddr = kernel_info->phys_region_start;

    /* init_downpages() — upstream default, maps only the elfloader's 2 MiB blocks.
     * init_downpages_full() — maps full 4 GiB as MT_NORMAL 1 GiB PUD blocks.
     * See comment above init_downpages_full() for why we use the full mapping. */
    init_downpages_full();

    _boot_pgd_up[GET_PGD_INDEX(first_vaddr)]
        = ((uintptr_t)_boot_pud_up) | BIT(1) | BIT(0); /* its a page table */

    _boot_pud_up[GET_PUD_INDEX(first_vaddr)]
        = ((uintptr_t)_boot_pmd_up) | BIT(1) | BIT(0); /* its a page table */

    /* We only map in 1 GiB, so check that the kernel doesn't cross 1GiB boundary. */
    if ((first_vaddr & ~MASK(ARM_1GB_BLOCK_BITS)) != (last_vaddr & ~MASK(ARM_1GB_BLOCK_BITS))) {
        printf("We only map 1GiB, but kernel vaddr range covers multiple GiB.\n");
        abort();
    }

    for (i = GET_PMD_INDEX(first_vaddr); i < BIT(PMD_BITS); i++) {
        _boot_pmd_up[i] = first_paddr
                          | BIT(10) /* access flag */
#if CONFIG_MAX_NUM_NODES > 1
                          | (3 << 8) /* make sure the shareability is the same as the kernel's */
#endif
                          | (4 << 2) /* MT_NORMAL memory */
                          | BIT(0); /* 2M block */
        first_paddr += BIT(ARM_2MB_BLOCK_BITS);
    }
}

void init_hyp_boot_vspace(struct image_info *kernel_info)
{
    word_t i;
    word_t pmd_index;
    vaddr_t first_vaddr = kernel_info->virt_region_start;
    paddr_t first_paddr = kernel_info->phys_region_start;

    /* See comment above init_downpages_full(). */
    init_downpages_full();

    _boot_pgd_down[GET_PGD_INDEX(first_vaddr)]
        = ((uintptr_t)_boot_pud_up) | BIT(1) | BIT(0); /* its a page table */

    _boot_pud_up[GET_PUD_INDEX(first_vaddr)]
        = ((uintptr_t)_boot_pmd_up) | BIT(1) | BIT(0); /* its a page table */

    pmd_index = GET_PMD_INDEX(first_vaddr);
    for (i = pmd_index; i < BIT(PMD_BITS); i++) {
        _boot_pmd_up[i] = (((i - pmd_index) << ARM_2MB_BLOCK_BITS) + first_paddr)
                          | BIT(10) /* access flag */
#if CONFIG_MAX_NUM_NODES > 1
                          | (3 << 8)
#endif
                          | (4 << 2) /* MT_NORMAL memory */
                          | BIT(0); /* 2M block */
    }
}
