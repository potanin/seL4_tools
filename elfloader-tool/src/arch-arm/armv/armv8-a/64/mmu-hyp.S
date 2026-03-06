/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <autoconf.h>
#include <assembler.h>
#include <armv/assembler.h>

#ifdef CONFIG_ARM_PA_SIZE_BITS_40
#define TCR_PS TCR_PS_1T
#else
#define TCR_PS TCR_PS_16T
#endif

.text

.extern flush_dcache
.extern invalidate_dcache
.extern invalidate_icache
.extern _boot_pgd_down

BEGIN_FUNC(disable_caches_hyp)
    /* T234 (Orin): DO NOT use flush_dcache (dc cisw) here.
     * dc cisw only reaches CPU caches, not the T234 system-level cache (SLC).
     * It causes data to get stuck in the SLC, making subsequent DRAM reads
     * return stale data after caches are disabled.
     *
     * The caller has already flushed all critical data to PoC (DRAM) using
     * dc cvac via flush_to_poc(). We just need dsb to ensure ordering,
     * then disable caches. No stack access after disable. */
    dsb     sy
    disable_id_cache sctlr_el2, x9
    ret
END_FUNC(disable_caches_hyp)

BEGIN_FUNC(leave_hyp)
    /* We call nested functions, follow the ABI. */
    stp     x29, x30, [sp, #-16]!
    mov     x29, sp

    bl      flush_dcache

    /* Ensure I-cache, D-cache and mmu are disabled for EL2/Stage1 */
    disable_mmu sctlr_el2, x9

    /*
     * Invalidate the local I-cache so that any instructions fetched
     * speculatively are discarded.
     */
    bl      invalidate_icache

    /* Ensure I-cache, D-cache and mmu are disabled for EL1/Stage2 */
    mov     x9, #(1 << 31)
    msr     hcr_el2, x9

    /* Ensure traps to EL2 are disabled */
    mov     x9, #0x33ff
    msr     cptr_el2, x9
    msr     hstr_el2, xzr
    msr     vttbr_el2, xzr

    /* Ensure I-cache, D-cache and mmu are disabled for EL1/Stage1 */
    disable_mmu sctlr_el1 , x9

    mov     x9, #(PSR_F_BIT | PSR_I_BIT | PSR_A_BIT | PSR_D_BIT | PSR_MODE_EL1h)
    msr     spsr_el2, x9

    /* Let's the caller use our stack, in case it needs to pop something */
    ldp     x29, x30, [sp], #16
    mov     x10, sp
    msr     sp_el1, x10
    msr     elr_el2, x30
    eret
END_FUNC(leave_hyp)

BEGIN_FUNC(arm_enable_hyp_mmu)
    /* T234 (Orin): caches are already disabled by disable_caches_hyp() before
     * this function is called. Page tables have been flushed to PoC (DRAM)
     * by the caller using dc cvac. We only need dsb to ensure all prior
     * stores are ordered, then disable MMU and switch page tables.
     *
     * DO NOT call flush_dcache here — dc cisw on T234 interacts badly with
     * the system-level cache (SLC) and causes intermittent hangs (~70%).
     *
     * Save LR/FP in callee-saved x19/x20, and caller's x19/x20 in x21/x22.
     * No stack access — all state in registers through the MMU transition. */
    mov     x21, x19            /* save caller's x19 */
    mov     x22, x20            /* save caller's x20 */
    mov     x19, x30            /* stash return address */
    mov     x20, x29            /* stash frame pointer */

    dsb     sy                  /* ensure all prior stores are visible */

    disable_mmu sctlr_el2, x8

    /* Inline invalidate_icache to avoid bl clobbering x30 */
    ic      iallu
    dsb     nsh
    isb

    /* T234: Mitigate stale firmware cache line RAS errors.
     *
     * Firmware (MB1/MB2/ATF/UEFI) leaves dirty cache lines at
     * protected addresses (paddr 0x0, 0xda0, etc.). When cache
     * pressure evicts them, writebacks trigger IOB RAS errors
     * → SError → ATF unconditionally powers off the core.
     *
     * Countermeasures applied here:
     *
     * 1. CBB ERD (Error Response Disable): makes CBB return
     *    0xdead2003 instead of SLV_ERR. Same as Linux tegra234-cbb.
     *    Doesn't prevent IOB from generating its own RAS error.
     *
     * 2. dc isw: invalidate CPU caches (L1/L2/L3) by set/way.
     *    Discards dirty lines without writeback (no bus transaction).
     *    Doesn't reach SLC (L4).
     *
     * Cannot reach SLC dirty lines from NS-EL2:
     * - dc civac/dc ivac: A78AE cleans dirty lines before
     *   invalidating → writeback → IOB RAS → ATF kills core.
     * - IOB/ACI ERRCTLR: Secure-only, writes silently ignored.
     * - MCE ROC_FLUSH_CACHE SMC: not supported on T234.
     * - SDEI handlers: ATF kills core after dispatch regardless.
     *
     * Remaining non-determinism depends on firmware cache state
     * at boot. Fix requires custom ATF (don't kill core on RAS). */

    /* CBB ERD: base 0x13a00000 + offset 0x3a004 */
    movz    x8, #0xa004
    movk    x8, #0x13a3, lsl #16
    mov     w9, #1
    str     w9, [x8]
    dsb     sy

    /* Invalidate CPU caches to discard stale firmware dirty lines.
     * Our data was already flushed to DRAM by flush_to_poc(dc cvac).
     * Caches are disabled and MMU is off, so no new dirty lines. */
    bl      invalidate_dcache

    /* SDEI handler registration is done later in the kernel
     * (boot.c) where the handler virtual address is valid. */

    /*
     *   DEVICE_nGnRnE      000     00000000
     *   DEVICE_nGnRE       001     00000100
     *   DEVICE_GRE         010     00001100
     *   NORMAL_NC          011     01000100
     *   NORMAL             100     11111111
     *   NORMAL_WT          101     10101010
     */
    ldr     x5, =MAIR(0x00, MT_DEVICE_nGnRnE) | \
                 MAIR(0x04, MT_DEVICE_nGnRE) | \
                 MAIR(0x0c, MT_DEVICE_GRE) | \
                 MAIR(0x44, MT_NORMAL_NC) | \
                 MAIR(0xff, MT_NORMAL) | \
                 MAIR(0xaa, MT_NORMAL_WT)
    msr     mair_el2, x5
    ldr     x8, =TCR_T0SZ(48) | TCR_IRGN0_WBWC | TCR_ORGN0_WBWC | TCR_SH0_ISH | TCR_TG0_4K | TCR_PS | TCR_EL2_RES1
    msr     tcr_el2, x8
    isb

    adrp    x8, _boot_pgd_down
    msr     ttbr0_el2, x8
    isb

    tlbi    alle2is
    dsb     ish
    isb

    enable_mmu  sctlr_el2, x8

    /* DO NOT invalidate I-cache (ic ialluis) here!
     * After enable_mmu, we execute from the identity map which uses
     * DEVICE_nGnRnE memory type. Instruction fetch from Device memory
     * is architecturally forbidden on ARMv8. The instructions currently
     * in the pipeline/icache work, but ic ialluis would force a refetch
     * from Device memory — causing hangs on Cortex-A78AE.
     * The I-cache was already invalidated before enable_mmu. */
    tlbi    alle2is
    dsb     ish
    isb

    /* MMU+caches back ON with seL4 page tables.
     * Restore all callee-saved regs from registers (no stack needed). */
    mov     x30, x19
    mov     x29, x20
    mov     x19, x21            /* restore caller's x19 */
    mov     x20, x22            /* restore caller's x20 */
    ret
END_FUNC(arm_enable_hyp_mmu)
