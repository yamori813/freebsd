/*-
 * Copyright (c) 2010 Adrian Chadd
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: head/sys/mips/atheros/ar5315_chip.c 234326 2012-04-15 22:34:22Z adrian $");

#include "opt_ddb.h"

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cons.h>
#include <sys/kdb.h>
#include <sys/reboot.h>

#include <vm/vm.h>
#include <vm/vm_page.h>

#include <net/ethernet.h>

#include <machine/clock.h>
#include <machine/cpu.h>
#include <machine/cpuregs.h>
#include <machine/hwfunc.h>
#include <machine/md_var.h>
#include <machine/trap.h>
#include <machine/vmparam.h>

#include <mips/atheros/ar531x/ar5315reg.h>
#include <mips/atheros/ar531x/ar5315_chip.h>
#include <mips/atheros/ar531x/ar5315_cpudef.h>

#include <mips/sentry5/s5reg.h>

/* XXX these shouldn't be in here - this file is a per-chip file */
/* XXX these should be in the top-level ar5315 type, not ar5315 -chip */
uint32_t u_ar5315_cpu_freq;
uint32_t u_ar5315_ahb_freq;
uint32_t u_ar5315_ddr_freq;

static void
ar5315_chip_detect_mem_size(void)
{
}

static void
ar5315_chip_detect_sys_frequency(void)
{
	uint32_t freq_ref, freq_pll;
	static const uint8_t pll_divide_table[] = {
		2, 3, 4, 6, 3,
		/*
		 * these entries are bogus, but it avoids a possible
		 * bad table dereference
		 */
		1, 1, 1
	};
	static const uint8_t pre_divide_table[] = {
		1, 2, 4, 5
	};

	const uint32_t pllc = ATH_READ_REG(AR5315_SYSREG_BASE + 
		AR5315_SYSREG_PLLC_CTL);

	const uint32_t refdiv = pre_divide_table[AR5315_PLLC_REF_DIV(pllc)];
	const uint32_t fbdiv = AR5315_PLLC_FB_DIV(pllc);
	const uint32_t div2 = (AR5315_PLLC_DIV_2(pllc) + 1) * 2; /* results in 2 or 4 */

	freq_ref = 40000000;

	/* 40MHz reference clk, reference and feedback dividers */
	freq_pll = (freq_ref / refdiv) * div2 * fbdiv;

	const uint32_t pllout[4] = {
	    /* CLKM select */
	    [0] = freq_pll / pll_divide_table[AR5315_PLLC_CLKM(pllc)],
	    [1] = freq_pll / pll_divide_table[AR5315_PLLC_CLKM(pllc)],

	    /* CLKC select */
	    [2] = freq_pll / pll_divide_table[AR5315_PLLC_CLKC(pllc)],

	    /* ref_clk select */
	    [3] = freq_ref, /* use original reference clock */
	};

	const uint32_t amba_clkctl = ATH_READ_REG(AR5315_SYSREG_BASE +
		AR5315_SYSREG_AMBACLK);
	uint32_t ambadiv = AR5315_CLOCKCTL_DIV(amba_clkctl);
	ambadiv = ambadiv ? (ambadiv * 2) : 1;
	u_ar5315_ahb_freq = pllout[AR5315_CLOCKCTL_SELECT(amba_clkctl)] / ambadiv;

	const uint32_t cpu_clkctl = ATH_READ_REG(AR5315_SYSREG_BASE +
		AR5315_SYSREG_CPUCLK);
	uint32_t cpudiv = AR5315_CLOCKCTL_DIV(cpu_clkctl);
	cpudiv = cpudiv ? (cpudiv * 2) : 1;
	u_ar5315_cpu_freq = pllout[AR5315_CLOCKCTL_SELECT(cpu_clkctl)] / cpudiv;

	u_ar5315_ddr_freq = 0;

//	u_ar5315_cpu_freq = 180 * 1000 * 1000;
//	u_ar5315_ddr_freq = 180 * 1000 * 1000;
//	u_ar5315_ahb_freq = 92 * 1000 * 1000;
}

/*
 * This does not lock the CPU whilst doing the work!
 */
static void
ar5315_chip_device_stop(uint32_t mask)
{
	uint32_t reg;

	reg = ATH_READ_REG(AR5315_SYSREG_BASE + AR5315_SYSREG_COLDRESET);
	ATH_WRITE_REG(AR5315_SYSREG_BASE + AR5315_SYSREG_COLDRESET,
		reg | mask);
}

static void
ar5315_chip_device_start(uint32_t mask)
{
	uint32_t reg;

	reg = ATH_READ_REG(AR5315_SYSREG_BASE + AR5315_SYSREG_COLDRESET);
	ATH_WRITE_REG(AR5315_SYSREG_BASE + AR5315_SYSREG_COLDRESET,
		reg & ~mask);
}

static int
ar5315_chip_device_stopped(uint32_t mask)
{
	uint32_t reg;

	reg = ATH_READ_REG(AR5315_SYSREG_BASE + AR5315_SYSREG_COLDRESET);
	return ((reg & mask) == mask);
}

void
ar5315_chip_set_mii_speed(uint32_t unit, uint32_t speed)
{
}

/* Speed is either 10, 100 or 1000 */
static void
ar5315_chip_set_pll_ge(int unit, int speed)
{
}

static void
ar5315_chip_ddr_flush_ge(int unit)
{
}

static void
ar5315_chip_ddr_flush_ip2(void)
{
}

static uint32_t
ar5315_chip_get_eth_pll(unsigned int mac, int speed)
{
	return 0;
}

static void
ar5315_chip_init_usb_peripheral(void)
{
}

struct ar5315_cpu_def ar5315_chip_def = {
	&ar5315_chip_detect_mem_size,
	&ar5315_chip_detect_sys_frequency,
	&ar5315_chip_device_stop,
	&ar5315_chip_device_start,
	&ar5315_chip_device_stopped,
	&ar5315_chip_set_pll_ge,
	&ar5315_chip_set_mii_speed,
	&ar5315_chip_ddr_flush_ge,
	&ar5315_chip_get_eth_pll,
	&ar5315_chip_ddr_flush_ip2,
	&ar5315_chip_init_usb_peripheral,
};
