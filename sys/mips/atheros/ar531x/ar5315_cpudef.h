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

/* $FreeBSD: head/sys/mips/atheros/ar5315_cpudef.h 233081 2012-03-17 07:25:23Z adrian $ */

#ifndef	__AR5315_CPUDEF_H__
#define	__AR5315_CPUDEF_H__

struct ar5315_cpu_def {
	void (* detect_mem_size) (void);
	void (* detect_sys_frequency) (void);
	void (* ar5315_chip_device_reset) (void);
	void (* ar5315_chip_device_start) (uint32_t);
	int (* ar5315_chip_device_stopped) (uint32_t);
	void (* ar5315_chip_set_pll_ge) (int, int);
	void (* ar5315_chip_set_mii_speed) (uint32_t, uint32_t);
	void (* ar5315_chip_ddr_flush_ge) (int);
	uint32_t (* ar5315_chip_get_eth_pll) (unsigned int, int);

	/*
	 * From Linux - Handling this IRQ is a bit special.
	 * AR71xx - AR5315_DDR_REG_FLUSH_PCI
	 * AR724x - AR724X_DDR_REG_FLUSH_PCIE
	 * AR91xx - AR91XX_DDR_REG_FLUSH_WMAC
	 *
	 * These are set when STATUSF_IP2 is set in regiser c0.
	 * This flush is done before the IRQ is handled to make
	 * sure the driver correctly sees any memory updates.
	 */
	void (* ar5315_chip_ddr_flush_ip2) (void);
	/*
	 * Allow to change MII bus mode:
	 * AR5315_ARGE_MII_MODE_MII
	 * AR5315_ARGE_MII_MODE_RMII
	 * AR5315_ARGE_MII_MODE_GMII
	 * AR5315_ARGE_MII_MODE_RGMII
	 * mii_mode(unit, mode);
	 */
#define	AR5315_ARGE_MII_MODE_MII	0x0100
#define	AR5315_ARGE_MII_MODE_RMII	0x0101
#define	AR5315_ARGE_MII_MODE_GMII	0x1000
#define	AR5315_ARGE_MII_MODE_RGMII	0x1001
	void (* ar5315_chip_set_mii_mode) (int, int, int);
};

extern struct ar5315_cpu_def * ar5315_cpu_ops;

static inline void ar5315_detect_sys_frequency(void)
{
	ar5315_cpu_ops->detect_sys_frequency();
}

static inline void ar5315_device_reset(void)
{
	ar5315_cpu_ops->ar5315_chip_device_reset();
}

static inline void ar5315_device_start(uint32_t mask)
{
	ar5315_cpu_ops->ar5315_chip_device_start(mask);
}

static inline int ar5315_device_stopped(uint32_t mask)
{
	return ar5315_cpu_ops->ar5315_chip_device_stopped(mask);
}

static inline void ar5315_device_set_pll_ge(int unit, int speed)
{
	ar5315_cpu_ops->ar5315_chip_set_pll_ge(unit, speed);
}

static inline void ar5315_device_set_mii_speed(int unit, int speed)
{
	ar5315_cpu_ops->ar5315_chip_set_mii_speed(unit, speed);
}

static inline void ar5315_device_flush_ddr_ge(int unit)
{
	ar5315_cpu_ops->ar5315_chip_ddr_flush_ge(unit);
}

static inline void ar5315_device_ddr_flush_ip2(void)
{
	ar5315_cpu_ops->ar5315_chip_ddr_flush_ip2();
}

static inline void ar5315_device_set_mii_mode(int unit, int mode, int speed)
{
	ar5315_cpu_ops->ar5315_chip_set_mii_mode(unit, mode, speed);
}

/* XXX shouldn't be here! */
extern uint32_t u_ar531x_cpu_freq;
extern uint32_t u_ar531x_ahb_freq;
extern uint32_t u_ar531x_ddr_freq;

extern uint32_t u_ar531x_uart_addr;

static inline uint32_t ar5315_cpu_freq(void) { return u_ar531x_cpu_freq; }
static inline uint32_t ar5315_ahb_freq(void) { return u_ar531x_ahb_freq; }
static inline uint32_t ar5315_ddr_freq(void) { return u_ar531x_ddr_freq; }

static inline uint32_t ar5315_uart_addr(void) { return u_ar531x_uart_addr; }
 
#endif
