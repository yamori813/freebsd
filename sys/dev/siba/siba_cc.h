/*-
 * Copyright (c) 2010, Aleksandr Rybalko <ray@ddteam.net>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
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

#ifndef _SIBA_CC_H_
#define _SIBA_CC_H_

/* offset of high ID register */
#define SIBA_CORE_IDLO		0x00000ff8
#define SIBA_CORE_IDHI		0x00000ffc

/*
 * Offsets of ChipCommon core registers.
 * XXX: move to siba_cc
 */
#define SIBA_CC_UART0	0x00000300	/* offset of UART0 */
#define SIBA_CC_UART1	0x00000400	/* offset of UART1 */

#define SIBA_CC_CCID 0x0000
#define  SIBA_CC_IDMASK 0x0000FFFF
#define  SIBA_CC_REVMASK 0x000F0000
#define  SIBA_CC_REVSHIFT 16
#define  SIBA_CC_PACKMASK 0x00F00000
#define  SIBA_CC_PACKSHIFT 20
#define  SIBA_CC_NRCORESMASK 0x0F000000
#define  SIBA_CC_NRCORESSHIFT 24

#define  SIBA_IDHIGH_RCLO	0x0000000F /* Revision Code (low part) */
#define  SIBA_IDHIGH_CC		0x00008FF0 /* Core Code */
#define  SIBA_IDHIGH_CC_SHIFT	4
#define  SIBA_IDHIGH_RCHI	0x00007000 /* Revision Code (high part) */
#define  SIBA_IDHIGH_RCHI_SHIFT	8
#define  SIBA_IDHIGH_VC		0xFFFF0000 /* Vendor Code */
#define  SIBA_IDHIGH_VC_SHIFT	16

#define SIBA_CCID_BCM4710	0x4710
#define SIBA_CCID_BCM4704	0x4704
#define SIBA_CCID_SENTRY5	0x5365
#define SIBA_CCID_BCM5354	0x5354

/* Interrupt control */
#define SIBA_CC_INTSTATUS	0x20

#define SIBA_CC_INTMASK		0x24
#define SIBA_CC_CHIPCONTROL 	0x28 /* rev >= 11 */
#define SIBA_CC_CHIPSTATUS	0x2c /* rev >= 11 */


/* intstatus/intmask */
#define	SIBA_CC_INT_GPIO	0x00000001	/* gpio intr */
#define	SIBA_CC_INT_EI		0x00000002	/* extif intr (corerev >= 3) */
#define	SIBA_CC_INT_TEMP	0x00000004	/* temp. ctrl intr (rev >= 15)*/
#define	SIBA_CC_INT_SIRQ	0x00000008	/* serial IRQ intr (rev >= 15)*/
#define	SIBA_CC_INT_ECI		0x00000010	/* eci intr (corerev >= 21) */
#define	SIBA_CC_INT_PMU		0x00000020	/* pmu intr (corerev >= 21) */
#define	SIBA_CC_INT_UART	0x00000040	/* uart intr (corerev >= 21) */
#define	SIBA_CC_INT_WDRESET	0x80000000	/* watchdog reset occurred */

#define	SIBA_CC_CAPABILITIES		4

#define SIBA_CC_GPIOIN			0x0060
#define SIBA_CC_GPIOOUT			0x0064
#define SIBA_CC_GPIOOUTEN		0x0068
#define SIBA_CC_GPIOCTL			0x006C
#define SIBA_CC_GPIOPOL			0x0070
#define SIBA_CC_GPIOIRQ			0x0074
#define SIBA_CC_WATCHDOG		0x0080
#define SIBA_CC_GPIOTIMER		0x0088		/* LED powersave (rev >= 16) */
#define SIBA_CC_GPIOTIMER_ONTIME_SHIFT	16
#define SIBA_CC_GPIOTOUTM		0x008C		/* LED powersave (rev >= 16) */

#define	SIBA_CC_CLOCKDEV 		0x00a4

#define SIBA_CC_CLOCKDEV_SFLASH             0x0f000000 
#define SIBA_CC_CLOCKDEV_SFLASH_SHIFT       24 
#define SIBA_CC_CLOCKDEV_OTP                0x000f0000 
#define SIBA_CC_CLOCKDEV_OTP_SHIFT          16 
#define SIBA_CC_CLOCKDEV_JTAG               0x00000f00 
#define SIBA_CC_CLOCKDEV_JTAG_SHIFT         8 
#define SIBA_CC_CLOCKDEV_UART               0x000000ff 


/* capabilities */
#define	SIBA_CC_CAP_UARTS_MASK		0x00000003	/* Number of uarts */
#define SIBA_CC_CAP_MIPSEB		0x00000004	/* MIPS is in big-endian mode */
#define SIBA_CC_CAP_UCLKSEL		0x00000018	/* UARTs clock select */
#define SIBA_CC_CAP_UINTCLK		0x00000008	/* UARTs are driven by internal divided clock */
#define SIBA_CC_CAP_UARTGPIO		0x00000020	/* UARTs own Gpio's 15:12 */
#define SIBA_CC_CAP_EXTBUS_MASK		0x000000c0	/* External bus mask */
#define SIBA_CC_CAP_EXTBUS_NONE		0x00000000	/* No ExtBus present */
#define SIBA_CC_CAP_EXTBUS_FULL		0x00000040	/* ExtBus: PCMCIA, IDE & Prog */
#define SIBA_CC_CAP_EXTBUS_PROG		0x00000080	/* ExtBus: ProgIf only */
#define	SIBA_CC_CAP_FLASH_MASK		0x00000700	/* Type of flash */
#define	SIBA_CC_CAP_FLASH_SHIFT		8
#define	SIBA_CC_CAP_PLL_MASK		0x00038000	/* Type of PLL */
#define	SIBA_CC_CAP_PLL_SHIFT		15
#define SIBA_CC_CAP_PWR_CTL		0x00040000	/* Power control */
#define SIBA_CC_CAP_OTPSIZE		0x00380000	/* OTP Size (0 = none) */
#define SIBA_CC_CAP_OTPSIZE_SHIFT	19		/* OTP Size shift */
#define SIBA_CC_CAP_OTPSIZE_BASE	5		/* OTP Size base */
#define SIBA_CC_CAP_JTAGP		0x00400000	/* JTAG Master Present */
#define SIBA_CC_CAP_ROM			0x00800000	/* Internal boot rom active */
#define SIBA_CC_CAP_BKPLN64		0x08000000	/* 64-bit backplane */
#define	SIBA_CC_CAP_PMU			0x10000000	/* PMU Present, rev >= 20 */
#define	SIBA_CC_CAP_ECI			0x20000000	/* ECI Present, rev >= 21 */



#define SIBA_CC_FLASH_NONE		0x0		/* No flash */
#define SIBA_CC_SERIAL_FLASH_ST		0x1		/* ST serial flash */
#define SIBA_CC_SERIAL_FLASH_AT		0x2		/* Atmel serial flash */
#define	SIBA_CC_PARALLEL_FLASH		0x7		/* Parallel flash */



#define siba_cc_read_1(sc, reg)				\
	bus_space_read_1((sc)->sc_bt, (sc)->sc_bh,(reg))

#define siba_cc_read_2(sc, reg)				\
	bus_space_read_2((sc)->sc_bt, (sc)->sc_bh,(reg))

#define siba_cc_read_4(sc, reg)				\
	bus_space_read_4((sc)->sc_bt, (sc)->sc_bh,(reg))

#define siba_cc_write_1(sc, reg, val)			\
	bus_space_write_1((sc)->sc_bt, (sc)->sc_bh,	\
			 (reg), (val))

#define siba_cc_write_2(sc, reg, val)			\
	bus_space_write_2((sc)->sc_bt, (sc)->sc_bh,	\
			 (reg), (val))

#define siba_cc_write_4(sc, reg, val)			\
	bus_space_write_4((sc)->sc_bt, (sc)->sc_bh,	\
			 (reg), (val))

enum siba_cc_device_ivars {
	SIBA_CC_IVAR_XTALFREQ
};

#endif /* _SIBA_CC_H_ */
