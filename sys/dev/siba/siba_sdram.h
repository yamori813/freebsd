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

/*
 * driver for SIBA SDRAM core.
 */

#ifndef	_SIBA_SDRAM_H_
#define	_SIBA_SDRAM_H_


#define	SIBA_SDRAM_CONTROL		0x00
#define	SIBA_SDRAM_CONFIG		0x04
#define	SIBA_SDRAM_REFRESH		0x08
#define	SIBA_SDRAM_BISTSTAT		0x0c
#define	SIBA_SDRAM_MODEBUF		0x10
#define	SIBA_SDRAM_BKCLS		0x14
#define	SIBA_SDRAM_PRIORINV		0x18
#define	SIBA_SDRAM_DRAMTIM		0x1c
#define	SIBA_SDRAM_INTSTAT		0x20
#define	SIBA_SDRAM_INTMASK		0x24
#define	SIBA_SDRAM_INTINFO		0x28
#define	SIBA_SDRAM_NCDLCTL		0x30
#define	SIBA_SDRAM_RDNCDLCOR		0x34
#define	SIBA_SDRAM_WRNCDLCOR		0x38
#define	SIBA_SDRAM_MISCDLYCTL		0x3c
#define	SIBA_SDRAM_DQSGATENCDL		0x40
#define	SIBA_SDRAM_SPARE		0x44
#define	SIBA_SDRAM_TPADDR		0x48
#define	SIBA_SDRAM_TPDATA		0x4c
#define	SIBA_SDRAM_BARRIER		0x50
#define	SIBA_SDRAM_CORE			0x54



/* SDRAM initialization control (initcontrol) register bits */
#define SIBA_SDRAM_CBR		0x0001	/* Writing 1 generates refresh cycle and toggles bit */
#define SIBA_SDRAM_PRE		0x0002	/* Writing 1 generates precharge cycle and toggles bit */
#define SIBA_SDRAM_MRS		0x0004	/* Writing 1 generates mode register select cycle and toggles bit */
#define SIBA_SDRAM_EN		0x0008	/* When set, enables access to SDRAM */
#define SIBA_SDRAM_16Mb		0x0000	/* Use 16 Megabit SDRAM */
#define SIBA_SDRAM_64Mb		0x0010	/* Use 64 Megabit SDRAM */
#define SIBA_SDRAM_128Mb	0x0020	/* Use 128 Megabit SDRAM */
#define SIBA_SDRAM_RSVMb	0x0030	/* Use special SDRAM */
#define SIBA_SDRAM_RST		0x0080	/* Writing 1 causes soft reset of controller */
#define SIBA_SDRAM_SELFREF	0x0100	/* Writing 1 enables self refresh mode */
#define SIBA_SDRAM_PWRDOWN	0x0200	/* Writing 1 causes controller to power down */
#define SIBA_SDRAM_32BIT	0x0400	/* When set, indicates 32 bit SDRAM interface */
#define SIBA_SDRAM_9BITCOL	0x0800	/* When set, indicates 9 bit column */

/* SDRAM configuration (config) register bits */
#define SIBA_SDRAM_BURSTFULL	0x0000	/* Use full page bursts */
#define SIBA_SDRAM_BURST8	0x0001	/* Use burst of 8 */
#define SIBA_SDRAM_BURST4	0x0002	/* Use burst of 4 */
#define SIBA_SDRAM_BURST2	0x0003	/* Use burst of 2 */
#define SIBA_SDRAM_CAS3		0x0000	/* Use CAS latency of 3 */
#define SIBA_SDRAM_CAS2		0x0004	/* Use CAS latency of 2 */

/* SDRAM refresh control (refresh) register bits */
#define SIBA_SDRAM_REF(p)	(((p)&0xff) | SIBA_SDRAM_REF_EN)	/* Refresh period */
#define SIBA_SDRAM_REF_EN	0x8000		/* Writing 1 enables periodic refresh */

/* SDRAM Core default Init values (OCP ID 0x803) */
#define SIBA_SDRAM_INIT		MEM4MX16X2
#define SIBA_SDRAM_CONFIG	SIBA_SDRAM_BURSTFULL
#define SIBA_SDRAM_REFRESH	SIBA_SDRAM_REF(0x40)

#define MEM1MX16	0x009	/* 2 MB */
#define MEM1MX16X2	0x409	/* 4 MB */
#define MEM2MX8X2	0x809	/* 4 MB */
#define MEM2MX8X4	0xc09	/* 8 MB */
#define MEM2MX32	0x439	/* 8 MB */
#define MEM4MX16	0x019	/* 8 MB */
#define MEM4MX16X2	0x419	/* 16 MB */
#define MEM8MX8X2	0x819	/* 16 MB */
#define MEM8MX16	0x829	/* 16 MB */
#define MEM4MX32	0x429	/* 16 MB */
#define MEM8MX8X4	0xc19	/* 32 MB */
#define MEM8MX16X2	0xc29	/* 32 MB */


#endif /* _SIBA_SDRAM_H_ */
