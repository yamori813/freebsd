/*-
 * Copyright (c) 2007 Bruce M. Simpson.
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
 *
 * $FreeBSD: head/sys/dev/siba/siba_ids.h 183371 2008-09-26 03:57:23Z imp $
 */

#ifndef _SIBA_SIBA_IDS_H_
#define	_SIBA_SIBA_IDS_H_

/*
 * Constants and structures for SiBa bus enumeration.
 */

struct siba_devid {
	uint16_t	 sd_vendor;
	uint16_t	 sd_device;
	uint8_t		 sd_rev;
	char		*sd_desc;
};

/*
 * Device IDs
 */
#define SIBA_DEVID_ANY		0xffff
#define SIBA_DEVID_CHIPCOMMON	0x0800
#define SIBA_DEVID_INSIDELINE	0x0801
#define SIBA_DEVID_ETHERNET	0x0806
#define SIBA_DEVID_MODEM	0x0807
#define SIBA_DEVID_SDRAMDDR	0x080f
#define SIBA_DEVID_IEEE_802_11	0x0812
#define SIBA_DEVID_MIPS_3302	0x0816
#define SIBA_DEVID_USB_2_0_HOST	0x0819
#define SIBA_DEVID_ROBOSWITCH	0x081C

/* core codes */
#define	SIBA_DEVID_NODEV	0x0700		/* Invalid coreid */
#define	SIBA_DEVID_CC		0x0800		/* chipcommon core */
#define	SIBA_DEVID_ILINE20	0x0801		/* iline20 core */
#define	SIBA_DEVID_SDRAM	0x0803		/* sdram core */
#define	SIBA_DEVID_PCI		0x0804		/* pci core */
#define	SIBA_DEVID_MIPS		0x0805		/* mips core */
#define	SIBA_DEVID_ENET		0x0806		/* enet mac core */
#define	SIBA_DEVID_CODEC	0x0807		/* v90 codec core */
#define	SIBA_DEVID_USB		0x0808		/* usb 1.1 host/device core */
#define	SIBA_DEVID_ADSL		0x0809		/* ADSL core */
#define	SIBA_DEVID_ILINE100	0x080a		/* iline100 core */
#define	SIBA_DEVID_IPSEC	0x080b		/* ipsec core */
#define	SIBA_DEVID_PCMCIA	0x080d		/* pcmcia core */
#define	SIBA_DEVID_SOCRAM	0x080e		/* internal memory core */
#define	SIBA_DEVID_MEMC		0x080f		/* memc sdram core */
#define	SIBA_DEVID_EXTIF	0x0811		/* external interface core */
#define	SIBA_DEVID_D11		0x0812		/* 802.11 MAC core */
#define	SIBA_DEVID_MIPS33	0x0816		/* mips3302 core */
#define	SIBA_DEVID_USB11H	0x0817		/* usb 1.1 host core */
#define	SIBA_DEVID_USB11D	0x0818		/* usb 1.1 device core */
#define	SIBA_DEVID_USB20H	0x0819		/* usb 2.0 host core */
#define	SIBA_DEVID_USB20D	0x081a		/* usb 2.0 device core */
#define	SIBA_DEVID_SDIOH	0x081b		/* sdio host core */
#define	SIBA_DEVID_ROBO		0x081c		/* roboswitch core */
#define	SIBA_DEVID_ATA100	0x081d		/* parallel ATA core */
#define	SIBA_DEVID_SATAXOR	0x081e		/* serial ATA & XOR DMA core */
#define	SIBA_DEVID_GIGETH	0x081f		/* gigabit ethernet core */
#define	SIBA_DEVID_PCIE		0x0820		/* pci express core */
#define	SIBA_DEVID_MIMO		0x0821		/* MIMO phy core */
#define	SIBA_DEVID_SRAMC	0x0822		/* SRAM controller core */
#define	SIBA_DEVID_MINIMAC	0x0823		/* MINI MAC/phy core */
#define	SIBA_DEVID_ARM7S	0x0825		/* ARM7tdmi-s core */
#define SIBA_DEVID_SDIOD	0x0829		/* SDIO device core */
#define SIBA_DEVID_ARMCM3	0x082a		/* ARM Cortex M3 core */
#define SIBA_DEVID_OCP		0x0830		/* OCP2OCP bridge core */
#define SIBA_DEVID_SC		0x0831		/* shared common core */
#define SIBA_DEVID_AHB		0x0832		/* OCP2AHB bridge core */

#define	SIBA_DEVID_CC_IDX	0		/* chipc, when present,
						 * always core 0 */

/*
 * Vendor IDs
 */
#define SIBA_VID_ANY		0xffff
#define SIBA_VID_BROADCOM	0x4243

/*
 * Revision IDs
 */
#define SIBA_REV_ANY		0xff

#endif /*_SIBA_SIBA_IDS_H_ */
