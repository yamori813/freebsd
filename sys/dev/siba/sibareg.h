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
 * $FreeBSD: head/sys/dev/siba/sibareg.h 183371 2008-09-26 03:57:23Z imp $
 */

/*
 * TODO: sprom
 * TODO: implement dma translation bits (if needed for system bus)
 */

#ifndef _SIBA_SIBAREG_H_
#define _SIBA_SIBAREG_H_

#define	PCI_DEVICE_ID_BCM4401		0x4401
#define	PCI_DEVICE_ID_BCM4401B0		0x4402
#define	PCI_DEVICE_ID_BCM4401B1		0x170c
#define	SIBA_PCIR_BAR			PCIR_BAR(0)
#define	SIBA_CCID_BCM4710		0x4710
#define	SIBA_CCID_BCM4704		0x4704
#define	SIBA_CCID_SENTRY5		0x5365


#define SIBA_MAX_WIN_COUNT		4 /* Max count of memmory window per core */

#define	SIBA_BAR0			0x80
#define	SIBA_IRQMASK			0x94
#define	SIBA_GPIO_IN			0xb0
#define	SIBA_GPIO_OUT			0xb4
#define	SIBA_GPIO_OUT_EN		0xb8
#define	SIBA_GPIO_CRYSTAL		0x40
#define	SIBA_GPIO_PLL			0x80

#define SIBA_CORE_IDLO		0x00000ff8
#define SIBA_CORE_IDHI		0x00000ffc

#define  SIBA_IDHIGH_RCLO	0x0000000F /* Revision Code (low part) */
#define  SIBA_IDHIGH_CC		0x00008FF0 /* Core Code */
#define  SIBA_IDHIGH_CC_SHIFT	4
#define  SIBA_IDHIGH_RCHI	0x00007000 /* Revision Code (high part) */
#define  SIBA_IDHIGH_RCHI_SHIFT	8
#define  SIBA_IDHIGH_VC		0xFFFF0000 /* Vendor Code */
#define  SIBA_IDHIGH_VC_SHIFT	16


#define	SIBA_REGWIN(x)							\
	(SIBA_ENUM_START + ((x) * SIBA_CORE_LEN))
#define SIBA_CORE_LEN		0x00001000	/* Size of cfg per core */
#define SIBA_CFG_END		0x00010000	/* Upper bound of cfg space */
#define SIBA_MAX_CORES		(SIBA_CFG_END/SIBA_CORE_LEN)	/* #max cores */
#define	SIBA_ENUM_START			0x18000000U
#define	SIBA_ENUM_END			0x18010000U

#define	SIBA_DMA_TRANSLATION_MASK	0xc0000000

#define	SIBA_PCI_DMA			0x40000000U
#define	SIBA_TPS			0x0f18
#define	SIBA_TPS_BPFLAG			0x0000003f
#define	SIBA_IAS			0x0f90     /* Initiator Agent State */
#define	SIBA_IAS_INBAND_ERR		0x00020000
#define	SIBA_IAS_TIMEOUT		0x00040000
#define	SIBA_INTR_MASK			0x0f94
#define	SIBA_TGSLOW			0x0f98
#define	SIBA_TGSLOW_RESET		0x00000001	/* target state low */
#define	SIBA_TGSLOW_REJECT_22		0x00000002
#define	SIBA_TGSLOW_REJECT_23		0x00000004
#define	SIBA_TGSLOW_CLOCK		0x00010000
#define	SIBA_TGSLOW_FGC			0x00020000
#define	SIBA_TGSHIGH			0x0f9c
#define	SIBA_TGSHIGH_SERR		0x00000001
#define	SIBA_TGSHIGH_BUSY		0x00000004
#define	SIBA_TGSHIGH_DMA64		0x10000000
#define	SIBA_IMCFGLO			0x0fa8
#define	SIBA_IMCFGLO_SERTO		0x00000007
#define	SIBA_IMCFGLO_REQTO		0x00000070
#define	SIBA_IDLOW			0x0ff8
#define	SIBA_IDLOW_SSBREV		0xf0000000
#define	SIBA_IDLOW_SSBREV_22		0x00000000
#define	SIBA_IDLOW_SSBREV_23		0x10000000
#define	SIBA_IDLOW_SSBREV_24		0x40000000
#define	SIBA_IDLOW_SSBREV_25		0x50000000
#define	SIBA_IDLOW_SSBREV_26		0x60000000
#define	SIBA_IDLOW_SSBREV_27		0x70000000
#define	SIBA_IDLOW_CS_MASK		0x3		/* config space */
#define	SIBA_IDLOW_ADDR_RANGE_MASK	0x38		/* # address ranges supported */
#define	SIBA_IDLOW_ADDR_RANGE_SHIFT	3
#define	SIBA_IDLOW_SYNCH		0x40		/* sync */
#define	SIBA_IDLOW_INIT		0x80		/* initiator */
#define	SIBA_IDLOW_MINLAT_MASK	0xf00		/* minimum backplane latency */
#define	SIBA_IDLOW_MINLAT_SHIFT	8
#define	SIBA_IDLOW_MAXLAT		0xf000		/* maximum backplane latency */
#define	SIBA_IDLOW_MAXLAT_SHIFT	12
#define	SIBA_IDLOW_FIRST		0x10000		/* this initiator is first */
#define	SIBA_IDLOW_CW_MASK		0xc0000		/* cycle counter width */
#define	SIBA_IDLOW_CW_SHIFT		18
#define	SIBA_IDLOW_TP_MASK		0xf00000	/* target ports */
#define	SIBA_IDLOW_TP_SHIFT		20
#define	SIBA_IDLOW_IP_MASK		0xf000000	/* initiator ports */
#define	SIBA_IDLOW_IP_SHIFT		24
#define	SIBA_IDLOW_RV_MASK		0xf0000000	/* sonics backplane revision code */
#define	SIBA_IDLOW_RV_SHIFT		28
#define	SIBA_IDLOW_RV_2_2		0x00000000	/* version 2.2 or earlier */
#define	SIBA_IDLOW_RV_2_3		0x10000000	/* version 2.3 */
#define	SIBA_IDHIGH			0x0ffc
#define	SIBA_IDHIGH_CORECODEMASK	0x00008FF0 /* Core Code */
#define	SIBA_IDHIGH_CORECODE_SHIFT	4
#define	SIBA_IDHIGH_CORECODE(id)					\
	((id & SIBA_IDHIGH_CORECODEMASK) >> SIBA_IDHIGH_CORECODE_SHIFT)
/* Revision Code (low part) */
#define	SIBA_IDHIGH_REVLO		0x0000000f
/* Revision Code (high part) */
#define	SIBA_IDHIGH_REVHI		0x00007000
#define	SIBA_IDHIGH_REVHI_SHIFT	8
#define	SIBA_IDHIGH_REV(id)						\
	((id & SIBA_IDHIGH_REVLO) | ((id & SIBA_IDHIGH_REVHI) >>	\
	    SIBA_IDHIGH_REVHI_SHIFT))
#define	SIBA_IDHIGH_VENDORMASK		0xFFFF0000 /* Vendor Code */
#define	SIBA_IDHIGH_VENDOR_SHIFT	16
#define	SIBA_IDHIGH_VENDOR(id)						\
	((id & SIBA_IDHIGH_VENDORMASK) >> SIBA_IDHIGH_VENDOR_SHIFT)


#define SIBA_IPSFLAG		0xf08
#define SIBA_IPSFLAGH		0xf0c
#define SIBA_ADMATCH3		0xf60
#define SIBA_ADMATCH2		0xf68
#define SIBA_ADMATCH1		0xf70
#define SIBA_IMSTATE		0xf90
#define SIBA_INTVEC		0xf94
#define SIBA_TMSLOW		0xf98
#define SIBA_TMSHIGH		0xf9c
#define SIBA_BWA0		0xfa0
#define SIBA_IMCONFIGLOW	0xfa8
#define SIBA_IMCONFIGHIGH	0xfac
#define SIBA_ADMATCH0		0xfb0




#define	SIBA_SPROMSIZE_R123		64
#define	SIBA_SPROMSIZE_R4		220
#define	SIBA_SPROM_BASE			0x1000
#define	SIBA_SPROM_REV_CRC		0xff00
#define	SIBA_SPROM1_MAC_80211BG		0x1048
#define	SIBA_SPROM1_MAC_ETH		0x104e
#define	SIBA_SPROM1_MAC_80211A		0x1054
#define	SIBA_SPROM1_ETHPHY		0x105a
#define	SIBA_SPROM1_ETHPHY_MII_ETH0	0x001f
#define	SIBA_SPROM1_ETHPHY_MII_ETH1	0x03e0
#define	SIBA_SPROM1_ETHPHY_MDIO_ETH0	(1 << 14)
#define	SIBA_SPROM1_ETHPHY_MDIO_ETH1	(1 << 15)
#define	SIBA_SPROM1_BOARDINFO		0x105c
#define	SIBA_SPROM1_BOARDINFO_BREV	0x00ff
#define	SIBA_SPROM1_BOARDINFO_CCODE	0x0f00
#define	SIBA_SPROM1_BOARDINFO_ANTBG	0x3000
#define	SIBA_SPROM1_BOARDINFO_ANTA	0xc000
#define	SIBA_SPROM1_PA0B0		0x105e
#define	SIBA_SPROM1_PA0B1		0x1060
#define	SIBA_SPROM1_PA0B2		0x1062
#define	SIBA_SPROM1_GPIOA		0x1064
#define	SIBA_SPROM1_GPIOA_P0		0x00ff
#define	SIBA_SPROM1_GPIOA_P1		0xff00
#define	SIBA_SPROM1_GPIOB		0x1066
#define	SIBA_SPROM1_GPIOB_P2		0x00ff
#define	SIBA_SPROM1_GPIOB_P3		0xff00
#define	SIBA_SPROM1_MAXPWR		0x1068
#define	SIBA_SPROM1_MAXPWR_BG		0x00ff
#define	SIBA_SPROM1_MAXPWR_A		0xff00
#define	SIBA_SPROM1_PA1B0		0x106a
#define	SIBA_SPROM1_PA1B1		0x106c
#define	SIBA_SPROM1_PA1B2		0x106e
#define	SIBA_SPROM1_TSSI		0x1070
#define	SIBA_SPROM1_TSSI_BG		0x00ff
#define	SIBA_SPROM1_TSSI_A		0xff00
#define	SIBA_SPROM1_BFLOW		0x1072
#define	SIBA_SPROM1_AGAIN		0x1074
#define	SIBA_SPROM1_AGAIN_BG		0x00ff
#define	SIBA_SPROM1_AGAIN_A		0xff00
#define	SIBA_SPROM2_BFHIGH		0x1038
#define	SIBA_SPROM3_MAC_80211BG		0x104a
#define	SIBA_SPROM4_MAC_80211BG		0x104c
#define	SIBA_SPROM4_ETHPHY		0x105a
#define	SIBA_SPROM4_ETHPHY_ET0A		0x001f
#define	SIBA_SPROM4_ETHPHY_ET1A		0x03e0
#define	SIBA_SPROM4_CCODE		0x1052
#define	SIBA_SPROM4_ANTAVAIL		0x105d
#define	SIBA_SPROM4_ANTAVAIL_A		0x00ff
#define	SIBA_SPROM4_ANTAVAIL_BG		0xff00
#define	SIBA_SPROM4_BFLOW		0x1044
#define	SIBA_SPROM4_AGAIN01		0x105e
#define	SIBA_SPROM4_AGAIN0		0x00ff
#define	SIBA_SPROM4_AGAIN1		0xff00
#define	SIBA_SPROM4_AGAIN23		0x1060
#define	SIBA_SPROM4_AGAIN2		0x00ff
#define	SIBA_SPROM4_AGAIN3		0xff00
#define	SIBA_SPROM4_BFHIGH		0x1046
#define	SIBA_SPROM4_MAXP_BG		0x1080
#define	SIBA_SPROM4_MAXP_BG_MASK	0x00ff
#define	SIBA_SPROM4_TSSI_BG		0xff00
#define	SIBA_SPROM4_MAXP_A		0x108a
#define	SIBA_SPROM4_MAXP_A_MASK		0x00ff
#define	SIBA_SPROM4_TSSI_A		0xff00
#define	SIBA_SPROM4_GPIOA		0x1056
#define	SIBA_SPROM4_GPIOA_P0		0x00ff
#define	SIBA_SPROM4_GPIOA_P1		0xff00
#define	SIBA_SPROM4_GPIOB		0x1058
#define	SIBA_SPROM4_GPIOB_P2		0x00ff
#define	SIBA_SPROM4_GPIOB_P3		0xff00
#define	SIBA_SPROM5_BFLOW		0x104a
#define	SIBA_SPROM5_BFHIGH		0x104c
#define	SIBA_SPROM5_MAC_80211BG		0x1052
#define	SIBA_SPROM5_CCODE		0x1044
#define	SIBA_SPROM5_GPIOA		0x1076
#define	SIBA_SPROM5_GPIOA_P0		0x00ff
#define	SIBA_SPROM5_GPIOA_P1		0xff00
#define	SIBA_SPROM5_GPIOB		0x1078
#define	SIBA_SPROM5_GPIOB_P2		0x00ff
#define	SIBA_SPROM5_GPIOB_P3		0xff00
#define	SIBA_SPROM8_BFLOW		0x1084
#define	SIBA_SPROM8_BFHIGH		0x1086
#define	SIBA_SPROM8_CCODE		0x1092
#define	SIBA_SPROM8_ANTAVAIL		0x109c
#define	SIBA_SPROM8_ANTAVAIL_A		0xff00
#define	SIBA_SPROM8_ANTAVAIL_BG		0x00ff
#define	SIBA_SPROM8_AGAIN01		0x109e
#define	SIBA_SPROM8_AGAIN0		0x00ff
#define	SIBA_SPROM8_AGAIN1		0xff00
#define	SIBA_SPROM8_AGAIN23		0x10a0
#define	SIBA_SPROM8_AGAIN2		0x00ff
#define	SIBA_SPROM8_AGAIN3		0xff00
#define	SIBA_SPROM8_GPIOA		0x1096
#define	SIBA_SPROM8_GPIOA_P0		0x00ff
#define	SIBA_SPROM8_GPIOA_P1		0xff00
#define	SIBA_SPROM8_GPIOB		0x1098
#define	SIBA_SPROM8_GPIOB_P2		0x00ff
#define	SIBA_SPROM8_GPIOB_P3		0xff00
#define	SIBA_SPROM8_MAXP_BG		0x10c0
#define	SIBA_SPROM8_MAXP_BG_MASK	0x00ff
#define	SIBA_SPROM8_TSSI_BG		0xff00
#define	SIBA_SPROM8_MAXP_A		0x10c8
#define	SIBA_SPROM8_MAXP_A_MASK		0x00ff
#define	SIBA_SPROM8_TSSI_A		0xff00

#define	SIBA_BOARDVENDOR_DELL		0x1028
#define	SIBA_BOARDVENDOR_BCM		0x14e4
#define	SIBA_BOARD_BCM4309G		0x0421
#define	SIBA_BOARD_MP4318		0x044a
#define	SIBA_BOARD_BU4306		0x0416
#define	SIBA_BOARD_BU4309		0x040a

#define	SIBA_PCICORE_BCAST_ADDR		SIBA_CC_BCAST_ADDR
#define	SIBA_PCICORE_BCAST_DATA		SIBA_CC_BCAST_DATA
#define	SIBA_PCICORE_SBTOPCI0		0x0100
#define	SIBA_PCICORE_SBTOPCI1		0x0104
#define	SIBA_PCICORE_SBTOPCI2		0x0108
#define	SIBA_PCICORE_MDIO_CTL		0x0128
#define	SIBA_PCICORE_MDIO_DATA		0x012c
#define	SIBA_PCICORE_SBTOPCI_PREF	0x00000004
#define	SIBA_PCICORE_SBTOPCI_BURST	0x00000008
#define	SIBA_PCICORE_SBTOPCI_MRM	0x00000020
#define SIBA_CCID_BCM5354	0x5354




/* Interrupt control */
#define SIBA_CC_INTSTATUS	0x20
#define SIBA_CC_INTMASK		0x24
#define SIBA_CC_CHIPCONTROL 	0x28 /* rev >= 11 */
#define SIBA_CC_CHIPSTATUS	0x2c /* rev >= 11 */


/* intstatus/intmask */
#define	CI_GPIO			0x00000001	/* gpio intr */
#define	CI_EI			0x00000002	/* extif intr (corerev >= 3) */
#define	CI_TEMP			0x00000004	/* temp. ctrl intr (corerev >= 15) */
#define	CI_SIRQ			0x00000008	/* serial IRQ intr (corerev >= 15) */
#define	CI_ECI			0x00000010	/* eci intr (corerev >= 21) */
#define	CI_PMU			0x00000020	/* pmu intr (corerev >= 21) */
#define	CI_UART			0x00000040	/* uart intr (corerev >= 21) */
#define	CI_WDRESET		0x80000000	/* watchdog reset occurred */



/*
 * Sonics Configuration Space Registers.
 */
#define SB_CORE_CFG_BASE	0xf00
 

#define SBIPSFLAG		0x08
#define SBTPSFLAG		0x18
#define	SBTMERRLOGA		0x48		/* sonics >= 2.3 */
#define	SBTMERRLOG		0x50		/* sonics >= 2.3 */
#define SBADMATCH3		0x60
#define SBADMATCH2		0x68
#define SBADMATCH1		0x70
#define SBIMSTATE		0x90
#define SBINTVEC		0x94
#define SBTMSTATELOW		0x98
#define SBTMSTATEHIGH		0x9c
#define SBBWA0			0xa0
#define SBIMCONFIGLOW		0xa8
#define SBIMCONFIGHIGH		0xac
#define SBADMATCH0		0xb0
#define SBTMCONFIGLOW		0xb8
#define SBTMCONFIGHIGH		0xbc
#define SBBCONFIG		0xc0
#define SBBSTATE		0xc8
#define SBACTCNFG		0xd8
#define	SBFLAGST		0xe8
#define SBIDLOW			0xf8
#define SBIDHIGH		0xfc

#define SSB_IPSFLAG		0x08
#define SSB_TPSFLAG		0x18
#define	SSB_TMERRLOGA		0x48		/* sonics >= 2.3 */
#define	SSB_TMERRLOG		0x50		/* sonics >= 2.3 */
#define SSB_TMCONFIGLOW		0xb8
#define SSB_TMCONFIGHIGH		0xbc
#define SSB_BCONFIG		0xc0
#define SSB_BSTATE		0xc8
#define SSB_ACTCNFG		0xd8
#define	SSB_FLAGST		0xe8
#define SSB_IDLOW			0xf8
#define SSB_IDHIGH		0xfc
/* All the previous registers are above SBCONFIGOFF, but with Sonics 2.3, we have
 * a few registers *below* that line. I think it would be very confusing to try
 * and change the value of SBCONFIGOFF, so I'm definig them as absolute offsets here,
 */

#define SBIMERRLOGA		0xea8
#define SBIMERRLOG		0xeb0
#define SBTMPORTCONNID0		0xed8
#define SBTMPORTLOCK0		0xef8
#ifndef PAD
#define	_PADLINE(line)	pad ## line
#define	_XSTR(line)	_PADLINE(line)
#define	PAD		_XSTR(__LINE__)
#endif	/* PAD */


/* sbipsflag */
#define	SBIPS_INT1_MASK		0x3f		/* which sbflags get routed to mips interrupt 1 */
#define	SBIPS_INT1_SHIFT	0
#define	SBIPS_INT2_MASK		0x3f00		/* which sbflags get routed to mips interrupt 2 */
#define	SBIPS_INT2_SHIFT	8
#define	SBIPS_INT3_MASK		0x3f0000	/* which sbflags get routed to mips interrupt 3 */
#define	SBIPS_INT3_SHIFT	16
#define	SBIPS_INT4_MASK		0x3f000000	/* which sbflags get routed to mips interrupt 4 */
#define	SBIPS_INT4_SHIFT	24

/* sbtpsflag */
#define	SBTPS_NUM0_MASK		0x3f		/* interrupt sbFlag # generated by this core */
#define	SBTPS_F0EN0		0x40		/* interrupt is always sent on the backplane */

/* sbtmerrlog */
#define	SBTMEL_CM		0x00000007	/* command */
#define	SBTMEL_CI		0x0000ff00	/* connection id */
#define	SBTMEL_EC		0x0f000000	/* error code */
#define	SBTMEL_ME		0x80000000	/* multiple error */

/* sbimstate */
#define	SBIM_PC			0xf		/* pipecount */
#define	SBIM_AP_MASK		0x30		/* arbitration policy */
#define	SBIM_AP_BOTH		0x00		/* use both timeslaces and token */
#define	SBIM_AP_TS		0x10		/* use timesliaces only */
#define	SBIM_AP_TK		0x20		/* use token only */
#define	SBIM_AP_RSV		0x30		/* reserved */
#define	SBIM_IBE		0x20000		/* inbanderror */
#define	SBIM_TO			0x40000		/* timeout */
#define	SBIM_BY			0x01800000	/* busy (sonics >= 2.3) */
#define	SBIM_RJ			0x02000000	/* reject (sonics >= 2.3) */
/* Aliases */
#define	SSB_IMSTATE_IBE		0x20000		/* inbanderror */
#define	SSB_IMSTATE_TO		0x40000		/* timeout */


/* sbtmstatelow */
#define	SBTML_RESET		0x1		/* reset */
#define	SBTML_REJ_MASK		0x6		/* reject */
#define	SBTML_REJ_SHIFT		1
#define	SBTML_CLK		0x10000		/* clock enable */
#define	SBTML_FGC		0x20000		/* force gated clocks on */
#define	SBTML_FL_MASK		0x3ffc0000	/* core-specific flags */
#define	SBTML_PE		0x40000000	/* pme enable */
#define	SBTML_BE		0x80000000	/* bist enable */

/* Aliases */
#define	SSB_TMSLOW_RESET	0x1		/* reset */
#define	SSB_TMSLOW_REJ_MASK	0x6		/* reject */
#define	SSB_TMSLOW_REJ_SHIFT	1
#define  SSB_TMSLOW_REJECT_22	0x00000002 /* Reject (Backplane rev 2.2) */
#define  SSB_TMSLOW_REJECT_23	0x00000004 /* Reject (Backplane rev 2.3) */
#define	SSB_TMSLOW_CLOCK	0x10000		/* clock enable */
#define	SSB_TMSLOW_FGC		0x20000		/* force gated clocks on */
#define	SSB_TMSLOW_FL_MASK	0x3ffc0000	/* core-specific flags */
#define	SSB_TMSLOW_PE		0x40000000	/* pme enable */
#define	SSB_TMSLOW_BE		0x80000000	/* bist enable */



/* sbtmstatehigh */
#define	SBTMH_SERR		0x1		/* serror */
#define	SBTMH_INT		0x2		/* interrupt */
#define	SBTMH_BUSY		0x4		/* busy */
#define	SBTMH_TO		0x00000020	/* timeout (sonics >= 2.3) */
#define	SBTMH_FL_MASK		0x0fff0000	/* core-specific flags */
#define SBTMH_DMA64		0x10000000      /* supports DMA with 64-bit addresses */
#define	SBTMH_GCR		0x20000000	/* gated clock request */
#define	SBTMH_BISTF		0x40000000	/* bist failed */
#define	SBTMH_BISTD		0x80000000	/* bist done */
/* Aliases */
#define	SSB_TMSHIGH_SERR	0x1		/* serror */
#define	SSB_TMSHIGH_INT		0x2		/* interrupt */
#define	SSB_TMSHIGH_BUSY	0x4		/* busy */
#define	SSB_TMSHIGH_TO		0x00000020	/* timeout (sonics >= 2.3) */
#define	SSB_TMSHIGH_FL_MASK	0x0fff0000	/* core-specific flags */
#define SSB_TMSHIGH_DMA64	0x10000000      /* supports DMA with 64-bit addresses */
#define	SSB_TMSHIGH_GCR		0x20000000	/* gated clock request */
#define	SSB_TMSHIGH_BISTF	0x40000000	/* bist failed */
#define	SSB_TMSHIGH_BISTD	0x80000000	/* bist done */


/* sbbwa0 */
#define	SBBWA_TAB0_MASK		0xffff		/* lookup table 0 */
#define	SBBWA_TAB1_MASK		0xffff		/* lookup table 1 */
#define	SBBWA_TAB1_SHIFT	16

/* sbimconfiglow */
#define	SBIMCL_STO_MASK		0x7		/* service timeout */
#define	SBIMCL_RTO_MASK		0x70		/* request timeout */
#define	SBIMCL_RTO_SHIFT	4
#define	SBIMCL_CID_MASK		0xff0000	/* connection id */
#define	SBIMCL_CID_SHIFT	16

/* sbimconfighigh */
#define	SBIMCH_IEM_MASK		0xc		/* inband error mode */
#define	SBIMCH_TEM_MASK		0x30		/* timeout error mode */
#define	SBIMCH_TEM_SHIFT	4
#define	SBIMCH_BEM_MASK		0xc0		/* bus error mode */
#define	SBIMCH_BEM_SHIFT	6

/* sbadmatch0 */
#define	SBAM_TYPE_MASK		0x3		/* address type */
#define	SBAM_AD64		0x4		/* reserved */
#define	SBAM_ADINT0_MASK	0xf8		/* type0 size */
#define	SBAM_ADINT0_SHIFT	3
#define	SBAM_ADINT1_MASK	0x1f8		/* type1 size */
#define	SBAM_ADINT1_SHIFT	3
#define	SBAM_ADINT2_MASK	0x1f8		/* type2 size */
#define	SBAM_ADINT2_SHIFT	3
#define	SBAM_ADEN		0x400		/* enable */
#define	SBAM_ADNEG		0x800		/* negative decode */
#define	SBAM_BASE0_MASK		0xffffff00	/* type0 base address */
#define	SBAM_BASE0_SHIFT	8
#define	SBAM_BASE1_MASK		0xfffff000	/* type1 base address for the core */
#define	SBAM_BASE1_SHIFT	12
#define	SBAM_BASE2_MASK		0xffff0000	/* type2 base address for the core */
#define	SBAM_BASE2_SHIFT	16

/* sbtmconfiglow */
#define	SBTMCL_CD_MASK		0xff		/* clock divide */
#define	SBTMCL_CO_MASK		0xf800		/* clock offset */
#define	SBTMCL_CO_SHIFT		11
#define	SBTMCL_IF_MASK		0xfc0000	/* interrupt flags */
#define	SBTMCL_IF_SHIFT		18
#define	SBTMCL_IM_MASK		0x3000000	/* interrupt mode */
#define	SBTMCL_IM_SHIFT		24

/* sbtmconfighigh */
#define	SBTMCH_BM_MASK		0x3		/* busy mode */
#define	SBTMCH_RM_MASK		0x3		/* retry mode */
#define	SBTMCH_RM_SHIFT		2
#define	SBTMCH_SM_MASK		0x30		/* stop mode */
#define	SBTMCH_SM_SHIFT		4
#define	SBTMCH_EM_MASK		0x300		/* sb error mode */
#define	SBTMCH_EM_SHIFT		8
#define	SBTMCH_IM_MASK		0xc00		/* int mode */
#define	SBTMCH_IM_SHIFT		10

/* sbbconfig */
#define	SBBC_LAT_MASK		0x3		/* sb latency */
#define	SBBC_MAX0_MASK		0xf0000		/* maxccntr0 */
#define	SBBC_MAX0_SHIFT		16
#define	SBBC_MAX1_MASK		0xf00000	/* maxccntr1 */
#define	SBBC_MAX1_SHIFT		20

/* sbbstate */
#define	SBBS_SRD		0x1		/* st reg disable */
#define	SBBS_HRD		0x2		/* hold reg disable */


/* sbidhigh */
#define	SBIDH_RC_MASK		0x000f		/* revision code */
#define	SBIDH_RCE_MASK		0x7000		/* revision code extension field */
#define	SBIDH_RCE_SHIFT		8
#define	SBCOREREV(sbidh) \
	((((sbidh) & SBIDH_RCE_MASK) >> SBIDH_RCE_SHIFT) | ((sbidh) & SBIDH_RC_MASK))
#define	SBIDH_CC_MASK		0x8ff0		/* core code */
#define	SBIDH_CC_SHIFT		4
#define	SBIDH_VC_MASK		0xffff0000	/* vendor code */
#define	SBIDH_VC_SHIFT		16

#define	SB_COMMIT		0xfd8		/* update buffered registers value */

/* vendor codes */
#define	SB_VEND_BCM		0x4243		/* Broadcom's SB vendor code */


#endif /* _SIBA_SIBAREG_H_ */
