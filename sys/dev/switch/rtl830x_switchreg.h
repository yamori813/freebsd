/*-
 * Copyright (c) 2011,2012 Aleksandr Rybalko.
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
 * $FreeBSD$
 */

#ifndef _RTL830X_SWITCHREG_H_
#define	_RTL830X_SWITCHREG_H_

enum phy_names {
	PHY0 = 0,
	PHY1,
	PHY2,
	PHY3,
	PHY4,
	PHY5,
	PHY6,
	PHY7,
	PHY8,
	PHY24 = 24
};

#define	ID(_phy, _reg)	((_phy)<<16 | (_reg))

#define	PHY_CTL(p)		ID(p, 0)
#define	PHY0_CTL		ID(PHY0, 0)
#define	PHY1_CTL		ID(PHY1, 0)
#define	PHY2_CTL		ID(PHY2, 0)
#define	PHY3_CTL		ID(PHY3, 0)
#define	PHY4_CTL		ID(PHY4, 0)
#define	PHY5_CTL		ID(PHY5, 0)
#define	PHY6_CTL		ID(PHY6, 0)
#define	PHY7_CTL		ID(PHY7, 0)
#define		PHY_CTL_RESET		(1<<15)
#define		PHY_CTL_LOOPBACK	(1<<14)
#define		PHY_CTL_100M		(1<<13)
#define		PHY_CTL_ANEG		(1<<12)
#define		PHY_CTL_POWERDOWN	(1<<11)
#define		PHY_CTL_ISOLATE		(1<<10)
#define		PHY_CTL_ANEG_RESTART	(1<< 9)
#define		PHY_CTL_FDX		(1<< 8)

#define	PHY_STS(p)		ID(p, 1)
#define	PHY0_STS		ID(PHY0, 1)
#define	PHY1_STS		ID(PHY1, 1)
#define	PHY2_STS		ID(PHY2, 1)
#define	PHY3_STS		ID(PHY3, 1)
#define	PHY4_STS		ID(PHY4, 1)
#define	PHY5_STS		ID(PHY5, 1)
#define	PHY6_STS		ID(PHY6, 1)
#define	PHY7_STS		ID(PHY7, 1)
#define		PHY_STS_100_T4		(1<<15)
#define		PHY_STS_100_F		(1<<14)
#define		PHY_STS_100_H		(1<<13)
#define		PHY_STS_10_F		(1<<12)
#define		PHY_STS_10_H		(1<<11)
#define		PHY_STS_MF_PRMBL	(1<< 6)
#define		PHY_STS_ANEG_CMPL	(1<< 5)
#define		PHY_STS_RMT_FLT		(1<< 4)
#define		PHY_STS_NW_ANEG		(1<< 3)
#define		PHY_STS_LINK_UP		(1<< 2)
#define		PHY_STS_JABBER		(1<< 1)
#define		PHY_STS_EXT_CAP		(1<< 0)

#define	PHY_ANEG_ADV(p)		ID(p, 4)
#define	PHY0_ANEG_ADV		ID(PHY0, 4)
#define	PHY1_ANEG_ADV		ID(PHY1, 4)
#define	PHY2_ANEG_ADV		ID(PHY2, 4)
#define	PHY3_ANEG_ADV		ID(PHY3, 4)
#define	PHY4_ANEG_ADV		ID(PHY4, 4)
#define	PHY5_ANEG_ADV		ID(PHY5, 4)
#define	PHY6_ANEG_ADV		ID(PHY6, 4)
#define	PHY7_ANEG_ADV		ID(PHY7, 4)
#define		PHY_ANEG_ADV_NP_DIS		(1<<15)
#define		PHY_ANEG_ADV_ACK		(1<<14)
#define		PHY_ANEG_ADV_FLT_ADV		(1<<13)
#define		PHY_ANEG_ADV_ADV		(1<<10)
#define		PHY_ANEG_ADV_100_T4		(1<< 9)
#define		PHY_ANEG_ADV_100_F		(1<< 8)
#define		PHY_ANEG_ADV_100_H		(1<< 7)
#define		PHY_ANEG_ADV_10_F		(1<< 6)
#define		PHY_ANEG_ADV_10_H		(1<< 5)
#define		PHY_ANEG_ADV_SEL_FLD_SHIFT	0
#define		PHY_ANEG_ADV_SEL_FLD_MASK	0x1f

#define	PHY_ANEG_PRTN(p)	ID(p,    5)
#define	PHY0_ANEG_PRTN		ID(PHY0, 5)
#define	PHY1_ANEG_PRTN		ID(PHY1, 5)
#define	PHY2_ANEG_PRTN		ID(PHY2, 5)
#define	PHY3_ANEG_PRTN		ID(PHY3, 5)
#define	PHY4_ANEG_PRTN		ID(PHY4, 5)
#define	PHY5_ANEG_PRTN		ID(PHY5, 5)
#define	PHY6_ANEG_PRTN		ID(PHY6, 5)
#define	PHY7_ANEG_PRTN		ID(PHY7, 5)
#define		PHY_ANEG_PRTN_NP_DIS		(1<<15)
#define		PHY_ANEG_PRTN_ACK		(1<<14)
#define		PHY_ANEG_PRTN_RMT_FLT		(1<<13)
#define		PHY_ANEG_PRTN_FC_ABLE		(1<<10)
#define		PHY_ANEG_PRTN_100_T4		(1<< 9)
#define		PHY_ANEG_PRTN_100_F		(1<< 8)
#define		PHY_ANEG_PRTN_100_H		(1<< 7)
#define		PHY_ANEG_PRTN_10_F		(1<< 6)
#define		PHY_ANEG_PRTN_10_H		(1<< 5)
#define		PHY_ANEG_PRTN_SEL_FLD_SHIFT	0
#define		PHY_ANEG_PRTN_SEL_FLD_MASK	0x1f

#define	PHY_ANEG_EXP(p)		ID(p,    6)
#define	PHY0_ANEG_EXP		ID(PHY0, 6)
#define	PHY1_ANEG_EXP		ID(PHY1, 6)
#define	PHY2_ANEG_EXP		ID(PHY2, 6)
#define	PHY3_ANEG_EXP		ID(PHY3, 6)
#define	PHY4_ANEG_EXP		ID(PHY4, 6)
#define	PHY5_ANEG_EXP		ID(PHY5, 6)
#define	PHY6_ANEG_EXP		ID(PHY6, 6)
#define	PHY7_ANEG_EXP		ID(PHY7, 6)
#define		PHY_ANEG_EXP_PD_FLT	(1<< 4)
#define		PHY_ANEG_EXP_PRTN_NXTPG	(1<< 3)
#define		PHY_ANEG_EXP_NXTPG	(1<< 2)
#define		PHY_ANEG_EXP_PG_RCV	(1<< 1)
#define		PHY_ANEG_EXP_PRTN_ANEG	(1<< 0)

#define	GCNTRL0 ID(PHY0, 16)
#define	GCNTRL0_LED_MODE_SHIFT	13
#define	GCNTRL0_LED_MODE_MASK	0xe000
#define		GCNTRL0_SOFT_RSET	(1<<12)
#define		GCNTRL0_VLAN_DIS	(1<<11)
#define		GCNTRL0_DOT1Q_DIS	(1<<10)
#define		GCNTRL0_INGRESS_CHECK_DIS (1<<9)
#define		GCNTRL0_TAG_ONLY_DIS	(1<<8)
#define		GCNTRL0_EEPROM 		(1<<7)
#define		GCNTRL0_FILTER_BAD	(1<<6)
#define		GCNTRL0_TX_FC		(1<<5)
#define		GCNTRL0_RX_FC		(1<<4)
#define		GCNTRL0_BCAST_IN_DROP	(1<<3)
#define		GCNTRL0_AGN_EN 		(1<<2)
#define		GCNTRL0_FAST_AGN	(1<<1)
#define		GCNTRL0_MACAT		(1<<0)

#define	GCNTRL1 ID(PHY0, 17)
#define	GCNTRL2 ID(PHY0, 18)
#define	GCNTRL3 ID(PHY0, 19)

#define	PHY_CTRL0(p)		ID(p, 22)
#define	PHY0_CTRL0		ID(PHY0, 22)
#define	PHY1_CTRL0		ID(PHY1, 22)
#define	PHY2_CTRL0		ID(PHY2, 22)
#define	PHY3_CTRL0		ID(PHY3, 22)
#define	PHY4_CTRL0		ID(PHY4, 22)
#define	PHY5_CTRL0		ID(PHY5, 22)
#define	PHY6_CTRL0		ID(PHY6, 22)
#define	PHY7_CTRL0		ID(PHY7, 22)
#define		CTRL0_LOOP		(1<<13)
#define		CTRL0_R_NL_VID		(1<<12)
#define		CTRL0_I_CH_EN		(1<<11)
#define		CTRL0_1P_DIS		(1<<10)
#define		CTRL0_DFSRV_DIS		(1<<9)
#define		CTRL0_PQOS_DIS		(1<<8)
#define		CTRL0_TAG_MODE_MASK	3
#define		CTRL0_TAG_NI_NR		3 /* Not insert, not remove */
#define		CTRL0_TAG_I_UTG		2 /* Ingress Insert for untagged */
#define		CTRL0_TAG_R_TG		1 /* Remove Tag on Egress */
#define		CTRL0_TAG_IR		0 /* Insert or Replace Tag */

#define	PHY_CTRL1(p)		ID(p, 23)
#define	PHY0_CTRL1		ID(PHY0, 23)
#define	PHY1_CTRL1		ID(PHY1, 23)
#define	PHY2_CTRL1		ID(PHY2, 23)
#define	PHY3_CTRL1		ID(PHY3, 23)
#define	PHY4_CTRL1		ID(PHY4, 23)
#define	PHY5_CTRL1		ID(PHY5, 23)
#define	PHY6_CTRL1		ID(PHY6, 23)
#define	PHY7_CTRL1		ID(PHY7, 23)
#define		CTRL1_TX_EN		(1<<11)
#define		CTRL1_RX_EN		(1<<10)
#define		CTRL1_LRN_EN		(1<<9)
#define		CTRL1_LOOP_DET		(1<<8)
#define		CTRL1_LQ_MASK		0x00f0
#define		CTRL1_LQ_SHIFT		4

#define	PORT_PVID(p)	ID((PHY0+p), 24)
#define	VLAN_MEMB(v)	ID((PHY0+v), 24)
#define	VLAN_ID(v)	ID((PHY0+v), 25)
#define		VLAN_ID_MASK		0x0fff
#define		VLAN_ID_SHIFT		0

#define	PORT0_PVID	ID(PHY0, 24)
#define	VLAN_A_MEMB	ID(PHY0, 24)
#define	VLAN_A_ID	ID(PHY0, 25)

#define	PORT8_PVID	ID(PHY5, 17)
#define	VLAN_I_MEMB	ID(PHY5, 17)
#define	VLAN_I_ID	ID(PHY5, 18)

#define	PORT0_PVID_IDX_SHIFT	12
#define	PORT0_PVID_IDX_MASK	0xf000
#define	PORT8_PVID_IDX_SHIFT	9
#define	PORT8_PVID_IDX_MASK	0x1e00
#define		VLAN_A_MEMB_SHIFT	0
#define		VLAN_A_MEMB_MASK	0x01ff

#endif /* _RTL830X_SWITCHREG_H_ */

