/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017~2018 NXP
 *
 */

#ifndef __DT_BINDINGS_CLOCK_IMX7ULP_H
#define __DT_BINDINGS_CLOCK_IMX7ULP_H

/* SCG1 */

#define IMX7ULP_CLK_DUMMY		0
#define IMX7ULP_CLK_ROSC		1
#define IMX7ULP_CLK_SOSC		2
#define IMX7ULP_CLK_FIRC		3
#define IMX7ULP_CLK_SPLL_PRE_SEL	4
#define IMX7ULP_CLK_SPLL_PRE_DIV	5
#define IMX7ULP_CLK_SPLL		6
#define IMX7ULP_CLK_SPLL_POST_DIV1	7
#define IMX7ULP_CLK_SPLL_POST_DIV2	8
#define IMX7ULP_CLK_SPLL_PFD0		9
#define IMX7ULP_CLK_SPLL_PFD1		10
#define IMX7ULP_CLK_SPLL_PFD2		11
#define IMX7ULP_CLK_SPLL_PFD3		12
#define IMX7ULP_CLK_SPLL_PFD_SEL	13
#define IMX7ULP_CLK_SPLL_SEL		14
#define IMX7ULP_CLK_APLL_PRE_SEL	15
#define IMX7ULP_CLK_APLL_PRE_DIV	16
#define IMX7ULP_CLK_APLL		17
#define IMX7ULP_CLK_APLL_POST_DIV1	18
#define IMX7ULP_CLK_APLL_POST_DIV2	19
#define IMX7ULP_CLK_APLL_PFD0		20
#define IMX7ULP_CLK_APLL_PFD1		21
#define IMX7ULP_CLK_APLL_PFD2		22
#define IMX7ULP_CLK_APLL_PFD3		23
#define IMX7ULP_CLK_APLL_PFD_SEL	24
#define IMX7ULP_CLK_APLL_SEL		25
#define IMX7ULP_CLK_UPLL		26
#define IMX7ULP_CLK_SYS_SEL		27
#define IMX7ULP_CLK_CORE_DIV		28
#define IMX7ULP_CLK_BUS_DIV		29
#define IMX7ULP_CLK_PLAT_DIV		30
#define IMX7ULP_CLK_DDR_SEL		31
#define IMX7ULP_CLK_DDR_DIV		32
#define IMX7ULP_CLK_NIC_SEL		33
#define IMX7ULP_CLK_NIC0_DIV		34
#define IMX7ULP_CLK_GPU_DIV		35
#define IMX7ULP_CLK_NIC1_DIV		36
#define IMX7ULP_CLK_NIC1_BUS_DIV	37
#define IMX7ULP_CLK_NIC1_EXT_DIV	38
#define IMX7ULP_CLK_MIPI_PLL		39
#define IMX7ULP_CLK_SIRC		40
#define IMX7ULP_CLK_SOSC_BUS_CLK	41
#define IMX7ULP_CLK_FIRC_BUS_CLK	42
#define IMX7ULP_CLK_SPLL_BUS_CLK	43
#define IMX7ULP_CLK_HSRUN_SYS_SEL	44
#define IMX7ULP_CLK_HSRUN_CORE_DIV	45

#define IMX7ULP_CLK_SCG1_END		46

/* PCC2 */
#define IMX7ULP_CLK_DMA1		0
#define IMX7ULP_CLK_RGPIO2P1		1
#define IMX7ULP_CLK_FLEXBUS		2
#define IMX7ULP_CLK_SEMA42_1		3
#define IMX7ULP_CLK_DMA_MUX1		4
#define IMX7ULP_CLK_CAAM		6
#define IMX7ULP_CLK_LPTPM4		7
#define IMX7ULP_CLK_LPTPM5		8
#define IMX7ULP_CLK_LPIT1		9
#define IMX7ULP_CLK_LPSPI2		10
#define IMX7ULP_CLK_LPSPI3		11
#define IMX7ULP_CLK_LPI2C4		12
#define IMX7ULP_CLK_LPI2C5		13
#define IMX7ULP_CLK_LPUART4		14
#define IMX7ULP_CLK_LPUART5		15
#define IMX7ULP_CLK_FLEXIO1		16
#define IMX7ULP_CLK_USB0		17
#define IMX7ULP_CLK_USB1		18
#define IMX7ULP_CLK_USB_PHY		19
#define IMX7ULP_CLK_USB_PL301		20
#define IMX7ULP_CLK_USDHC0		21
#define IMX7ULP_CLK_USDHC1		22
#define IMX7ULP_CLK_WDG1		23
#define IMX7ULP_CLK_WDG2		24

#define IMX7ULP_CLK_PCC2_END		25

/* PCC3 */
#define IMX7ULP_CLK_LPTPM6		0
#define IMX7ULP_CLK_LPTPM7		1
#define IMX7ULP_CLK_LPI2C6		2
#define IMX7ULP_CLK_LPI2C7		3
#define IMX7ULP_CLK_LPUART6		4
#define IMX7ULP_CLK_LPUART7		5
#define IMX7ULP_CLK_VIU			6
#define IMX7ULP_CLK_DSI			7
#define IMX7ULP_CLK_LCDIF		8
#define IMX7ULP_CLK_MMDC		9
#define IMX7ULP_CLK_PCTLC		10
#define IMX7ULP_CLK_PCTLD		11
#define IMX7ULP_CLK_PCTLE		12
#define IMX7ULP_CLK_PCTLF		13
#define IMX7ULP_CLK_GPU3D		14
#define IMX7ULP_CLK_GPU2D		15

#define IMX7ULP_CLK_PCC3_END		16

/* SMC1 */
#define IMX7ULP_CLK_ARM			0

#define IMX7ULP_CLK_SMC1_END		1

#endif /* __DT_BINDINGS_CLOCK_IMX7ULP_H */
