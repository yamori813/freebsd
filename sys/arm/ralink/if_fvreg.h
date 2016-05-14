/*-
 * Copyright (C) 2007 
 *	Oleksandr Tymoshenko <gonzo@freebsd.org>. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR HIS RELATIVES BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF MIND, USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 *
 */

#ifndef __IF_FVREG_H__
#define __IF_FVREG_H__

#define	FV_ETHINTFC 	0x0000	/* Ethernet interface control             */
#define		ETH_INTFC_EN 	0x0001
#define		ETH_INTFC_RIP 	0x0004
#define		ETH_INTFC_EN 	0x0001
#define	FV_ETHFIFOTT	0x0004	/* Ethernet FIFO transmit threshold       */
#define	FV_ETHARC   	0x0008	/* Ethernet address recognition control   */
#define	FV_ETHHASH0 	0x000C	/* Ethernet hash table 0                  */
#define	FV_ETHHASH1 	0x0010	/* Ethernet hash table 1                  */
#define	FV_ETHPFS   	0x0024	/* Ethernet pause frame status            */
#define	FV_ETHMCP   	0x0028	/* Ethernet management clock prescalar    */
#define	FV_ETHSAL0  	0x0100	/* Ethernet station address 0 low         */
#define	FV_ETHSAH0  	0x0104	/* Ethernet station address 0 high        */
#define	FV_ETHSAL1  	0x0108	/* Ethernet station address 1 low         */
#define	FV_ETHSAH1  	0x010C	/* Ethernet station address 1 high        */
#define	FV_ETHSAL2  	0x0110	/* Ethernet station address 2 low         */
#define	FV_ETHSAH2  	0x0114	/* Ethernet station address 2 high        */
#define	FV_ETHSAL3  	0x0118	/* Ethernet station address 3 low         */
#define	FV_ETHSAH3  	0x011C	/* Ethernet station address 3 high        */
#define	FV_ETHRBC   	0x0120	/* Ethernet receive byte count            */
#define	FV_ETHRPC   	0x0124	/* Ethernet receive packet count          */
#define	FV_ETHRUPC  	0x0128	/* Ethernet receive undersized packet cnt */
#define	FV_ETHRFC   	0x012C	/* Ethernet receive fragment count        */
#define	FV_ETHTBC   	0x0130	/* Ethernet transmit byte count           */
#define	FV_ETHGPF   	0x0134	/* Ethernet generate pause frame          */
#define	FV_ETHMAC1 	0x0200	/* Ethernet MAC configuration 1           */
#define		FV_ETH_MAC1_RE	0x01
#define		FV_ETH_MAC1_PAF	0x02
#define		FV_ETH_MAC1_MR	0x80
#define	FV_ETHMAC2 	0x0204	/* Ethernet MAC configuration 2           */
#define		FV_ETH_MAC2_FD	0x01
#define		FV_ETH_MAC2_FLC	0x02
#define		FV_ETH_MAC2_HFE	0x04
#define		FV_ETH_MAC2_DC	0x08
#define		FV_ETH_MAC2_CEN	0x10
#define		FV_ETH_MAC2_PEN	0x20
#define		FV_ETH_MAC2_VPE	0x08
#define	FV_ETHIPGT 	0x0208	/* Ethernet back-to-back inter-packet gap */
#define	FV_ETHIPGR 	0x020C	/* Ethernet non back-to-back inter-packet gap */
#define	FV_ETHCLRT 	0x0210	/* Ethernet collision window retry        */
#define	FV_ETHMAXF 	0x0214	/* Ethernet maximum frame length          */
#define	FV_ETHMTEST	0x021C	/* Ethernet MAC test                      */
#define	FV_MIIMCFG 	0x0220	/* MII management configuration           */
#define		FV_MIIMCFG_R	0x8000 	
#define	FV_MIIMCMD 	0x0224	/* MII management command                 */
#define		FV_MIIMCMD_RD 	0x01
#define		FV_MIIMCMD_SCN 	0x02
#define	FV_MIIMADDR	0x0228	/* MII management address                 */
#define	FV_MIIMWTD 	0x022C	/* MII management write data              */
#define	FV_MIIMRDD 	0x0230	/* MII management read data               */
#define	FV_MIIMIND 	0x0234	/* MII management indicators              */
#define		FV_MIIMIND_BSY 	0x1
#define		FV_MIIMIND_SCN 	0x2
#define		FV_MIIMIND_NV 	0x4
#define	FV_ETHCFSA0	0x0240	/* Ethernet control frame station address 0   */
#define	FV_ETHCFSA1	0x0244	/* Ethernet control frame station address 1   */
#define	FV_ETHCFSA2	0x0248	/* Ethernet control frame station address 2   */

#define	FV_ETHIPGT_HALF_DUPLEX	0x12
#define	FV_ETHIPGT_FULL_DUPLEX	0x15

#define FV_TIMEOUT	0xf000
#define FV_MII_TIMEOUT	0xf000

#define FV_RX_IRQ	40
#define FV_TX_IRQ	41
#define FV_RX_UND_IRQ	42
#define FV_TX_OVR_IRQ	43
//#define RC32434_DMA_BASE_ADDR	MIPS_PHYS_TO_KSEG1(0x18040000)
#define RC32434_DMA_BASE_ADDR	0x00
#define		DMA_C		0x00
#define			DMA_C_R		0x01
#define			DMA_C_ABORT	0x10
#define		DMA_S		0x04
#define			DMA_S_F		0x01
#define			DMA_S_D		0x02
#define			DMA_S_C		0x04
#define			DMA_S_E		0x08
#define			DMA_S_H		0x10
#define		DMA_SM		0x08
#define			DMA_SM_F	0x01
#define			DMA_SM_D	0x02
#define			DMA_SM_C	0x04
#define			DMA_SM_E	0x08
#define			DMA_SM_H	0x10
#define		DMA_DPTR	0x0C
#define		DMA_NDPTR	0x10

#define	RC32434_DMA_CHAN_SIZE	0x14
#define FV_DMA_RXCHAN		0
#define FV_DMA_TXCHAN		1

#define	FV_DMA_READ_REG(chan, reg) \
	(*(volatile uint32_t *)	\
	    (RC32434_DMA_BASE_ADDR + chan * RC32434_DMA_CHAN_SIZE + reg))

#define	FV_DMA_WRITE_REG(chan, reg, val) \
	((*(volatile uint32_t *)	\
	    (RC32434_DMA_BASE_ADDR + chan * RC32434_DMA_CHAN_SIZE + reg)) = val)

#define	FV_DMA_SETBITS_REG(chan, reg, bits) \
	FV_DMA_WRITE_REG((chan), (reg), FV_DMA_READ_REG((chan), (reg)) | (bits))

#define	FV_DMA_CLEARBITS_REG(chan, reg, bits)		\
	FV_DMA_WRITE_REG((chan), (reg),			\
	    FV_DMA_READ_REG((chan), (reg)) & ~(bits))

struct fv_desc {
	uint32_t	fv_ctl;
	uint32_t	fv_ca;
	uint32_t	fv_devcs;
	uint32_t	fv_link;
};


#define FV_DMASIZE(len)		((len)  & ((1 << 18)-1))		
#define FV_PKTSIZE(len)		((len & 0xffff0000) >> 16)

#define	FV_CTL_COF	0x02000000
#define	FV_CTL_COD	0x04000000
#define	FV_CTL_IOF	0x08000000
#define	FV_CTL_IOD	0x10000000
#define	FV_CTL_T	0x20000000
#define	FV_CTL_D	0x40000000
#define	FV_CTL_F	0x80000000

#define	FV_DMARX_DEVCS_RSV	0x00000001
#define	FV_DMARX_DEVCS_LD	0x00000002
#define	FV_DMARX_DEVCS_ROK	0x00000004
#define	FV_DMARX_DEVCS_FM	0x00000008
#define	FV_DMARX_DEVCS_MP	0x00000010
#define	FV_DMARX_DEVCS_BP	0x00000020
#define	FV_DMARX_DEVCS_VLT	0x00000040
#define	FV_DMARX_DEVCS_CF	0x00000080
#define	FV_DMARX_DEVCS_OVR	0x00000100
#define	FV_DMARX_DEVCS_CRC	0x00000200
#define	FV_DMARX_DEVCS_CV	0x00000400
#define	FV_DMARX_DEVCS_DB	0x00000800
#define	FV_DMARX_DEVCS_LE	0x00001000
#define	FV_DMARX_DEVCS_LOR	0x00002000
#define	FV_DMARX_DEVCS_CES	0x00004000

#define	FV_DMATX_DEVCS_FD	0x00000001
#define	FV_DMATX_DEVCS_LD	0x00000002
#define	FV_DMATX_DEVCS_OEN	0x00000004
#define	FV_DMATX_DEVCS_PEN	0x00000008
#define	FV_DMATX_DEVCS_CEN	0x00000010
#define	FV_DMATX_DEVCS_HEN	0x00000020
#define	FV_DMATX_DEVCS_TOK	0x00000040
#define	FV_DMATX_DEVCS_MP	0x00000080
#define	FV_DMATX_DEVCS_BP	0x00000100
#define	FV_DMATX_DEVCS_UND	0x00000200
#define	FV_DMATX_DEVCS_OF	0x00000400
#define	FV_DMATX_DEVCS_ED	0x00000800
#define	FV_DMATX_DEVCS_EC	0x00001000
#define	FV_DMATX_DEVCS_LC	0x00002000
#define	FV_DMATX_DEVCS_TD	0x00004000
#define	FV_DMATX_DEVCS_CRC	0x00008000
#define	FV_DMATX_DEVCS_LE	0x00010000

#define FV_RX_RING_CNT		128
#define FV_TX_RING_CNT		128
#define FV_TX_RING_SIZE		sizeof(struct fv_desc) * FV_TX_RING_CNT
#define FV_RX_RING_SIZE		sizeof(struct fv_desc) * FV_RX_RING_CNT
#define FV_RING_ALIGN		sizeof(struct fv_desc)
#define FV_RX_ALIGN		sizeof(uint32_t)
#define FV_MAXFRAGS		8
#define FV_TX_INTR_THRESH	8

#define	FV_TX_RING_ADDR(sc, i)	\
    ((sc)->fv_rdata.fv_tx_ring_paddr + sizeof(struct fv_desc) * (i))
#define	FV_RX_RING_ADDR(sc, i)	\
    ((sc)->fv_rdata.fv_rx_ring_paddr + sizeof(struct fv_desc) * (i))
#define	FV_INC(x,y)		(x) = (((x) + 1) % y)

struct fv_txdesc {
	struct mbuf	*tx_m;
	bus_dmamap_t	tx_dmamap;
};

struct fv_rxdesc {
	struct mbuf	*rx_m;
	bus_dmamap_t	rx_dmamap;
	struct fv_desc	*desc;
	/* Use this values on error instead of allocating new mbuf */
	uint32_t	saved_ctl, saved_ca; 
};

struct fv_chain_data {
	bus_dma_tag_t		fv_parent_tag;
	bus_dma_tag_t		fv_tx_tag;
	struct fv_txdesc	fv_txdesc[FV_TX_RING_CNT];
	bus_dma_tag_t		fv_rx_tag;
	struct fv_rxdesc	fv_rxdesc[FV_RX_RING_CNT];
	bus_dma_tag_t		fv_tx_ring_tag;
	bus_dma_tag_t		fv_rx_ring_tag;
	bus_dmamap_t		fv_tx_ring_map;
	bus_dmamap_t		fv_rx_ring_map;
	bus_dmamap_t		fv_rx_sparemap;
	int			fv_tx_pkts;
	int			fv_tx_prod;
	int			fv_tx_cons;
	int			fv_tx_cnt;
	int			fv_rx_cons;
};

struct fv_ring_data {
	struct fv_desc		*fv_rx_ring;
	struct fv_desc		*fv_tx_ring;
	bus_addr_t		fv_rx_ring_paddr;
	bus_addr_t		fv_tx_ring_paddr;
};

struct fv_softc {
	struct ifnet		*fv_ifp;	/* interface info */
	bus_space_handle_t	fv_bhandle;	/* bus space handle */
	bus_space_tag_t		fv_btag;	/* bus space tag */
	device_t		fv_dev;
	struct resource		*fv_res;
	int			fv_rid;
	struct resource		*fv_rx_irq;
	void			*fv_rx_intrhand;
	struct resource		*fv_tx_irq;
	void			*fv_tx_intrhand;
	struct resource		*fv_rx_und_irq;
	void			*fv_rx_und_intrhand;
	struct resource		*fv_tx_ovr_irq;
	void			*fv_tx_ovr_intrhand;
	device_t		fv_miibus;
	bus_dma_tag_t		fv_parent_tag;
	bus_dma_tag_t		fv_tag;
	struct mtx		fv_mtx;
	struct callout		fv_stat_callout;
	struct task		fv_link_task;
	struct fv_chain_data	fv_cdata;
	struct fv_ring_data	fv_rdata;
	int			fv_link_status;
	int			fv_detach;
};

#define	FV_LOCK(_sc)		mtx_lock(&(_sc)->fv_mtx)
#define	FV_UNLOCK(_sc)		mtx_unlock(&(_sc)->fv_mtx)
#define	FV_LOCK_ASSERT(_sc)	mtx_assert(&(_sc)->fv_mtx, MA_OWNED)

/*
 * register space access macros
 */
#define CSR_WRITE_4(sc, reg, val)	\
	bus_space_write_4(sc->fv_btag, sc->fv_bhandle, reg, val)

#define CSR_READ_4(sc, reg)		\
	bus_space_read_4(sc->fv_btag, sc->fv_bhandle, reg)

#endif /* __IF_FVREG_H__ */
