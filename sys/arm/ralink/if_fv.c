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
 * $Id: $
 * 
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * RC32434 Ethernet interface driver
 */
#include <sys/param.h>
#include <sys/endian.h>
#include <sys/systm.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/socket.h>
#include <sys/taskqueue.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>

#include <net/bpf.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

MODULE_DEPEND(fv, ether, 1, 1, 1);
MODULE_DEPEND(fv, miibus, 1, 1, 1);

#include "miibus_if.h"

#include <arm/ralink/if_fvreg.h>

#define FV_DEBUG

static int fv_attach(device_t);
static int fv_detach(device_t);
static int fv_ifmedia_upd(struct ifnet *);
static void fv_ifmedia_sts(struct ifnet *, struct ifmediareq *);
static int fv_ioctl(struct ifnet *, u_long, caddr_t);
static void fv_init(void *);
static void fv_init_locked(struct fv_softc *);
static void fv_link_task(void *, int);
static int fv_miibus_readreg(device_t, int, int);
static void fv_miibus_statchg(device_t);
static int fv_miibus_writereg(device_t, int, int, int);
static int fv_probe(device_t);
static void fv_reset(struct fv_softc *);
static int fv_resume(device_t);
static int fv_rx_ring_init(struct fv_softc *);
static int fv_tx_ring_init(struct fv_softc *);
static int fv_shutdown(device_t);
static void fv_start(struct ifnet *);
static void fv_start_locked(struct ifnet *);
static void fv_stop(struct fv_softc *);
static int fv_suspend(device_t);

static void fv_rx(struct fv_softc *);
static void fv_tx(struct fv_softc *);
static void fv_rx_intr(void *);
static void fv_tx_intr(void *);
static void fv_rx_und_intr(void *);
static void fv_tx_ovr_intr(void *);
static void fv_tick(void *);

static void fv_dmamap_cb(void *, bus_dma_segment_t *, int, int);
static int fv_dma_alloc(struct fv_softc *);
static void fv_dma_free(struct fv_softc *);
static int fv_newbuf(struct fv_softc *, int);
static __inline void fv_fixup_rx(struct mbuf *);

static device_method_t fv_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		fv_probe),
	DEVMETHOD(device_attach,	fv_attach),
	DEVMETHOD(device_detach,	fv_detach),
	DEVMETHOD(device_suspend,	fv_suspend),
	DEVMETHOD(device_resume,	fv_resume),
	DEVMETHOD(device_shutdown,	fv_shutdown),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	fv_miibus_readreg),
	DEVMETHOD(miibus_writereg,	fv_miibus_writereg),
	DEVMETHOD(miibus_statchg,	fv_miibus_statchg),

	DEVMETHOD_END
};

static driver_t fv_driver = {
	"fv",
	fv_methods,
	sizeof(struct fv_softc)
};

static devclass_t fv_devclass;

DRIVER_MODULE(fv, obio, fv_driver, fv_devclass, 0, 0);
DRIVER_MODULE(miibus, fv, miibus_driver, miibus_devclass, 0, 0);

static int 
fv_probe(device_t dev)
{

	device_set_desc(dev, "RC32434 Ethernet interface");
	return (0);
}

static int
fv_attach(device_t dev)
{
	uint8_t			eaddr[ETHER_ADDR_LEN];
	struct ifnet		*ifp;
	struct fv_softc		*sc;
	int			error = 0, rid;
	int			unit;

	sc = device_get_softc(dev);
	unit = device_get_unit(dev);
	sc->fv_dev = dev;

	mtx_init(&sc->fv_mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK,
	    MTX_DEF);
	callout_init_mtx(&sc->fv_stat_callout, &sc->fv_mtx, 0);
	TASK_INIT(&sc->fv_link_task, 0, fv_link_task, sc);
	pci_enable_busmaster(dev);

	/* Map control/status registers. */
	sc->fv_rid = 0;
	sc->fv_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->fv_rid, 
	    RF_ACTIVE);

	if (sc->fv_res == NULL) {
		device_printf(dev, "couldn't map memory\n");
		error = ENXIO;
		goto fail;
	}

	sc->fv_btag = rman_get_bustag(sc->fv_res);
	sc->fv_bhandle = rman_get_bushandle(sc->fv_res);

	/* Allocate interrupts */
	rid = 0;
	sc->fv_rx_irq = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, FV_RX_IRQ,
	    FV_RX_IRQ, 1, RF_SHAREABLE | RF_ACTIVE);

	if (sc->fv_rx_irq == NULL) {
		device_printf(dev, "couldn't map rx interrupt\n");
		error = ENXIO;
		goto fail;
	}

	rid = 0;
	sc->fv_tx_irq = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, FV_TX_IRQ,
	    FV_TX_IRQ, 1, RF_SHAREABLE | RF_ACTIVE);

	if (sc->fv_tx_irq == NULL) {
		device_printf(dev, "couldn't map tx interrupt\n");
		error = ENXIO;
		goto fail;
	}

	rid = 0;
	sc->fv_rx_und_irq = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, 
	    FV_RX_UND_IRQ, FV_RX_UND_IRQ, 1, RF_SHAREABLE | RF_ACTIVE);

	if (sc->fv_rx_und_irq == NULL) {
		device_printf(dev, "couldn't map rx underrun interrupt\n");
		error = ENXIO;
		goto fail;
	}

	rid = 0;
	sc->fv_tx_ovr_irq = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, 
	    FV_TX_OVR_IRQ, FV_TX_OVR_IRQ, 1, RF_SHAREABLE | RF_ACTIVE);

	if (sc->fv_tx_ovr_irq == NULL) {
		device_printf(dev, "couldn't map tx overrun interrupt\n");
		error = ENXIO;
		goto fail;
	}

	/* Allocate ifnet structure. */
	ifp = sc->fv_ifp = if_alloc(IFT_ETHER);

	if (ifp == NULL) {
		device_printf(dev, "couldn't allocate ifnet structure\n");
		error = ENOSPC;
		goto fail;
	}
	ifp->if_softc = sc;
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = fv_ioctl;
	ifp->if_start = fv_start;
	ifp->if_init = fv_init;

	/* XXX: add real size */
	IFQ_SET_MAXLEN(&ifp->if_snd, 9);
	ifp->if_snd.ifq_maxlen = 9;
	IFQ_SET_READY(&ifp->if_snd);

	ifp->if_capenable = ifp->if_capabilities;

	eaddr[0] = 0x00;
	eaddr[1] = 0x0C;
	eaddr[2] = 0x42;
	eaddr[3] = 0x09;
	eaddr[4] = 0x5E;
	eaddr[5] = 0x6B;

	if (fv_dma_alloc(sc) != 0) {
		error = ENXIO;
		goto fail;
	}

	/* TODO: calculate prescale */
	CSR_WRITE_4(sc, FV_ETHMCP, (165000000 / (1250000 + 1)) & ~1);

	CSR_WRITE_4(sc, FV_MIIMCFG, FV_MIIMCFG_R);
	DELAY(1000);
	CSR_WRITE_4(sc, FV_MIIMCFG, 0);

	/* Do MII setup. */
	error = mii_attach(dev, &sc->fv_miibus, ifp, fv_ifmedia_upd,
	    fv_ifmedia_sts, BMSR_DEFCAPMASK, MII_PHY_ANY, MII_OFFSET_ANY, 0);
	if (error != 0) {
		device_printf(dev, "attaching PHYs failed\n");
		goto fail;
	}

	/* Call MI attach routine. */
	ether_ifattach(ifp, eaddr);

	/* Hook interrupt last to avoid having to lock softc */
	error = bus_setup_intr(dev, sc->fv_rx_irq, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, fv_rx_intr, sc, &sc->fv_rx_intrhand);

	if (error) {
		device_printf(dev, "couldn't set up rx irq\n");
		ether_ifdetach(ifp);
		goto fail;
	}

	error = bus_setup_intr(dev, sc->fv_tx_irq, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, fv_tx_intr, sc, &sc->fv_tx_intrhand);

	if (error) {
		device_printf(dev, "couldn't set up tx irq\n");
		ether_ifdetach(ifp);
		goto fail;
	}

	error = bus_setup_intr(dev, sc->fv_rx_und_irq, 
	    INTR_TYPE_NET | INTR_MPSAFE, NULL, fv_rx_und_intr, sc, 
	    &sc->fv_rx_und_intrhand);

	if (error) {
		device_printf(dev, "couldn't set up rx underrun irq\n");
		ether_ifdetach(ifp);
		goto fail;
	}

	error = bus_setup_intr(dev, sc->fv_tx_ovr_irq, 
	    INTR_TYPE_NET | INTR_MPSAFE, NULL, fv_tx_ovr_intr, sc, 
	    &sc->fv_tx_ovr_intrhand);

	if (error) {
		device_printf(dev, "couldn't set up tx overrun irq\n");
		ether_ifdetach(ifp);
		goto fail;
	}

fail:
	if (error) 
		fv_detach(dev);

	return (error);
}

static int
fv_detach(device_t dev)
{
	struct fv_softc		*sc = device_get_softc(dev);
	struct ifnet		*ifp = sc->fv_ifp;

	KASSERT(mtx_initialized(&sc->fv_mtx), ("vr mutex not initialized"));

	/* These should only be active if attach succeeded */
	if (device_is_attached(dev)) {
		FV_LOCK(sc);
		sc->fv_detach = 1;
		fv_stop(sc);
		FV_UNLOCK(sc);
		taskqueue_drain(taskqueue_swi, &sc->fv_link_task);
		ether_ifdetach(ifp);
	}
	if (sc->fv_miibus)
		device_delete_child(dev, sc->fv_miibus);
	bus_generic_detach(dev);

	if (sc->fv_rx_intrhand)
		bus_teardown_intr(dev, sc->fv_rx_irq, sc->fv_rx_intrhand);
	if (sc->fv_rx_irq)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->fv_rx_irq);
	if (sc->fv_tx_intrhand)
		bus_teardown_intr(dev, sc->fv_tx_irq, sc->fv_tx_intrhand);
	if (sc->fv_tx_irq)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->fv_tx_irq);
	if (sc->fv_rx_und_intrhand)
		bus_teardown_intr(dev, sc->fv_rx_und_irq, 
		    sc->fv_rx_und_intrhand);
	if (sc->fv_rx_und_irq)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->fv_rx_und_irq);
	if (sc->fv_tx_ovr_intrhand)
		bus_teardown_intr(dev, sc->fv_tx_ovr_irq, 
		    sc->fv_tx_ovr_intrhand);
	if (sc->fv_tx_ovr_irq)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->fv_tx_ovr_irq);

	if (sc->fv_res)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->fv_rid, 
		    sc->fv_res);

	if (ifp)
		if_free(ifp);

	fv_dma_free(sc);

	mtx_destroy(&sc->fv_mtx);

	return (0);

}

static int
fv_suspend(device_t dev)
{

	panic("%s", __func__);
	return 0;
}

static int
fv_resume(device_t dev)
{

	panic("%s", __func__);
	return 0;
}

static int
fv_shutdown(device_t dev)
{
	struct fv_softc	*sc;

	sc = device_get_softc(dev);

	FV_LOCK(sc);
	fv_stop(sc);
	FV_UNLOCK(sc);

	return (0);
}

static int
fv_miibus_readreg(device_t dev, int phy, int reg)
{
	struct fv_softc * sc = device_get_softc(dev);
	int i, result;

	i = FV_MII_TIMEOUT;
	while ((CSR_READ_4(sc, FV_MIIMIND) & FV_MIIMIND_BSY) && i)
		i--;

	if (i == 0)
		device_printf(dev, "phy mii is busy %d:%d\n", phy, reg);

	CSR_WRITE_4(sc, FV_MIIMADDR, (phy << 8) | reg);

	i = FV_MII_TIMEOUT;
	while ((CSR_READ_4(sc, FV_MIIMIND) & FV_MIIMIND_BSY) && i)
		i--;

	if (i == 0)
		device_printf(dev, "phy mii is busy %d:%d\n", phy, reg);

	CSR_WRITE_4(sc, FV_MIIMCMD, FV_MIIMCMD_RD);

	i = FV_MII_TIMEOUT;
	while ((CSR_READ_4(sc, FV_MIIMIND) & FV_MIIMIND_BSY) && i)
		i--;

	if (i == 0)
		device_printf(dev, "phy mii read is timed out %d:%d\n", phy, 
		    reg);

	if (CSR_READ_4(sc, FV_MIIMIND) & FV_MIIMIND_NV)
		printf("phy mii readreg failed %d:%d: data not valid\n",
		    phy, reg);

	result = CSR_READ_4(sc , FV_MIIMRDD);
	CSR_WRITE_4(sc, FV_MIIMCMD, 0);

	return (result);
}

static int
fv_miibus_writereg(device_t dev, int phy, int reg, int data)
{
	struct fv_softc * sc = device_get_softc(dev);
	int i;

	i = FV_MII_TIMEOUT;
	while ((CSR_READ_4(sc, FV_MIIMIND) & FV_MIIMIND_BSY) && i)
		i--;

	if (i == 0)
		device_printf(dev, "phy mii is busy %d:%d\n", phy, reg);

	CSR_WRITE_4(sc, FV_MIIMADDR, (phy << 8) | reg);

	i = FV_MII_TIMEOUT;
	while ((CSR_READ_4(sc, FV_MIIMIND) & FV_MIIMIND_BSY) && i)
		i--;

	if (i == 0)
		device_printf(dev, "phy mii is busy %d:%d\n", phy, reg);

	CSR_WRITE_4(sc, FV_MIIMWTD, data);

	i = FV_MII_TIMEOUT;
	while ((CSR_READ_4(sc, FV_MIIMIND) & FV_MIIMIND_BSY) && i)
		i--;

	if (i == 0)
		device_printf(dev, "phy mii is busy %d:%d\n", phy, reg);

	return (0);
}

static void
fv_miibus_statchg(device_t dev)
{
	struct fv_softc		*sc;

	sc = device_get_softc(dev);
	taskqueue_enqueue(taskqueue_swi, &sc->fv_link_task);
}

static void
fv_link_task(void *arg, int pending)
{
	struct fv_softc		*sc;
	struct mii_data		*mii;
	struct ifnet		*ifp;
	/* int			lfdx, mfdx; */

	sc = (struct fv_softc *)arg;

	FV_LOCK(sc);
	mii = device_get_softc(sc->fv_miibus);
	ifp = sc->fv_ifp;
	if (mii == NULL || ifp == NULL ||
	    (ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
		FV_UNLOCK(sc);
		return;
	}

	if (mii->mii_media_status & IFM_ACTIVE) {
		if (IFM_SUBTYPE(mii->mii_media_active) != IFM_NONE)
			sc->fv_link_status = 1;
	} else
		sc->fv_link_status = 0;

	FV_UNLOCK(sc);
}

static void
fv_reset(struct fv_softc *sc)
{
	int		i;

	CSR_WRITE_4(sc, FV_ETHINTFC, 0);

	for (i = 0; i < FV_TIMEOUT; i++) {
		DELAY(10);
		if (!(CSR_READ_4(sc, FV_ETHINTFC) & ETH_INTFC_RIP))
			break;
	}

	if (i == FV_TIMEOUT)
		device_printf(sc->fv_dev, "reset time out\n");
}

static void
fv_init(void *xsc)
{
	struct fv_softc	 *sc = xsc;

	FV_LOCK(sc);
	fv_init_locked(sc);
	FV_UNLOCK(sc);
}

static void
fv_init_locked(struct fv_softc *sc)
{
	struct ifnet		*ifp = sc->fv_ifp;
	struct mii_data		*mii;

	FV_LOCK_ASSERT(sc);

	mii = device_get_softc(sc->fv_miibus);

	fv_stop(sc);
	fv_reset(sc);

	CSR_WRITE_4(sc, FV_ETHINTFC, ETH_INTFC_EN);

	/* Init circular RX list. */
	if (fv_rx_ring_init(sc) != 0) {
		device_printf(sc->fv_dev,
		    "initialization failed: no memory for rx buffers\n");
		fv_stop(sc);
		return;
	}

	/* Init tx descriptors. */
	fv_tx_ring_init(sc);

	FV_DMA_WRITE_REG(FV_DMA_RXCHAN, DMA_S, 0);
	FV_DMA_WRITE_REG(FV_DMA_RXCHAN, DMA_NDPTR, 0);
	FV_DMA_WRITE_REG(FV_DMA_RXCHAN, DMA_DPTR, 
	    sc->fv_rdata.fv_rx_ring_paddr);


	FV_DMA_CLEARBITS_REG(FV_DMA_RXCHAN, DMA_SM, 
	    DMA_SM_H | DMA_SM_E | DMA_SM_D) ;

	FV_DMA_WRITE_REG(FV_DMA_TXCHAN, DMA_S, 0);
	FV_DMA_WRITE_REG(FV_DMA_TXCHAN, DMA_NDPTR, 0);
	FV_DMA_WRITE_REG(FV_DMA_TXCHAN, DMA_DPTR, 0);
	FV_DMA_CLEARBITS_REG(FV_DMA_TXCHAN, DMA_SM, 
	    DMA_SM_F | DMA_SM_E);


	/* Accept only packets destined for THIS Ethernet device address */
	CSR_WRITE_4(sc, FV_ETHARC, 1);

	/* 
	 * Set all Ethernet address registers to the same initial values
	 * set all four addresses to 66-88-aa-cc-dd-ee 
	 */
	CSR_WRITE_4(sc, FV_ETHSAL0, 0x42095E6B);
	CSR_WRITE_4(sc, FV_ETHSAH0, 0x0000000C);

	CSR_WRITE_4(sc, FV_ETHSAL1, 0x42095E6B);
	CSR_WRITE_4(sc, FV_ETHSAH1, 0x0000000C);

	CSR_WRITE_4(sc, FV_ETHSAL2, 0x42095E6B);
	CSR_WRITE_4(sc, FV_ETHSAH2, 0x0000000C);

	CSR_WRITE_4(sc, FV_ETHSAL3, 0x42095E6B);
	CSR_WRITE_4(sc, FV_ETHSAH3, 0x0000000C);

	CSR_WRITE_4(sc, FV_ETHMAC2, 
	    FV_ETH_MAC2_PEN | FV_ETH_MAC2_CEN | FV_ETH_MAC2_FD);

	CSR_WRITE_4(sc, FV_ETHIPGT, FV_ETHIPGT_FULL_DUPLEX);
	CSR_WRITE_4(sc, FV_ETHIPGR, 0x12); /* minimum value */

	CSR_WRITE_4(sc, FV_MIIMCFG, FV_MIIMCFG_R);
	DELAY(1000);
	CSR_WRITE_4(sc, FV_MIIMCFG, 0);

	/* TODO: calculate prescale */
	CSR_WRITE_4(sc, FV_ETHMCP, (165000000 / (1250000 + 1)) & ~1);

	/* FIFO Tx threshold level */
	CSR_WRITE_4(sc, FV_ETHFIFOTT, 0x30);

	CSR_WRITE_4(sc, FV_ETHMAC1, FV_ETH_MAC1_RE);

	sc->fv_link_status = 0;
	mii_mediachg(mii);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	callout_reset(&sc->fv_stat_callout, hz, fv_tick, sc);
}

static void
fv_start(struct ifnet *ifp)
{
	struct fv_softc	 *sc;

	sc = ifp->if_softc;

	FV_LOCK(sc);
	fv_start_locked(ifp);
	FV_UNLOCK(sc);
}

/*
 * Encapsulate an mbuf chain in a descriptor by coupling the mbuf data
 * pointers to the fragment pointers.
 */
static int
fv_encap(struct fv_softc *sc, struct mbuf **m_head)
{
	struct fv_txdesc	*txd;
	struct fv_desc		*desc, *prev_desc;
	bus_dma_segment_t	txsegs[FV_MAXFRAGS];
	uint32_t		link_addr;
	int			error, i, nsegs, prod, si, prev_prod;

	FV_LOCK_ASSERT(sc);

	prod = sc->fv_cdata.fv_tx_prod;
	txd = &sc->fv_cdata.fv_txdesc[prod];
	error = bus_dmamap_load_mbuf_sg(sc->fv_cdata.fv_tx_tag, txd->tx_dmamap,
	    *m_head, txsegs, &nsegs, BUS_DMA_NOWAIT);
	if (error == EFBIG) {
		panic("EFBIG");
	} else if (error != 0)
		return (error);
	if (nsegs == 0) {
		m_freem(*m_head);
		*m_head = NULL;
		return (EIO);
	}

	/* Check number of available descriptors. */
	if (sc->fv_cdata.fv_tx_cnt + nsegs >= (FV_TX_RING_CNT - 1)) {
		bus_dmamap_unload(sc->fv_cdata.fv_tx_tag, txd->tx_dmamap);
		return (ENOBUFS);
	}

	txd->tx_m = *m_head;
	bus_dmamap_sync(sc->fv_cdata.fv_tx_tag, txd->tx_dmamap,
	    BUS_DMASYNC_PREWRITE);

	si = prod;

	/* 
	 * Make a list of descriptors for this packet. DMA controller will
	 * walk through it while fv_link is not zero. The last one should
	 * have COF flag set, to pickup next chain from NDPTR
	 */
	prev_prod = prod;
	desc = prev_desc = NULL;
	for (i = 0; i < nsegs; i++) {
		desc = &sc->fv_rdata.fv_tx_ring[prod];
		desc->fv_ctl = FV_DMASIZE(txsegs[i].ds_len) | FV_CTL_IOF;
		if (i == 0)
			desc->fv_devcs = FV_DMATX_DEVCS_FD;
		desc->fv_ca = txsegs[i].ds_addr;
		desc->fv_link = 0;
		/* link with previous descriptor */
		if (prev_desc)
			prev_desc->fv_link = FV_TX_RING_ADDR(sc, prod);

		sc->fv_cdata.fv_tx_cnt++;
		prev_desc = desc;
		FV_INC(prod, FV_TX_RING_CNT);
	}

	/* 
	 * Set COF for last descriptor and mark last fragment with LD flag
	 */
	if (desc) {
		desc->fv_ctl |=  FV_CTL_COF;
		desc->fv_devcs |= FV_DMATX_DEVCS_LD;
	}

	/* Update producer index. */
	sc->fv_cdata.fv_tx_prod = prod;

	/* Sync descriptors. */
	bus_dmamap_sync(sc->fv_cdata.fv_tx_ring_tag,
	    sc->fv_cdata.fv_tx_ring_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	/* Start transmitting */
	/* Check if new list is queued in NDPTR */
	if (FV_DMA_READ_REG(FV_DMA_TXCHAN, DMA_NDPTR) == 0) {
		/* NDPTR is not busy - start new list */
		FV_DMA_WRITE_REG(FV_DMA_TXCHAN, DMA_NDPTR, 
		    FV_TX_RING_ADDR(sc, si));
	}
	else {
		link_addr = FV_TX_RING_ADDR(sc, si);
		/* Get previous descriptor */
		si = (si + FV_TX_RING_CNT - 1) % FV_TX_RING_CNT;
		desc = &sc->fv_rdata.fv_tx_ring[si];
		desc->fv_link = link_addr;
	}

	return (0);
}

static void
fv_start_locked(struct ifnet *ifp)
{
	struct fv_softc		*sc;
	struct mbuf		*m_head;
	int			enq;

	sc = ifp->if_softc;

	FV_LOCK_ASSERT(sc);

	if ((ifp->if_drv_flags & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING || sc->fv_link_status == 0 )
		return;

	for (enq = 0; !IFQ_DRV_IS_EMPTY(&ifp->if_snd) &&
	    sc->fv_cdata.fv_tx_cnt < FV_TX_RING_CNT - 2; ) {
		IFQ_DRV_DEQUEUE(&ifp->if_snd, m_head);
		if (m_head == NULL)
			break;
		/*
		 * Pack the data into the transmit ring. If we
		 * don't have room, set the OACTIVE flag and wait
		 * for the NIC to drain the ring.
		 */
		if (fv_encap(sc, &m_head)) {
			if (m_head == NULL)
				break;
			IFQ_DRV_PREPEND(&ifp->if_snd, m_head);
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}

		enq++;
		/*
		 * If there's a BPF listener, bounce a copy of this frame
		 * to him.
		 */
		ETHER_BPF_MTAP(ifp, m_head);
	}
}

static void
fv_stop(struct fv_softc *sc)
{
	struct ifnet	    *ifp;

	FV_LOCK_ASSERT(sc);


	ifp = sc->fv_ifp;
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);
	callout_stop(&sc->fv_stat_callout);

	/* mask out RX interrupts */
	FV_DMA_SETBITS_REG(FV_DMA_RXCHAN, DMA_SM, 
	    DMA_SM_D | DMA_SM_H | DMA_SM_E);

	/* mask out TX interrupts */
	FV_DMA_SETBITS_REG(FV_DMA_TXCHAN, DMA_SM, 
	    DMA_SM_F | DMA_SM_E);

	/* Abort RX DMA transactions */
	if (FV_DMA_READ_REG(FV_DMA_RXCHAN, DMA_C) & DMA_C_R) {
		/* Set ABORT bit if trunsuction is in progress */
		FV_DMA_WRITE_REG(FV_DMA_RXCHAN, DMA_C, DMA_C_ABORT);
		/* XXX: Add timeout */
		while ((FV_DMA_READ_REG(FV_DMA_RXCHAN, DMA_S) & DMA_S_H) == 0)
			DELAY(10);
		FV_DMA_WRITE_REG(FV_DMA_RXCHAN, DMA_S, 0);
	}
	FV_DMA_WRITE_REG(FV_DMA_RXCHAN, DMA_DPTR, 0);
	FV_DMA_WRITE_REG(FV_DMA_RXCHAN, DMA_NDPTR, 0);

	/* Abort TX DMA transactions */
	if (FV_DMA_READ_REG(FV_DMA_TXCHAN, DMA_C) & DMA_C_R) {
		/* Set ABORT bit if trunsuction is in progress */
		FV_DMA_WRITE_REG(FV_DMA_TXCHAN, DMA_C, DMA_C_ABORT);
		/* XXX: Add timeout */
		while ((FV_DMA_READ_REG(FV_DMA_TXCHAN, DMA_S) & DMA_S_H) == 0)
			DELAY(10);
		FV_DMA_WRITE_REG(FV_DMA_TXCHAN, DMA_S, 0);
	}
	FV_DMA_WRITE_REG(FV_DMA_TXCHAN, DMA_DPTR, 0);
	FV_DMA_WRITE_REG(FV_DMA_TXCHAN, DMA_NDPTR, 0);

	CSR_WRITE_4(sc, FV_ETHINTFC, 0);
}


static int
fv_ioctl(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct fv_softc		*sc = ifp->if_softc;
	struct ifreq		*ifr = (struct ifreq *) data;
	struct mii_data		*mii;
	int			error;

	switch (command) {
	case SIOCSIFFLAGS:
#if 0
		FV_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
				if ((ifp->if_flags ^ sc->fv_if_flags) &
				    (IFF_PROMISC | IFF_ALLMULTI))
					fv_set_filter(sc);
			} else {
				if (sc->fv_detach == 0)
					fv_init_locked(sc);
			}
		} else {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				fv_stop(sc);
		}
		sc->fv_if_flags = ifp->if_flags;
		FV_UNLOCK(sc);
#endif
		error = 0;
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
#if 0
		FV_LOCK(sc);
		fv_set_filter(sc);
		FV_UNLOCK(sc);
#endif
		error = 0;
		break;
	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		mii = device_get_softc(sc->fv_miibus);
		error = ifmedia_ioctl(ifp, ifr, &mii->mii_media, command);
		break;
	case SIOCSIFCAP:
		error = 0;
#if 0
		mask = ifr->ifr_reqcap ^ ifp->if_capenable;
		if ((mask & IFCAP_HWCSUM) != 0) {
			ifp->if_capenable ^= IFCAP_HWCSUM;
			if ((IFCAP_HWCSUM & ifp->if_capenable) &&
			    (IFCAP_HWCSUM & ifp->if_capabilities))
				ifp->if_hwassist = FV_CSUM_FEATURES;
			else
				ifp->if_hwassist = 0;
		}
		if ((mask & IFCAP_VLAN_HWTAGGING) != 0) {
			ifp->if_capenable ^= IFCAP_VLAN_HWTAGGING;
			if (IFCAP_VLAN_HWTAGGING & ifp->if_capenable &&
			    IFCAP_VLAN_HWTAGGING & ifp->if_capabilities &&
			    ifp->if_drv_flags & IFF_DRV_RUNNING) {
				FV_LOCK(sc);
				fv_vlan_setup(sc);
				FV_UNLOCK(sc);
			}
		}
		VLAN_CAPABILITIES(ifp);
#endif
		break;
	default:
		error = ether_ioctl(ifp, command, data);
		break;
	}

	return (error);
}

/*
 * Set media options.
 */
static int
fv_ifmedia_upd(struct ifnet *ifp)
{
	struct fv_softc		*sc;
	struct mii_data		*mii;
	struct mii_softc	*miisc;
	int			error;

	sc = ifp->if_softc;
	FV_LOCK(sc);
	mii = device_get_softc(sc->fv_miibus);
	LIST_FOREACH(miisc, &mii->mii_phys, mii_list)
		PHY_RESET(miisc);
	error = mii_mediachg(mii);
	FV_UNLOCK(sc);

	return (error);
}

/*
 * Report current media status.
 */
static void
fv_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct fv_softc		*sc = ifp->if_softc;
	struct mii_data		*mii;

	mii = device_get_softc(sc->fv_miibus);
	FV_LOCK(sc);
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
	FV_UNLOCK(sc);
}

struct fv_dmamap_arg {
	bus_addr_t	fv_busaddr;
};

static void
fv_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	struct fv_dmamap_arg	*ctx;

	if (error != 0)
		return;
	ctx = arg;
	ctx->fv_busaddr = segs[0].ds_addr;
}

static int
fv_dma_alloc(struct fv_softc *sc)
{
	struct fv_dmamap_arg	ctx;
	struct fv_txdesc	*txd;
	struct fv_rxdesc	*rxd;
	int			error, i;

	/* Create parent DMA tag. */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->fv_dev),	/* parent */
	    1, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    BUS_SPACE_MAXSIZE_32BIT,	/* maxsize */
	    0,				/* nsegments */
	    BUS_SPACE_MAXSIZE_32BIT,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->fv_cdata.fv_parent_tag);
	if (error != 0) {
		device_printf(sc->fv_dev, "failed to create parent DMA tag\n");
		goto fail;
	}
	/* Create tag for Tx ring. */
	error = bus_dma_tag_create(
	    sc->fv_cdata.fv_parent_tag,	/* parent */
	    FV_RING_ALIGN, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    FV_TX_RING_SIZE,		/* maxsize */
	    1,				/* nsegments */
	    FV_TX_RING_SIZE,		/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->fv_cdata.fv_tx_ring_tag);
	if (error != 0) {
		device_printf(sc->fv_dev, "failed to create Tx ring DMA tag\n");
		goto fail;
	}

	/* Create tag for Rx ring. */
	error = bus_dma_tag_create(
	    sc->fv_cdata.fv_parent_tag,	/* parent */
	    FV_RING_ALIGN, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    FV_RX_RING_SIZE,		/* maxsize */
	    1,				/* nsegments */
	    FV_RX_RING_SIZE,		/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->fv_cdata.fv_rx_ring_tag);
	if (error != 0) {
		device_printf(sc->fv_dev, "failed to create Rx ring DMA tag\n");
		goto fail;
	}

	/* Create tag for Tx buffers. */
	error = bus_dma_tag_create(
	    sc->fv_cdata.fv_parent_tag,	/* parent */
	    sizeof(uint32_t), 0,	/* alignment, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES * FV_MAXFRAGS,	/* maxsize */
	    FV_MAXFRAGS,		/* nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->fv_cdata.fv_tx_tag);
	if (error != 0) {
		device_printf(sc->fv_dev, "failed to create Tx DMA tag\n");
		goto fail;
	}

	/* Create tag for Rx buffers. */
	error = bus_dma_tag_create(
	    sc->fv_cdata.fv_parent_tag,	/* parent */
	    FV_RX_ALIGN, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES,			/* maxsize */
	    1,				/* nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->fv_cdata.fv_rx_tag);
	if (error != 0) {
		device_printf(sc->fv_dev, "failed to create Rx DMA tag\n");
		goto fail;
	}

	/* Allocate DMA'able memory and load the DMA map for Tx ring. */
	error = bus_dmamem_alloc(sc->fv_cdata.fv_tx_ring_tag,
	    (void **)&sc->fv_rdata.fv_tx_ring, BUS_DMA_WAITOK |
	    BUS_DMA_COHERENT | BUS_DMA_ZERO, &sc->fv_cdata.fv_tx_ring_map);
	if (error != 0) {
		device_printf(sc->fv_dev,
		    "failed to allocate DMA'able memory for Tx ring\n");
		goto fail;
	}

	ctx.fv_busaddr = 0;
	error = bus_dmamap_load(sc->fv_cdata.fv_tx_ring_tag,
	    sc->fv_cdata.fv_tx_ring_map, sc->fv_rdata.fv_tx_ring,
	    FV_TX_RING_SIZE, fv_dmamap_cb, &ctx, 0);
	if (error != 0 || ctx.fv_busaddr == 0) {
		device_printf(sc->fv_dev,
		    "failed to load DMA'able memory for Tx ring\n");
		goto fail;
	}
	sc->fv_rdata.fv_tx_ring_paddr = ctx.fv_busaddr;

	/* Allocate DMA'able memory and load the DMA map for Rx ring. */
	error = bus_dmamem_alloc(sc->fv_cdata.fv_rx_ring_tag,
	    (void **)&sc->fv_rdata.fv_rx_ring, BUS_DMA_WAITOK |
	    BUS_DMA_COHERENT | BUS_DMA_ZERO, &sc->fv_cdata.fv_rx_ring_map);
	if (error != 0) {
		device_printf(sc->fv_dev,
		    "failed to allocate DMA'able memory for Rx ring\n");
		goto fail;
	}

	ctx.fv_busaddr = 0;
	error = bus_dmamap_load(sc->fv_cdata.fv_rx_ring_tag,
	    sc->fv_cdata.fv_rx_ring_map, sc->fv_rdata.fv_rx_ring,
	    FV_RX_RING_SIZE, fv_dmamap_cb, &ctx, 0);
	if (error != 0 || ctx.fv_busaddr == 0) {
		device_printf(sc->fv_dev,
		    "failed to load DMA'able memory for Rx ring\n");
		goto fail;
	}
	sc->fv_rdata.fv_rx_ring_paddr = ctx.fv_busaddr;

	/* Create DMA maps for Tx buffers. */
	for (i = 0; i < FV_TX_RING_CNT; i++) {
		txd = &sc->fv_cdata.fv_txdesc[i];
		txd->tx_m = NULL;
		txd->tx_dmamap = NULL;
		error = bus_dmamap_create(sc->fv_cdata.fv_tx_tag, 0,
		    &txd->tx_dmamap);
		if (error != 0) {
			device_printf(sc->fv_dev,
			    "failed to create Tx dmamap\n");
			goto fail;
		}
	}
	/* Create DMA maps for Rx buffers. */
	if ((error = bus_dmamap_create(sc->fv_cdata.fv_rx_tag, 0,
	    &sc->fv_cdata.fv_rx_sparemap)) != 0) {
		device_printf(sc->fv_dev,
		    "failed to create spare Rx dmamap\n");
		goto fail;
	}
	for (i = 0; i < FV_RX_RING_CNT; i++) {
		rxd = &sc->fv_cdata.fv_rxdesc[i];
		rxd->rx_m = NULL;
		rxd->rx_dmamap = NULL;
		error = bus_dmamap_create(sc->fv_cdata.fv_rx_tag, 0,
		    &rxd->rx_dmamap);
		if (error != 0) {
			device_printf(sc->fv_dev,
			    "failed to create Rx dmamap\n");
			goto fail;
		}
	}

fail:
	return (error);
}

static void
fv_dma_free(struct fv_softc *sc)
{
	struct fv_txdesc	*txd;
	struct fv_rxdesc	*rxd;
	int			i;

	/* Tx ring. */
	if (sc->fv_cdata.fv_tx_ring_tag) {
		if (sc->fv_rdata.fv_tx_ring_paddr)
			bus_dmamap_unload(sc->fv_cdata.fv_tx_ring_tag,
			    sc->fv_cdata.fv_tx_ring_map);
		if (sc->fv_rdata.fv_tx_ring)
			bus_dmamem_free(sc->fv_cdata.fv_tx_ring_tag,
			    sc->fv_rdata.fv_tx_ring,
			    sc->fv_cdata.fv_tx_ring_map);
		sc->fv_rdata.fv_tx_ring = NULL;
		sc->fv_rdata.fv_tx_ring_paddr = 0;
		bus_dma_tag_destroy(sc->fv_cdata.fv_tx_ring_tag);
		sc->fv_cdata.fv_tx_ring_tag = NULL;
	}
	/* Rx ring. */
	if (sc->fv_cdata.fv_rx_ring_tag) {
		if (sc->fv_rdata.fv_rx_ring_paddr)
			bus_dmamap_unload(sc->fv_cdata.fv_rx_ring_tag,
			    sc->fv_cdata.fv_rx_ring_map);
		if (sc->fv_rdata.fv_rx_ring)
			bus_dmamem_free(sc->fv_cdata.fv_rx_ring_tag,
			    sc->fv_rdata.fv_rx_ring,
			    sc->fv_cdata.fv_rx_ring_map);
		sc->fv_rdata.fv_rx_ring = NULL;
		sc->fv_rdata.fv_rx_ring_paddr = 0;
		bus_dma_tag_destroy(sc->fv_cdata.fv_rx_ring_tag);
		sc->fv_cdata.fv_rx_ring_tag = NULL;
	}
	/* Tx buffers. */
	if (sc->fv_cdata.fv_tx_tag) {
		for (i = 0; i < FV_TX_RING_CNT; i++) {
			txd = &sc->fv_cdata.fv_txdesc[i];
			if (txd->tx_dmamap) {
				bus_dmamap_destroy(sc->fv_cdata.fv_tx_tag,
				    txd->tx_dmamap);
				txd->tx_dmamap = NULL;
			}
		}
		bus_dma_tag_destroy(sc->fv_cdata.fv_tx_tag);
		sc->fv_cdata.fv_tx_tag = NULL;
	}
	/* Rx buffers. */
	if (sc->fv_cdata.fv_rx_tag) {
		for (i = 0; i < FV_RX_RING_CNT; i++) {
			rxd = &sc->fv_cdata.fv_rxdesc[i];
			if (rxd->rx_dmamap) {
				bus_dmamap_destroy(sc->fv_cdata.fv_rx_tag,
				    rxd->rx_dmamap);
				rxd->rx_dmamap = NULL;
			}
		}
		if (sc->fv_cdata.fv_rx_sparemap) {
			bus_dmamap_destroy(sc->fv_cdata.fv_rx_tag,
			    sc->fv_cdata.fv_rx_sparemap);
			sc->fv_cdata.fv_rx_sparemap = 0;
		}
		bus_dma_tag_destroy(sc->fv_cdata.fv_rx_tag);
		sc->fv_cdata.fv_rx_tag = NULL;
	}

	if (sc->fv_cdata.fv_parent_tag) {
		bus_dma_tag_destroy(sc->fv_cdata.fv_parent_tag);
		sc->fv_cdata.fv_parent_tag = NULL;
	}
}

/*
 * Initialize the transmit descriptors.
 */
static int
fv_tx_ring_init(struct fv_softc *sc)
{
	struct fv_ring_data	*rd;
	struct fv_txdesc	*txd;
	bus_addr_t		addr;
	int			i;

	sc->fv_cdata.fv_tx_prod = 0;
	sc->fv_cdata.fv_tx_cons = 0;
	sc->fv_cdata.fv_tx_cnt = 0;
	sc->fv_cdata.fv_tx_pkts = 0;

	rd = &sc->fv_rdata;
	bzero(rd->fv_tx_ring, FV_TX_RING_SIZE);
	for (i = 0; i < FV_TX_RING_CNT; i++) {
		if (i == FV_TX_RING_CNT - 1)
			addr = FV_TX_RING_ADDR(sc, 0);
		else
			addr = FV_TX_RING_ADDR(sc, i + 1);
		rd->fv_tx_ring[i].fv_ctl = FV_CTL_IOF;
		rd->fv_tx_ring[i].fv_ca = 0;
		rd->fv_tx_ring[i].fv_devcs = 0;
		rd->fv_tx_ring[i].fv_link = 0;
		txd = &sc->fv_cdata.fv_txdesc[i];
		txd->tx_m = NULL;
	}

	bus_dmamap_sync(sc->fv_cdata.fv_tx_ring_tag,
	    sc->fv_cdata.fv_tx_ring_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	return (0);
}

/*
 * Initialize the RX descriptors and allocate mbufs for them. Note that
 * we arrange the descriptors in a closed ring, so that the last descriptor
 * points back to the first.
 */
static int
fv_rx_ring_init(struct fv_softc *sc)
{
	struct fv_ring_data	*rd;
	struct fv_rxdesc	*rxd;
	bus_addr_t		addr;
	int			i;

	sc->fv_cdata.fv_rx_cons = 0;

	rd = &sc->fv_rdata;
	bzero(rd->fv_rx_ring, FV_RX_RING_SIZE);
	for (i = 0; i < FV_RX_RING_CNT; i++) {
		rxd = &sc->fv_cdata.fv_rxdesc[i];
		rxd->rx_m = NULL;
		rxd->desc = &rd->fv_rx_ring[i];
		if (i == FV_RX_RING_CNT - 1)
			addr = FV_RX_RING_ADDR(sc, 0);
		else
			addr = FV_RX_RING_ADDR(sc, i + 1);
		rd->fv_rx_ring[i].fv_ctl = FV_CTL_IOD;
		if (i == FV_RX_RING_CNT - 1)
			rd->fv_rx_ring[i].fv_ctl |= FV_CTL_COD;
		rd->fv_rx_ring[i].fv_devcs = 0;
		rd->fv_rx_ring[i].fv_ca = 0;
		rd->fv_rx_ring[i].fv_link = addr;
		if (fv_newbuf(sc, i) != 0)
			return (ENOBUFS);
	}

	bus_dmamap_sync(sc->fv_cdata.fv_rx_ring_tag,
	    sc->fv_cdata.fv_rx_ring_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	return (0);
}

/*
 * Initialize an RX descriptor and attach an MBUF cluster.
 */
static int
fv_newbuf(struct fv_softc *sc, int idx)
{
	struct fv_desc		*desc;
	struct fv_rxdesc	*rxd;
	struct mbuf		*m;
	bus_dma_segment_t	segs[1];
	bus_dmamap_t		map;
	int			nsegs;

	m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL)
		return (ENOBUFS);
	m->m_len = m->m_pkthdr.len = MCLBYTES;
	m_adj(m, sizeof(uint64_t));

	if (bus_dmamap_load_mbuf_sg(sc->fv_cdata.fv_rx_tag,
	    sc->fv_cdata.fv_rx_sparemap, m, segs, &nsegs, 0) != 0) {
		m_freem(m);
		return (ENOBUFS);
	}
	KASSERT(nsegs == 1, ("%s: %d segments returned!", __func__, nsegs));

	rxd = &sc->fv_cdata.fv_rxdesc[idx];
	if (rxd->rx_m != NULL) {
		bus_dmamap_sync(sc->fv_cdata.fv_rx_tag, rxd->rx_dmamap,
		    BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(sc->fv_cdata.fv_rx_tag, rxd->rx_dmamap);
	}
	map = rxd->rx_dmamap;
	rxd->rx_dmamap = sc->fv_cdata.fv_rx_sparemap;
	sc->fv_cdata.fv_rx_sparemap = map;
	bus_dmamap_sync(sc->fv_cdata.fv_rx_tag, rxd->rx_dmamap,
	    BUS_DMASYNC_PREREAD);
	rxd->rx_m = m;
	desc = rxd->desc;
	desc->fv_ca = segs[0].ds_addr;
	desc->fv_ctl |= FV_DMASIZE(segs[0].ds_len);
	rxd->saved_ca = desc->fv_ca ;
	rxd->saved_ctl = desc->fv_ctl ;

	return (0);
}

static __inline void
fv_fixup_rx(struct mbuf *m)
{
        int		i;
        uint16_t	*src, *dst;

	src = mtod(m, uint16_t *);
	dst = src - 1;

	for (i = 0; i < (m->m_len / sizeof(uint16_t) + 1); i++)
		*dst++ = *src++;

	m->m_data -= ETHER_ALIGN;
}


static void
fv_tx(struct fv_softc *sc)
{
	struct fv_txdesc	*txd;
	struct fv_desc		*cur_tx;
	struct ifnet		*ifp;
	uint32_t		ctl, devcs;
	int			cons, prod;

	FV_LOCK_ASSERT(sc);

	cons = sc->fv_cdata.fv_tx_cons;
	prod = sc->fv_cdata.fv_tx_prod;
	if (cons == prod)
		return;

	bus_dmamap_sync(sc->fv_cdata.fv_tx_ring_tag,
	    sc->fv_cdata.fv_tx_ring_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	ifp = sc->fv_ifp;
	/*
	 * Go through our tx list and free mbufs for those
	 * frames that have been transmitted.
	 */
	for (; cons != prod; FV_INC(cons, FV_TX_RING_CNT)) {
		cur_tx = &sc->fv_rdata.fv_tx_ring[cons];
		ctl = cur_tx->fv_ctl;
		devcs = cur_tx->fv_devcs;
		/* Check if descriptor has "finished" flag */
		if ((ctl & FV_CTL_F) == 0)
			break;

		sc->fv_cdata.fv_tx_cnt--;
		ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

		txd = &sc->fv_cdata.fv_txdesc[cons];

		if (devcs & FV_DMATX_DEVCS_TOK)
			if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
		else {
			if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
			/* collisions: medium busy, late collision */
			if ((devcs & FV_DMATX_DEVCS_EC) || 
			    (devcs & FV_DMATX_DEVCS_LC))
				if_inc_counter(ifp, IFCOUNTER_COLLISIONS, 1);
		}

		bus_dmamap_sync(sc->fv_cdata.fv_tx_tag, txd->tx_dmamap,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->fv_cdata.fv_tx_tag, txd->tx_dmamap);

		/* Free only if it's first descriptor in list */
		if (txd->tx_m)
			m_freem(txd->tx_m);
		txd->tx_m = NULL;

		/* reset descriptor */
		cur_tx->fv_ctl = FV_CTL_IOF;
		cur_tx->fv_devcs = 0;
		cur_tx->fv_ca = 0;
		cur_tx->fv_link = 0; 
	}

	sc->fv_cdata.fv_tx_cons = cons;

	bus_dmamap_sync(sc->fv_cdata.fv_tx_ring_tag,
	    sc->fv_cdata.fv_tx_ring_map, BUS_DMASYNC_PREWRITE);
}


static void
fv_rx(struct fv_softc *sc)
{
	struct fv_rxdesc	*rxd;
	struct ifnet		*ifp = sc->fv_ifp;
	int			cons, prog, packet_len, count, error;
	struct fv_desc		*cur_rx;
	struct mbuf		*m;

	FV_LOCK_ASSERT(sc);

	cons = sc->fv_cdata.fv_rx_cons;

	bus_dmamap_sync(sc->fv_cdata.fv_rx_ring_tag,
	    sc->fv_cdata.fv_rx_ring_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	for (prog = 0; prog < FV_RX_RING_CNT; FV_INC(cons, FV_RX_RING_CNT)) {
		cur_rx = &sc->fv_rdata.fv_rx_ring[cons];
		rxd = &sc->fv_cdata.fv_rxdesc[cons];
		m = rxd->rx_m;

		if ((cur_rx->fv_ctl & FV_CTL_D) == 0)
		       break;	

		prog++;

		packet_len = FV_PKTSIZE(cur_rx->fv_devcs);
		count = m->m_len - FV_DMASIZE(cur_rx->fv_ctl);
		/* Assume it's error */
		error = 1;

		if (packet_len != count)
			if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
		else if (count < 64)
			if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
		else if ((cur_rx->fv_devcs & FV_DMARX_DEVCS_LD) == 0)
			if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
		else if ((cur_rx->fv_devcs & FV_DMARX_DEVCS_ROK) != 0) {
			error = 0;
			bus_dmamap_sync(sc->fv_cdata.fv_rx_tag, rxd->rx_dmamap,
			    BUS_DMASYNC_PREREAD);
			m = rxd->rx_m;
			fv_fixup_rx(m);
			m->m_pkthdr.rcvif = ifp;
			/* Skip 4 bytes of CRC */
			m->m_pkthdr.len = m->m_len = packet_len - ETHER_CRC_LEN;
			if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);

			FV_UNLOCK(sc);
			(*ifp->if_input)(ifp, m);
			FV_LOCK(sc);
		}

		if (error) {
			/* Restore CONTROL and CA values, reset DEVCS */
			cur_rx->fv_ctl = rxd->saved_ctl;
			cur_rx->fv_ca = rxd->saved_ca;
			cur_rx->fv_devcs = 0;
		}
		else {
			/* Reinit descriptor */
			cur_rx->fv_ctl = FV_CTL_IOD;
			if (cons == FV_RX_RING_CNT - 1)
				cur_rx->fv_ctl |= FV_CTL_COD;
			cur_rx->fv_devcs = 0;
			cur_rx->fv_ca = 0;
			if (fv_newbuf(sc, cons) != 0) {
				device_printf(sc->fv_dev, 
				    "Failed to allocate buffer\n");
				break;
			}
		}

		bus_dmamap_sync(sc->fv_cdata.fv_rx_ring_tag,
		    sc->fv_cdata.fv_rx_ring_map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	}

	if (prog > 0) {
		sc->fv_cdata.fv_rx_cons = cons;

		bus_dmamap_sync(sc->fv_cdata.fv_rx_ring_tag,
		    sc->fv_cdata.fv_rx_ring_map,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	}
}

static void
fv_rx_intr(void *arg)
{
	struct fv_softc		*sc = arg;
	uint32_t		status;

	FV_LOCK(sc);

	/* mask out interrupts */
	FV_DMA_SETBITS_REG(FV_DMA_RXCHAN, DMA_SM, 
	    DMA_SM_D | DMA_SM_H | DMA_SM_E);

	status = FV_DMA_READ_REG(FV_DMA_RXCHAN, DMA_S);
	if (status & (DMA_S_D | DMA_S_E | DMA_S_H)) {
		fv_rx(sc);

		if (status & DMA_S_E)
			device_printf(sc->fv_dev, "RX DMA error\n");
	}

	/* Reread status */
	status = FV_DMA_READ_REG(FV_DMA_RXCHAN, DMA_S);

	/* restart DMA RX  if it has been halted */
	if (status & DMA_S_H) {
		FV_DMA_WRITE_REG(FV_DMA_RXCHAN, DMA_DPTR, 
		    FV_RX_RING_ADDR(sc, sc->fv_cdata.fv_rx_cons));
	}

	FV_DMA_WRITE_REG(FV_DMA_RXCHAN, DMA_S, ~status);

	/* Enable F, H, E interrupts */
	FV_DMA_CLEARBITS_REG(FV_DMA_RXCHAN, DMA_SM, 
	    DMA_SM_D | DMA_SM_H | DMA_SM_E);

	FV_UNLOCK(sc);
}

static void
fv_tx_intr(void *arg)
{
	struct fv_softc		*sc = arg;
	uint32_t		status;

	FV_LOCK(sc);

	/* mask out interrupts */
	FV_DMA_SETBITS_REG(FV_DMA_TXCHAN, DMA_SM, 
	    DMA_SM_F | DMA_SM_E);

	status = FV_DMA_READ_REG(FV_DMA_TXCHAN, DMA_S);
	if (status & (DMA_S_F | DMA_S_E)) {
		fv_tx(sc);
		if (status & DMA_S_E)
			device_printf(sc->fv_dev, "DMA error\n");
	}

	FV_DMA_WRITE_REG(FV_DMA_TXCHAN, DMA_S, ~status);

	/* Enable F, E interrupts */
	FV_DMA_CLEARBITS_REG(FV_DMA_TXCHAN, DMA_SM, 
	    DMA_SM_F | DMA_SM_E);

	FV_UNLOCK(sc);

}

static void
fv_rx_und_intr(void *arg)
{

	panic("interrupt: %s\n", __func__);
}

static void
fv_tx_ovr_intr(void *arg)
{

	panic("interrupt: %s\n", __func__);
}

static void
fv_tick(void *xsc)
{
	struct fv_softc		*sc = xsc;
	struct mii_data		*mii;

	FV_LOCK_ASSERT(sc);

	mii = device_get_softc(sc->fv_miibus);
	mii_tick(mii);
	callout_reset(&sc->fv_stat_callout, hz, fv_tick, sc);
}
