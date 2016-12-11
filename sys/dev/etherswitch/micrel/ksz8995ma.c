/*-
 * Copyright (c) 2016 Hiroki Mori
 * Copyright (c) 2013 Luiz Otavio O Souza.
 * Copyright (c) 2011-2012 Stefan Bethke.
 * Copyright (c) 2012 Adrian Chadd.
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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/ethernet.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <machine/bus.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/etherswitch/etherswitch.h>

#include <dev/spibus/spi.h>

#include "spibus_if.h"
#include "miibus_if.h"
#include "etherswitch_if.h"

#define	KSZ8995MA_PRODUCT_CODE	0x7102

#define	KSZ8995MA_SC3		0x11
#define	KSZ8995MA_VF0L		0x40
#define	KSZ8995MA_VF0H		0x41
#define	KSZ8995MA_CI0		0xa0
#define	KSZ8995MA_CI1		0xa1
#define	KSZ8995MA_PHY_C0	0x200

#define	KSZ8995MA_PC_SHIFT	4
#define	KSZ8995MA_TBV_SHIFT	5
#define	KSZ8995MA_PVID_SHIFT	10
#define	KSZ8995MA_OPTE_SHIFT	4
#define	KSZ8995MA_VV_SHIFT	15

#define	KSZ8995MA_PHY_SIZE	0x20

MALLOC_DECLARE(M_KSZ8995MA);
MALLOC_DEFINE(M_KSZ8995MA, "ksz8995ma", "ksz8995ma data structures");

struct ksz8995ma_softc {
	struct mtx	sc_mtx;		/* serialize access to softc */
	device_t	sc_dev;
	int		vlan_mode;
	int		media;		/* cpu port media */
	int		cpuport;	/* which PHY is connected to the CPU */
	int		phymask;	/* PHYs we manage */
	int		numports;	/* number of ports */
	int		ifpport[MII_NPHY];
	int		*portphy;
	char		**ifname;
	device_t	**miibus;
	struct ifnet	**ifp;
	struct callout	callout_tick;
	etherswitch_info_t	info;
};

#define	KSZ8995MA_LOCK(_sc)			\
	    mtx_lock(&(_sc)->sc_mtx)
#define	KSZ8995MA_UNLOCK(_sc)			\
	    mtx_unlock(&(_sc)->sc_mtx)
#define	KSZ8995MA_LOCK_ASSERT(_sc, _what)	\
	    mtx_assert(&(_sc)->sc_mtx, (_what))
#define	KSZ8995MA_TRYLOCK(_sc)			\
	    mtx_trylock(&(_sc)->sc_mtx)

#if defined(DEBUG)
#define	DPRINTF(dev, args...) device_printf(dev, args)
#else
#define	DPRINTF(dev, args...)
#endif

static inline int ksz8995ma_portforphy(struct ksz8995ma_softc *, int);
static void ksz8995ma_tick(void *);
static int ksz8995ma_ifmedia_upd(struct ifnet *);
static void ksz8995ma_ifmedia_sts(struct ifnet *, struct ifmediareq *);

static int
ksz8995ma_probe(device_t dev)
{
	int data1, data2;
	int pc;
	struct ksz8995ma_softc *sc;
	uint8_t txBuf[8], rxBuf[8];
	struct spi_command cmd;
	int err;

printf("MORI MORI ksz8995ma_probe\n");

	memset(&cmd, 0, sizeof(cmd));
	memset(txBuf, 0, sizeof(txBuf));
	memset(rxBuf, 0, sizeof(rxBuf));

	/* read spi */
	txBuf[0] = 0x03;
	txBuf[1] = 0x00;
	cmd.tx_cmd = &txBuf;
	cmd.rx_cmd = &rxBuf;
	cmd.tx_cmd_sz = 3;
	cmd.rx_cmd_sz = 3;
        err = SPIBUS_TRANSFER(device_get_parent(dev), dev, &cmd);
	if (err)
		return(0);

printf("SPI Trans %x\n", rxBuf[2]);

	sc = device_get_softc(dev);
	bzero(sc, sizeof(*sc));

	data1 = 0;
	data2 = 0;
	pc = ((data2 << 16) | data1) >> KSZ8995MA_PC_SHIFT;
	if (bootverbose)
		device_printf(dev,"Chip Identifier Register %x %x\n", data1,
		    data2);

	/* check Product Code */
	if (pc != KSZ8995MA_PRODUCT_CODE) {
		return (ENXIO);
	}

	device_set_desc_copy(dev, "Infineon KSZ8995MA/M/MX MDIO switch driver");
	return (BUS_PROBE_DEFAULT);
}

static int
ksz8995ma_attach_phys(struct ksz8995ma_softc *sc)
{
	int phy, port, err;
	char name[IFNAMSIZ];

	port = 0;
	err = 0;
	/* PHYs need an interface, so we generate a dummy one */
	snprintf(name, IFNAMSIZ, "%sport", device_get_nameunit(sc->sc_dev));
	for (phy = 0; phy < sc->numports; phy++) {
		if (((1 << phy) & sc->phymask) == 0)
			continue;
		sc->ifpport[phy] = port;
		sc->portphy[port] = phy;
		sc->ifp[port] = if_alloc(IFT_ETHER);
		sc->ifp[port]->if_softc = sc;
		sc->ifp[port]->if_flags |= IFF_UP | IFF_BROADCAST |
		    IFF_DRV_RUNNING | IFF_SIMPLEX;
		if_initname(sc->ifp[port], name, port);
		sc->miibus[port] = malloc(sizeof(device_t), M_KSZ8995MA,
		    M_WAITOK | M_ZERO);
		if (sc->miibus[port] == NULL) {
			err = ENOMEM;
			goto failed;
		}
		err = mii_attach(sc->sc_dev, sc->miibus[port], sc->ifp[port],
		    ksz8995ma_ifmedia_upd, ksz8995ma_ifmedia_sts, \
		    BMSR_DEFCAPMASK, phy, MII_OFFSET_ANY, 0);
		DPRINTF(sc->sc_dev, "%s attached to pseudo interface %s\n",
		    device_get_nameunit(*sc->miibus[port]),
		    sc->ifp[port]->if_xname);
		if (err != 0) {
			device_printf(sc->sc_dev,
			    "attaching PHY %d failed\n",
			    phy);
			goto failed;
		}
		++port;
	}
	sc->info.es_nports = port;
	if (sc->cpuport != -1) {
		/* assume cpuport is last one */
		sc->ifpport[sc->cpuport] = port;
		sc->portphy[port] = sc->cpuport;
		++sc->info.es_nports;
	}
	return (0);

failed:
	for (phy = 0; phy < sc->numports; phy++) {
		if (((1 << phy) & sc->phymask) == 0)
			continue;
		port = ksz8995ma_portforphy(sc, phy);
		if (sc->miibus[port] != NULL)
			device_delete_child(sc->sc_dev, (*sc->miibus[port]));
		if (sc->ifp[port] != NULL)
			if_free(sc->ifp[port]);
		if (sc->ifname[port] != NULL)
			free(sc->ifname[port], M_KSZ8995MA);
		if (sc->miibus[port] != NULL)
			free(sc->miibus[port], M_KSZ8995MA);
	}
	return (err);
}

static int
ksz8995ma_attach(device_t dev)
{
	struct ksz8995ma_softc	*sc;
	int			 err;

	err = 0;
	sc = device_get_softc(dev);

	sc->sc_dev = dev;
	mtx_init(&sc->sc_mtx, "ksz8995ma", NULL, MTX_DEF);
	strlcpy(sc->info.es_name, device_get_desc(dev),
	    sizeof(sc->info.es_name));

	/* KSZ8995MA Defaults */
	sc->numports = 6;
	sc->phymask = 0x1f;
	sc->cpuport = 5;
	sc->media = 100;

	sc->info.es_nvlangroups = 16;
	sc->info.es_vlan_caps = ETHERSWITCH_VLAN_PORT | ETHERSWITCH_VLAN_DOT1Q;

	sc->ifp = malloc(sizeof(struct ifnet *) * sc->numports, M_KSZ8995MA,
	    M_WAITOK | M_ZERO);
	sc->ifname = malloc(sizeof(char *) * sc->numports, M_KSZ8995MA,
	    M_WAITOK | M_ZERO);
	sc->miibus = malloc(sizeof(device_t *) * sc->numports, M_KSZ8995MA,
	    M_WAITOK | M_ZERO);
	sc->portphy = malloc(sizeof(int) * sc->numports, M_KSZ8995MA,
	    M_WAITOK | M_ZERO);

	if (sc->ifp == NULL || sc->ifname == NULL || sc->miibus == NULL ||
	    sc->portphy == NULL) {
		err = ENOMEM;
		goto failed;
	}

	/*
	 * Attach the PHYs and complete the bus enumeration.
	 */
	err = ksz8995ma_attach_phys(sc);
	if (err != 0)
		goto failed;

	bus_generic_probe(dev);
	bus_enumerate_hinted_children(dev);
	err = bus_generic_attach(dev);
	if (err != 0)
		goto failed;
	
	callout_init(&sc->callout_tick, 0);

	ksz8995ma_tick(sc);
	
	return (0);

failed:
	if (sc->portphy != NULL)
		free(sc->portphy, M_KSZ8995MA);
	if (sc->miibus != NULL)
		free(sc->miibus, M_KSZ8995MA);
	if (sc->ifname != NULL)
		free(sc->ifname, M_KSZ8995MA);
	if (sc->ifp != NULL)
		free(sc->ifp, M_KSZ8995MA);

	return (err);
}

static int
ksz8995ma_detach(device_t dev)
{
	struct ksz8995ma_softc	*sc;
	int			 i, port;

	sc = device_get_softc(dev);

	callout_drain(&sc->callout_tick);

	for (i = 0; i < MII_NPHY; i++) {
		if (((1 << i) & sc->phymask) == 0)
			continue;
		port = ksz8995ma_portforphy(sc, i);
		if (sc->miibus[port] != NULL)
			device_delete_child(dev, (*sc->miibus[port]));
		if (sc->ifp[port] != NULL)
			if_free(sc->ifp[port]);
		free(sc->ifname[port], M_KSZ8995MA);
		free(sc->miibus[port], M_KSZ8995MA);
	}

	free(sc->portphy, M_KSZ8995MA);
	free(sc->miibus, M_KSZ8995MA);
	free(sc->ifname, M_KSZ8995MA);
	free(sc->ifp, M_KSZ8995MA);

	bus_generic_detach(dev);
	mtx_destroy(&sc->sc_mtx);

	return (0);
}

/*
 * Convert PHY number to port number.
 */
static inline int
ksz8995ma_portforphy(struct ksz8995ma_softc *sc, int phy)
{

	return (sc->ifpport[phy]);
}

static inline struct mii_data *
ksz8995ma_miiforport(struct ksz8995ma_softc *sc, int port)
{

	if (port < 0 || port > sc->numports)
		return (NULL);
	if (port == sc->cpuport)
		return (NULL);
	return (device_get_softc(*sc->miibus[port]));
}

static inline struct ifnet *
ksz8995ma_ifpforport(struct ksz8995ma_softc *sc, int port)
{

	if (port < 0 || port > sc->numports)
		return (NULL);
	return (sc->ifp[port]);
}

/*
 * Poll the status for all PHYs.
 */
static void
ksz8995ma_miipollstat(struct ksz8995ma_softc *sc)
{
	int i, port;
	struct mii_data *mii;
	struct mii_softc *miisc;

	KSZ8995MA_LOCK_ASSERT(sc, MA_NOTOWNED);

	for (i = 0; i < MII_NPHY; i++) {
		if (((1 << i) & sc->phymask) == 0)
			continue;
		port = ksz8995ma_portforphy(sc, i);
		if ((*sc->miibus[port]) == NULL)
			continue;
		mii = device_get_softc(*sc->miibus[port]);
		LIST_FOREACH(miisc, &mii->mii_phys, mii_list) {
			if (IFM_INST(mii->mii_media.ifm_cur->ifm_media) !=
			    miisc->mii_inst)
				continue;
			ukphy_status(miisc);
			mii_phy_update(miisc, MII_POLLSTAT);
		}
	}
}

static void
ksz8995ma_tick(void *arg)
{
	struct ksz8995ma_softc *sc;

	sc = arg;

	ksz8995ma_miipollstat(sc);
	callout_reset(&sc->callout_tick, hz, ksz8995ma_tick, sc);
}

static void
ksz8995ma_lock(device_t dev)
{
	struct ksz8995ma_softc *sc;

	sc = device_get_softc(dev);

	KSZ8995MA_LOCK_ASSERT(sc, MA_NOTOWNED);
	KSZ8995MA_LOCK(sc);
}

static void
ksz8995ma_unlock(device_t dev)
{
	struct ksz8995ma_softc *sc;

	sc = device_get_softc(dev);

	KSZ8995MA_LOCK_ASSERT(sc, MA_OWNED);
	KSZ8995MA_UNLOCK(sc);
}

static etherswitch_info_t *
ksz8995ma_getinfo(device_t dev)
{
	struct ksz8995ma_softc *sc;

	sc = device_get_softc(dev);
	
	return (&sc->info);
}

static int
ksz8995ma_getport(device_t dev, etherswitch_port_t *p)
{
	return (0);
}

static int
ksz8995ma_setport(device_t dev, etherswitch_port_t *p)
{
	return (0);
}

static int
ksz8995ma_getvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	return (0);
}

static int
ksz8995ma_setvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	return (0);
}

static int
ksz8995ma_getconf(device_t dev, etherswitch_conf_t *conf)
{
	struct ksz8995ma_softc *sc;

	sc = device_get_softc(dev);

	/* Return the VLAN mode. */
	conf->cmd = ETHERSWITCH_CONF_VLAN_MODE;
	conf->vlan_mode = sc->vlan_mode;

	return (0);
}

static int
ksz8995ma_setconf(device_t dev, etherswitch_conf_t *conf)
{
	return (0);
}

static void
ksz8995ma_statchg(device_t dev)
{

	DPRINTF(dev, "%s\n", __func__);
}

static int
ksz8995ma_ifmedia_upd(struct ifnet *ifp)
{
	struct ksz8995ma_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = ksz8995ma_miiforport(sc, ifp->if_dunit);

	DPRINTF(sc->sc_dev, "%s\n", __func__);
	if (mii == NULL)
		return (ENXIO);
	mii_mediachg(mii);
	return (0);
}

static void
ksz8995ma_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct ksz8995ma_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = ksz8995ma_miiforport(sc, ifp->if_dunit);

	DPRINTF(sc->sc_dev, "%s\n", __func__);

	if (mii == NULL)
		return;
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
}

static int
ksz8995ma_readphy(device_t dev, int phy, int reg)
{

	return (0);
}

static int
ksz8995ma_writephy(device_t dev, int phy, int reg, int data)
{

	return (0);
}

static int
ksz8995ma_readreg(device_t dev, int addr)
{

	return (0);
}

static int
ksz8995ma_writereg(device_t dev, int addr, int value)
{

	return (0);
}

static device_method_t ksz8995ma_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		ksz8995ma_probe),
	DEVMETHOD(device_attach,	ksz8995ma_attach),
	DEVMETHOD(device_detach,	ksz8995ma_detach),
	
	/* bus interface */
	DEVMETHOD(bus_add_child,	device_add_child_ordered),
	
	/* MII interface */
	DEVMETHOD(miibus_readreg,	ksz8995ma_readphy),
	DEVMETHOD(miibus_writereg,	ksz8995ma_writephy),
	DEVMETHOD(miibus_statchg,	ksz8995ma_statchg),

	/* MDIO interface */
/*
	DEVMETHOD(mdio_readreg,		ksz8995ma_readphy),
	DEVMETHOD(mdio_writereg,	ksz8995ma_writephy),
*/

	/* etherswitch interface */
	DEVMETHOD(etherswitch_lock,	ksz8995ma_lock),
	DEVMETHOD(etherswitch_unlock,	ksz8995ma_unlock),
	DEVMETHOD(etherswitch_getinfo,	ksz8995ma_getinfo),
	DEVMETHOD(etherswitch_readreg,	ksz8995ma_readreg),
	DEVMETHOD(etherswitch_writereg,	ksz8995ma_writereg),
	DEVMETHOD(etherswitch_readphyreg,	ksz8995ma_readphy),
	DEVMETHOD(etherswitch_writephyreg,	ksz8995ma_writephy),
	DEVMETHOD(etherswitch_getport,	ksz8995ma_getport),
	DEVMETHOD(etherswitch_setport,	ksz8995ma_setport),
	DEVMETHOD(etherswitch_getvgroup,	ksz8995ma_getvgroup),
	DEVMETHOD(etherswitch_setvgroup,	ksz8995ma_setvgroup),
	DEVMETHOD(etherswitch_setconf,	ksz8995ma_setconf),
	DEVMETHOD(etherswitch_getconf,	ksz8995ma_getconf),

	DEVMETHOD_END
};

DEFINE_CLASS_0(ksz8995ma, ksz8995ma_driver, ksz8995ma_methods,
    sizeof(struct ksz8995ma_softc));
static devclass_t ksz8995ma_devclass;

DRIVER_MODULE(ksz8995ma, spibus, ksz8995ma_driver, ksz8995ma_devclass, 0, 0);
DRIVER_MODULE(etherswitch, ksz8995ma, etherswitch_driver, etherswitch_devclass,
    0, 0);
MODULE_VERSION(ksz8995ma, 1);
MODULE_DEPEND(ksz8995ma, spibus, 1, 1, 1); /* XXX which versions? */
MODULE_DEPEND(ksz8995ma, etherswitch, 1, 1, 1); /* XXX which versions? */
