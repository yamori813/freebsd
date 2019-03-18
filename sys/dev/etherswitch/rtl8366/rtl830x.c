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

/*
 * This is RTL8305SB, RTL8305SB Ver.D driver.
 * RTL8305SC, RTL8306SD is not support yet.
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
#include <dev/mdio/mdio.h>

#include <dev/etherswitch/etherswitch.h>

#include "mdio_if.h"
#include "miibus_if.h"
#include "etherswitch_if.h"

#define	RTL830X_SB		1
#define	RTL830X_SC		2

#define	SB_DISVLAN		(1 << 5)

MALLOC_DECLARE(M_RTL830X);
MALLOC_DEFINE(M_RTL830X, "rtl830x", "rtl830x data structures");

struct rtl830x_softc {
	struct mtx	sc_mtx;		/* serialize access to softc */
	device_t	sc_dev;
	int		dev_type;
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

#define	RTL830X_LOCK(_sc)			\
	    mtx_lock(&(_sc)->sc_mtx)
#define	RTL830X_UNLOCK(_sc)			\
	    mtx_unlock(&(_sc)->sc_mtx)
#define	RTL830X_LOCK_ASSERT(_sc, _what)	\
	    mtx_assert(&(_sc)->sc_mtx, (_what))
#define	RTL830X_TRYLOCK(_sc)			\
	    mtx_trylock(&(_sc)->sc_mtx)

#if defined(DEBUG)
#define	DPRINTF(dev, args...) device_printf(dev, args)
#else
#define	DPRINTF(dev, args...)
#endif

static inline int rtl830x_portforphy(struct rtl830x_softc *, int);
static void rtl830x_tick(void *);
static int rtl830x_ifmedia_upd(struct ifnet *);
static void rtl830x_ifmedia_sts(struct ifnet *, struct ifmediareq *);

#define	RTL830X_READREG(dev, x)					\
	MDIO_READREG(dev, ((x) >> 5), ((x) & 0x1f));
#define	RTL830X_WRITEREG(dev, x, v)					\
	MDIO_WRITEREG(dev, ((x) >> 5), ((x) & 0x1f), v);

#define	RTL830X_PVIDBYDATA(data1, data2)				\
	((((data1) >> RTL830X_PVID_SHIFT) & 0x0f) | ((data2) << 4))

static int
rtl830x_probe(device_t dev)
{

	device_set_desc_copy(dev, "Realtek RTL830X MDIO switch driver");
	return (BUS_PROBE_DEFAULT);
}

static int
rtl830x_attach_phys(struct rtl830x_softc *sc)
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
		if (sc->ifp[port] == NULL) {
			device_printf(sc->sc_dev, "couldn't allocate ifnet structure\n");
			err = ENOMEM;
			break;
		}

		sc->ifp[port]->if_softc = sc;
		sc->ifp[port]->if_flags |= IFF_UP | IFF_BROADCAST |
		    IFF_DRV_RUNNING | IFF_SIMPLEX;
		if_initname(sc->ifp[port], name, port);
		sc->miibus[port] = malloc(sizeof(device_t), M_RTL830X,
		    M_WAITOK | M_ZERO);
		if (sc->miibus[port] == NULL) {
			err = ENOMEM;
			goto failed;
		}
		err = mii_attach(sc->sc_dev, sc->miibus[port], sc->ifp[port],
		    rtl830x_ifmedia_upd, rtl830x_ifmedia_sts, \
		    BMSR_DEFCAPMASK, phy, MII_OFFSET_ANY, 0x0);
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
		port = rtl830x_portforphy(sc, phy);
		if (sc->miibus[port] != NULL)
			device_delete_child(sc->sc_dev, (*sc->miibus[port]));
		if (sc->ifp[port] != NULL)
			if_free(sc->ifp[port]);
		if (sc->ifname[port] != NULL)
			free(sc->ifname[port], M_RTL830X);
		if (sc->miibus[port] != NULL)
			free(sc->miibus[port], M_RTL830X);
	}
	return (err);
}

static int
rtl830x_attach(device_t dev)
{
	struct rtl830x_softc	*sc;
	int			 err;
	int			 reg;

	err = 0;
	sc = device_get_softc(dev);

	sc->sc_dev = dev;
	mtx_init(&sc->sc_mtx, "rtl830x", NULL, MTX_DEF);
	strlcpy(sc->info.es_name, device_get_desc(dev),
	    sizeof(sc->info.es_name));

	reg = RTL830X_READREG(device_get_parent(dev), 2);
	if (reg == 0)
		sc->dev_type = RTL830X_SB;
	else
		sc->dev_type = RTL830X_SC;

	/* RTL830X Defaults */
	sc->numports = 5;
	sc->phymask = 0x0f;
	sc->cpuport = 4;
	sc->media = 100;

	if (sc->dev_type == RTL830X_SB)
		sc->info.es_nvlangroups = 5;
	else {
/* Not support yet
		sc->info.es_nvlangroups = 16;
*/
		err = ENXIO;
		goto failed;
	}

	sc->info.es_vlan_caps = ETHERSWITCH_VLAN_PORT;

	sc->ifp = malloc(sizeof(struct ifnet *) * sc->numports, M_RTL830X,
	    M_WAITOK | M_ZERO);
	sc->ifname = malloc(sizeof(char *) * sc->numports, M_RTL830X,
	    M_WAITOK | M_ZERO);
	sc->miibus = malloc(sizeof(device_t *) * sc->numports, M_RTL830X,
	    M_WAITOK | M_ZERO);
	sc->portphy = malloc(sizeof(int) * sc->numports, M_RTL830X,
	    M_WAITOK | M_ZERO);

	if (sc->ifp == NULL || sc->ifname == NULL || sc->miibus == NULL ||
	    sc->portphy == NULL) {
		err = ENOMEM;
		goto failed;
	}

	/*
	 * Attach the PHYs and complete the bus enumeration.
	 */
	err = rtl830x_attach_phys(sc);
	if (err != 0)
		goto failed;

	bus_generic_probe(dev);
	bus_enumerate_hinted_children(dev);
	err = bus_generic_attach(dev);
	if (err != 0)
		goto failed;
	
	callout_init(&sc->callout_tick, 0);

	rtl830x_tick(sc);
	
	return (0);

failed:
	if (sc->portphy != NULL)
		free(sc->portphy, M_RTL830X);
	if (sc->miibus != NULL)
		free(sc->miibus, M_RTL830X);
	if (sc->ifname != NULL)
		free(sc->ifname, M_RTL830X);
	if (sc->ifp != NULL)
		free(sc->ifp, M_RTL830X);

	return (err);
}

static int
rtl830x_detach(device_t dev)
{
	struct rtl830x_softc	*sc;
	int			 i, port;

	sc = device_get_softc(dev);

	callout_drain(&sc->callout_tick);

	for (i = 0; i < MII_NPHY; i++) {
		if (((1 << i) & sc->phymask) == 0)
			continue;
		port = rtl830x_portforphy(sc, i);
		if (sc->miibus[port] != NULL)
			device_delete_child(dev, (*sc->miibus[port]));
		if (sc->ifp[port] != NULL)
			if_free(sc->ifp[port]);
		free(sc->ifname[port], M_RTL830X);
		free(sc->miibus[port], M_RTL830X);
	}

	free(sc->portphy, M_RTL830X);
	free(sc->miibus, M_RTL830X);
	free(sc->ifname, M_RTL830X);
	free(sc->ifp, M_RTL830X);

	bus_generic_detach(dev);
	mtx_destroy(&sc->sc_mtx);

	return (0);
}

/*
 * Convert PHY number to port number.
 */
static inline int
rtl830x_portforphy(struct rtl830x_softc *sc, int phy)
{

	return (sc->ifpport[phy]);
}

static inline struct mii_data *
rtl830x_miiforport(struct rtl830x_softc *sc, int port)
{

	if (port < 0 || port > sc->numports)
		return (NULL);
	if (port == sc->cpuport)
		return (NULL);
	return (device_get_softc(*sc->miibus[port]));
}

static inline struct ifnet *
rtl830x_ifpforport(struct rtl830x_softc *sc, int port)
{

	if (port < 0 || port > sc->numports)
		return (NULL);
	return (sc->ifp[port]);
}

/*
 * Poll the status for all PHYs.
 */
static void
rtl830x_miipollstat(struct rtl830x_softc *sc)
{
	int i, port;
	struct mii_data *mii;
	struct mii_softc *miisc;

	RTL830X_LOCK_ASSERT(sc, MA_NOTOWNED);

	for (i = 0; i < MII_NPHY; i++) {
		if (((1 << i) & sc->phymask) == 0)
			continue;
		port = rtl830x_portforphy(sc, i);
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
rtl830x_tick(void *arg)
{
	struct rtl830x_softc *sc;

	sc = arg;

	rtl830x_miipollstat(sc);
	callout_reset(&sc->callout_tick, hz, rtl830x_tick, sc);
}

static void
rtl830x_lock(device_t dev)
{
	struct rtl830x_softc *sc;

	sc = device_get_softc(dev);

	RTL830X_LOCK_ASSERT(sc, MA_NOTOWNED);
	RTL830X_LOCK(sc);
}

static void
rtl830x_unlock(device_t dev)
{
	struct rtl830x_softc *sc;

	sc = device_get_softc(dev);

	RTL830X_LOCK_ASSERT(sc, MA_OWNED);
	RTL830X_UNLOCK(sc);
}

static etherswitch_info_t *
rtl830x_getinfo(device_t dev)
{
	struct rtl830x_softc *sc;

	sc = device_get_softc(dev);
	
	return (&sc->info);
}

static int
rtl830x_getport(device_t dev, etherswitch_port_t *p)
{
	struct rtl830x_softc	*sc;
	struct mii_data		*mii;
	struct ifmediareq	*ifmr;
	int 			 err, phy;
	device_t		 parent;

	sc = device_get_softc(dev);
	parent = device_get_parent(dev);
	ifmr = &p->es_ifmr;

	if (p->es_port < 0 || p->es_port >= sc->numports)
		return (ENXIO);

	if (sc->vlan_mode == ETHERSWITCH_VLAN_PORT) {
		p->es_pvid = 0;
	} else {
		p->es_pvid = 0;
	}

	phy = sc->portphy[p->es_port];
	mii = rtl830x_miiforport(sc, p->es_port);
	if (sc->cpuport != -1 && phy == sc->cpuport) {
		/* fill in fixed values for CPU port */
		p->es_flags |= ETHERSWITCH_PORT_CPU;
		ifmr->ifm_count = 0;
		if (sc->media == 100)
			ifmr->ifm_current = ifmr->ifm_active =
			    IFM_ETHER | IFM_100_TX | IFM_FDX;
		else
			ifmr->ifm_current = ifmr->ifm_active =
			    IFM_ETHER | IFM_1000_T | IFM_FDX;
		ifmr->ifm_mask = 0;
		ifmr->ifm_status = IFM_ACTIVE | IFM_AVALID;
	} else if (mii != NULL) {
		err = ifmedia_ioctl(mii->mii_ifp, &p->es_ifr,
		    &mii->mii_media, SIOCGIFMEDIA);
		if (err)
			return (err);
	} else {
		return (ENXIO);
	}

	return (0);
}

static int
rtl830x_setport(device_t dev, etherswitch_port_t *p)
{
	struct rtl830x_softc	*sc;
	struct ifmedia		*ifm;
	struct mii_data		*mii;
	struct ifnet		*ifp;
	device_t		 parent;
	int 			 err;

	sc = device_get_softc(dev);
	parent = device_get_parent(dev);

	if (p->es_port < 0 || p->es_port >= sc->numports)
		return (ENXIO);

	if (sc->portphy[p->es_port] != sc->cpuport) {
		mii = rtl830x_miiforport(sc, p->es_port);
		if (mii == NULL)
			return (ENXIO);

		ifp = rtl830x_ifpforport(sc, p->es_port);

		ifm = &mii->mii_media;
		err = ifmedia_ioctl(ifp, &p->es_ifr, ifm, SIOCSIFMEDIA);
	} else {
		return (ENXIO);
	}

	return (err);
}

static int
rtl830x_getvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	struct rtl830x_softc	*sc;
	device_t		 parent;
	int			 data;
	int			 off;

	sc = device_get_softc(dev);
	parent = device_get_parent(dev);

	if (sc->vlan_mode == ETHERSWITCH_VLAN_PORT) {
		off = 0;
		switch(vg->es_vlangroup) {
			case 4:
				off = 3;
			case 2:
				off += 3;
			case 0:
				vg->es_vid = ETHERSWITCH_VID_VALID;
				data = RTL830X_READREG(parent,
				    (1 << 5) | (24 + off));
				data = data & 0xfff;
				vg->es_vid |= data;
				data = RTL830X_READREG(parent,
				    (1 << 5) | (25 + off));
				data = data & 0x1f;
				vg->es_member_ports = data;
				vg->es_untagged_ports = vg->es_member_ports;
				break;
			case 3:
				off = 3;
			case 1:
				vg->es_vid = ETHERSWITCH_VID_VALID;
				data = RTL830X_READREG(parent,
				    (1 << 5) | (25 + off));
				data = data >> 8;
				vg->es_vid |= data;
				data = RTL830X_READREG(parent,
				    (1 << 5) | (26 + off));
				vg->es_vid |= (data & 0xf) << 8;
				vg->es_member_ports = (data >> 8) & 0x1f;
				vg->es_untagged_ports = vg->es_member_ports;
				break;
		}
		vg->es_fid = 0;
	} else {
		vg->es_fid = 0;
	}

	return (0);
}

static int
rtl830x_setvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	struct rtl830x_softc	*sc;
	device_t		 parent;
	int 			 data;
	int 			 reg;
	
	sc = device_get_softc(dev);
	parent = device_get_parent(dev);

	if (sc->vlan_mode == ETHERSWITCH_VLAN_PORT) {
		if (vg->es_vlangroup % 2 == 0) {
			reg = (1 << 5) | (24 + vg->es_vlangroup / 2 * 3);
			RTL830X_WRITEREG(parent, reg, vg->es_vid);
			reg = (1 << 5) | (25 + vg->es_vlangroup / 2 * 3);
			data = RTL830X_READREG(parent, reg);
			RTL830X_WRITEREG(parent, reg,
			    (data & ~0x1f) | vg->es_member_ports);
		} else {
			reg = (1 << 5) | (25 + vg->es_vlangroup / 2 * 3);
			data = RTL830X_READREG(parent, reg);
			data = (data & 0xff) | ((vg->es_vid & 0xff) << 8);
			RTL830X_WRITEREG(parent, reg, data);
			data = (vg->es_vlangroup << 8) | (vg->es_vid >> 8);
			RTL830X_WRITEREG(parent, reg + 1, data);
		}
	}

	return (0);
}

static int
rtl830x_getconf(device_t dev, etherswitch_conf_t *conf)
{
	struct rtl830x_softc *sc;

	sc = device_get_softc(dev);

	/* Return the VLAN mode. */
	conf->cmd = ETHERSWITCH_CONF_VLAN_MODE;
	conf->vlan_mode = sc->vlan_mode;

	return (0);
}

static int
rtl830x_setconf(device_t dev, etherswitch_conf_t *conf)
{
	struct rtl830x_softc	*sc;
	device_t		 parent;
	int 			 data;

	sc = device_get_softc(dev);
	parent = device_get_parent(dev);

	if ((conf->cmd & ETHERSWITCH_CONF_VLAN_MODE) == 0)
		return (0);

	if (conf->vlan_mode == ETHERSWITCH_VLAN_PORT) {
		sc->vlan_mode = ETHERSWITCH_VLAN_PORT;
		data = RTL830X_READREG(parent, (2 << 5) | 17);
		data &= ~SB_DISVLAN;
		RTL830X_WRITEREG(parent, (2 << 5) | 17, data);
	} else {
		sc->vlan_mode = 0;
		data = RTL830X_READREG(parent, (2 << 5) | 17);
		data |= SB_DISVLAN;
		RTL830X_WRITEREG(parent, (2 << 5) | 17, data);
	}

	return (0);
}

static void
rtl830x_statchg(device_t dev)
{

	DPRINTF(dev, "%s\n", __func__);
}

static int
rtl830x_ifmedia_upd(struct ifnet *ifp)
{
	struct rtl830x_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = rtl830x_miiforport(sc, ifp->if_dunit);

	DPRINTF(sc->sc_dev, "%s\n", __func__);
	if (mii == NULL)
		return (ENXIO);
	mii_mediachg(mii);
	return (0);
}

static void
rtl830x_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct rtl830x_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = rtl830x_miiforport(sc, ifp->if_dunit);

	DPRINTF(sc->sc_dev, "%s\n", __func__);

	if (mii == NULL)
		return;
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
}

static int
rtl830x_readphy(device_t dev, int phy, int reg)
{
	struct rtl830x_softc	*sc;
	int			 data;

	sc = device_get_softc(dev);
	RTL830X_LOCK_ASSERT(sc, MA_NOTOWNED);

	if (phy < 0 || phy >= 32)
		return (ENXIO);
	if (reg < 0 || reg >= 32)
		return (ENXIO);

	RTL830X_LOCK(sc);
	data = RTL830X_READREG(device_get_parent(dev),
	    (phy << 5) + reg);
	RTL830X_UNLOCK(sc);

	return (data);
}

static int
rtl830x_writephy(device_t dev, int phy, int reg, int data)
{
	struct rtl830x_softc *sc;
	int err;

	sc = device_get_softc(dev);
	RTL830X_LOCK_ASSERT(sc, MA_NOTOWNED);

	if (phy < 0 || phy >= 32)
		return (ENXIO);
	if (reg < 0 || reg >= 32)
		return (ENXIO);

	RTL830X_LOCK(sc);
	err = RTL830X_WRITEREG(device_get_parent(dev),
	    (phy << 5) + reg, data);
	RTL830X_UNLOCK(sc);

	return (err);
}

static int
rtl830x_readreg(device_t dev, int addr)
{

	return RTL830X_READREG(device_get_parent(dev),  addr);
}

static int
rtl830x_writereg(device_t dev, int addr, int value)
{
	int err;

	err = RTL830X_WRITEREG(device_get_parent(dev), addr, value);
	return (err);
}

static device_method_t rtl830x_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rtl830x_probe),
	DEVMETHOD(device_attach,	rtl830x_attach),
	DEVMETHOD(device_detach,	rtl830x_detach),
	
	/* bus interface */
	DEVMETHOD(bus_add_child,	device_add_child_ordered),
	
	/* MII interface */
	DEVMETHOD(miibus_readreg,	rtl830x_readphy),
	DEVMETHOD(miibus_writereg,	rtl830x_writephy),
	DEVMETHOD(miibus_statchg,	rtl830x_statchg),

	/* MDIO interface */
	DEVMETHOD(mdio_readreg,		rtl830x_readphy),
	DEVMETHOD(mdio_writereg,	rtl830x_writephy),

	/* etherswitch interface */
	DEVMETHOD(etherswitch_lock,	rtl830x_lock),
	DEVMETHOD(etherswitch_unlock,	rtl830x_unlock),
	DEVMETHOD(etherswitch_getinfo,	rtl830x_getinfo),
	DEVMETHOD(etherswitch_readreg,	rtl830x_readreg),
	DEVMETHOD(etherswitch_writereg,	rtl830x_writereg),
	DEVMETHOD(etherswitch_readphyreg,	rtl830x_readphy),
	DEVMETHOD(etherswitch_writephyreg,	rtl830x_writephy),
	DEVMETHOD(etherswitch_getport,	rtl830x_getport),
	DEVMETHOD(etherswitch_setport,	rtl830x_setport),
	DEVMETHOD(etherswitch_getvgroup,	rtl830x_getvgroup),
	DEVMETHOD(etherswitch_setvgroup,	rtl830x_setvgroup),
	DEVMETHOD(etherswitch_setconf,	rtl830x_setconf),
	DEVMETHOD(etherswitch_getconf,	rtl830x_getconf),

	DEVMETHOD_END
};

DEFINE_CLASS_0(rtl830x, rtl830x_driver, rtl830x_methods,
    sizeof(struct rtl830x_softc));
static devclass_t rtl830x_devclass;

DRIVER_MODULE(rtl830x, mdio, rtl830x_driver, rtl830x_devclass, 0, 0);
DRIVER_MODULE(miibus, rtl830x, miibus_driver, miibus_devclass, 0, 0);
DRIVER_MODULE(mdio, rtl830x, mdio_driver, mdio_devclass, 0, 0);
DRIVER_MODULE(etherswitch, rtl830x, etherswitch_driver, etherswitch_devclass,
    0, 0);
MODULE_VERSION(rtl830x, 1);
MODULE_DEPEND(rtl830x, miibus, 1, 1, 1); /* XXX which versions? */
MODULE_DEPEND(rtl830x, etherswitch, 1, 1, 1); /* XXX which versions? */
