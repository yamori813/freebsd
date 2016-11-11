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
#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/mdio/mdio.h>

#include <dev/etherswitch/etherswitch.h>

#include "spibus_if.h"
#include "mdio_if.h"
#include "miibus_if.h"
#include "etherswitch_if.h"

MALLOC_DECLARE(M_KS8995MA);
MALLOC_DEFINE(M_KS8995MA, "ks8995ma", "ks8995ma data structures");

struct ks8995ma_softc {
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

#define KS8995MA_LOCK(_sc)			\
	    mtx_lock(&(_sc)->sc_mtx)
#define KS8995MA_UNLOCK(_sc)			\
	    mtx_unlock(&(_sc)->sc_mtx)
#define KS8995MA_LOCK_ASSERT(_sc, _what)	\
	    mtx_assert(&(_sc)->sc_mtx, (_what))
#define KS8995MA_TRYLOCK(_sc)			\
	    mtx_trylock(&(_sc)->sc_mtx)

#if defined(DEBUG)
#define	DPRINTF(dev, args...) device_printf(dev, args)
#else
#define	DPRINTF(dev, args...)
#endif

static inline int ks8995ma_portforphy(struct ks8995ma_softc *, int);
static void ks8995ma_tick(void *);
static int ks8995ma_ifmedia_upd(struct ifnet *);
static void ks8995ma_ifmedia_sts(struct ifnet *, struct ifmediareq *);


static void
ks8995ma_identify(driver_t *driver, device_t parent)
{
	device_t child;
	struct spibus_ivar *devi;

	if (device_find_child(parent, "ks8995ma", -1) == NULL) {
		child = BUS_ADD_CHILD(parent, 0, "ks8995ma", -1);
		devi = SPIBUS_IVAR(child);
	}
}

#define READ_COMMAND	0x02
#define WRITE_COMMAND	0x03

static int spi_readreg(device_t dev, int addr)
{
	uint8_t txBuf[8], rxBuf[8];
	struct spi_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	memset(txBuf, 0, sizeof(txBuf));
	memset(rxBuf, 0, sizeof(rxBuf));

	txBuf[0] = READ_COMMAND;
	txBuf[1] = addr;
	cmd.tx_cmd = &txBuf;
	cmd.rx_cmd = &rxBuf;

	cmd.tx_cmd_sz = 2;
	cmd.rx_cmd_sz = 1;
	err = SPIBUS_TRANSFER(device_get_parent(dev), dev, &cmd);
	return rxBuf[0];
}

static int spi_writereg(device_t dev, int addr, int data)
{
	uint8_t txBuf[8], rxBuf[8];
	struct spi_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	memset(txBuf, 0, sizeof(txBuf));
	memset(rxBuf, 0, sizeof(rxBuf));

	txBuf[0] = WRITE_COMMAND;
	txBuf[1] = addr;
	txBuf[2] = data;
	cmd.tx_cmd = &txBuf;
	cmd.rx_cmd = &rxBuf;

	cmd.tx_cmd_sz = 3;
	cmd.rx_cmd_sz = 0;
	err = SPIBUS_TRANSFER(device_get_parent(dev), dev, &cmd);
	return rxBuf[0];
}

static int
ks8995ma_probe(device_t dev)
{
	int data;
	struct ks8995ma_softc *sc;

	sc = device_get_softc(dev);
	bzero(sc, sizeof(*sc));

	data = spi_readreg(dev, 0);
	spi_writereg(dev, 0, 0);
	int i;
	for(i = 0; i < 32; ++i) {
	data = MDIO_READREG(device_get_parent(dev), 0x0, i);
	device_printf(dev,"Switch Identifier Register %d %x\n", i, data);
	}
	for(i = 0; i < 32; ++i) {
	data = MDIO_READREG(device_get_parent(dev), i, 0);
	device_printf(dev,"Switch Identifier Register %d %x\n", i, data);
	}

	if((data >> 4) != 0x060) {
		return (ENXIO);
	}

	device_set_desc_copy(dev, "Marvell 88E6060 MDIO switch driver");
	return (BUS_PROBE_DEFAULT);
}

static int
ks8995ma_attach_phys(struct ks8995ma_softc *sc)
{
	int phy, port = 0, err = 0;
	char name[IFNAMSIZ];

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
		sc->miibus[port] = malloc(sizeof(device_t), M_KS8995MA,
		    M_WAITOK | M_ZERO);
		err = mii_attach(sc->sc_dev, sc->miibus[port], sc->ifp[port],
		    ks8995ma_ifmedia_upd, ks8995ma_ifmedia_sts, \
		    BMSR_DEFCAPMASK, phy, MII_OFFSET_ANY, 0);
		DPRINTF(sc->sc_dev, "%s attached to pseudo interface %s\n",
		    device_get_nameunit(*sc->miibus[port]),
		    sc->ifp[port]->if_xname);
		if (err != 0) {
			device_printf(sc->sc_dev,
			    "attaching PHY %d failed\n",
			    phy);
			break;
		}
		++port;
	}
	sc->info.es_nports = port;
	if(sc->cpuport != -1) {
		/* assume cpuport is last one */
		sc->ifpport[sc->cpuport] = port;
		sc->portphy[port] = sc->cpuport;
		++sc->info.es_nports;
	}
	return (err);
}

static int
ks8995ma_attach(device_t dev)
{
	struct ks8995ma_softc *sc;
	int err = 0;

	sc = device_get_softc(dev);

	sc->sc_dev = dev;
	mtx_init(&sc->sc_mtx, "ks8995ma", NULL, MTX_DEF);
	strlcpy(sc->info.es_name, device_get_desc(dev),
	    sizeof(sc->info.es_name));

	/* XXX Defaults */
	sc->numports = 6;
	sc->phymask = 0x1f;
	sc->cpuport = 5;
	sc->media = 100;

	(void) resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "numports", &sc->numports);
	(void) resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "phymask", &sc->phymask);
	(void) resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "cpuport", &sc->cpuport);
	(void) resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "media", &sc->media);

	sc->info.es_nvlangroups = sc->numports;
	sc->info.es_vlan_caps = ETHERSWITCH_VLAN_PORT;

	sc->ifp = malloc(sizeof(struct ifnet *) * sc->numports, M_KS8995MA,
	    M_WAITOK | M_ZERO);
	sc->ifname = malloc(sizeof(char *) * sc->numports, M_KS8995MA,
	    M_WAITOK | M_ZERO);
	sc->miibus = malloc(sizeof(device_t *) * sc->numports, M_KS8995MA,
	    M_WAITOK | M_ZERO);
	sc->portphy = malloc(sizeof(int) * sc->numports, M_KS8995MA,
	    M_WAITOK | M_ZERO);

	/*
	 * Attach the PHYs and complete the bus enumeration.
	 */
	err = ks8995ma_attach_phys(sc);
	if (err != 0)
		return (err);

	bus_generic_probe(dev);
	bus_enumerate_hinted_children(dev);
	err = bus_generic_attach(dev);
	if (err != 0)
		return (err);
	
	callout_init(&sc->callout_tick, 0);

	ks8995ma_tick(sc);
	
	return (err);
}

static int
ks8995ma_detach(device_t dev)
{
	struct ks8995ma_softc *sc = device_get_softc(dev);
	int i, port;

	callout_drain(&sc->callout_tick);

	for (i=0; i < MII_NPHY; i++) {
		if (((1 << i) & sc->phymask) == 0)
			continue;
		port = ks8995ma_portforphy(sc, i);
		if (sc->miibus[port] != NULL)
			device_delete_child(dev, (*sc->miibus[port]));
		if (sc->ifp[port] != NULL)
			if_free(sc->ifp[port]);
		free(sc->ifname[port], M_KS8995MA);
		free(sc->miibus[port], M_KS8995MA);
	}

	free(sc->portphy, M_KS8995MA);
	free(sc->miibus, M_KS8995MA);
	free(sc->ifname, M_KS8995MA);
	free(sc->ifp, M_KS8995MA);

	bus_generic_detach(dev);
	mtx_destroy(&sc->sc_mtx);

	return (0);
}

/*
 * Convert PHY number to port number.
 */
static inline int
ks8995ma_portforphy(struct ks8995ma_softc *sc, int phy)
{

	return (sc->ifpport[phy]);
}

static inline struct mii_data *
ks8995ma_miiforport(struct ks8995ma_softc *sc, int port)
{

	if (port < 0 || port > sc->numports)
		return (NULL);
	if (port == sc->cpuport)
		return (NULL);
	return (device_get_softc(*sc->miibus[port]));
}

static inline struct ifnet *
ks8995ma_ifpforport(struct ks8995ma_softc *sc, int port)
{

	if (port < 0 || port > sc->numports)
		return (NULL);
	return (sc->ifp[port]);
}

/*
 * Poll the status for all PHYs.
 */
static void
ks8995ma_miipollstat(struct ks8995ma_softc *sc)
{
	int i, port;
	struct mii_data *mii;
	struct mii_softc *miisc;

	KS8995MA_LOCK_ASSERT(sc, MA_NOTOWNED);

	for (i = 0; i < MII_NPHY; i++) {
		if (((1 << i) & sc->phymask) == 0)
			continue;
		port = ks8995ma_portforphy(sc, i);
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
ks8995ma_tick(void *arg)
{
	struct ks8995ma_softc *sc = arg;

	ks8995ma_miipollstat(sc);
	callout_reset(&sc->callout_tick, hz, ks8995ma_tick, sc);
}

static void
ks8995ma_lock(device_t dev)
{
	struct ks8995ma_softc *sc = device_get_softc(dev);

	KS8995MA_LOCK_ASSERT(sc, MA_NOTOWNED);
	KS8995MA_LOCK(sc);
}

static void
ks8995ma_unlock(device_t dev)
{
	struct ks8995ma_softc *sc = device_get_softc(dev);

	KS8995MA_LOCK_ASSERT(sc, MA_OWNED);
	KS8995MA_UNLOCK(sc);
}

static etherswitch_info_t *
ks8995ma_getinfo(device_t dev)
{
	struct ks8995ma_softc *sc = device_get_softc(dev);
	
	return (&sc->info);
}

static int
ks8995ma_getport(device_t dev, etherswitch_port_t *p)
{
	struct ks8995ma_softc *sc = device_get_softc(dev);
	struct mii_data *mii;
	struct ifmediareq *ifmr = &p->es_ifmr;
	int err, phy;

	if (p->es_port < 0 || p->es_port >= sc->numports)
		return (ENXIO);
	p->es_pvid = 0;

	phy = sc->portphy[p->es_port];
	mii = ks8995ma_miiforport(sc, p->es_port);
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
ks8995ma_setport(device_t dev, etherswitch_port_t *p)
{
	struct ks8995ma_softc *sc = device_get_softc(dev);
	struct ifmedia *ifm;
	struct mii_data *mii;
	struct ifnet *ifp;
	int err;

	if (p->es_port < 0 || p->es_port >= sc->numports)
		return (ENXIO);

	if (sc->portphy[p->es_port] == sc->cpuport)
		return (ENXIO);

	mii = ks8995ma_miiforport(sc, p->es_port);
	if (mii == NULL)
		return (ENXIO);

	ifp = ks8995ma_ifpforport(sc, p->es_port);

	ifm = &mii->mii_media;
	err = ifmedia_ioctl(ifp, &p->es_ifr, ifm, SIOCSIFMEDIA);
	return (err);
}

static int
ks8995ma_getvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	struct ks8995ma_softc *sc = device_get_softc(dev);
	int data;

	if (sc->vlan_mode == ETHERSWITCH_VLAN_PORT) {
		vg->es_vid = ETHERSWITCH_VID_VALID;
		vg->es_vid |= vg->es_vlangroup;
		data = MDIO_READREG(device_get_parent(dev), 0x18 + vg->es_vlangroup, 6);
		vg->es_member_ports = data & 0x3f;
		vg->es_untagged_ports = vg->es_member_ports;
		vg->es_fid = 0;
	} else {
		vg->es_vid = 0;
	}
	return (0);
}

static int
ks8995ma_setvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	struct ks8995ma_softc *sc = device_get_softc(dev);
	int data;

	if (sc->vlan_mode == ETHERSWITCH_VLAN_PORT) {
		data = MDIO_READREG(device_get_parent(dev), 0x18 + vg->es_vlangroup, 6);
		data &= ~0x3f;
		data |= vg->es_member_ports;
		MDIO_WRITEREG(device_get_parent(dev), 0x18 + vg->es_vlangroup, 6, data);
	} 

	return (0);
}

static void
ks8995ma_reset_vlans(device_t dev)
{
	struct ks8995ma_softc *sc;
	uint32_t ports;
	int i, j;
	int data;

	sc = device_get_softc(dev);

	for (i = 0; i <= sc->numports; i++) {
		ports = 0;
		for (j = 0; j <= sc->numports; j++)
			if(i != j)
				ports |= (1 << j);
		if(sc->vlan_mode == ETHERSWITCH_VLAN_PORT) {
			data = i << 12;
		} else {
			data = 0;
		}
		data |= ports;
		MDIO_WRITEREG(device_get_parent(dev), 0x18 + i, 6, data);
	}
}

static int
ks8995ma_getconf(device_t dev, etherswitch_conf_t *conf)
{
	struct ks8995ma_softc *sc;
	
	sc = device_get_softc(dev);

	/* Return the VLAN mode. */
	conf->cmd = ETHERSWITCH_CONF_VLAN_MODE;
	conf->vlan_mode = sc->vlan_mode;

	return (0);
}

static int
ks8995ma_setconf(device_t dev, etherswitch_conf_t *conf)
{
	struct ks8995ma_softc *sc;

	sc = device_get_softc(dev);

	/* Set the VLAN mode. */
	if (conf->cmd & ETHERSWITCH_CONF_VLAN_MODE) {
		if(conf->vlan_mode == ETHERSWITCH_VLAN_PORT) {
			sc->vlan_mode = ETHERSWITCH_VLAN_PORT;
		} else {
			sc->vlan_mode = 0;
		}

		/* Reset VLANs. */
		ks8995ma_reset_vlans(dev);
	}

	return (0);
}

static void
ks8995ma_statchg(device_t dev)
{

	DPRINTF(dev, "%s\n", __func__);
}

static int
ks8995ma_ifmedia_upd(struct ifnet *ifp)
{
	struct ks8995ma_softc *sc = ifp->if_softc;
	struct mii_data *mii = ks8995ma_miiforport(sc, ifp->if_dunit);

	DPRINTF(sc->sc_dev, "%s\n", __func__);
	if (mii == NULL)
		return (ENXIO);
	mii_mediachg(mii);
	return (0);
}

static void
ks8995ma_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct ks8995ma_softc *sc = ifp->if_softc;
	struct mii_data *mii = ks8995ma_miiforport(sc, ifp->if_dunit);

	DPRINTF(sc->sc_dev, "%s\n", __func__);

	if (mii == NULL)
		return;
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
}

static int
ks8995ma_readphy(device_t dev, int phy, int reg)
{
	struct ks8995ma_softc *sc;
	int data;

	phy += 0x10;

	sc = device_get_softc(dev);
	KS8995MA_LOCK_ASSERT(sc, MA_NOTOWNED);

	if (phy < 0 || phy >= 32)
		return (ENXIO);
	if (reg < 0 || reg >= 32)
		return (ENXIO);

	KS8995MA_LOCK(sc);
	data = MDIO_READREG(device_get_parent(dev), phy, reg);
	KS8995MA_UNLOCK(sc);

	return (data);
}

static int
ks8995ma_writephy(device_t dev, int phy, int reg, int data)
{
	struct ks8995ma_softc *sc;
	int err;

	phy += 0x10;

	sc = device_get_softc(dev);
	KS8995MA_LOCK_ASSERT(sc, MA_NOTOWNED);

	if (phy < 0 || phy >= 32)
		return (ENXIO);
	if (reg < 0 || reg >= 32)
		return (ENXIO);

	KS8995MA_LOCK(sc);
	err = MDIO_WRITEREG(device_get_parent(dev), phy, reg, data);
	KS8995MA_UNLOCK(sc);

	return (err);
}

/* addr is 5-8 bit is SMI Device Addres, 0-4 bit is SMI Register Address */

static int
ks8995ma_readreg(device_t dev, int addr)
{
	int devaddr, regaddr;

	devaddr = (addr >> 5) & 0xf;
	regaddr = addr & 0x1f;

	return MDIO_READREG(device_get_parent(dev), devaddr+0x10, regaddr);
}

/* addr is 5-8 bit is SMI Device Addres, 0-4 bit is SMI Register Address */

static int
ks8995ma_writereg(device_t dev, int addr, int value)
{
	int devaddr, regaddr;

	devaddr = (addr >> 5) & 0xf;
	regaddr = addr & 0x1f;

	return (MDIO_WRITEREG(device_get_parent(dev), devaddr+0x10, regaddr, value));
}

static device_method_t ks8995ma_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	ks8995ma_identify),
	DEVMETHOD(device_probe,		ks8995ma_probe),
	DEVMETHOD(device_attach,	ks8995ma_attach),
	DEVMETHOD(device_detach,	ks8995ma_detach),
	
	/* bus interface */
	DEVMETHOD(bus_add_child,	device_add_child_ordered),
	
	/* MII interface */
	DEVMETHOD(miibus_readreg,	ks8995ma_readphy),
	DEVMETHOD(miibus_writereg,	ks8995ma_writephy),
	DEVMETHOD(miibus_statchg,	ks8995ma_statchg),

	/* MDIO interface */
	DEVMETHOD(mdio_readreg,		ks8995ma_readphy),
	DEVMETHOD(mdio_writereg,	ks8995ma_writephy),

	/* etherswitch interface */
	DEVMETHOD(etherswitch_lock,	ks8995ma_lock),
	DEVMETHOD(etherswitch_unlock,	ks8995ma_unlock),
	DEVMETHOD(etherswitch_getinfo,	ks8995ma_getinfo),
	DEVMETHOD(etherswitch_readreg,	ks8995ma_readreg),
	DEVMETHOD(etherswitch_writereg,	ks8995ma_writereg),
	DEVMETHOD(etherswitch_readphyreg,	ks8995ma_readphy),
	DEVMETHOD(etherswitch_writephyreg,	ks8995ma_writephy),
	DEVMETHOD(etherswitch_getport,	ks8995ma_getport),
	DEVMETHOD(etherswitch_setport,	ks8995ma_setport),
	DEVMETHOD(etherswitch_getvgroup,	ks8995ma_getvgroup),
	DEVMETHOD(etherswitch_setvgroup,	ks8995ma_setvgroup),
	DEVMETHOD(etherswitch_setconf,	ks8995ma_setconf),
	DEVMETHOD(etherswitch_getconf,	ks8995ma_getconf),

	DEVMETHOD_END
};

DEFINE_CLASS_0(ks8995ma, ks8995ma_driver, ks8995ma_methods,
    sizeof(struct ks8995ma_softc));
static devclass_t ks8995ma_devclass;

DRIVER_MODULE(ks8995ma, mdio, ks8995ma_driver, ks8995ma_devclass, 0, 0);
DRIVER_MODULE(miibus, ks8995ma, miibus_driver, miibus_devclass, 0, 0);
DRIVER_MODULE(mdio, ks8995ma, mdio_driver, mdio_devclass, 0, 0);
DRIVER_MODULE(etherswitch, ks8995ma, etherswitch_driver, etherswitch_devclass, 0, 0);
MODULE_VERSION(ks8995ma, 1);
MODULE_DEPEND(ks8995ma, miibus, 1, 1, 1); /* XXX which versions? */
MODULE_DEPEND(ks8995ma, etherswitch, 1, 1, 1); /* XXX which versions? */
