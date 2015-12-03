/*-
 * Copyright (c) 2011-2012 Aleksandr Rybalko
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/bus.h>
#include <sys/sysctl.h>

/* Required for struct switch_softc */
#include <machine/bus.h>

#include <net/if.h>
#include <net/if_media.h>

/* MII interface */
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miibus_if.h"
#include "miidevs.h"

/* Switch interface */
#include <dev/switch/switchvar.h>
#include "switch_if.h"
#include "switchb_if.h"
#include "switchpub_if.h"

// copy from dev/mii/mii_physubr.c

enum { 
        MII_MEDIA_NONE = 0,
        MII_MEDIA_10_T,
        MII_MEDIA_10_T_FDX,
        MII_MEDIA_100_T4,
        MII_MEDIA_100_TX,
        MII_MEDIA_100_TX_FDX,
        MII_MEDIA_1000_X,
        MII_MEDIA_1000_X_FDX,
        MII_MEDIA_1000_T,
        MII_MEDIA_1000_T_FDX,
        MII_NMEDIA,
};

/*
 * Float PHY satellite driver, used for devices that have PHY but can't
 * communicate directly to it
 */

static int 	floatphy_probe(device_t dev);
static int 	floatphy_attach(device_t dev);

struct floatphy_softc {
	struct mii_softc miisc;
	device_t	dev;
	device_t	master;
	const char	*master_name;
	int		master_unit;
	uint32_t	master_phys;
	uint32_t	flags;
	int		speed; /* TODO: Maybe better to save media type? */
	uint32_t	debug;
};

static device_method_t floatphy_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe,		floatphy_probe),
	DEVMETHOD(device_attach,	floatphy_attach),
	DEVMETHOD(device_detach,	mii_phy_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	{ 0, 0 }
};

static devclass_t floatphy_devclass;

static driver_t floatphy_driver = {
	"floatphy",
	floatphy_methods,
	sizeof(struct floatphy_softc)
};

DRIVER_MODULE(floatphy, miibus, floatphy_driver, floatphy_devclass, 0, 0);
DRIVER_MODULE(floatphy, mii, floatphy_driver, floatphy_devclass, 0, 0);

static int	floatphy_service(struct mii_softc *, struct mii_data *, int);
static void	floatphy_status(struct mii_softc *);

static const struct mii_phy_funcs floatphy_funcs = {
	floatphy_service,
	floatphy_status,
	mii_phy_reset
};

#define	SWITCH_PHY_REG(_p, _r)	(0x40000000 + ((_p) << 8) + (_r))

#define	FLOATPHY_DEBUG_PHYREG	0x00000001
#define	FLOATPHY_DEBUG_LINK	0x00000002
#define	FLOATPHY_DEBUG_SPEED	0x00000004
#define	FLOATPHY_DEBUG_CALLS	0x00000100

#define	FLOATPHY_FLAGS_LINK_TYPE_MASK		0x00000003
#define		FLOATPHY_FLAGS_LINK_TYPE_ANY	0x00000000
#define		FLOATPHY_FLAGS_LINK_TYPE_ALL	0x00000001
#define	FLOATPHY_FLAGS_LOCK_SPEED		0x00000004

static void
floatphy_sysctl(struct floatphy_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;

	ctx = device_get_sysctl_ctx(sc->dev);
	tree = device_get_sysctl_tree(sc->dev);
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "debug", CTLFLAG_RW, &sc->debug, 0,
	    "enable floatphy debugging");
}

static int
floatphy_probe(device_t dev)
{

	return (BUS_PROBE_NOWILDCARD);
}

static int
floatphy_attach(device_t dev)
{
	struct floatphy_softc *sc;
	struct mii_softc *miisc;
	struct mii_data		*miidata;
	const char *devname;
	int devunit;

	sc = device_get_softc(dev);
	sc->dev = dev;
	miisc = &sc->miisc;
	miidata = device_get_softc(device_get_parent(dev));
	mii_phy_dev_attach(dev, MIIF_NOISOLATE | MIIF_NOMANPAUSE,
	    &floatphy_funcs, 0);

	/*
	 * hint.floatphy.0.master="ar8x16_switch0"
	 * hint.floatphy.0.master_phys=0x0000000f # Sense PHY0-PHY4
	 * hint.floatphy.0.flags=0x00000001
	 * hint.floatphy.0.speed=1000
	 * Flags:
	 *	0x00000000	Notify parent about link up if any of PHYs in
	 *		master_phys mask stay in link.
	 *	0x00000001	Same, but if all in link.
	 *	0x00000004	use hint .speed always, otherwise if master
	 *		is found, use master info.
	 */
	devname = device_get_name(dev);
	devunit = device_get_unit(dev);
	/* Default to switch0 */
	sc->master_name = "switch";
	/* master_unit, master_phys, flags, speed is 0 */
	resource_string_value(devname, devunit, "master", &sc->master_name);
	resource_int_value(devname, devunit, "master_unit", &sc->master_unit);
	resource_int_value(devname, devunit, "master_phys", &sc->master_phys);
	resource_int_value(devname, devunit, "flags", &sc->flags);
	resource_int_value(devname, devunit, "speed", &sc->speed);

	ifmedia_add(&miidata->mii_media,
	    IFM_MAKEWORD(IFM_ETHER, IFM_100_TX, 0, miisc->mii_inst),
	    MII_MEDIA_100_TX, NULL);
	ifmedia_add(&miidata->mii_media,
	    IFM_MAKEWORD(IFM_ETHER, IFM_100_TX, 0, miisc->mii_inst),
	    MII_MEDIA_100_TX_FDX, NULL);
	ifmedia_add(&miidata->mii_media,
	    IFM_MAKEWORD(IFM_ETHER, IFM_1000_T, 0, miisc->mii_inst),
	    MII_MEDIA_1000_T, NULL);
	ifmedia_add(&miidata->mii_media,
	    IFM_MAKEWORD(IFM_ETHER, IFM_1000_T, 0, miisc->mii_inst),
	    MII_MEDIA_1000_T_FDX, NULL);
	ifmedia_add(&miidata->mii_media,
	    IFM_MAKEWORD(IFM_ETHER, IFM_1000_SX, 0, miisc->mii_inst),
	    MII_MEDIA_1000_X, NULL);
	ifmedia_add(&miidata->mii_media,
	    IFM_MAKEWORD(IFM_ETHER, IFM_1000_SX, 0, miisc->mii_inst),
	    MII_MEDIA_1000_X_FDX, NULL);

	if (sc->flags & FLOATPHY_FLAGS_LOCK_SPEED) {
		switch (sc->speed) {
		case 10:
			miisc->mii_capabilities = BMSR_10THDX | BMSR_10TFDX |
			    BMSR_ANEG;
			miisc->mii_extcapabilities = 0;
			break;
		case 100:
			miisc->mii_capabilities = BMSR_100TXHDX |
			    BMSR_100TXFDX | BMSR_ANEG;
			miisc->mii_extcapabilities = 0;
			break;
		case 1000:
			miisc->mii_capabilities = BMSR_EXTCAP | BMSR_ANEG;
			miisc->mii_extcapabilities = EXTSR_1000TFDX |
			    EXTSR_1000THDX | EXTSR_1000XFDX | EXTSR_1000XHDX;
			break;
		default: /* 100 */
			miisc->mii_capabilities = BMSR_100TXHDX |
			    BMSR_100TXFDX | BMSR_EXTCAP | BMSR_ANEG;
			miisc->mii_extcapabilities = 0;
			break;
		}
	} else {
		miisc->mii_capabilities = BMSR_10THDX | BMSR_10TFDX |
		    BMSR_100TXHDX | BMSR_100TXFDX | BMSR_EXTCAP | BMSR_ANEG;
		miisc->mii_extcapabilities = EXTSR_1000TFDX | EXTSR_1000THDX |
		    EXTSR_1000XFDX | EXTSR_1000XHDX;
	}

	floatphy_sysctl(sc);

	device_printf(dev, " ");
	mii_phy_add_media(miisc);
	printf("\n");

	MIIBUS_MEDIAINIT(miisc->mii_dev);
	return (0);
}

static int
floatphy_find_master(struct floatphy_softc *sc)
{
	device_t switchdev;

	if (sc->debug & FLOATPHY_DEBUG_CALLS)
		device_printf(sc->dev, "DEBUG: %s(sc=%p)\n", __func__, sc);

	if (sc->master)
		return (0);

	if (sc->master_name) {
		switchdev = devclass_get_device(
		    devclass_find(sc->master_name), sc->master_unit);
		if (switchdev) {
			device_printf(sc->dev, "found master %s\n",
			    device_get_nameunit(switchdev));
			sc->master = switchdev;
			return (0);
		}
		return (ENOENT);
	}
	return (ENXIO);
}

static int
floatphy_query_link(struct floatphy_softc *sc)
{
	uint32_t reg, status, control, out;
	int error, p;

	/* TODO: Check if we lost master */
	if (sc->debug & FLOATPHY_DEBUG_CALLS)
		device_printf(sc->dev, "DEBUG: %s(sc=%p)\n", __func__, sc);

	out = 0;
	/* If no master, always return linked */
	if (floatphy_find_master(sc) != 0)
		return (1);

	/* TODO: Ask PHY driver and return here */
	for (p = 0; p < 32; p ++) {

		if (!(sc->master_phys & (1 << p)))
			/* Skip PHY's which not in mask */
			continue;

		reg = SWITCH_PHY_REG(p, MII_BMSR);
		error = SWITCHPUB_GET_REG(sc->master, reg, &status);
		if (error) {
			return (-1);
		}
		/* TODO: handle error */
		if ((status & 0xffff) == 0xffff)
			/* Isolated or NC PHY */
			continue;

		if (sc->debug & FLOATPHY_DEBUG_PHYREG)
			device_printf(sc->dev, "DEBUG: PHY(%d) BMSR=%04x\n",
			    p, status);

		/* Update speed only if linked */
		/* TODO: better on linkup event, to reduce MDIO access */
		/* TODO: move to separate sub */
		if (((sc->flags & FLOATPHY_FLAGS_LOCK_SPEED) == 0) &&
		    (status & BMSR_LINK)) {
			/*
			 * Speed setting not locked and port linked, so we can
			 * update speed.
			 */
			reg = SWITCH_PHY_REG(p, MII_BMCR);
			error = SWITCHPUB_GET_REG(sc->master, reg, &control);
			if (error) {
				return (-1);
			}
			if (sc->debug & FLOATPHY_DEBUG_PHYREG)
				device_printf(sc->dev,
				    "DEBUG: PHY(%d) BMCR=%04x\n", p, control);

			switch (control & (BMCR_S100 | BMCR_S1000)) {
			case 0:
				sc->speed = 10;
				break;
			case BMCR_S100:
				sc->speed = 100;
				break;
			case BMCR_S1000:
				sc->speed = 1000;
				break;
			case (BMCR_S100|BMCR_S1000):
				/* Here is we catch 0xffff */
				break;
			}

			if (sc->debug & FLOATPHY_DEBUG_SPEED)
				device_printf(sc->dev,
				    "DEBUG: PHY(%d) speed=%d\n",
				    p, sc->speed);
		}
		switch (sc->flags & FLOATPHY_FLAGS_LINK_TYPE_MASK) {
		case FLOATPHY_FLAGS_LINK_TYPE_ANY:
			if (status & BMSR_LINK) {
				/* First linked sutisfy */
				if (sc->debug & FLOATPHY_DEBUG_LINK)
					device_printf(sc->dev, 
					    "DEBUG: PHY(%d) linkup\n", p);
				return (1);
			}
			break;
		case FLOATPHY_FLAGS_LINK_TYPE_ALL:
			out &= (status & BMSR_LINK);
			break;
		}
	}

	if (((sc->flags & FLOATPHY_FLAGS_LINK_TYPE_MASK) ==
	    FLOATPHY_FLAGS_LINK_TYPE_ALL) && out) {
		if (sc->debug & FLOATPHY_DEBUG_LINK)
			device_printf(sc->dev, "DEBUG: ALL(0x%08x) linkup\n",
			    sc->master_phys);
		return (1);
	}

	return (0);
}

static uint32_t
floatphy_speed_to_media(int speed, int fdx)
{

	switch (speed) {
	case 1000:
		return (IFM_1000_T | fdx);
		break;
	case 100:
		return (IFM_100_TX | fdx);
		break;
	case 10:
		return (IFM_10_T | fdx);
		break;
	default:
		return (IFM_100_TX | fdx);
		break;
	}
}

static int
floatphy_service(struct mii_softc *miisc, struct mii_data *mii, int cmd)
{
	struct ifmedia_entry *ife;
	struct floatphy_softc *sc;
	uint32_t media;
	int link;

	ife = mii->mii_media.ifm_cur;
	sc = (struct floatphy_softc *)miisc;

	if (sc->debug & FLOATPHY_DEBUG_CALLS)
		device_printf(sc->dev, "DEBUG: %s(sc=%p)\n", __func__, sc);

	switch (cmd) {
	case MII_POLLSTAT:
		link = floatphy_query_link(sc);
		if (link == -1) {
			/* Error detected, no update */
			return (0);
		}
		media = floatphy_speed_to_media(sc->speed, IFM_FDX);
		mii->mii_media_status = IFM_AVALID |
		    (link == 1) ? IFM_ACTIVE : 0;
		mii->mii_media_active = IFM_ETHER | media;
		break;
	case MII_MEDIACHG:
		link = floatphy_query_link(sc);
		if (link == -1) {
			/* Error detected, no update */
			return (0);
		}
		link = (link == 1) ? IFM_ACTIVE : 0;
		/* XXX: get FDX link status */
		media = floatphy_speed_to_media(sc->speed, IFM_FDX);

		switch (IFM_SUBTYPE(ife->ifm_media)) {
		case IFM_NONE:
			mii->mii_media_status = IFM_AVALID;
			mii->mii_media_active = IFM_ETHER | media;
			break;
		case IFM_AUTO:
		default:
			mii->mii_media_status = IFM_AVALID | link;
			mii->mii_media_active = IFM_ETHER | media;
			break;
		}
		break;
	case MII_TICK:
		if (mii_phy_tick(miisc) == EJUSTRETURN)
			return (0);
		break;
	}

	/* Update the media status. */
	PHY_STATUS(miisc);

	/* Callback if something changed. */
	mii_phy_update(miisc, cmd);
	return (0);
}

static void
floatphy_status(struct mii_softc *miisc)
{
	struct floatphy_softc *sc;
	struct ifmedia_entry *ife;
	struct mii_data *mii;
	uint32_t media;
	int link;

	sc = (struct floatphy_softc *)miisc;
	mii = miisc->mii_pdata;
	ife = mii->mii_media.ifm_cur;

	if (sc->debug & FLOATPHY_DEBUG_CALLS)
		device_printf(sc->dev, "DEBUG: %s(sc=%p)\n", __func__, sc);

	link = floatphy_query_link(sc);
	if (link == -1) {
		/* Error detected, no update */
		return;
	}
	link = (link == 1) ? IFM_ACTIVE : 0;
	media = floatphy_speed_to_media(sc->speed, IFM_FDX);

	mii->mii_media_status = IFM_AVALID | link;
	mii->mii_media_active = IFM_ETHER | media;
}
