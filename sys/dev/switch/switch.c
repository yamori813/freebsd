/*-
 * Copyright (c) 2010-2012 Aleksandr Rybalko
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
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>

#include <sys/conf.h>
#include <sys/ioccom.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/systm.h>
#include <sys/uio.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/switch/switch_ioctl.h>
#include <dev/switch/switchvar.h>

#include "switch_if.h"
#include "switchb_if.h"

#undef	CREATE_CHILD_MDIO_BUS

#ifdef CREATE_CHILD_MDIO_BUS
#include <sys/socket.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include "miibus_if.h"
#endif

static	d_open_t	switch_open;
static	d_close_t	switch_close;
static	d_ioctl_t	switch_ioctl;

static int	switch_config_sub(struct switch_softc *,
    struct switch_config *);
static int	switch_caps_sub(struct switch_softc *, struct switch_caps *);
static int	switch_getreg_sub(struct switch_softc *, struct switch_reg *);
static int	switch_setreg_sub(struct switch_softc *, struct switch_reg *);
static int	vlan_config_sub(struct switch_softc *, struct vlan_config *);
static int	vlan_vlan_config_sub(struct switch_softc *,
    struct vlan_vlan_config *);

static struct cdevsw switch_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	switch_open,
	.d_close =	switch_close,
	.d_ioctl =	switch_ioctl,
	.d_name =	"switch"
};

static	int
switch_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{

	return (0);
}

static	int
switch_close(struct cdev *dev, int fflags, int devtype, struct thread *td)
{

	return (0);
}


static int
switch_config_sub(struct switch_softc *sc, struct switch_config	*sw)
{

	SWITCH_ASSERT_LOCKED(sc);

	if (sw->version != SWITCH_API_VERSION) {
		return (EINVAL);
	}

	if (sw->cmd == 1) {
		/* Set */
		if (sw->enable != sc->enable)
			sc->enable = sw->enable;
#ifdef notyet
			SWITCHB_ENABLE()
#endif
	} else {
		/* Get */
		sw->enable = sc->enable;
	}

	return (0);
}

static int
switch_caps_sub(struct switch_softc *sc, struct switch_caps *cap)
{

	SWITCH_ASSERT_LOCKED(sc);

	if (cap->version != SWITCH_API_VERSION) {
		return (EINVAL);
	}

	if (cap->cmd == 1) {
		/* Set */
	} else {
		/* Get */
		cap->ports = sc->caps->ports;
		cap->main = sc->caps->main;
		cap->vlan = sc->caps->vlan;
		cap->qos = sc->caps->qos;
		cap->lacp = sc->caps->lacp;
		cap->stp = sc->caps->stp;
		cap->acl = sc->caps->acl;
		cap->stat = sc->caps->stat;
	}
	return (0);
}

static int
switch_getreg_sub(struct switch_softc *sc, struct switch_reg *reg)
{
	int error = 0;

	SWITCH_ASSERT_LOCKED(sc);

	error = SWITCH_GET_REG(sc->child, reg->addr, &reg->value);

	return (error);
}

static int
switch_setreg_sub(struct switch_softc *sc, struct switch_reg *reg)
{
	int error = 0;

	SWITCH_ASSERT_LOCKED(sc);

	error = SWITCH_SET_REG(sc->child, reg->addr, &reg->value);

	return (error);
}

static int
switch_pbvlan_sub(struct switch_softc *sc, struct vlan_vlan_config *vlan)
{
	int error = 0;

	SWITCH_ASSERT_LOCKED(sc);

	if (vlan->cmd == 1) {
		/* Set */
		error = SWITCH_PBVLAN_SETPORTS(sc->child, vlan->index,
		    vlan->d.port.allowed);
	} else {
		/* Get */
		error = SWITCH_PBVLAN_GETPORTS(sc->child, vlan->index,
		    &vlan->d.port.allowed);
	}

	return (error);
}

static int
vlan_config_sub(struct switch_softc *sc, struct vlan_config *vlan_config)
{

	SWITCH_ASSERT_LOCKED(sc);

	if (vlan_config->version != SWITCH_API_VERSION) {
		return (EINVAL);
	}

	if (vlan_config->cmd == 1) {
		/* TODO: Set VLAN mode (portbased/802.1q)*/
	} else {
		/* Get */
	}

	return (0);
}

static int
vlan_vlan_config_sub(struct switch_softc *sc, struct vlan_vlan_config *vlan)
{
	int port, error;
	uint32_t ports, uports;

	SWITCH_ASSERT_LOCKED(sc);

	if (vlan->version != SWITCH_API_VERSION) {
		return (EINVAL);
	}

	switch (vlan->vlan_type) {
	case VLAN_TYPE_PORT:
		error = switch_pbvlan_sub(sc, vlan);
		return (error);
		break;
	case VLAN_TYPE_DOT1Q:
		/* Handled here */
		break;
	case VLAN_TYPE_NONE:
		return (0);
	case VLAN_TYPE_ISL:
	default:
		return (EINVAL);
	}

	if (vlan->cmd == 1) {
		/* Set */
		if (vlan->vlan_type == VLAN_TYPE_DOT1Q) {
			ports = uports = 0;

			for (port = 0; port < sc->caps->ports; port ++) {
				switch (vlan->d.dot1q.port_config[port] &
				    DOT1Q_PORT_VLAN_CONFIG_VLAN_MASK) {
				case DOT1Q_PORT_VLAN_CONFIG_VLAN_NONE:
					/* FALLTHROUGH */
				case DOT1Q_PORT_VLAN_CONFIG_VLAN_FORBIDDEN:
					break;
				case DOT1Q_PORT_VLAN_CONFIG_VLAN_UNTAGGED:
					uports |= (1 << port);
					/* FALLTHROUGH */
				case DOT1Q_PORT_VLAN_CONFIG_VLAN_TAGGED:
					ports |= (1 << port);
					break;
				default:
					device_printf(sc->sc_dev,
					    "Wrong port member type\n");
					return (EINVAL);
				}
			}
			SWITCH_SET_VID(sc->child, vlan->index,
			    vlan->d.dot1q.vid);
			SWITCH_SET_VLANPORTS(sc->child, vlan->index, ports);
			SWITCH_SET_VLANUTPORTS(sc->child, vlan->index, uports);
		}
	} else {
		/* Get */
		if (vlan->vlan_type == VLAN_TYPE_DOT1Q) {
			error = SWITCH_GET_VID(sc->child, vlan->index,
			    &vlan->d.dot1q.vid);
			if (error)
				return (error);

			error = SWITCH_GET_VLANPORTS(sc->child, vlan->index,
			    &ports);
			if (error)
				return (error);
			error = SWITCH_GET_VLANUTPORTS(sc->child, vlan->index,
			    &uports);
			if (error)
				return (error);

			for (port = 0; port < sc->caps->ports; port ++) {
				if (ports & (1 << port)) {
					vlan->d.dot1q.port_config[port] =
					    (uports & (1 << port))?
					    DOT1Q_PORT_VLAN_CONFIG_VLAN_UNTAGGED:
					    DOT1Q_PORT_VLAN_CONFIG_VLAN_TAGGED;
				} else {
					vlan->d.dot1q.port_config[port] =
					    DOT1Q_PORT_VLAN_CONFIG_VLAN_NONE;
				}
			}
		}
	}

	return (0);
}

static int
vlan_port_config_sub(struct switch_softc *sc,
		     struct vlan_port_config *vlan_port)
{
	int error;

	SWITCH_ASSERT_LOCKED(sc);

	if (vlan_port->version != SWITCH_API_VERSION) {
		return (EINVAL);
	}

	if (vlan_port->cmd == 1) {
		/* Set */
		if (vlan_port->vlan_type == VLAN_TYPE_DOT1Q) {
			SWITCH_SET_PVID(sc->child, vlan_port->index,
			    vlan_port->d.dot1q.vid);
			SWITCH_SET_PFLAGS(sc->child, vlan_port->index,
			    vlan_port->d.dot1q.flags);
			printf("%s: SWITCH_SET_PFLAGS(sw, %d, %04x)\n",
			    __func__, vlan_port->index,
			    vlan_port->d.dot1q.flags);
		}
	} else {
		/* Get */
		if (vlan_port->vlan_type == VLAN_TYPE_DOT1Q) {
			error = SWITCH_GET_PVID(sc->child, vlan_port->index,
			    &vlan_port->d.dot1q.vid);
			if (error)
				return (error);
			error = SWITCH_GET_PFLAGS(sc->child, vlan_port->index,
			    &vlan_port->d.dot1q.flags);
			if (error)
				return (error);
		}

	}

	return (0);
}


static int
switch_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct switch_softc *sc = dev->si_drv1;
	int ret = 0;
	void * kdata = data;

	SWITCH_LOCK(sc);
	switch ( cmd ) {
	case IOCTL_SWITCH_CONFIG:
		ret = switch_config_sub(sc, (struct switch_config *)kdata);
		if (ret) {
			device_printf(sc->sc_dev,
			    "switch_config_sub return error\n");
			/* XXX: handle error */
			break;
		}
		break;
	case IOCTL_SWITCH_CAP:
		ret = switch_caps_sub(sc, (struct switch_caps *)kdata);
		if (ret) {
			device_printf(sc->sc_dev,
			    "switch_caps_sub return error\n");
			/* XXX: handle error */
			break;
		}
		break;
	case IOCTL_SWITCH_GETREG:
		ret = switch_getreg_sub(sc, (struct switch_reg *)kdata);
		if (ret) {
			device_printf(sc->sc_dev,
			    "switch_getreg_sub return error\n");
			/* XXX: handle error */
			break;
		}
		break;
	case IOCTL_SWITCH_SETREG:
		ret = switch_setreg_sub(sc, (struct switch_reg *)kdata);
		if (ret) {
			device_printf(sc->sc_dev,
			    "switch_setreg_sub return error\n");
			/* XXX: handle error */
			break;
		}
		break;
	case IOCTL_SWITCH_RESETSUB:
		ret = SWITCH_RESET_SUBSYS(sc->child, *((uint32_t *)kdata));
		if (ret) {
			device_printf(sc->sc_dev,
			    "SWITCH_RESET_SUBSYS return error\n");
			/* XXX: handle error */
			break;
		}
		break;
	case IOCTL_VLAN_CONFIG:
		ret = vlan_config_sub(sc, (struct vlan_config *)kdata);
		if (ret) {
			device_printf(sc->sc_dev,
			    "vlan_config_sub return error\n");
			/* XXX: handle error */
			break;
		}
		break;
	case IOCTL_VLAN_VLAN_CONFIG:
		ret = vlan_vlan_config_sub(sc,
		    (struct vlan_vlan_config *)kdata);
		if (ret) {
			device_printf(sc->sc_dev,
				      "vlan_vlan_config_sub return error\n");
			/* XXX: handle error */
			break;
		}
		break;
	case IOCTL_VLAN_PORT_CONFIG:
		ret = vlan_port_config_sub(sc,
		    (struct vlan_port_config *)kdata);
		if (ret) {
			device_printf(sc->sc_dev,
			    "vlan_port_config_sub return error\n");
			/* XXX: handle error */
			break;
		}
		break;
	/* default case already checked */
	}
	SWITCH_UNLOCK(sc);

	return (ret);
}

#ifdef CREATE_CHILD_MDIO_BUS
int
switch_miibus_writereg(device_t dev, int phy, int reg, int value)
{
	struct switch_softc *sc;

	sc = device_get_softc(dev);
	if (sc || !sc->child)
		return (0);
	return (SWITCH_MIIBUS_WRITEREG(sc->child, phy, reg, value));
}

int
switch_miibus_readreg(device_t dev, int phy, int reg)
{
	struct switch_softc *sc;

	sc = device_get_softc(dev);
	if (sc || !sc->child)
		return (0);
	return (SWITCH_MIIBUS_READREG(sc->child, phy, reg));
}

void
switch_miibus_statchg(device_t dev)
{
	struct switch_softc *sc;
	struct mii_data *mii;

	sc = device_get_softc(dev);
	mii = device_get_softc(sc->child_miibus);

	if ((mii->mii_media_status & (IFM_ACTIVE | IFM_AVALID)) ==
	    (IFM_ACTIVE | IFM_AVALID)) {
		switch (IFM_SUBTYPE(mii->mii_media_active)) {
		case IFM_10_T:
		case IFM_100_TX:
			/* XXX check link here */
			/* sc->flags |= 1; */
			break;
		default:
			break;
		}
	}
}

/*
 * Set media options.
 */
static int
switch_ifmedia_upd(struct ifnet *ifp)
{
#ifdef notyet
	struct rt_softc *sc;
	struct mii_data *mii;
	int error = 0;

	sc = ifp->if_softc;
	RT_SOFTC_LOCK(sc);

	mii = device_get_softc(sc->child_miibus);
	if (mii->mii_instance) {
		struct mii_softc *miisc;
		for (miisc = LIST_FIRST(&mii->mii_phys); miisc != NULL;
				miisc = LIST_NEXT(miisc, mii_list))
			mii_phy_reset(miisc);
	}
	if (mii)
		error = mii_mediachg(mii);
	RT_SOFTC_UNLOCK(sc);

	return (error);
#endif
	return (0);
}

/*
 * Report current media status.
 */
static void
switch_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
#ifdef notyet
	struct rt_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;

	RT_SOFTC_LOCK(sc);
	mii = device_get_softc(sc->child_miibus);
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
	ifmr->ifm_active = IFM_ETHER | IFM_100_TX | IFM_FDX;
	ifmr->ifm_status = IFM_AVALID | IFM_ACTIVE;
	RT_SOFTC_UNLOCK(sc);
#endif
}
#endif /* CREATE_CHILD_MDIO_BUS */
static void switch_tick(void *arg);

int
switch_init(struct switch_softc *sc)
{
	char 	*var;
#ifdef CREATE_CHILD_MDIO_BUS
	int	error;
#endif /* CREATE_CHILD_MDIO_BUS */

	if (sc->child) {
		device_printf(sc->sc_dev, "Only one child allowed\n");
		return (ENXIO);
	}

	sc->args = malloc(sizeof(struct child_res_avl), M_DEVBUF, M_NOWAIT);
	if (sc->args == NULL)
		return (1);

	/* Hint child that we have some memory */
	if (sc->mem_res)
		sc->args->memres_size = rman_get_size(sc->mem_res);
	/* Hint child that we have some IRQs */
	if (sc->irq_res)
		sc->args->irqs = rman_get_size(sc->irq_res);
	/* Hint child that we have some PHY IDs */
	sc->args->phys = sc->phys;

	bus_generic_probe(sc->sc_dev);

	if (!resource_string_value(device_get_name(sc->sc_dev), 
	    device_get_unit(sc->sc_dev),
	    "driver", (const char **)&var)) {
		/* If driver hinted, add this driver */
		sc->child = device_add_child(sc->sc_dev, var, -1);
	} else {
		/* Else try any driver */
		sc->child = device_add_child(sc->sc_dev, NULL, -1);
	}
	/* XXX: if add child filed we still continue XXX */

	if (sc->child == NULL) {
		free(sc->args, M_DEVBUF);
		return (ENXIO);
	}
	device_set_ivars(sc->child, (void *)sc->args);

	device_probe_and_attach(sc->child);

	sc->caps = SWITCH_GET_CAPS(sc->child);

	/* Create device node */
	sc->sc_cdev = make_dev(&switch_cdevsw, 0, UID_ROOT, GID_WHEEL, 0644,
			      "switch%d", device_get_unit(sc->sc_dev));
	sc->sc_cdev->si_drv1 = sc;

#ifdef CREATE_CHILD_MDIO_BUS
	sc->ifp = if_alloc(IFT_OTHER);
	if_initname(sc->ifp, device_get_name(sc->sc_dev),
	    device_get_unit(sc->sc_dev));
	sc->ifp->if_mtu = 0;
	IFQ_SET_MAXLEN(&sc->ifp->if_snd, 0);
	sc->ifp->if_snd.ifq_drv_maxlen = 0;
	IFQ_SET_READY(&sc->ifp->if_snd);

	error = mii_attach(sc->sc_dev, &sc->child_miibus, sc->ifp,
	    switch_ifmedia_upd, switch_ifmedia_sts,
	    BMSR_DEFCAPMASK, MII_PHY_ANY, MII_OFFSET_ANY, 0);
	if (error != 0) {
		device_printf(sc->sc_dev, "attaching PHYs failed\n");
	}
#endif

	callout_init(&sc->tick_callout, 1);
	callout_reset(&sc->tick_callout, hz, switch_tick, sc);

	return (0);
}

static void
switch_tick(void *arg)
{
	struct switch_softc *sc;

	sc = arg;

	SWITCH_TICK(sc->child);
	callout_reset(&sc->tick_callout, hz, switch_tick, sc);
}

int
switch_deinit(struct switch_softc *sc)
{

	/* Destroy device node */
	if (sc->sc_cdev != NULL)
		destroy_dev(sc->sc_cdev);

	callout_drain(&sc->tick_callout);

	return (0);
}

int
switchpub_get_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct switch_softc *sc;
	int error;

	sc = device_get_softc(dev);
	SWITCH_LOCK(sc);
	error = SWITCH_GET_REG(sc->child, reg, value);
	SWITCH_UNLOCK(sc);

	return (0);
}

int
switchpub_set_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct switch_softc *sc;
	int error;

	sc = device_get_softc(dev);
	SWITCH_LOCK(sc);
	error = SWITCH_SET_REG(sc->child, reg, value);
	SWITCH_UNLOCK(sc);

	return (error);
}

