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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <sys/module.h>
#include <sys/bus.h>

#include <machine/bus.h>

#include <net/if.h>
#include <net/if_media.h>

#include <dev/switch/switchvar.h>
#include <dev/switch/rtl830x_switchreg.h>
#include <dev/switch/rtl830x_switchvar.h>
#include <dev/switch/switch_ioctl.h>
#include <dev/mii/mii.h>

#include "switch_if.h"
#include "switchb_if.h"

/* TODO */
static int find_mac_addr(device_t dev, uint64_t mac);
/* TODO */
static int mac_table_write(device_t dev, uint64_t mac, int idx,
    uint32_t port_map, uint8_t age, int *hash_idx );

static int	set_port_vid_idx(struct rtl830x_switch_softc *sc, int port,
		int idx);
static int	get_reg(device_t dev, uint32_t reg, uint32_t *value);
static int	set_reg(device_t dev, uint32_t reg, uint32_t *value);
static int	set_port_vid(device_t dev, int port, uint16_t pvid);
static int	get_port_vid(device_t dev, int port, uint16_t *pvid);
static int	set_vid(device_t dev, int idx, uint16_t vid);
static int	get_vid(device_t dev, int idx, uint16_t *vid);
static int	set_vlan_ports(device_t dev, int idx, uint32_t memb);
static int	get_vlan_ports(device_t dev, int idx, uint32_t *memb);

static int	get_port_link(device_t dev, int port);
static int	get_port_speed(device_t dev, int port);
static int	force_port_mode(device_t dev, int port, uint32_t mode);

static int
rtl830x_switch_probe(device_t dev)
{
	struct child_res_avl *res;

	res = device_get_ivars(dev);

	device_set_desc(dev, "RTL830x ethernet switch");
	return (BUS_PROBE_DEFAULT);
}

static int
rtl830x_switch_attach(device_t dev)
{
	struct rtl830x_switch_softc *sc;
	struct switch_softc *ssc;
	struct mii_softc *miisc;
	uint32_t 	reg;
	int 		i;

	sc = device_get_softc(dev);
	sc->parent = device_get_parent(dev);

	/* 10/100 only, so no 1000baseT */
	ssc = device_get_softc(sc->parent);
	miisc = &ssc->sc_mii;
	miisc->mii_capabilities &= ~BMSR_EXTCAP;
	miisc->mii_extcapabilities = 0;

	sc->caps = malloc(sizeof(struct switch_capability), M_DEVBUF,
	    M_WAITOK | M_ZERO);

	if (!sc->caps)
		return (ENXIO);

	sc->caps->ports = sc->ports = 9;
	sc->vlans = 9;
	sc->sc_dev = dev;

#define S_C(x) SWITCH_CAPS_ ## x
	sc->caps->main = S_C(MAIN_PORT_POWER);
	sc->caps->vlan = S_C(VLAN_DOT1Q) | S_C(VLAN_DOUBLE_TAG) |
	    ((sc->vlans << S_C(VLAN_MAX_SHIFT_SHIFT)) &
	    S_C(VLAN_MAX_SHIFT_MASK));
	sc->caps->qos = (2 << S_C(QOS_QUEUES_SHIFT)) & S_C(QOS_QUEUES_MASK);
	sc->caps->lacp = 0; /* No LACP caps */
	sc->caps->stp = 0; /* No STP caps */
	sc->caps->acl = 0x21020304;
	sc->caps->stat = 0x31020304;
#undef S_C
	reg = READ4(sc, PHY0_CTL);
	device_printf(dev, "PHY0_CTL=%08x\n", reg);
#define DUMP(_reg) device_printf(dev, #_reg "=%08x\n", READ4(sc, _reg))
	DUMP(GCNTRL0);
	DUMP(GCNTRL1);
	DUMP(GCNTRL2);
	DUMP(GCNTRL3);

	DUMP(PORT_PVID(0));
	DUMP(PORT_PVID(1));
	DUMP(PORT_PVID(2));
	DUMP(PORT_PVID(3));
	DUMP(PORT_PVID(4));
	DUMP(PORT_PVID(5));
	DUMP(PORT_PVID(6));
	DUMP(PORT_PVID(7));
	DUMP(PORT8_PVID  );

	DUMP(VLAN_A_MEMB);
	DUMP(VLAN_A_ID);
#undef DUMP

	/* Reset PHYs */
	for (i = 1; i < 8; i++)
		WRITE4(sc, PHY_CTL(i), PHY_CTL_RESET);

	/* Set to Auto-Negotiation and reset Auto-Negotiation process */
	for (i = 1; i < 8; i++)
		WRITE4(sc, PHY_CTL(i), PHY_CTL_ANEG|PHY_CTL_ANEG_RESTART);

	/* Insert tag on ingress packet */
	for (i = 1; i < 8; i++)
		WRITE4(sc, PHY_CTRL0(i), (READ4(sc, PHY_CTRL0(i)) & 0xfffc) |
		    CTRL0_TAG_I_UTG);

	WRITE4(sc, GCNTRL0, 
	    (6 << GCNTRL0_LED_MODE_SHIFT) | /* LED Mode6: Activity,Speed,Link */
	    GCNTRL0_INGRESS_CHECK_DIS |
	    GCNTRL0_TAG_ONLY_DIS |
	    GCNTRL0_TX_FC |
	    GCNTRL0_RX_FC |
	    GCNTRL0_AGN_EN);

	/* set_vid(swdev, Idx, VID) */
	set_vid(dev, 0, 1);
	set_vid(dev, 1, 2);
	set_vid(dev, 2, 3);
	set_vid(dev, 3, 4);
	set_vid(dev, 4, 5);
	set_vid(dev, 5, 6);
	set_vid(dev, 6, 7);
	set_vid(dev, 7, 8);
	set_vid(dev, 8, 9);

	/* All ports are members of VLAN0 (VID1) */
	set_vlan_ports(dev, 0, 0x01ff);
	/* Other VLANs have no members */
	for (i = 1; i < 9; i++)
		set_vlan_ports(dev, i, 0x00);
	/* VLAN1 to all ports */
	for (i = 0; i < 9; i++)
		set_port_vid_idx(sc, i, 0);

	/* Remove isolate flag */
	for (i = 1; i < 8; i++)
		WRITE4(sc, PHY_CTL(i),
		    READ4(sc, PHY_CTL(i)) & ~PHY_CTL_ISOLATE);

	return (0);
}

static int
rtl830x_switch_detach(device_t dev)
{
	struct rtl830x_switch_softc *sc;

	sc = device_get_softc(dev);

	if (sc->caps)
		free(sc->caps, M_DEVBUF);

	return (0);
}

/*
 * Switch capability
 */
static struct switch_capability *
get_caps(device_t dev)
{
	struct rtl830x_switch_softc *sc;

	sc = device_get_softc(dev);

	return (sc->caps);
}

static int
get_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct rtl830x_switch_softc *sc;

	sc = device_get_softc(dev);

	*value = READ4(sc, reg);
	return (0);
}

static int
set_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct rtl830x_switch_softc *sc;
	uint32_t old;

	sc = device_get_softc(dev);
	old = READ4(sc, reg);
	WRITE4(sc, reg, *value);

	*value = old;
	return (0);
}

static int
find_mac_addr(device_t dev, uint64_t mac)
{
	struct rtl830x_switch_softc *sc;
	int idx = -1;

	sc = device_get_softc(dev);

	return (idx);
}

static int
mac_table_write(device_t dev, uint64_t mac, int idx, uint32_t port_map,
    uint8_t age, int *hash_idx )
{

	return (0);
}

static int
add_vlan(struct rtl830x_switch_softc *sc, int vid)
{
	uint32_t reg;
	int i;

	if (vid & ~VLAN_ID_MASK)
		return (-1);

	/* Check VLANs A-H for zero VID */
	for (i = 0; i < 8; i++) {
		reg = READ4(sc, VLAN_ID(i)) & VLAN_ID_MASK;
		if (reg == 0) {
			reg |= (vid << VLAN_ID_SHIFT);
			WRITE4(sc, VLAN_ID(i), reg);
			return (i);
		}
	}

	/* Check VLAN I for zero VID */
	reg = READ4(sc, VLAN_I_ID) & VLAN_ID_MASK;
	if (reg == 0) {
		reg |= (vid << VLAN_ID_SHIFT);
		WRITE4(sc, VLAN_I_ID, reg);
		return (8);
	}

	return (-1);
}

static int
get_vlan_idx(struct rtl830x_switch_softc *sc, int vid)
{
	uint32_t reg;
	int i;

	vid <<= VLAN_ID_SHIFT;

	/* Check VLANs A-H for zero VID */
	for (i = 0; i < 8; i++) {
		reg = READ4(sc, VLAN_ID(i)) & VLAN_ID_MASK;
		if (reg == vid) {
			return (i);
		}
	}

	/* Check VLAN I for zero VID */
	reg = READ4(sc, VLAN_I_ID) & VLAN_ID_MASK;
	if (reg == vid) {
		return (8);
	}

	return (-1);
}

static int
get_vlan_by_index(struct rtl830x_switch_softc *sc, int idx)
{
	uint32_t reg;

	if (idx > (sc->vlans - 1))
		return (-1);

	reg = READ4(sc, ((idx < 8)?VLAN_ID(idx):VLAN_I_ID));
	reg &= VLAN_ID_MASK;

	return (reg);
}

static int
set_port_vid_idx(struct rtl830x_switch_softc *sc, int port, int idx)
{
	uint32_t reg;

	if (port > (sc->ports - 1))
		return (EINVAL);

	/* Fetch PVID index reg */
	reg = READ4(sc, ((port < 8)?PORT_PVID(port):PORT8_PVID));
	/* Clear old PVID index */
	reg &= ~((port < 8)?PORT0_PVID_IDX_MASK:PORT8_PVID_IDX_MASK);

	/* Preapre new PVID index */
	idx <<= ((port < 8)?PORT0_PVID_IDX_SHIFT:PORT8_PVID_IDX_SHIFT);
	idx &= ((port < 8)?PORT0_PVID_IDX_MASK:PORT8_PVID_IDX_MASK);

	/* Add new PVID index */
	reg |= idx;

	/* Save new value */
	WRITE4(sc, ((port < 8)?PORT_PVID(port):PORT8_PVID), reg);

	return (0);
}

static int
set_port_vid(device_t dev, int port, uint16_t pvid)
{
	struct rtl830x_switch_softc *sc;
	int idx;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	/* Get index for VID from "VID Table" */
	idx = get_vlan_idx(sc, pvid);

	/* Add VID to "VID Table" */
	if (idx < 0)
		idx = add_vlan(sc, pvid);

	/* Can't allocate "VID Table" entry */
	if (idx < 0)
		return (EINVAL);

	return (set_port_vid_idx(sc, port, idx));
}

static int
get_port_vid(device_t dev, int port, uint16_t *pvid)
{
	struct rtl830x_switch_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	reg = READ4(sc, ((port < 8)?PORT_PVID(port):PORT8_PVID));
	reg &= ((port < 8)?PORT0_PVID_IDX_MASK:PORT8_PVID_IDX_MASK);
	reg >>= ((port < 8)?PORT0_PVID_IDX_SHIFT:PORT8_PVID_IDX_SHIFT);

	*pvid = get_vlan_by_index(sc, reg);
	return (0);
}

static int
get_port_flags(device_t dev, int port, uint32_t *flags)
{
	struct rtl830x_switch_softc *sc;
	uint32_t reg = 0;

	*flags = 0;
	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	reg = READ4(sc, PHY_CTRL0(port));
	if (reg & CTRL0_I_CH_EN)
		*flags |= DOT1Q_VLAN_PORT_FLAG_INGRESS;

	switch (reg & CTRL0_TAG_MODE_MASK){

	case CTRL0_TAG_NI_NR:
		*flags |= DOT1Q_VLAN_PORT_FLAG_TAGGED;
		break;
	case CTRL0_TAG_I_UTG:
		*flags |= DOT1Q_VLAN_PORT_FLAG_UNTAGGED;
		break;
	case CTRL0_TAG_R_TG:
		*flags |= DOT1Q_VLAN_PORT_FLAG_UNTAGGED|
		    DOT1Q_VLAN_PORT_FLAG_FORCE_UNTAGGED;
		break;
	case CTRL0_TAG_IR:
		*flags |= DOT1Q_VLAN_PORT_FLAG_FORCE_PVID|
		    DOT1Q_VLAN_PORT_FLAG_UNTAGGED;
		break;
	}

	return (0);
}

static int
set_port_flags(device_t dev, int port, uint32_t flags)
{
	struct rtl830x_switch_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	reg = READ4(sc, PHY_CTRL0(port));

	if (flags & DOT1Q_VLAN_PORT_FLAG_INGRESS)
		reg |= CTRL0_I_CH_EN;
	else
		reg &= ~CTRL0_I_CH_EN;

	reg &= ~CTRL0_TAG_MODE_MASK;
	if (flags & DOT1Q_VLAN_PORT_FLAG_TAGGED)
		reg |= CTRL0_TAG_NI_NR;
	else if (flags & DOT1Q_VLAN_PORT_FLAG_UNTAGGED) {
		if (flags & DOT1Q_VLAN_PORT_FLAG_FORCE_UNTAGGED) {
			reg |= CTRL0_TAG_R_TG;
		} else if (flags & DOT1Q_VLAN_PORT_FLAG_FORCE_PVID) {
			reg |= CTRL0_TAG_IR;
		} else {
			reg |= CTRL0_TAG_I_UTG;
		}
	}

	return (0);
}

static int
set_vid(device_t dev, int idx, uint16_t vid)
{
	struct rtl830x_switch_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (idx > (sc->vlans - 1))
		return (EINVAL);

	reg = READ4(sc, ((idx<8)?VLAN_ID(idx):VLAN_I_ID)) & VLAN_ID_MASK;
	reg &= ~VLAN_ID_MASK;
	reg |= vid;
	WRITE4(sc, ((idx<8)?VLAN_ID(idx):VLAN_I_ID), reg);

	return (0);
}

static int
get_vid(device_t dev, int idx, uint16_t *vid)
{
	struct rtl830x_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	reg = READ4(sc, ((idx < 8)?VLAN_ID(idx):VLAN_I_ID));
	reg &= VLAN_ID_MASK;

	*vid = reg;
	return (0);
}

static int
set_vlan_ports(device_t dev, int idx, uint32_t memb)
{
	struct rtl830x_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;

	printf("%s: idx=%d, memb=%08x\n", __func__, idx, memb);

	if (idx > (sc->vlans - 1))
		return (EINVAL);
	if (memb & ~((1 << sc->ports) - 1))
		return (EINVAL);

	reg = READ4(sc, ((idx < 8)?VLAN_MEMB(idx):VLAN_I_MEMB));
	reg &= ~(VLAN_A_MEMB_MASK << VLAN_A_MEMB_SHIFT);
	reg |= ((memb & VLAN_A_MEMB_MASK) << VLAN_A_MEMB_SHIFT);
	WRITE4(sc, ((idx < 8)?VLAN_MEMB(idx):VLAN_I_MEMB), reg);

	return (0);
}

static int
get_vlan_ports(device_t dev, int idx, uint32_t *memb)
{
	struct rtl830x_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	reg = READ4(sc, ((idx < 8)?VLAN_MEMB(idx):VLAN_I_MEMB));
	reg &= VLAN_A_MEMB_MASK;
	reg >>= VLAN_A_MEMB_SHIFT;

	printf("%s: idx=%d, memb=%08x\n", __func__, idx, reg);
	*memb = reg;
	return (0);
}

static int
get_port_link(device_t dev, int port)
{
	struct rtl830x_switch_softc *sc = device_get_softc(dev);
	int link;

	if (port > (sc->ports - 1))
		return (-1);

	link = READ4(sc, PHY0_CTL) >> 25;
	link = (link >> port) & 1;

	return (link);
}

static int
get_port_speed(device_t dev, int port)
{
	struct rtl830x_switch_softc *sc = device_get_softc(dev);
	uint32_t link, fdx;

	link = READ4(sc, PHY0_CTL);

	fdx = (link & (1 << (port + 9)))?IFM_FDX:IFM_HDX;

	if (port < 5)
		return ((((link >> port) & 1)?IFM_100_TX:IFM_10_T) | fdx);

	/* GDMA0 */
	if (port == 5)
		link = (link >> 5) & 0x03;

	/* GDMA1 */
	if (port == 6)
		link = (link >> 7) & 0x03;

	if (link == 2)
		return (IFM_1000_T | fdx);

	if (link == 1)
		return (IFM_100_TX | fdx);

	return (IFM_10_T | fdx);
}

static int
force_port_mode(device_t dev, int port, uint32_t mode)
{
	struct rtl830x_switch_softc *sc = device_get_softc(dev);
	uint32_t reg = READ4(sc, PHY0_CTL);

	if (port > (sc->ports - 2/* MII */ - 1))
		return (EINVAL);

	reg &= ~(((1 << 27) | (1 << 22) | (1 << 16) | (1 << 8) | 1) << port);

	switch (IFM_SUBTYPE(mode)) {
	case IFM_10_T:
		/* Already zero */
		break;
	case IFM_100_TX:
		reg |= (1 << port);
		break;
	default:
		/* error unsupported media */
		return (EINVAL);
	}

	switch (mode & IFM_GMASK) {
	case IFM_FDX:
		reg |= (1 << port) << 8;
		break;
	case IFM_HDX:
		/* Already zero */
		break;
	case IFM_FLOW:
		reg |= (1 << port) << 16;
		break;
	case IFM_FLAG0: /* XXX: Used as enable force mode */
		reg |= (1 << port) << 27;
		break;
	case IFM_FLAG1: /* XXX: Used as force link */
		reg |= (1 << port) << 22;
		break;
	case IFM_FLAG2:
		break;
	case IFM_LOOP:
		break;
	}

	WRITE4(sc, PHY0_CTL, reg);

	return (0);
}

static void
rtl830x_tick(device_t dev)
{
	struct rtl830x_switch_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	reg = READ4(sc, PHY0_CTL);

	return;
}

static device_method_t rtl830x_switch_methods[] = {
	DEVMETHOD(device_probe,		rtl830x_switch_probe),
	DEVMETHOD(device_attach,	rtl830x_switch_attach),
	DEVMETHOD(device_detach,	rtl830x_switch_detach),

	/* Capability */
	DEVMETHOD(switch_get_caps,	get_caps),
	DEVMETHOD(switch_set_reg,	set_reg),
	DEVMETHOD(switch_get_reg,	get_reg),

	/* MAC address table */
	DEVMETHOD(switch_find_mac,	find_mac_addr),
	DEVMETHOD(switch_mac_write,	mac_table_write),

	/* 802.1q */
	DEVMETHOD(switch_set_pvid,	set_port_vid),
	DEVMETHOD(switch_get_pvid,	get_port_vid),
	DEVMETHOD(switch_set_pflags,	set_port_flags),
	DEVMETHOD(switch_get_pflags,	get_port_flags),
	DEVMETHOD(switch_set_vid,	set_vid),
	DEVMETHOD(switch_get_vid,	get_vid),
	DEVMETHOD(switch_set_vlanports,	set_vlan_ports),
	DEVMETHOD(switch_get_vlanports,	get_vlan_ports),

	/* Port state */
	DEVMETHOD(switch_get_portlink,	get_port_link),
	DEVMETHOD(switch_get_portspeed,	get_port_speed),
	DEVMETHOD(switch_force_mode,	force_port_mode),
	DEVMETHOD(switch_tick,		rtl830x_tick),

	{0, 0},
};

static driver_t rtl830x_switch_driver = {
	"rtl830x_switch",
	rtl830x_switch_methods,
	sizeof(struct rtl830x_switch_softc),
};
static devclass_t rtl830x_switch_devclass;

DRIVER_MODULE(rtl830x_switch, switch, rtl830x_switch_driver,
	      rtl830x_switch_devclass, 0, 0);
MODULE_VERSION(rtl830x_switch, 1);
MODULE_DEPEND(rtl830x_switch, switch, 1, 1, 1);

