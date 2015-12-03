/*-
 * Copyright (c) 2010-2012 Aleksandr Rybalko.
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
#include <dev/switch/rt305x_switchreg.h>
#include <dev/switch/rt305x_switchvar.h>
#include <dev/switch/switch_ioctl.h>

#include "switch_if.h"
#include "switchb_if.h"

/* TODO */
static int find_mac_addr(device_t dev, uint64_t mac);
/* TODO */
static int mac_table_write(device_t dev, uint64_t mac, int idx, 
    uint32_t port_map, uint8_t age, int *hash_idx );

static int	set_port_vid(device_t dev, int port, uint16_t pvid);
static int	get_port_vid(device_t dev, int port, uint16_t *pvid);
static int	set_port_flags(device_t dev, int port, uint32_t flags);
static int	get_port_flags(device_t dev, int port, uint32_t *flags);
static int	set_vid(device_t dev, int idx, uint16_t vid);
static int	get_vid(device_t dev, int idx, uint16_t *vid);
static int	set_vlan_ports(device_t dev, int idx, uint32_t memb);
static int	get_vlan_ports(device_t dev, int idx, uint32_t *memb);

static int	get_port_link(device_t dev, int port);
static int	get_port_speed(device_t dev, int port);
static int	force_port_mode(device_t dev, int port, uint32_t mode);

static int	rt305x_switch_isr(void *arg);

static int
rt305x_switch_probe(device_t dev)
{
	struct child_res_avl *res;

	res = device_get_ivars(dev);

	/* rt305x internal switch require mem region */
	if (res->memres_size < (RT_SW_P5PC - RT_SW_ISR))
		return (ENXIO);

	/* and can serve IRQ */
	if (res->irqs < 1)
		return (ENXIO);

	device_set_desc(dev, "RT305XF internal ethernet switch");
	return (BUS_PROBE_DEFAULT);
}

static int
rt305x_switch_attach(device_t dev)
{
	struct rt305x_switch_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	sc->parent = device_get_parent(dev);

	reg = READ4(sc, RT_SW_MTI);
	if (reg & AT_RAM_TEST_FAIL) {
		device_printf(dev, "Address Table RAM test failed\n");
		return (ENXIO);
	}
	if (reg & LK_RAM_TEST_FAIL) {
		device_printf(dev, "Link RAM test failed\n");
		return (ENXIO);
	}
	if (reg & DT_RAM_TEST_FAIL) {
		device_printf(dev, "Data buffer RAM test failed\n");
		return (ENXIO);
	}

	if (!(reg & SW_RAM_TEST_DONE))
		device_printf(dev, "WARNING: Switch memory test not done\n");
	if (!(reg & AT_RAM_TEST_DONE))
		device_printf(dev,
		    "WARNING: Address Table RAM test not done\n");
	if (!(reg & LK_RAM_TEST_DONE))
		device_printf(dev, "WARNING: Link RAM test not done\n");
	if (!(reg & DT_RAM_TEST_DONE))
		device_printf(dev, "WARNING: Data buffer RAM test not done\n");

	if (!(reg & (SW_RAM_TEST_DONE | AT_RAM_TEST_DONE |
		    LK_RAM_TEST_DONE | DT_RAM_TEST_DONE))) {
		device_printf(dev,
		    "ERROR: All RAM tests not done or not RT305xF"
		    " internal switch\n");
		return (ENXIO);
	}

	sc->caps = malloc(sizeof(struct switch_capability), M_DEVBUF,
	    M_WAITOK | M_ZERO);

	if (!sc->caps)
		return (ENXIO);

	sc->caps->ports = sc->ports = 7;
	sc->vlans = 16;
	sc->sc_dev = dev;
	SWITCHB_REGISTER_ISR(sc->parent, rt305x_switch_isr, dev);

#define S_C(x) SWITCH_CAPS_ ## x
	sc->caps->main = S_C(MAIN_PORT_POWER);
	sc->caps->vlan = S_C(VLAN_GLBL_UNTG) | S_C(VLAN_DOT1Q) |
	    S_C(VLAN_DOUBLE_TAG) |
	    ((sc->vlans << S_C(VLAN_MAX_SHIFT_SHIFT)) & S_C(VLAN_MAX_SHIFT_MASK));
	sc->caps->qos = (2 << S_C(QOS_QUEUES_SHIFT)) & S_C(QOS_QUEUES_MASK);
	sc->caps->lacp = 0; /* No LACP caps */
	sc->caps->stp = 0; /* No STP caps */
	sc->caps->acl = 0x21020304;
	sc->caps->stat = 0x31020304;
#undef S_C

	reg = READ4(sc, RT_SW_IMR);
	reg &= ~(HAS_INTRUDER | PORT_ST_CHG | BC_STORM);
	WRITE4(sc, RT_SW_IMR, reg);

#ifdef RT305X_SWITCH_DEBUG
	int i;
	for (i = 0; i < RT_SW_P5PC; i += 4) {
		printf("%08x%c", READ4(sc, i), ((i+4)%16)?' ':'\n');
	}
#endif

#define RT305X_SWITCH_INIT_OPEN
#ifdef RT305X_SWITCH_INIT_OPEN
	set_port_vid(dev, 0, 1);
	set_port_vid(dev, 1, 1);
	set_port_vid(dev, 2, 1);
	set_port_vid(dev, 3, 1);
	set_port_vid(dev, 4, 1);
	set_port_vid(dev, 5, 1);
	set_port_vid(dev, 6, 1);
	set_port_flags(dev, 0, DOT1Q_VLAN_PORT_FLAG_UNTAGGED|
	    DOT1Q_VLAN_PORT_FLAG_LAN);
	set_port_flags(dev, 1, DOT1Q_VLAN_PORT_FLAG_UNTAGGED|
	    DOT1Q_VLAN_PORT_FLAG_LAN);
	set_port_flags(dev, 2, DOT1Q_VLAN_PORT_FLAG_UNTAGGED|
	    DOT1Q_VLAN_PORT_FLAG_LAN);
	set_port_flags(dev, 3, DOT1Q_VLAN_PORT_FLAG_UNTAGGED|
	    DOT1Q_VLAN_PORT_FLAG_LAN);
	set_port_flags(dev, 4, DOT1Q_VLAN_PORT_FLAG_UNTAGGED|
	    DOT1Q_VLAN_PORT_FLAG_LAN);
	set_port_flags(dev, 5, DOT1Q_VLAN_PORT_FLAG_UNTAGGED|
	    DOT1Q_VLAN_PORT_FLAG_LAN);
	set_port_flags(dev, 6, DOT1Q_VLAN_PORT_FLAG_UNTAGGED|
	    DOT1Q_VLAN_PORT_FLAG_LAN);

	set_vid(dev, 0, 1);
	set_vlan_ports(dev, 0, 0x7f);

#endif

	return (0);
}

static int
rt305x_switch_detach(device_t dev)
{
	struct rt305x_switch_softc *sc;

	sc = device_get_softc(dev);

	if (sc->parent)
		SWITCHB_UNREGISTER_ISR(sc->parent, dev);

	if (sc->caps)
		free(sc->caps, M_DEVBUF);

	return (0);
}

static void
rt305x_switch_wdog(struct rt305x_switch_softc *sc, int wdog)
{

	printf("%s: wdog=%d\n", __func__, wdog);
}

static void
rt305x_switch_intruder_alert(struct rt305x_switch_softc *sc)
{

	printf("%s\n", __func__);
}

static void
rt305x_switch_port_state_change(struct rt305x_switch_softc *sc)
{
	int port;

	printf("%s: Link status:\n", __func__);
	for (port = 0; port < sc->ports; port++)
		printf("%c ",
		    get_port_link(sc->sc_dev, port)?'*':' ');
	printf("\n");
}

static void
rt305x_switch_broadcast_storm(struct rt305x_switch_softc *sc)
{

	printf("%s\n", __func__);
}

static void
rt305x_switch_global_queue_full(struct rt305x_switch_softc *sc)
{

	printf("%s\n", __func__);
}

static void
rt305x_switch_lan_queue_full(struct rt305x_switch_softc *sc, int port)
{

	printf("%s: port=%d\n", __func__, port);
}

static int
rt305x_switch_isr(void *arg)
{
	struct rt305x_switch_softc *sc;
	uint32_t isr;

	sc = (struct rt305x_switch_softc *)arg;

	isr = READ4(sc, RT_SW_ISR);
	if (!isr)
		return (0);

	if (isr & WATCHDOG1_TMR_EXPIRED) {
		/* Handle Watchdog timer 1 */
		rt305x_switch_wdog(sc, 1);
	}

	if (isr & WATCHDOG0_TMR_EXPIRED) {
		/* Handle Watchdog timer 0 */
		rt305x_switch_wdog(sc, 0);
	}

	if (isr & HAS_INTRUDER) {
		/*
		 * Handle detected Intrusion, read bits 0-6 of RT_SW_PTS
		 * to get port.
		 */
		rt305x_switch_intruder_alert(sc);
	}

	if (isr & PORT_ST_CHG) {
		/* Handle port state change */
		rt305x_switch_port_state_change(sc);
	}

	if (isr & BC_STORM) {
		/* Handle Broadcast Storm */
		rt305x_switch_broadcast_storm(sc);
	}

	if (isr & MUST_DROP_LAN) {
		/* Handle  */
		/* XXX: check what is it */
	}

	if (isr & GLOBAL_QUE_FULL) {
		/* Handle global queue full intr */
		rt305x_switch_global_queue_full(sc);
	}

	if (isr & LAN_QUE_FULL6) {
		/* Handle LAN port 6 queue full intr */
		rt305x_switch_lan_queue_full(sc, 6);
	}

	if (isr & LAN_QUE_FULL5) {
		/* Handle LAN port 5 queue full intr */
		rt305x_switch_lan_queue_full(sc, 5);
	}

	if (isr & LAN_QUE_FULL4) {
		/* Handle LAN port 4 queue full intr */
		rt305x_switch_lan_queue_full(sc, 4);
	}

	if (isr & LAN_QUE_FULL3) {
		/* Handle LAN port 3 queue full intr */
		rt305x_switch_lan_queue_full(sc, 3);
	}

	if (isr & LAN_QUE_FULL2) {
		/* Handle LAN port 2 queue full intr */
		rt305x_switch_lan_queue_full(sc, 2);
	}

	if (isr & LAN_QUE_FULL1) {
		/* Handle LAN port 1 queue full intr */
		rt305x_switch_lan_queue_full(sc, 1);
	}

	if (isr & LAN_QUE_FULL0) {
		/* Handle LAN port 0 queue full intr */
		rt305x_switch_lan_queue_full(sc, 0);
	}

	/* Ack interrupts */
	WRITE4(sc, RT_SW_ISR, isr);
	return (FILTER_HANDLED);
}

/*
 * Switch interface methods
 */
static struct switch_capability *
get_caps(device_t dev)
{
	struct rt305x_switch_softc *sc;

	sc = device_get_softc(dev);

	return (sc->caps);
}

static int
find_mac_addr(device_t dev, uint64_t mac)
{
	struct rt305x_switch_softc *sc;
	int idx = -1;
	uint32_t reg;

	sc = device_get_softc(dev);
	reg = READ4(sc, RT_SW_ATS);
#ifdef notyet
	reg = READ4(sc, RT_SW_ATS0);
	reg = READ4(sc, RT_SW_ATS1);
	reg = READ4(sc, RT_SW_ATS2);
#endif

	return (idx);
}

static int
mac_table_write(device_t dev, uint64_t mac, int idx, uint32_t port_map,
    uint8_t age, int *hash_idx )
{
	/* TODO */

	return (0);
}

static int
set_port_vid(device_t dev, int port, uint16_t pvid)
{
	struct rt305x_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;

	if (port > (sc->ports - 1))
		return (EINVAL);

	reg = READ4(sc, RT_SW_PVIDC0 + ((port / 2) * 4));
	reg &= ~(0xfff << (12 * (port % 2)));
	reg |= (0xfff & pvid) << (12 * (port % 2));
	WRITE4(sc, RT_SW_PVIDC0 + ((port / 2) * 4), reg);

	return (0);
}

static int
get_port_vid(device_t dev, int port, uint16_t *pvid)
{
	struct rt305x_switch_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	reg = READ4(sc, RT_SW_PVIDC0 + ((port / 2) * 4));

	*pvid = (reg >> (12 * (port % 2))) & 0xfff;
	return (0);
}

static int
get_port_flags(device_t dev, int port, uint32_t *flags)
{
	struct rt305x_switch_softc *sc;
	uint32_t reg = 0;

	*flags = 0;
	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	reg = READ4(sc, RT_SW_SGC2);
	*flags |= (reg & (1 << port))?DOT1Q_VLAN_PORT_FLAG_DOUBLE_TAG:0;
	*flags |= (reg & (1 << (port+24)))?
	    DOT1Q_VLAN_PORT_FLAG_LAN:DOT1Q_VLAN_PORT_FLAG_WAN;

	reg = READ4(sc, RT_SW_POC2);
	*flags |= (reg & (1 << port))?DOT1Q_VLAN_PORT_FLAG_UNTAGGED:
	    DOT1Q_VLAN_PORT_FLAG_UNTAGGED;

	return (0);
}

static int
set_port_flags(device_t dev, int port, uint32_t flags)
{
	struct rt305x_switch_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	reg = READ4(sc, RT_SW_POC2);
	if (flags & DOT1Q_VLAN_PORT_FLAG_TAGGED) {
		reg &= ~(1 << port);
	} else if (flags & DOT1Q_VLAN_PORT_FLAG_UNTAGGED) {
		reg |= (1 << port);
	}
	WRITE4(sc, RT_SW_POC2, reg);

	/* Q-in-Q */
	reg = READ4(sc, RT_SW_SGC2);
	if (flags & DOT1Q_VLAN_PORT_FLAG_DOUBLE_TAG)
		reg |= (1 << port);
	else
		reg &= ~(1 << port);

	/* LAN/WAN */
	if (flags & DOT1Q_VLAN_PORT_FLAG_LAN)
		reg |= (1 << (port+24));
	/* Reset to WAN port type only if WAN specified */
	else if (flags & DOT1Q_VLAN_PORT_FLAG_WAN)
		reg &= ~(1 << (port+24));

	WRITE4(sc, RT_SW_SGC2, reg);

	return (0);
}

static int
set_vid(device_t dev, int idx, uint16_t vid)
{
	struct rt305x_switch_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (idx > (sc->vlans - 1))
		return (EINVAL);

	reg = READ4(sc, RT_SW_VID0 + idx / 2 * 4);
	reg &= ~(0xfff << (12 * (idx % 2)));
	reg |= (0xfff & vid) << (12 * (idx % 2));
	WRITE4(sc, RT_SW_VID0 + idx / 2 * 4, reg);

	return (0);
}

static int
get_vid(device_t dev, int idx, uint16_t *vid)
{
	struct rt305x_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	reg = READ4(sc, RT_SW_VID0 + idx / 2 * 4);

	*vid = (reg >> (12 * (idx % 2))) & 0xfff;
	return (0);
}

static int
set_vlan_ports(device_t dev, int idx, uint32_t memb)
{
	struct rt305x_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;

	printf("%s: idx=%d, memb=%08x\n", __func__, idx, memb);

	if (idx > (sc->vlans - 1))
		return (EINVAL);
	if (memb & ~((1 << sc->ports) - 1))
		return (EINVAL);

	reg = READ4(sc, RT_SW_VMSC0 + (idx & ~0x03));
	reg &= ~(((1 << sc->ports) - 1) << (8 * (idx % 4)));
	reg |= (memb & ((1 << sc->ports) - 1)) << (8 * (idx % 4));
	WRITE4(sc, RT_SW_VMSC0 + (idx & ~0x03), reg);

	return (0);
}

static int
get_vlan_ports(device_t dev, int idx, uint32_t *memb)
{
	struct rt305x_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	reg = READ4(sc, RT_SW_VMSC0 + (idx & ~0x03));
	reg = (reg >> (8 * (idx % 4))) & ((1 << sc->ports) - 1);

	printf("%s: idx=%d, memb=%08x\n", __func__, idx, reg);
	*memb = reg;
	return (0);
}

static int
get_port_link(device_t dev, int port)
{
	struct rt305x_switch_softc *sc = device_get_softc(dev);
	int link;

	if (port > (sc->ports - 1))
		return (-1);

	link = READ4(sc, RT_SW_POA) >> 25;
	link = (link >> port) & 1;

	return (link);
}

static int
get_port_speed(device_t dev, int port)
{
	struct rt305x_switch_softc *sc = device_get_softc(dev);
	uint32_t link, fdx;

	link = READ4(sc, RT_SW_POA);

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
	struct rt305x_switch_softc *sc = device_get_softc(dev);
	uint32_t reg = READ4(sc, RT_SW_FPA);

	if (port > (sc->ports - 2/* MII */ - 1))
		return (1);

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
		return (1);
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

	WRITE4(sc, RT_SW_FPA, reg);

	return (0);
}

static int
miibus_writereg(device_t dev, int phy, int reg, int value)
{
	struct rt305x_switch_softc *sc;

	sc = device_get_softc(dev);
	WRITE4(sc, RT_SW_PCR1,
	    ((value << PCR1_PHYDATA_SHIFT) & PCR1_PHYDATA_MASK));
	WRITE4(sc, RT_SW_PCR0,
	    PCR0_PHY_WRITE |
	    (phy << PCR0_PHYADDR_SHIFT) |
	    (reg << PCR0_REGADDR_SHIFT));

	if (WAIT4(sc, RT_SW_PCR1, PCR1_PHY_WRITE_DONE, PCR1_PHY_WRITE_DONE,
	    2)) /* 2 ticks */
		return (-1);

	return (0);
}

static int
miibus_readreg(device_t dev, int phy, int reg)
{
	struct rt305x_switch_softc *sc;

	sc = device_get_softc(dev);
	WRITE4(sc, RT_SW_PCR0,
	    PCR0_PHY_READ |
	    (phy << PCR0_PHYADDR_SHIFT) |
	    (reg << PCR0_REGADDR_SHIFT));

	if (WAIT4(sc, RT_SW_PCR1, PCR1_PHY_READ_DONE, PCR1_PHY_READ_DONE,
	    2)) /* 2 ticks */
		return (0xffff);

	return ((READ4(sc, RT_SW_PCR1) >> PCR1_PHYDATA_SHIFT) & 0xffff);
}

static int
get_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct rt305x_switch_softc *sc;

	sc = device_get_softc(dev);
	if (reg & SWITCH_REG_TYPE_RAW)
		*value = READ4(sc, reg & 0xffff);
	else if (reg & SWITCH_REG_TYPE_PHY)
		*value = miibus_readreg(dev, ((reg >> 8) & 0xff),
		    (reg & 0xff));
	else
		*value = 0; /* Raw reg */
	return (0);
}

static int
set_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct rt305x_switch_softc *sc;
	uint32_t old;

	sc = device_get_softc(dev);
	if (reg & SWITCH_REG_TYPE_RAW) {
		old = READ4(sc, reg & 0xffff);
		WRITE4(sc, reg & 0xffff, *value);
	} else if (reg & SWITCH_REG_TYPE_PHY) {
		old = miibus_readreg(dev, ((reg >> 8) & 0xff), (reg & 0xff));
		*value = miibus_writereg(dev, ((reg >> 8) & 0xff),
		    (reg & 0xff), *value);
	} else {
		old = 0; /* Raw reg */
	}

	*value = old;
	return (0);
}

static int
reset_subsys(device_t dev, int subsys)
{
	struct rt305x_switch_softc *sc = device_get_softc(dev);
	int port;

	switch (subsys & SWITCH_RESETSUB_MASK) {
	case SWITCH_RESETSUB_SWITCH:
		WRITE4(sc, RT_SW_STRT, 1);
		break;
	case SWITCH_RESETSUB_PORT:
		if ((subsys & SWITCH_RESETSUB_PORT_MASK) ==
		    SWITCH_RESETSUB_ALLPORTS) {
			/* Reset all PHYs */
#ifdef notyet
			for (port = 0; port < sc->ports; port ++)
				reset_port(sc, port);
#endif
		} else {
			/* Reset syngle PHY */
			port = (subsys & SWITCH_RESETSUB_PORT_MASK) >>
			    SWITCH_RESETSUB_PORT_SHIFT;
#ifdef notyet
			reset_port(sc, port);
#endif
		}
		break;
	case SWITCH_RESETSUB_VLANS:
		/* TODO */
		break;
	case SWITCH_RESETSUB_QOS:
		break;
	}

	return (0);
}

static device_method_t rt305x_switch_methods[] = {
	DEVMETHOD(device_probe,		rt305x_switch_probe),
	DEVMETHOD(device_attach,	rt305x_switch_attach),
	DEVMETHOD(device_detach,	rt305x_switch_detach),

	/* Capability */
	DEVMETHOD(switch_get_caps,	get_caps),
	DEVMETHOD(switch_set_reg,	set_reg),
	DEVMETHOD(switch_get_reg,	get_reg),
	DEVMETHOD(switch_reset_subsys,	reset_subsys),

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

	{0, 0},
};

static driver_t rt305x_switch_driver = {
	"rt305x_switch",
	rt305x_switch_methods,
	sizeof(struct rt305x_switch_softc),
};
static devclass_t rt305x_switch_devclass;

DRIVER_MODULE(rt305x_switch, switch, rt305x_switch_driver,
	      rt305x_switch_devclass, 0, 0);
MODULE_VERSION(rt305x_switch, 1);
MODULE_DEPEND(rt305x_switch, switch, 1, 1, 1);

