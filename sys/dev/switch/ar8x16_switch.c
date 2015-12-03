/*-
 * Copyright (c) 2011 Luiz Otavio O Souza.
 * Copyright (c) 2011 Aleksandr Rybalko.
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
#include <dev/switch/switch_ioctl.h>
#include <dev/switch/ar8x16_switchreg.h>
#include <dev/switch/ar8x16_switchvar.h>

#include "switch_if.h"
#include "switchb_if.h"

static int	get_reg(device_t dev, uint32_t reg, uint32_t *value);
static int 	set_reg(device_t dev, uint32_t reg, uint32_t *value);
static int	reset_subsys(device_t dev, int subsys);

static void
ar8x16_set_page(struct ar8x16_switch_softc *sc, int page)
{

	if (sc->curpage != page) {
		MII_WRITE(sc, sc->page_phy, sc->page_reg, page);
		/* Wait for the page switch to propagate */
		DELAY(2000);
		/* Update current page */
		sc->curpage = MII_READ(sc, sc->page_phy, sc->page_reg);
	}
}

static uint32_t
ar8x16_reg_read(struct ar8x16_switch_softc *sc, uint32_t reg)
{
	uint16_t lo, hi;

	ar8x16_set_page(sc, REG_PAGE(reg));

	lo = MII_READ(sc, (0x10 | REG_PHY(reg)), REG_REG(reg));
	hi = MII_READ(sc, (0x10 | REG_PHY(reg)), REG_REG(reg) + 1);

	return ((hi << 16) | lo);
}

static void
ar8x16_reg_write(struct ar8x16_switch_softc *sc, uint32_t reg, uint32_t val)
{

	ar8x16_set_page(sc, REG_PAGE(reg));

	MII_WRITE(sc, (0x10 | REG_PHY(reg)), REG_REG(reg) + 1, val >> 16);
	MII_WRITE(sc, (0x10 | REG_PHY(reg)), REG_REG(reg), val & 0xffff);
}

static int
miibus_writereg(device_t dev, int phy, int reg, int value)
{
	struct ar8x16_switch_softc *sc;

	sc = device_get_softc(dev);
	WRITE4(sc, AR8X16_REG_MDIO_CTRL,
	    AR8X16_MDIO_CTRL_BUSY |
	    AR8X16_MDIO_CTRL_MASTER_EN |
	    AR8X16_MDIO_CTRL_CMD_WRITE |
	    (phy << AR8X16_MDIO_CTRL_PHY_ADDR_SHIFT) |
	    (reg << AR8X16_MDIO_CTRL_REG_ADDR_SHIFT) |
	    (value & AR8X16_MDIO_CTRL_DATA_MASK));

	if (WAIT4(sc, AR8X16_REG_MDIO_CTRL, AR8X16_MDIO_CTRL_BUSY, 0, 1000))
		return (-1);

	return (0);
}

static int
miibus_readreg(device_t dev, int phy, int reg)
{
	struct ar8x16_switch_softc *sc;

	sc = device_get_softc(dev);
	WRITE4(sc, AR8X16_REG_MDIO_CTRL,
	    AR8X16_MDIO_CTRL_BUSY |
	    AR8X16_MDIO_CTRL_MASTER_EN |
	    AR8X16_MDIO_CTRL_CMD_READ |
	    (phy << AR8X16_MDIO_CTRL_PHY_ADDR_SHIFT) |
	    (reg << AR8X16_MDIO_CTRL_REG_ADDR_SHIFT));

	if (WAIT4(sc, AR8X16_REG_MDIO_CTRL, AR8X16_MDIO_CTRL_BUSY, 0, 1000))
		return (0xffff);

	return (READ4(sc, AR8X16_REG_MDIO_CTRL) & 0xffff);
}

static int
get_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct ar8x16_switch_softc *sc;

	sc = device_get_softc(dev);
	if (reg & SWITCH_REG_TYPE_RAW)
		*value = READ4(sc, reg);
	else if (reg & SWITCH_REG_TYPE_PHY)
		*value = miibus_readreg(dev, ((reg >> 8) & 0xff),
		    (reg & 0xff));
	else
		*value = MII_SW_READ4(sc, reg);
	return (0);
}

static int
set_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct ar8x16_switch_softc *sc;
	uint32_t old = 0xffff;
	int error = 0;

	sc = device_get_softc(dev);
	if (reg & SWITCH_REG_TYPE_RAW) {
		old = READ4(sc, reg);
		WRITE4(sc, reg, *value);
	} else if (reg & SWITCH_REG_TYPE_PHY) {
		old = miibus_readreg(dev, ((reg >> 8) & 0xff),
		    (reg & 0xff));
		error = miibus_writereg(dev, ((reg >> 8) & 0xff),
		    (reg & 0xff), *value);
		if (error)
			device_printf(dev, "ERROR: writing reg=%08x\n", reg);
	} else {
		old = MII_SW_READ4(sc, reg);
		MII_SW_WRITE4(sc, reg, *value);
	}

	*value = old;
	return (error);
}

static int
ar8x16_switch_reset(struct ar8x16_switch_softc *sc)
{
	int ret;

	/* Reset the switch. */
	WRITE4(sc, AR8X16_REG_MASK_CTRL, AR8X16_MASK_CTRL_SOFT_RESET);

	ret = WAIT4(sc, AR8X16_REG_MASK_CTRL, AR8X16_MASK_CTRL_SOFT_RESET, 0,
	    1000);
	return ret;
}

#if 0
static int
ar8x16_set_addr(struct ar8x16_switch_softc *sc, uint8_t *mac)
{

	WRITE4(sc, AR8X16_REG_MAC_ADDR0, (mac[4] << 8) | mac[5]);
	WRITE4(sc, AR8X16_REG_MAC_ADDR1, (mac[0] << 24) | (mac[1] << 16) |
	    (mac[2] << 8) | mac[3]);

	return 0;
}
#endif

static int
ar8x16_switch_detect(struct ar8x16_switch_softc *sc, int phy, int reg)
{
	uint32_t rev;

	sc->page_phy = phy;
	sc->page_reg = reg;

	/* Force page switching, to init sc->curpage */
	rev = READ4(sc, AR8X16_REG_PORT_STS(0));
	rev = READ4(sc, AR8X16_REG_MASK_CTRL);

	switch (rev & (AR8X16_MASK_CTRL_VER_MASK|AR8X16_MASK_CTRL_REV_MASK)) {
	case 0x0101:
	case 0x0102:
		sc->devid = 8216;
		sc->revid = (rev & AR8X16_MASK_CTRL_REV_MASK);
		break;
	case 0x0201:
		sc->devid = 8226;
		sc->revid = (rev & AR8X16_MASK_CTRL_REV_MASK);
		break;
	case 0x1001:
		sc->devid = 8316;
		sc->revid = (rev & AR8X16_MASK_CTRL_REV_MASK);
		break;
	default:
		sc->devid = ((rev & AR8X16_MASK_CTRL_VER_MASK) >>
			     AR8X16_MASK_CTRL_VER_SHIFT);
		sc->revid = (rev & AR8X16_MASK_CTRL_REV_MASK);
		break;
	}

	return (rev);
}

static int
ar8x16_switch_probe(device_t dev)
{
	struct ar8x16_switch_softc *sc;
	struct child_res_avl *res;
	char	name[64];

	res = device_get_ivars(dev);

	sc = device_get_softc(dev);
	sc->parent = device_get_parent(dev); /* switchX device */
	sc->sc_dev = dev;

	if (ar8x16_switch_detect(sc, 0x18, 0x00) == 0xffff)
		if (ar8x16_switch_detect(sc, 0x1f, 0x10) == 0xffff) {
			device_printf(dev, "Can't access device regs\n");
			return (ENXIO);
		}

	if (sc->devid == 8216 || sc->devid == 8316) {
		sprintf(name, "AR%d.%d Ethernet switch", sc->devid, sc->revid);
		device_set_desc_copy(dev, name);
		return (BUS_PROBE_SPECIFIC);
	}

	sprintf(name, "Unknown AR8x16 like Ethernet switch (%02x.%02x)",
		sc->devid, sc->revid);
	device_set_desc_copy(dev, name);

	return (BUS_PROBE_DEFAULT);
}

static int
ar8316_init(struct ar8x16_switch_softc *sc)
{
	uint32_t mode;

	mode = READ4(sc, AR8X16_REG_MODE);
	if (sc->sc_mii_mode != mode) {
		device_printf(sc->sc_dev, "Initializing the switch mode.\n");
		WRITE4(sc, AR8X16_REG_MODE, sc->sc_mii_mode);
	}

	/* Standard Atheros magic */
	/* XXX, find what magic in those value,
	 * somewhere I(ray) already seen it, but forget where */
	WRITE4(sc, 0x38, AR8X16_MAGIC);

	return (0);
}

static int
ar8x16_init(device_t dev)
{
	struct ar8x16_switch_softc *sc;
	int port, err;

	sc = device_get_softc(dev);

	resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "mii_mode", &sc->sc_mii_mode);

	/* AR8316 specific init routine */
	if (sc->devid == 8316) {
		/* XXX - Default setting for RSPRO */
		/* XXX - Need to "isolate" of isolate bits :),
		 * seems port1 (first Physical) isolate on set 0x400 bit
		 */
		if (sc->sc_mii_mode == 0)
			sc->sc_mii_mode = AR8X16_MODE_RGMII_PORT4_ISO;
		err = ar8316_init(sc);
		if (err != 0)
			return (err);
	}
	if ((sc->devid == 8216) && (sc->revid == 2)) {
		if (sc->sc_mii_mode == 0)
			sc->sc_mii_mode = AR8X16_MODE_RGMII_PORT4_SWITCH;
		WRITE4(sc, AR8X16_REG_MODE, sc->sc_mii_mode);
	}

	/* Reset the switch */
	ar8x16_switch_reset(sc);

	/* Enable CPU port, and disable mirror port */
	WRITE4(sc, AR8X16_REG_CPU_PORT,
			   AR8X16_CPU_PORT_EN |
			   (15 << AR8X16_MIRROR_PORT_SHIFT));

	/* Setup TAG priority mapping */
	WRITE4(sc, AR8X16_REG_TAG_PRIO, 0xfa50);

	/* Enable ARP frame acknowledge */
	SET4(sc, AR8X16_REG_AT_CTRL, 0, AR8X16_AT_CTRL_ARP_EN);

	/* Enable Broadcast frames transmitted to the CPU */
	SET4(sc, AR8X16_REG_FLOOD_MASK, 0, AR8X16_FLOOD_MASK_BCAST_TO_CPU);

	/* setup MTU */
	SET4(sc, AR8X16_REG_GLOBAL_CTRL, AR8X16_GLOBAL_CTRL_MTU_MASK, 1536);

	/* setup Service TAG */
	SET4(sc, AR8X16_REG_SERVICE_TAG, AR8X16_SERVICE_TAG_MASK, 0);

	/* Port0 - CPU */
	/* XXX: good to know which speed to set */
	WRITE4(sc, AR8X16_REG_PORT_STS(0),
	    /* TODO: deal with interface link speed */
	    /* AR8X16_PORT_STS_SPEED_10	| */
	    /* AR8X16_PORT_STS_SPEED_100	| */
	    AR8X16_PORT_STS_SPEED_1000	|
	    AR8X16_PORT_STS_TXMAC	|
	    AR8X16_PORT_STS_RXMAC	|
	    AR8X16_PORT_STS_TXFLOW	|
	    AR8X16_PORT_STS_RXFLOW	|
	    AR8X16_PORT_STS_DUPLEX);
	WRITE4(sc, AR8X16_REG_PORT_CTRL(0),
	    READ4(sc, AR8X16_REG_PORT_CTRL(0)) & ~AR8X16_PORT_CTRL_HEADER);

	for (port = 1; port < AR8X16_NUM_PORTS; port++) {
		/* Set ports to autoneg */
		WRITE4(sc, AR8X16_REG_PORT_STS(port),
		    AR8X16_PORT_STS_LINK_AUTO);
		WRITE4(sc, AR8X16_REG_PORT_CTRL(port),
		    READ4(sc, AR8X16_REG_PORT_CTRL(port)) &
			~AR8X16_PORT_CTRL_HEADER);
	}
#if 0
	pause("PhyNeg", hz*3);
#endif

	return (0);
}


static int
ar8x16_switch_attach(device_t dev)
{
	struct ar8x16_switch_softc *sc;

	sc = device_get_softc(dev);

	if (ar8x16_init(dev) != 0)
		return (ENXIO);

	sc->ports = 6;
	sc->vlans = 16;
	sc->sc_dev = dev;
	sc->vlan_idx = malloc(sizeof(uint16_t) * sc->vlans, M_DEVBUF,
			      M_WAITOK|M_ZERO);
	if (!sc->vlan_idx)
		return (ENXIO);

	sc->caps = malloc(sizeof(struct switch_capability), M_DEVBUF,
	    M_WAITOK | M_ZERO);

	if (!sc->caps) {
		free(sc->vlan_idx, M_DEVBUF);
		return (ENXIO);
	}

#define S_C(x) SWITCH_CAPS_ ## x
	sc->caps->ports = sc->ports;
	sc->caps->main = S_C(MAIN_PORT_POWER);
	sc->caps->vlan = S_C(VLAN_GLBL_UNTG) | S_C(VLAN_DOT1Q) |
	    S_C(VLAN_DOUBLE_TAG) |
	    ((sc->vlans << S_C(VLAN_MAX_SHIFT_SHIFT)) &
	    S_C(VLAN_MAX_SHIFT_MASK));
	sc->caps->qos = (2 << S_C(QOS_QUEUES_SHIFT)) & S_C(QOS_QUEUES_MASK);
	sc->caps->lacp = 0; /* No LACP caps */
	sc->caps->stp = 0; /* No STP caps */
	sc->caps->acl = 0;
	sc->caps->stat = 0;
#undef S_C

	/*
	 * XXX: if last request in attach will be AR8X16_REG_MASK_CTRL,
	 * then arge1 will have luck to find some PHYs in miibus call.
	 */
	READ4(sc, AR8X16_REG_MASK_CTRL);
	return (0);
}

static int
ar8x16_switch_detach(device_t dev)
{
	struct ar8x16_switch_softc *sc;

	sc = device_get_softc(dev);

#ifdef AR8X16_USE_INTERRUPT
	if (sc->parent)
		SWITCHB_UNREGISTER_ISR(sc->parent, dev);
#endif

	if (sc->caps)
		free(sc->caps, M_DEVBUF);

	if (sc->vlan_idx);
		free(sc->vlan_idx, M_DEVBUF);

	return (0);
}

/*
 * Switch interface methods
 */
static struct switch_capability *
get_caps(device_t dev)
{
	struct ar8x16_switch_softc *sc;

	sc = device_get_softc(dev);

	return (sc->caps);
}


static int
find_mac_addr(device_t dev, uint64_t mac)
{
	struct ar8x16_switch_softc *sc;
	int idx = -1;

	sc = device_get_softc(dev);

	/* TODO */

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
	struct ar8x16_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;

	if (port > (sc->ports - 1))
		return (EINVAL);

	reg = READ4(sc, AR8X16_REG_PORT_VLAN(port));
	reg &= ~0xfff;
	reg |= (0xfff & pvid);
	WRITE4(sc, AR8X16_REG_PORT_VLAN(port), reg);

	return (0);
}

static int
get_port_vid(device_t dev, int port, uint16_t *pvid)
{
	struct ar8x16_switch_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	reg = READ4(sc, AR8X16_REG_PORT_VLAN(port));

	*pvid = reg & 0xfff;
	return (0);
}

static int
get_port_flags(device_t dev, int port, uint32_t *flags)
{
	struct ar8x16_switch_softc *sc;
	uint32_t reg = 0;

	*flags = 0;
	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	reg = READ4(sc, AR8X16_REG_PORT_CTRL(port));
	*flags |= (reg & AR8X16_PORT_CTRL_DOUBLE_TAG)?
	    DOT1Q_VLAN_PORT_FLAG_DOUBLE_TAG:0;

	if (((reg & 0x300) >> 8) == AR8X16_PORT_CTRL_EGRESS_VLAN_MODE_ADD)
		*flags |= DOT1Q_VLAN_PORT_FLAG_TAGGED;
	else if (((reg & 0x300) >> 8) ==
	    AR8X16_PORT_CTRL_EGRESS_VLAN_MODE_STRIP)
		*flags |= DOT1Q_VLAN_PORT_FLAG_UNTAGGED;

	/* Ingress filter */
	reg = READ4(sc, AR8X16_REG_PORT_VLAN(port));
	if ((reg >> AR8X16_PORT_VLAN_MODE_SHIFT) &
	    AR8X16_PORT_VLAN_MODE_SECURE)
		*flags |= DOT1Q_VLAN_PORT_FLAG_INGRESS;

	return (0);
}

static int
set_port_flags(device_t dev, int port, uint32_t flags)
{
	struct ar8x16_switch_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (1);

	/* Q-in-Q */
	reg = READ4(sc, AR8X16_REG_PORT_CTRL(port));
	if (flags & DOT1Q_VLAN_PORT_FLAG_DOUBLE_TAG)
		reg |= AR8X16_PORT_CTRL_DOUBLE_TAG;
	else
		reg &= ~AR8X16_PORT_CTRL_DOUBLE_TAG;

	if (flags & DOT1Q_VLAN_PORT_FLAG_TAGGED) {
		reg &= ~(3 << 8);
		reg |= AR8X16_PORT_CTRL_EGRESS_VLAN_MODE_ADD << 8;
	} else if (flags & DOT1Q_VLAN_PORT_FLAG_UNTAGGED) {
		reg &= ~(3 << 8);
		reg |= AR8X16_PORT_CTRL_EGRESS_VLAN_MODE_STRIP << 8;
	}

	WRITE4(sc, AR8X16_REG_PORT_CTRL(port), reg);

	/* Ingress filter */
	reg = READ4(sc, AR8X16_REG_PORT_VLAN(port));
	reg &= ~AR8X16_PORT_VLAN_MODE_MASK;
	if (flags & DOT1Q_VLAN_PORT_FLAG_INGRESS) {
		reg |= (AR8X16_PORT_VLAN_MODE_SECURE <<
		    AR8X16_PORT_VLAN_MODE_SHIFT);
	}
	WRITE4(sc, AR8X16_REG_PORT_VLAN(port), reg);

 	return (0);
}

static int
set_vid(device_t dev, int idx, uint16_t vid)
{
	struct ar8x16_switch_softc *sc;

	sc = device_get_softc(dev);
	if (idx > (sc->vlans - 1))
		return (1);

	sc->vlan_idx[idx] = VLAN_IDX_VALID | vid;

	return (0);
}

static int
get_vid(device_t dev, int idx, uint16_t *vid)
{
	struct ar8x16_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	reg = sc->vlan_idx[idx];

	if (reg & VLAN_IDX_VALID) {
		*vid = reg & 0xfff;
		return (0);
	} else
		return (ENOENT);
}

static int
set_vlan_ports(device_t dev, int idx, uint32_t memb)
{
	struct ar8x16_switch_softc *sc = device_get_softc(dev);
	uint16_t vlan;
	int error = 0;

	printf("%s: idx=%d, memb=%08x\n", __func__, idx, memb);

	if (idx > (sc->vlans - 1))
		return (EINVAL);
	if (memb & ~((1 << sc->ports) - 1))
		return (EINVAL);

	error = get_vid(dev, idx, &vlan);
	if (error == ENOENT) {
		/* If vid not mapped to vid, try to create it */
		error = set_vid(dev, idx, vlan);
	}
	if (error)
		return (error);

	if (WAIT4(sc, AR8X16_REG_VLAN_CTRL, AR8X16_VLAN_ACTIVE, 0, 5))
		return (EBUSY);

	WRITE4(sc, AR8X16_REG_VLAN_DATA, (memb & AR8X16_VLAN_MEMBER) |
	       AR8X16_VLAN_VALID);

	WRITE4(sc, AR8X16_REG_VLAN_CTRL, (vlan << AR8X16_VLAN_VID_SHIFT) |
	       AR8X16_VLAN_ACTIVE | AR8X16_VLAN_OP_LOAD);

	/* Wait for command done */
	if (WAIT4(sc, AR8X16_REG_VLAN_CTRL, AR8X16_VLAN_ACTIVE, 0, 5))
		return (EBUSY);

	if (READ4(sc, AR8X16_REG_VLAN_CTRL) & AR8X16_VLAN_FULL)
		return (EINVAL);

	return (0);
}

static int
get_vlan_ports(device_t dev, int idx, uint32_t *memb)
{
	struct ar8x16_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;
	uint16_t vlan;
	int error = 0;

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	error = get_vid(dev, idx, &vlan);
	if (error)
		return (error);

	if (WAIT4(sc, AR8X16_REG_VLAN_CTRL, AR8X16_VLAN_ACTIVE, 0, 5))
		return (EBUSY);

	WRITE4(sc, AR8X16_REG_VLAN_CTRL, (vlan << AR8X16_VLAN_VID_SHIFT) |
	       AR8X16_VLAN_ACTIVE | AR8X16_VLAN_OP_GET);

	/* Wait for data */
	if (WAIT4(sc, AR8X16_REG_VLAN_CTRL, AR8X16_VLAN_ACTIVE, 0, 5))
		return (EBUSY);

	reg = READ4(sc, AR8X16_REG_VLAN_DATA);
	if (reg & AR8X16_VLAN_VALID) {
		reg &= ((1 << sc->ports) - 1);
	} else {
		reg = 0;
		error = EINVAL;
	}

	printf("%s: idx=%d, memb=%08x, error=%d\n", __func__, idx, reg,
	    error);
	*memb = reg;
	return (error);
}

static int
reset_subsys(device_t dev, int subsys)
{
	struct ar8x16_switch_softc *sc = device_get_softc(dev);
	int port;

	switch (subsys & SWITCH_RESETSUB_MASK) {
	case SWITCH_RESETSUB_SWITCH:
		ar8x16_switch_reset(sc);
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
		if (WAIT4(sc, AR8X16_REG_VLAN_CTRL, AR8X16_VLAN_ACTIVE, 0, 5))
			return (EBUSY);
		WRITE4(sc, AR8X16_REG_VLAN_CTRL,
		    AR8X16_VLAN_ACTIVE | AR8X16_VLAN_OP_FLUSH);
		break;
	case SWITCH_RESETSUB_QOS:
		break;
	}

	return (0);
}

static device_method_t ar8x16_switch_methods[] = {
	DEVMETHOD(device_probe,		ar8x16_switch_probe),
	DEVMETHOD(device_attach,	ar8x16_switch_attach),
	DEVMETHOD(device_detach,	ar8x16_switch_detach),

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

	/* ports PHY access */
	DEVMETHOD(switch_miibus_readreg,	miibus_readreg),
	DEVMETHOD(switch_miibus_writereg,	miibus_writereg),

	{0, 0},
};

static driver_t ar8x16_switch_driver = {
	"ar8x16_switch",
	ar8x16_switch_methods,
	sizeof(struct ar8x16_switch_softc),
};
static devclass_t ar8x16_switch_devclass;

DRIVER_MODULE(ar8x16_switch, switch, ar8x16_switch_driver, ar8x16_switch_devclass, 0, 0);
MODULE_VERSION(ar8x16_switch, 1);
MODULE_DEPEND(ar8x16_switch, switch, 1, 1, 1);


