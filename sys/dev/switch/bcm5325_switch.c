/*-
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
#include <dev/switch/bcm5325_switchvar.h>
#include <dev/switch/bcm5325_switchreg.h>
#include <dev/switch/switch_ioctl.h>

#include "switch_if.h"
#include "switchb_if.h"

static int	bcm5325_switch_probe(device_t dev);
static int	bcm5325_switch_attach(device_t dev);
static int	bcm5325_switch_detach(device_t dev);

/* TODO */
static int find_mac_addr(device_t dev, uint64_t mac);
/* TODO */
static int mac_table_write(device_t dev, uint64_t mac, int idx,
    uint32_t port_map, uint8_t age, int *hash_idx );

static int	get_reg(device_t dev, uint32_t reg, uint32_t *value);
static int 	set_reg(device_t dev, uint32_t reg, uint32_t *value);
static int	set_port_vid(device_t dev, int port, uint16_t pvid);
static int	get_port_vid(device_t dev, int port, uint16_t *pvid);
static int	set_vid(device_t dev, int idx, uint16_t vid);
static int	get_vid(device_t dev, int idx, uint16_t *vid);
static int	set_vlan_ports(device_t dev, int idx, uint32_t memb);
static int	get_vlan_ports(device_t dev, int idx, uint32_t *memb);
static int	set_vlan_untagged_ports(device_t dev, int idx, uint32_t memb);
static int	get_vlan_untagged_ports(device_t dev, int idx, uint32_t *memb);

static int	bcm5325_write(struct bcm5325_switch_softc *sc, uint32_t reg,
		    uint64_t val);
static int	bcm5325_read(struct bcm5325_switch_softc *sc, uint32_t reg,
		    uint64_t *val);

static int	bcm5325_vlan_write(struct bcm5325_switch_softc *sc, uint16_t vid,
		    uint32_t tports, uint32_t uports);
static int	bcm5325_vlan_read(struct bcm5325_switch_softc *sc, uint16_t vid,
		    uint32_t *tports, uint32_t *uports);
static int	bcm5395_vlan_write(struct bcm5325_switch_softc *sc, uint16_t vid,
		    uint32_t tports, uint32_t uports);
static int	bcm5395_vlan_read(struct bcm5325_switch_softc *sc, uint16_t vid,
		    uint32_t *tports, uint32_t *uports);

#define	PSPHY_WRITE(_sc, _reg, _val)			\
	    MII_SW_WRITE4((_sc), ((PSEUDOPHY_ADDR << 8) | (_reg)), (_val))
#define	PSPHY_READ(_sc, _reg)				\
	    MII_SW_READ4((_sc), ((PSEUDOPHY_ADDR << 8) | (_reg)))

#define	WRITE4(_sc, _reg, _val)			\
	    MII_SW_WRITE4((_sc), ((PSEUDOPHY_ADDR << 8) | (_reg)), (_val))

#define	WRITE(_sc, _reg, _val)	bcm5325_write((_sc), (_reg), (_val))
#define	READ(_sc, _reg, _val)	bcm5325_read((_sc), (_reg), (_val))

static uint32_t
READ4(struct bcm5325_switch_softc *sc, uint32_t reg)
{
	uint64_t val;
	int error;

	error = bcm5325_read(sc, reg, &val);

	if (error)
		return (0xffffffff);

	return (val & 0xffffffff);
}

static int
bcm5325_write(struct bcm5325_switch_softc *sc, uint32_t reg, uint64_t val)
{
	int i;
	uint8_t len, page;

	len = (reg & 0x00ff0000) >> 16;
	page = (reg & 0x0000ff00) >> 8;
	reg &= 0x000000ff;

	PSPHY_WRITE(sc, BCM5325_ACCESS_CONTROL_REG,
		  (((page << ACCESS_CONTROL_PAGE_SHIFT) &
		    ACCESS_CONTROL_PAGE_MASK) | ACCESS_CONTROL_RW));

	for (i = 0; i < len; i+=2){
		PSPHY_WRITE(sc, BCM5325_DATA_15_0_REG + (i/2),
			  (val >> (8*i)) & 0xffff);
	}

	PSPHY_WRITE(sc, BCM5325_RW_CONTROL_REG,
		  ((reg << RW_CONTROL_ADDR_SHIFT) & RW_CONTROL_ADDR_MASK) |
		  RW_CONTROL_WRITE);

	/* is operation finished? */
	for (i = BCM5325_OP_RETRY; i > 0; i --) {
		if ((PSPHY_READ(sc, BCM5325_RW_CONTROL_REG) &
		     RW_CONTROL_OP_MASK) == RW_CONTROL_NOP)
			break;
	}

	/* timed out */
	if (!i) {
		printf("mii_wreg: timeout\n");
		return (EBUSY);
	}

	i = PSPHY_READ(sc, BCM5325_RW_STATUS_REG);
	if (i & 0x0003)
		printf("XXX: reg=%08x BCM5325_RW_STATUS_REG=%d\n", reg, i);

	return (0);
}

static int
bcm5325_read(struct bcm5325_switch_softc *sc, uint32_t reg, uint64_t *val)
{
	int i;
	uint8_t len, page;

	*val = 0;
	len = (reg & 0x00ff0000) >> 16;
	page = (reg & 0x0000ff00) >> 8;
	reg &= 0x000000ff;

	PSPHY_WRITE(sc, BCM5325_ACCESS_CONTROL_REG,
		  (((page << ACCESS_CONTROL_PAGE_SHIFT) &
		    ACCESS_CONTROL_PAGE_MASK) | ACCESS_CONTROL_RW));

	PSPHY_WRITE(sc, BCM5325_RW_CONTROL_REG,
		  ((reg << RW_CONTROL_ADDR_SHIFT) & RW_CONTROL_ADDR_MASK) |
		  RW_CONTROL_READ);

	/* is operation finished? */
	for (i = BCM5325_OP_RETRY; i > 0; i --) {
		if ((PSPHY_READ(sc, BCM5325_RW_CONTROL_REG) &
		     RW_CONTROL_OP_MASK) == RW_CONTROL_NOP)
			break;
	}
	/* timed out */
	if (!i) {
		return (EBUSY);
	}

	for (i = 0; i < len; i+=2){
		*val |= PSPHY_READ(sc, BCM5325_DATA_15_0_REG + (i/2)) << (8*i);
	}

	i = PSPHY_READ(sc, BCM5325_RW_STATUS_REG);
	if (i & 0x0003)
		printf("XXX: reg=%08x BCM5325_RW_STATUS_REG=%d\n", reg, i);

	return (0);
}

static int
bcm5325_switch_probe(device_t dev)
{
	struct child_res_avl *res;

	res = device_get_ivars(dev);

	/* XXX: bcm5325 show at leaset 5 PHYs */
	if (!res->phys)
		return (ENXIO);

	device_set_desc(dev, "BCM5325 family ethernet switch");
	return (BUS_PROBE_DEFAULT);
}

static int
bcm5325_switch_attach(device_t dev)
{
	struct bcm5325_switch_softc *sc;
	struct mii_softc	*miisc;
	struct switch_softc	*ssc;
	uint64_t reg;
	uint64_t reg32;

	sc = device_get_softc(dev);
	sc->parent = device_get_parent(dev);
	/* Use BCM5354 mode by default */
	/* XXX: about 5354 we can decide at robo switch core on SSB */
	sc->devid = 0x5325;

	reg = 0;
	READ(sc, SWITCH_DEVICEID, &reg);
	reg32 = reg & 0xfffff;
	if (reg32) {
#if 1
		/* At least BCM53115 return 0x53115 if query in 32bit mode */
		sc->devid = reg32;
#else
		/* XXX: maybe wrong, based on known chips */
		if (reg32 < 0x100)
			sc->devid = reg32 + 0x5300;
		else
			sc->devid = reg32 + 0x50000;
#endif
	}

	resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "devid", &sc->devid);
	device_printf(dev, "\t%switch model is BCM%x\n",
	    ((resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "devid", &sc->devid))?"S":"Hinted s"), sc->devid);

	sc->caps = malloc(sizeof(struct switch_capability), M_DEVBUF,
	    M_WAITOK | M_ZERO);

	if (!sc->caps)
		return (ENXIO);

	switch (sc->devid) {
	case 0x5395:
	case 0x53115:
	case 0x53118:
		sc->vlan_write = &bcm5395_vlan_write;
		sc->vlan_read = &bcm5395_vlan_read;
		sc->vlans = 4096;
		break;
	case 0x5325:
	case 0x5352:
	case 0x5354:
	default:
		sc->vlan_write = &bcm5325_vlan_write;
		sc->vlan_read = &bcm5325_vlan_read;
		sc->vlans = 16;
		break;
	}

	switch (sc->devid) {
	case 0x53118:
	case 0x53115:	/* 53118 w/o ports 6-7 */
	case 0x5325:	/* MII port is port8 */
	case 0x5352:
	case 0x5354:
	case 0x5395:
		sc->caps->ports = sc->ports = 9;
		break;
	case 0x5380:
		/* 8 - 10/100, 2 - 10/100/1000 */
		sc->caps->ports = sc->ports = 10;
		break;
	default:
		/* XXX: trick, last digit of id + 1 MII port */
		sc->caps->ports = sc->ports = (sc->devid % 0xf) + 1;
		break;
	}

	switch (sc->devid) {
	/* XXX Incomplete list of 1000base.* switches */
	case 0x53118:
	case 0x53115:
		break;
	default:
		ssc = device_get_softc(device_get_parent(dev));
		miisc = &ssc->sc_mii;
		/* Remove 1000base.* capabilities for 10/100 switches */
#define	BMSR_EXTCAP	0x0001	/* Extended capability */
		miisc->mii_capabilities &= ~BMSR_EXTCAP;
		miisc->mii_extcapabilities = 0;
		break;
	}
	sc->sc_dev = dev;

#define S_C(x) SWITCH_CAPS_ ## x
	sc->caps->main = S_C(MAIN_PORT_POWER);
	sc->caps->vlan = S_C(VLAN_DOT1Q) |
	    ((sc->vlans << S_C(VLAN_MAX_SHIFT_SHIFT)) &
		S_C(VLAN_MAX_SHIFT_MASK));
	sc->caps->qos = (2 << S_C(QOS_QUEUES_SHIFT)) & S_C(QOS_QUEUES_MASK);
	sc->caps->lacp = 0; /* No LACP caps */
	sc->caps->stp = 0; /* No STP caps */
	sc->caps->acl = 0;
	sc->caps->stat = 0;
#undef S_C

#define DUMP(_reg) device_printf(dev, #_reg "=%08x\n", READ4(sc, _reg))
	DUMP(PORT_CTL(PORT0));
	DUMP(PORT_CTL(PORT1));
	DUMP(PORT_CTL(PORT2));
	DUMP(PORT_CTL(PORT3));
	DUMP(PORT_CTL(PORT4));
	DUMP(PORT_CTL(PORT5));
	DUMP(PORT_CTL(PORT6));
	DUMP(PORT_CTL(PORT7));
	DUMP(PORT_CTL(PORTMII));

	DUMP(PORTMII_STATUS_OVERRIDE);
	DUMP(SWITCH_DEVICEID);
	DUMP(BIST_STATUS_RC);
	DUMP(VLAN_GLOBAL_CTL0);
	DUMP(VLAN_GLOBAL_CTL1);
	DUMP(VLAN_GLOBAL_CTL2);
	DUMP(VLAN_DROP_UNTAGGED);
	DUMP(VLAN_GLOBAL_CTL4);
	DUMP(VLAN_GLOBAL_CTL5);
#undef DUMP

	/* MII port state override (page 0 register 14) */
	READ(sc, PORTMII_STATUS_OVERRIDE , &reg);

	/* Bit 4 enables reverse MII mode */
	if (!(reg & PORTMII_STATUS_REVERSE_MII))
	{
		/* Enable RvMII */
		reg |= PORTMII_STATUS_REVERSE_MII;
		WRITE(sc, PORTMII_STATUS_OVERRIDE, reg);
		/* Read back */
		READ(sc, PORTMII_STATUS_OVERRIDE, &reg);
		if (!(reg & PORTMII_STATUS_REVERSE_MII))
		{
			device_printf(dev, "Unable to set RvMII mode\n");
			bcm5325_switch_detach(dev);
			return (ENXIO);
		}
	}

	/*
	 * XXX: We need prefetch existing sc->base_vlan here.
	 */
	/*
	 * XXX: Avoid default configuration, bootloader must set it or we
	 * must load user defined
	 */

	return (0);
}

static int
bcm5325_switch_detach(device_t dev)
{
	struct bcm5325_switch_softc *sc;

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
	struct bcm5325_switch_softc *sc;

	sc = device_get_softc(dev);

	return (sc->caps);
}

/*
 * Variable holding upper 32bits of get_reg/set_reg requests.
 * Accessible with special reg address 0x0fffffff
 */
static uint32_t get_set_upper32 = 0;

static int
get_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct bcm5325_switch_softc *sc;
	uint64_t val64;
	int error = 0;

	sc = device_get_softc(dev);

	switch (reg & SWITCH_REG_TYPE_MASK) {
	case SWITCH_REG_TYPE_PHY: /* Same in BCM53xx case */
	case SWITCH_REG_TYPE_RAW:
		*value = MII_SW_READ4(sc, reg);
		return (0);
	}

	if (reg == 0x0ffffffful) {
		*value = get_set_upper32;
	} else if (reg == 0x0ffffffeul) {
		*value = sc->base_vlan;
	} else {
		error = READ(sc, reg, &val64);
		if (((reg & 0x00ff0000) >> 16) > 4)
			printf("\t%08x = %016jx\n", reg, val64);
		if (error == 0) {
			*value = (uint32_t)(val64 & 0xffffffff);
			get_set_upper32 =
			    (uint32_t)((val64 >> 32) & 0xffffffff);
		}
	}
	return (error);
}

static int
set_reg(device_t dev, uint32_t reg, uint32_t *value)
{
	struct bcm5325_switch_softc *sc;
	uint64_t val64;
	uint32_t old;
	int error = 0;

	sc = device_get_softc(dev);

	switch (reg & SWITCH_REG_TYPE_MASK) {
	case SWITCH_REG_TYPE_PHY: /* Same in BCM53xx case */
	case SWITCH_REG_TYPE_RAW:
		old = MII_SW_READ4(sc, reg);
		MII_SW_WRITE4(sc, reg, *value);
		*value = old;
		return (0);
	}

	if (reg == 0x0ffffffful) {
		old = get_set_upper32;
		get_set_upper32 = *value;
		*value = old;
	} else {
		error = READ(sc, reg, &val64);
		if (error == 0) {
			/*
			 * If old value required for 64bits registers,
			 * use get_reg first
			 */
			old = (uint32_t)(val64 & 0xffffffff);
		} else {
			return (error);
		}
		/*
		 * When write 64bits value always set 0x0fffffff reg
		 * to upper 32 bit
		 */
		val64 = ((uint64_t)get_set_upper32 << 32) | (*value);
		error = WRITE(sc, reg, val64);
		if (error == 0)
			return (error);

		*value = old;
	}
	return (error);
}

static int
find_mac_addr(device_t dev, uint64_t mac)
{
	struct bcm5325_switch_softc *sc;
	int idx = -1;

	sc = device_get_softc(dev);

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
	struct bcm5325_switch_softc *sc;
	int error = 0;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	if (pvid > 0xfff)
		return (EINVAL);

	error = WRITE(sc, VLAN_DEFAULT_PORT_TAG(port), pvid);

	return (error);
}

static int
get_port_vid(device_t dev, int port, uint16_t *pvid)
{
	struct bcm5325_switch_softc *sc;
	uint64_t reg;
	int error = 0;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	error = READ(sc, VLAN_DEFAULT_PORT_TAG(port), &reg);
	*pvid = reg & 0xfff;

	return (error);
}

static int
get_port_flags(device_t dev, int port, uint32_t *flags)
{
	struct bcm5325_switch_softc *sc;
	uint64_t reg = 0;
	int error = 0;

	*flags = 0;
	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	error = READ(sc, VLAN_DROP_UNTAGGED, &reg);
	if (error)
		return (error);
	if (reg & VLAN_DROP_UNTAGGED_ONPORT(port))
		*flags |= DOT1Q_VLAN_PORT_FLAG_DROP_UNTAGGED;

	return (0);
}

static int
set_port_flags(device_t dev, int port, uint32_t flags)
{
	struct bcm5325_switch_softc *sc;
	uint64_t reg;
	int error = 0;

	sc = device_get_softc(dev);
	if (port > (sc->ports - 1))
		return (EINVAL);

	error = READ(sc, VLAN_DROP_UNTAGGED, &reg);
	if (error)
		return (error);

	if (flags & DOT1Q_VLAN_PORT_FLAG_DROP_UNTAGGED)
		reg |= VLAN_DROP_UNTAGGED_ONPORT(port);
	else
		reg &= ~VLAN_DROP_UNTAGGED_ONPORT(port);

	error = WRITE(sc, VLAN_DROP_UNTAGGED, reg);

	return (error);
}

static int
bcm5325_vlan_write(struct bcm5325_switch_softc *sc, uint16_t vid,
		   uint32_t tports, uint32_t uports)
{
	uint64_t reg;
	int error = 0;

	reg = VLAN_RW_VALID |
	    ((tports << VLAN_RW_MEMBER_SHIFT) & VLAN_RW_MEMBER_MASK) |
	    ((uports << VLAN_RW_UNTAGGED_SHIFT) & VLAN_RW_UNTAGGED_MASK);
	error = WRITE(sc, VLAN_WRITE, reg);
	if (error)
		return (error);
	reg = VLAN_TABLE_ACCESS_RW_ENABLE | VLAN_TABLE_ACCESS_WRITE |
	    (vid & VLAN_TABLE_ACCESS_VID_MASK);
	error = WRITE(sc, VLAN_TABLE_ACCESS, reg);

	return (error);
}

static int
bcm5325_vlan_read(struct bcm5325_switch_softc *sc, uint16_t vid,
		  uint32_t *tports, uint32_t *uports)
{
	uint64_t reg;
	int error = 0;

	reg = VLAN_TABLE_ACCESS_RW_ENABLE |
	    (vid & VLAN_TABLE_ACCESS_VID_MASK);
	error = WRITE(sc, VLAN_TABLE_ACCESS, reg);
	if (error)
		return (error);

	error = READ(sc, VLAN_READ, &reg);
	if (error)
		return (error);
	if (!(reg & VLAN_RW_VALID))
		return (ENOENT);
	if (tports != NULL)
		*tports = (reg & VLAN_RW_MEMBER_MASK) >> VLAN_RW_MEMBER_SHIFT;
	if (uports != NULL)
		*uports =
		    (reg & VLAN_RW_UNTAGGED_MASK) >> VLAN_RW_UNTAGGED_SHIFT;

	return (error);
}

static int
bcm5395_vlan_write(struct bcm5325_switch_softc *sc, uint16_t vid,
		   uint32_t tports, uint32_t uports)
{
	int error = 0;

	error = WRITE(sc, VLAN_TABLE_ENTRY_5395,
	    (uports << VLAN_RW_UNTAG_SHIFT_5395) | tports);
	if (error)
		return (error);
	error = WRITE(sc, VLAN_TABLE_INDX_5395, vid);
	if (error)
		return (error);
	error = WRITE(sc, VLAN_TABLE_ACCESS_5395, VLAN_TABLE_ACCESS_5395_RUN);
	if (error)
		return (error);

	return (error);
}

static int
bcm5395_vlan_read(struct bcm5325_switch_softc *sc, uint16_t vid,
		  uint32_t *tports, uint32_t *uports)
{
	uint64_t reg, mask;
	int error = 0;

	error = WRITE(sc, VLAN_TABLE_INDX_5395, vid);
	if (error)
		return (error);
	error = WRITE(sc, VLAN_TABLE_ACCESS_5395,
	    VLAN_TABLE_ACCESS_5395_RUN |VLAN_TABLE_ACCESS_5395_READ);
	if (error)
		return (error);

	error = READ(sc, VLAN_TABLE_ENTRY_5395, &reg);
	if (error)
		return (error);

	mask = (1 << VLAN_RW_UNTAG_SHIFT_5395) - 1;
	if (tports != NULL)
		*tports = reg & mask;
	if (uports != NULL)
		*uports = (reg >> VLAN_RW_UNTAG_SHIFT_5395) & mask;

	return (error);
}

static inline int
bcmXXXX_vlan_write(struct bcm5325_switch_softc *sc, uint16_t vid,
		   uint32_t tports, uint32_t uports)
{

	return (sc->vlan_write(sc, vid, tports, uports));
}

static inline int
bcmXXXX_vlan_read(struct bcm5325_switch_softc *sc, uint16_t vid,
		  uint32_t *tports, uint32_t *uports)
{

	return (sc->vlan_read(sc, vid, tports, uports));
}
/*
 * set_vid(dev, idx, vid)
 * Define a VLAN, since BCM5325 family use only lower 4-8 bits of VID -
 * idx parameter ignored.
 * VID = base_vlan << 4(or 8, dep on chip) + idx.
 * When base_vlan not equal with previouse value, WARNING displayed.
 */
static int
set_vid(device_t dev, int idx, uint16_t vid)
{
	struct bcm5325_switch_softc *sc;
	uint16_t base_vlan_mask;
	int error = 0;

	sc = device_get_softc(dev);
	if (idx > (sc->vlans - 1))
		return (EINVAL);

	base_vlan_mask = ~(sc->vlans - 1);
	if ((vid & base_vlan_mask) != sc->base_vlan) {
		sc->base_vlan = (vid & base_vlan_mask);
		device_printf(sc->sc_dev, "WARNING: Base VLAN changed %04x\n",
			      sc->base_vlan);
	}

	error = bcmXXXX_vlan_read(sc, vid, NULL, NULL);
	if (error == ENOENT) {
		/* Create empty valid VLAN port set */
		bcmXXXX_vlan_write(sc, vid, 0, 0);
		return (0);
	}

	return (error);
}

static int
get_vid(device_t dev, int idx, uint16_t *vid)
{
	struct bcm5325_switch_softc *sc = device_get_softc(dev);

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	*vid = sc->base_vlan * sc->vlans + idx;

	return (0);
}

static int
set_vlan_ports(device_t dev, int idx, uint32_t memb)
{
	struct bcm5325_switch_softc *sc = device_get_softc(dev);
	uint32_t umemb;
	int error = 0;

	printf("%s: idx=%d, memb=%08x\n", __func__, idx, memb);

	if (idx > (sc->vlans - 1))
		return (EINVAL);
	if (memb & ~((1 << sc->ports) - 1))
		return (EINVAL);

	bcmXXXX_vlan_read(sc, idx /* must be vid */, NULL, &umemb);
	error = bcmXXXX_vlan_write(sc, idx /* must be vid */, memb, umemb);

	return (error);
}

static int
get_vlan_ports(device_t dev, int idx, uint32_t *memb)
{
	struct bcm5325_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;
	int error;

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	error = bcmXXXX_vlan_read(sc, idx /* must be vid */, &reg, NULL);
	printf("%s: error=%d idx=%d, memb=%08x\n", __func__, error, idx, reg);
	if (error == ENOENT) {
		*memb = 0;
		return (0);
	}
	if (error)
		return (error);
	*memb = reg;

	return (0);
}

static int
set_vlan_untagged_ports(device_t dev, int idx, uint32_t umemb)
{
	struct bcm5325_switch_softc *sc = device_get_softc(dev);
	uint32_t memb;
	int error = 0;

	printf("%s: idx=%d, memb=%08x\n", __func__, idx, umemb);

	if (idx > (sc->vlans - 1))
		return (EINVAL);
	if (memb & ~((1 << sc->ports) - 1))
		return (EINVAL);

	bcmXXXX_vlan_read(sc, idx /* must be vid */, &memb, NULL);
	error = bcmXXXX_vlan_write(sc, idx /* must be vid */, memb | umemb,
	    umemb);

	return (error);
}

static int
get_vlan_untagged_ports(device_t dev, int idx, uint32_t *memb)
{
	struct bcm5325_switch_softc *sc = device_get_softc(dev);
	uint32_t reg;
	int error;

	if (idx > (sc->vlans - 1))
		return (EINVAL);

	error = bcmXXXX_vlan_read(sc, idx /* must be vid */, NULL, &reg);
	printf("%s: error=%d idx=%d, memb=%08x\n", __func__, error, idx, reg);
	if (error == ENOENT) {
		*memb = 0;
		return (0);
	}
	if (error)
		return (error);
	*memb = reg;

	return (0);
}

static int
pbvlan_setports(device_t dev, int port,	uint32_t allowed)
{
	struct bcm5325_switch_softc *sc;
	int error;

	sc = device_get_softc(dev);

	if (port > (sc->ports - 1))
		return (EINVAL);

	error = WRITE(sc, PBVLAN_ALLOWED_PORTS(port), allowed);
	if (error)
		return (error);

	return (0);
}

static int
pbvlan_getports(device_t dev, int port,	uint32_t *allowed)
{
	struct bcm5325_switch_softc *sc;
	uint64_t reg;
	int error;

	sc = device_get_softc(dev);

	if (port > (sc->ports - 1))
		return (EINVAL);

	error = READ(sc, PBVLAN_ALLOWED_PORTS(port), &reg);
	if (error)
		return (error);

	*allowed = (uint32_t)reg;
	return (0);
}

static int
reset_subsys(device_t dev, int subsys)
{
	struct bcm5325_switch_softc *sc;
	int error, port;

	sc = device_get_softc(dev);
	error = 0;
	switch (subsys & SWITCH_RESETSUB_MASK) {
	case SWITCH_RESETSUB_SWITCH:
		/* XXX: Hope it will reset any switch */
		error = WRITE(sc, SWITCH_RESET, 0xffff);
		if (error)
			return (error);

		DELAY(10000);

		error = WRITE(sc, SWITCH_RESET, 0x00);
		if (error)
			return (error);
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
		/* TODO */
		break;
	}

	return (error);
}

static device_method_t bcm5325_switch_methods[] = {
	DEVMETHOD(device_probe,		bcm5325_switch_probe),
	DEVMETHOD(device_attach,	bcm5325_switch_attach),
	DEVMETHOD(device_detach,	bcm5325_switch_detach),

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
	DEVMETHOD(switch_set_vlanutports,	set_vlan_untagged_ports),
	DEVMETHOD(switch_get_vlanutports,	get_vlan_untagged_ports),

	/* Port based VLAN */
	DEVMETHOD(switch_pbvlan_getports,	pbvlan_getports),
	DEVMETHOD(switch_pbvlan_setports,	pbvlan_setports),

	{0, 0},
};

static driver_t bcm5325_switch_driver = {
	"bcm5325_switch",
	bcm5325_switch_methods,
	sizeof(struct bcm5325_switch_softc),
};
static devclass_t bcm5325_switch_devclass;

DRIVER_MODULE(bcm5325_switch, switch, bcm5325_switch_driver,
	      bcm5325_switch_devclass, 0, 0);
MODULE_VERSION(bcm5325_switch, 1);
MODULE_DEPEND(bcm5325_switch, switch, 1, 1, 1);
