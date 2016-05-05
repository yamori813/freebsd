/*-
 * Copyright (c) 2007 Bruce M. Simpson.
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: head/sys/dev/siba/siba.c 227848 2011-11-22 21:55:40Z marius $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/malloc.h>
#include <sys/interrupt.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/siba/sibavar.h>
#include <dev/siba/sibareg.h>
#include <dev/siba/siba_ids.h>
#include <dev/siba/siba_cc.h>
#include <dev/siba/sibautils.h>

/*
 * TODO: De-mipsify this code.
 * TODO: cpu clock calculation. -> move to siba_cc instance
 * TODO: Hardwire IRQs for attached cores on siba at probe time.
 * TODO: Support detach.
 * TODO: Power management.
 * TODO: code cleanup.
 * TODO: Support deployments of siba other than as a system bus.
 */

extern int rman_debug;

static int siba_debug = 1;
static const char descfmt[] = "Sonics SiliconBackplane rev 0x%x";
#define SIBA_DEVDESCLEN sizeof(descfmt) + 8

/*
 * Device identifiers and descriptions.
 */
static struct siba_devid siba_devids[] = {
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_CHIPCOMMON,	SIBA_REV_ANY,
	  "ChipCommon" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_SDRAM,	SIBA_REV_ANY,
	  "SDRAM controller" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_PCI,		SIBA_REV_ANY,
	  "PCI host interface" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_MIPS,	SIBA_REV_ANY,
	  "MIPS core" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_ETHERNET,	SIBA_REV_ANY,
	  "Ethernet core" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_USB,		SIBA_REV_ANY,
	  "USB host controller" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_IPSEC,	SIBA_REV_ANY,
	  "IPSEC accelerator" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_SDRAMDDR,	SIBA_REV_ANY,
	  "SDRAM/DDR controller" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_MIPS_3302,	SIBA_REV_ANY,
	  "MIPS 3302 core" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_IEEE_802_11,	SIBA_REV_ANY,
	  "IEEE 802.11" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_USB_2_0_HOST,SIBA_REV_ANY,
	  "USB 2.0 Host" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_ROBOSWITCH,	SIBA_REV_ANY,
	  "Roboswitch" },
	{ SIBA_VID_BROADCOM,	SIBA_DEVID_CODEC,	SIBA_REV_ANY,
	  "V90 Codec Core" },
	{ 0, 0, 0, NULL }
};

static int	siba_activate_resource(device_t, device_t, int, int,
		    struct resource *);
static device_t	siba_add_child(device_t, u_int, const char *, int);
static struct resource *
		siba_alloc_resource(device_t, device_t, int, int *, rman_res_t,
		    rman_res_t, rman_res_t, u_int);
static int	siba_attach(device_t);
static int	siba_detach(device_t);
static void	siba_destroy_devinfo(device_t);
static struct siba_devid *
		siba_dev_match(uint16_t, uint16_t, uint8_t);
static struct resource_list *
		siba_get_reslist(device_t, device_t);
static uint8_t	siba_getirq(uint16_t);
static uint8_t	siba_getncores(device_t, uint16_t);
static int	siba_print_all_resources(device_t dev);
static int	siba_print_child(device_t, device_t);
static int	siba_probe(device_t);
static void	siba_probe_nomatch(device_t, device_t);
int		siba_read_ivar(device_t, device_t, int, uintptr_t *);
static struct siba_devinfo *
		siba_setup_devinfo(device_t, uint8_t);
int		siba_write_ivar(device_t, device_t, int, uintptr_t);

static void 	siba_print_regions(struct siba_softc *, uint8_t , 
		    uint16_t, device_t, struct siba_devinfo *);

extern int 	badaddr(char *addr, int len);
extern char cpu_model[80];
extern char cpu_board[80];

/*
 * Earlier ChipCommon revisions have hardcoded number of cores
 * present dependent on the ChipCommon ID.
 */
uint8_t
siba_getncores(device_t dev, uint16_t chipid)
{
	struct siba_softc *sc = device_get_softc(dev);
	uint8_t ncores = 0;
	switch (chipid) {
	case 0x4401:
	case 0x4402:
		return (3);
	case 0x4301:
	case 0x4307:
		return (5);
	case 0x4306:
		return (6);
	case SIBA_CCID_SENTRY5:
		return (7);
	case 0x4310:
		return (8);
	case SIBA_CCID_BCM4710:
	case 0x4610:
	case SIBA_CCID_BCM4704:
		return (9);
	default:
		for ( ncores = 0 ; 
		    ncores < SIBA_MAX_CORES ; 
		    ncores ++ )
			if (badaddr((char *)(sc->sc_bh | (ncores << 12) | 0xffc), 4)) 
			break;
		if (ncores > 0) return (ncores);
		device_printf(dev, "unknown the chipset ID %#x\n", chipid);
	}

	return (1);
}

/*
 * On the Sentry5, the system bus IRQs are the same as the
 * MIPS IRQs. Particular cores are hardwired to certain IRQ lines.
 */
static uint8_t
siba_getirq(uint16_t devid)
{
	uint8_t irq;

	switch (devid) {
	case SIBA_DEVID_CHIPCOMMON:
		irq = 0;
		break;
	case SIBA_DEVID_IPSEC:
		irq = 1;
		break;
	case SIBA_DEVID_ETHERNET:
		irq = 2;
		break;
	case SIBA_DEVID_USB:
	case SIBA_DEVID_USB_2_0_HOST:
		irq = 3;
		break;
	case SIBA_DEVID_PCI:
	case SIBA_DEVID_IEEE_802_11:
		irq = 4;
		break;
#if 0
	/*
	 * 5 is reserved for the MIPS on-chip timer interrupt;
	 * it is hard-wired by the tick driver.
	 */
	case SIBA_DEVID_MIPS:
	case SIBA_DEVID_MIPS_3302:
		irq = 5;
		break;
#endif
	default:
		irq = 0xFF;	/* this core does not need an irq */
		break;
	}

	return (irq);
}



static int
siba_probe(device_t dev)
{
	struct siba_softc *sc = device_get_softc(dev);
	uint16_t vid, devid, ccid, cc_id, cc_rev;
	uint32_t idlo, idhi, ccidreg;
	char soc_name[20];
	int idx, rid;

	sc->sc_dev = dev;

	/*
	 * Map the ChipCommon register set using the hints the kernel
	 * was compiled with.
	 */
	rid = 0;
	sc->sc_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		device_printf(dev, "unable to allocate probe aperture\n");
		return (ENXIO);
	}
	sc->sc_bt = rman_get_bustag(sc->sc_mem);
	sc->sc_bh = rman_get_bushandle(sc->sc_mem);
	sc->sc_maddr = rman_get_start(sc->sc_mem);
	sc->sc_msize = rman_get_size(sc->sc_mem);

	if (siba_debug) {
		device_printf(dev, "start %08x len %08x\n",
		    sc->sc_maddr, sc->sc_msize);
	}

	idlo = siba_read_4(sc, 0, SIBA_CORE_IDLO);
	idhi = siba_read_4(sc, 0, SIBA_CORE_IDHI);
	ccid = ((idhi & 0x8ff0) >> 4);
	if (siba_debug) {
		device_printf(dev, "idlo = %08x\n", idlo);
		device_printf(dev, "idhi = %08x\n", idhi);
		device_printf(dev, " chipcore id = %08x\n", ccid);
	}

	/*
	 * For now, check that the first core is the ChipCommon core.
	 */
	if (ccid != SIBA_DEVID_CHIPCOMMON) {
		if (siba_debug)
			device_printf(dev, "first core is not ChipCommon\n");
		return (ENXIO);
	}

	/*
	 * Determine backplane revision and set description string.
	 */
	char descbuf[SIBA_DEVDESCLEN];

	sc->sc_rev = (uint8_t) idlo >> 28 & 0x0F;

	(void)snprintf(descbuf, sizeof(descbuf), descfmt, sc->sc_rev);
	device_set_desc_copy(dev, descbuf);

	/*
	 * Determine how many cores are present on this siba bus, so
	 * that we may map them all.
	 */
	ccidreg = siba_read_4(sc, 0, SIBA_CC_CCID);
	cc_id = (ccidreg & SIBA_CC_IDMASK);
	cc_rev = (ccidreg & SIBA_CC_REVMASK) >> SIBA_CC_REVSHIFT;
	if (siba_debug) {
		device_printf(dev, "ccid = %08x, cc_id = %04x, cc_rev = %04x\n",
		     ccidreg, cc_id, cc_rev);
	}

	if (strlen(cpu_model) == 0) {
		vid = (siba_read_4(sc, 0, SIBA_CORE_IDHI) & SIBA_IDHIGH_VC) >>
		    SIBA_IDHIGH_VC_SHIFT;
		if (vid == SIBA_VID_BROADCOM) {
			sprintf(soc_name,
			    "Vendor: %04x Core: BCM%04x rev: %d",
			    vid, cc_id, cc_rev);
		} else {
			sprintf(soc_name, "Vendor: %04x Core: %04x rev: %d",
			    vid, cc_id, cc_rev);
		}
		(void)snprintf(cpu_model, sizeof(cpu_model)-1, "%s", soc_name);
	}
	/* Get board name */
	if (strlen(cpu_board) == 0) {
#ifdef	TARGET_BOARD_NAME
		(void)snprintf(cpu_board, sizeof(cpu_board)-1, "%s",
		    TARGET_BOARD_NAME);
#else
		(void)snprintf(cpu_board, sizeof(cpu_board)-1, "%s",
		    kern_ident);
#endif
	}

	sc->sc_ncores = siba_getncores(dev, cc_id);

	if (siba_debug) {
		device_printf(dev, "%d cores detected.\n", sc->sc_ncores);
	}

	/*
	 * Now we know how many cores are on this siba, release the
	 * mapping and allocate a new mapping spanning all cores on the bus.
	 */
	rid = 0;
	int result;
	result = bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->sc_mem);
	if (result != 0) {
		device_printf(dev, "error %d releasing resource\n", result);
		return (ENXIO);
	}

	uint32_t total;
	total = sc->sc_ncores * SIBA_CORE_LEN;

	/* XXX Don't allocate the entire window until we
	 * enumerate the bus. Once the bus has been enumerated,
	 * and instance variables/children instantiated + populated,
	 * release the resource so children may attach.
	 */
	sc->sc_mem = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid,
	    sc->sc_maddr, sc->sc_maddr + total - 1, total, RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		device_printf(dev, "unable to allocate entire aperture\n");
		return (ENXIO);
	}
	sc->sc_bt = rman_get_bustag(sc->sc_mem);
	sc->sc_bh = rman_get_bushandle(sc->sc_mem);
	sc->sc_maddr = rman_get_start(sc->sc_mem);
	sc->sc_msize = rman_get_size(sc->sc_mem);

	if (siba_debug) {
		device_printf(dev, "after remapping: start %08x len %08x\n",
		    sc->sc_maddr, sc->sc_msize);
	}


	bus_set_resource(dev, SYS_RES_MEMORY, rid, sc->sc_maddr, sc->sc_msize);



	/*
	 * We need a manager for the space we claim on nexus to
	 * satisfy requests from children.
	 * We need to keep the source reservation we took because
	 * otherwise it may be claimed elsewhere.
	 */
	sc->mem_rman.rm_start = sc->sc_maddr;
	sc->mem_rman.rm_end = sc->sc_maddr + sc->sc_msize - 1;
	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "SiBa I/O memory addresses";
	if (rman_init(&sc->mem_rman) != 0 ||
	    rman_manage_region(&sc->mem_rman, sc->mem_rman.rm_start, sc->mem_rman.rm_end) != 0) {
		panic("%s: sc->mem_rman", __func__);
	}

	/*
	 * Find core that route IRQ for as,
	 * Resul is MIPS core ID(prefered) or PCI[E] core ID
	 */
	sc->sc_irq_route_core = 255;
	for (idx = 0; idx < sc->sc_ncores; idx++) {

		devid = ((siba_read_4(sc, idx, SIBA_CORE_IDHI) & 0x8ff0) >> 4);

		if ( devid == SIBA_DEVID_MIPS_3302)
		     sc->sc_irq_route_core = idx;

		if ( sc->sc_irq_route_core == 255 &&
			(devid == SIBA_DEVID_PCI ||
			 devid == SIBA_DEVID_PCIE) )
		     sc->sc_irq_route_core = idx;
	}


	return (0);
}

static int
siba_attach(device_t dev)
{
	struct siba_softc	*sc = device_get_softc(dev);
	struct siba_devinfo	*sdi;
	device_t		 child;
	int			 idx;
	uint16_t		 devid;

	if (siba_debug)
		printf("%s: entry\n", __func__);
	bus_generic_probe(dev);

	/*
	 * Now that all bus space is mapped and visible to the CPU,
	 * enumerate its children.
	 * NB: only one core may be mapped at any time if the siba bus
	 * is the child of a PCI or PCMCIA bus.
	 */
	for (idx = 0; idx < sc->sc_ncores; idx++) {
		devid = ((siba_read_4(sc, idx, SIBA_CORE_IDHI) & 0x8ff0) >> 4);
		/* Require to reset switch and usb cores */
		if (( devid == SIBA_DEVID_ROBO ||
		     devid == SIBA_DEVID_USB20H )) {
			if ( devid == SIBA_DEVID_ROBO   ) 
				printf("\tReset switch core\n");
			if ( devid == SIBA_DEVID_USB20H ) 
				printf("\tReset USB core\n");
			siba_dev_enable(dev, idx, 0);
			DELAY(100);
		}

		sdi = siba_setup_devinfo(dev, idx);
		child = device_add_child(dev, NULL, -1);
		if (child == NULL)
			panic("%s: device_add_child() failed\n", __func__);
		device_set_ivars(child, sdi);
	}

	return (bus_generic_attach(dev));
}

static int
siba_detach(device_t dev)
{
	device_t *devlistp;
	int devcnt, error = 0, i;

	error = device_get_children(dev, &devlistp, &devcnt);
	if (error != 0)
		return (0);

	for ( i = 0 ; i < devcnt ; i++)
	{
		siba_destroy_devinfo(devlistp[i]);
		device_delete_child(dev, devlistp[i]);
	}
	free(devlistp, M_TEMP);
	return (0);
}




static struct siba_devid *
siba_dev_match(uint16_t vid, uint16_t devid, uint8_t rev)
{
	size_t			 i;
	struct siba_devid	*sd;

	sd = &siba_devids[0];
	for (i = 0; i < nitems(siba_devids); i++, sd++) {
		if (((vid == SIBA_VID_ANY) || (vid == sd->sd_vendor)) &&
		    ((devid == SIBA_DEVID_ANY) || (devid == sd->sd_device)) &&
		    ((rev == SIBA_REV_ANY) || (rev == sd->sd_rev) ||
		     (sd->sd_rev == SIBA_REV_ANY)))
			return(sd);
	}

	return (NULL);
}

static int
siba_print_child(device_t bus, device_t child)
{
	int retval = 0;

	retval += bus_print_child_header(bus, child);
	retval += siba_print_all_resources(child);
	if (device_get_flags(child))
		retval += printf(" flags %#x", device_get_flags(child));
	retval += printf(" on %s\n", device_get_nameunit(bus));

	return (retval);
}

static struct resource *
siba_alloc_resource(device_t bus, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct resource			*rv;
	struct resource_list		*rl;
	struct resource_list_entry	*rle;
	int				 isdefault, needactivate;
	struct siba_softc		*sc = device_get_softc(bus);

	isdefault = (RMAN_IS_DEFAULT_RANGE(start, end) && count == 1);
	needactivate = flags & RF_ACTIVE;
	rl = BUS_GET_RESOURCE_LIST(bus, child);
	rle = NULL;

	if (isdefault) {
		rle = resource_list_find(rl, type, *rid);
		if (rle == NULL)
			return (NULL);
		if (rle->res != NULL)
			panic("%s: resource entry is busy", __func__);
		start = rle->start;
		end = rle->end;
		count = rle->count;
	}

	/*
	 * If the request is for a resource which we manage,
	 * attempt to satisfy the allocation ourselves.
	 */
	if (type == SYS_RES_MEMORY &&
	    start >= sc->mem_rman.rm_start && end <= sc->mem_rman.rm_end) {
// siba_cc0
if(start == 0x18000000 && end == 0x18000fff) {
end = start + 0x300 - 1;count = 0x300;}

		rv = rman_reserve_resource(&sc->mem_rman, start, end, count,
		    flags, child);
		if (rv == 0) {
			printf("%s: could not reserve resource\n", __func__);
			return (0);
		}

		rman_set_rid(rv, *rid);

		if (needactivate) {
			if (bus_activate_resource(child, type, *rid, rv)) {
				printf("%s: could not activate resource\n",
				    __func__);
				rman_release_resource(rv);
				return (0);
			}
		}

		return (rv);
	}

	/*
	 * Pass the request to the parent, usually MIPS nexus.
	 */
	return (resource_list_alloc(rl, bus, child, type, rid,
	    start, end, count, flags));
}

/*
 * The parent bus is responsible for resource activation; in the
 * case of MIPS, this boils down to setting the virtual address and
 * bus handle by mapping the physical address into KSEG1.
 */
static int
siba_activate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *r)
{

	return (BUS_ACTIVATE_RESOURCE(device_get_parent(bus), child, type,
	    rid, r));
}

static struct siba_devinfo *
siba_setup_devinfo(device_t dev, uint8_t idx)
{
	struct siba_softc *sc = device_get_softc(dev);
	struct siba_devinfo *sdi;
	uint32_t idlo, idhi, addrrange;
	uint8_t win, wincount;
	uint32_t admatch[4] = {
		SIBA_ADMATCH0, 
		SIBA_ADMATCH1, 
		SIBA_ADMATCH2, 
		SIBA_ADMATCH3
	};

	sdi = malloc(sizeof(*sdi), M_DEVBUF, M_WAITOK | M_ZERO);
	resource_list_init(&sdi->sdi_rl);

	idlo = siba_read_4(sc, idx, SIBA_CORE_IDLO);
	idhi = siba_read_4(sc, idx, SIBA_CORE_IDHI);

	sdi->sdi_vid = (idhi & SIBA_IDHIGH_VC) >> SIBA_IDHIGH_VC_SHIFT;
	sdi->sdi_devid = ((idhi & 0x8ff0) >> 4);
	sdi->sdi_rev = (idhi & SIBA_IDHIGH_RCLO);
	sdi->sdi_rev |= (idhi & SIBA_IDHIGH_RCHI) >> SIBA_IDHIGH_RCHI_SHIFT;

	sdi->sdi_idx = idx;
	sdi->sdi_irq = siba_getirq(sdi->sdi_devid);
	sdi->sdi_flag = siba_read_4(sc, idx, SIBA_TPS) & SIBA_TPS_BPFLAG;

	device_printf(dev, "core=%d Vendor=%04x Dev=%03x "
		    "Rev=%03x IRQ flag=%x\n",
	    sdi->sdi_idx, sdi->sdi_vid, sdi->sdi_devid, 
	    sdi->sdi_rev, sdi->sdi_flag);

	/* Route interrupt */
	int irq_core = sc->sc_irq_route_core;
	uint32_t flags;
	int n;
	/* XXX should set by IRQ usage */
//	siba_write_4(sc, irq_core, SIBA_IPSFLAG, 0x03020100);
//	siba_write_4(sc, irq_core, SIBA_INTVEC, 0xf1);
	if (sdi->sdi_irq == 0)
	{
	    flags = siba_read_4(sc, irq_core, SIBA_IPSFLAG);
	    /* Delete old flag */
	    for ( n = 0; n < 4; n ++ )
		if (((flags >> (n*8)) & 0x3f) == sdi->sdi_flag)
		    flags &= ~(0x3f << (n*8));

	    siba_write_4(sc, irq_core, SIBA_IPSFLAG, flags);
	    /* Readback */
	    siba_read_4(sc, irq_core, SIBA_IPSFLAG);

	    /* Set flag to route via IRQ0 */
	    siba_write_4(sc, irq_core, SIBA_INTVEC, 
		siba_read_4(sc, irq_core, SIBA_INTVEC) | (1 << sdi->sdi_flag));
	    /* Readback */
	    siba_read_4(sc, irq_core, SIBA_INTVEC);
	}
	else if (sdi->sdi_irq >= 1 && sdi->sdi_irq <= 4)
	{
	    flags = siba_read_4(sc, irq_core, SIBA_IPSFLAG);

	    /* Delete old flag */
	    for ( n = 0; n < 4; n ++ )
		if (((flags >> (n*8)) & 0x3f) == sdi->sdi_flag)
		    flags &= ~(0x3f << (n*8));

	    /* Add new flag */
	    flags &= ~(0x3f << ((sdi->sdi_irq - 1)*8));
	    flags |= sdi->sdi_flag << ((sdi->sdi_irq - 1)*8);

	    siba_write_4(sc, irq_core, SIBA_IPSFLAG, flags);
	    /* Readback */
	    siba_read_4(sc, irq_core, SIBA_IPSFLAG);

	    /* Clear flag to not route via IRQ0 */
	    siba_write_4(sc, irq_core, SIBA_INTVEC, 
		siba_read_4(sc, irq_core, SIBA_INTVEC) & ~(1 << sdi->sdi_flag));
	    /* Readback */
	    siba_read_4(sc, irq_core, SIBA_INTVEC);
	}

	/*
	 * Determine memory window on bus and irq if one is needed.
	 */
	wincount = (siba_read_4(sc, idx, SIBA_IDLOW) & 
		SIBA_IDLOW_ADDR_RANGE_MASK) >> SIBA_IDLOW_ADDR_RANGE_SHIFT;
	for (win = 0; win <= wincount; win++ )
	{
		addrrange = siba_read_4(sc, idx, admatch[win]);
		sdi->sdi_addr_win[win] = 0;
		sdi->sdi_addr_win_size[win] = 0; 

		switch (addrrange & SBAM_TYPE_MASK) {
		case 0:
			sdi->sdi_addr_win[win] = (addrrange & SBAM_BASE0_MASK);
			sdi->sdi_addr_win_size[win] = 1 << 
			    (((addrrange & SBAM_ADINT0_MASK) >> 
			    SBAM_ADINT0_SHIFT) + 1);
			break;
		case 1:
			sdi->sdi_addr_win[win] = (addrrange & SBAM_BASE1_MASK);
			sdi->sdi_addr_win_size[win] = 1 << 
			    (((addrrange & SBAM_ADINT1_MASK) >> 
			    SBAM_ADINT1_SHIFT) + 1);
			break;
		case 2:
			sdi->sdi_addr_win[win] = (addrrange & SBAM_BASE2_MASK);
			sdi->sdi_addr_win_size[win] = 1 << 
			    (((addrrange & SBAM_ADINT2_MASK) >> 
			    SBAM_ADINT2_SHIFT) + 1);
			break;
		default:
			device_printf(dev, "core=%d Vendor=%04x Dev=%03x "
			    "Rev=%03x unknown address range type %#x\n", 
			    sdi->sdi_idx, sdi->sdi_vid, sdi->sdi_devid, 
			    sdi->sdi_rev, addrrange);
			return (0);
		}
	}
	/* Map core space for drivers access */
	if (sdi->sdi_addr_win_size[0])
		resource_list_add(&sdi->sdi_rl, SYS_RES_MEMORY,
		    0, 
		    sdi->sdi_addr_win[0], 
		    sdi->sdi_addr_win[0] + sdi->sdi_addr_win_size[0] - 1, 
		    sdi->sdi_addr_win_size[0]);

	if (sdi->sdi_irq != 0xff) {
		resource_list_add(&sdi->sdi_rl, SYS_RES_IRQ,
		    0, sdi->sdi_irq, sdi->sdi_irq, 1);
	}
	siba_print_regions(sc, idx, sdi->sdi_devid, dev, sdi);
	return (sdi);
}

static void 
siba_print_regions(struct siba_softc *sc, uint8_t idx, uint16_t coreid, device_t dev, struct siba_devinfo *sdi)
{
	uint32_t id, i;
	struct siba_devid *sd;
	id = siba_read_4(sc, idx, 0x0ffc);

	sd = siba_dev_match(
	    ( id & SIBA_IDHIGH_VC  ) >> SIBA_IDHIGH_VC_SHIFT, 
	    ( id & SIBA_IDHIGH_CC  ) >> SIBA_IDHIGH_CC_SHIFT, 
	    ((id & SIBA_IDHIGH_RCHI) >> SIBA_IDHIGH_RCHI_SHIFT) | (id & SIBA_IDHIGH_RCLO));


	if ( sd ) { 
		device_printf(dev, "<%s> corid=%04x regions: ", sd->sd_desc, coreid);
		for (i = 0; i < SIBA_MAX_WIN_COUNT; i ++)
		{
			if (sdi->sdi_addr_win_size[i] > 0)
	    			printf(" %d=%08xx%08x", i,
	    	    		    sdi->sdi_addr_win[i], 
	    	    		    sdi->sdi_addr_win_size[i]);

		}
		printf("\n");
	}
}

static void
siba_destroy_devinfo(device_t child)
{
	struct siba_devinfo *sdi;
	sdi = device_get_ivars(child);
	resource_list_free(&sdi->sdi_rl);
	free(sdi, M_DEVBUF);
}

/* XXX is this needed? */
static device_t
siba_add_child(device_t dev, u_int order, const char *name, int unit)
{
#if 1

	device_printf(dev, "%s: entry\n", __func__);
	return (NULL);
#else
	device_t child;
	struct siba_devinfo *sdi;

	child = device_add_child_ordered(dev, order, name, unit);
	if (child == NULL)
		return (NULL);

	sdi = malloc(sizeof(struct siba_devinfo), M_DEVBUF, M_NOWAIT|M_ZERO);
	if (sdi == NULL)
		return (NULL);

	device_set_ivars(child, sdi);
	return (child);
#endif
}

int
siba_read_ivar(device_t dev, device_t child, int which, uintptr_t *result)
{
	struct siba_devinfo *sdi;

	sdi = device_get_ivars(child);

	switch (which) {
	case SIBA_IVAR_VENDOR:
		*result = sdi->sdi_vid;
		break;
	case SIBA_IVAR_DEVICE:
		*result = sdi->sdi_devid;
		break;
	case SIBA_IVAR_REVID:
		*result = sdi->sdi_rev;
		break;
	case SIBA_IVAR_CORE_INDEX:
		*result = sdi->sdi_idx;
		break;
	case SIBA_IVAR_CORE_ADDRESS_SPACE0:
		*result = sdi->sdi_addr_win[0];
		break;
	case SIBA_IVAR_CORE_ADDRESS_SPACE1:
		*result = sdi->sdi_addr_win[1];
		break;
	case SIBA_IVAR_CORE_ADDRESS_SPACE2:
		*result = sdi->sdi_addr_win[2];
		break;
	case SIBA_IVAR_CORE_ADDRESS_SPACE3:
		*result = sdi->sdi_addr_win[3];
		break;
	case SIBA_IVAR_CORE_ADDRESS_SPACE0_SIZE:
		*result = sdi->sdi_addr_win_size[0];
		break;
	case SIBA_IVAR_CORE_ADDRESS_SPACE1_SIZE:
		*result = sdi->sdi_addr_win_size[1];
		break;
	case SIBA_IVAR_CORE_ADDRESS_SPACE2_SIZE:
		*result = sdi->sdi_addr_win_size[2];
		break;
	case SIBA_IVAR_CORE_ADDRESS_SPACE3_SIZE:
		*result = sdi->sdi_addr_win_size[3];
		break;
	default:
		return (ENOENT);
	}

	return (0);
}

int
siba_write_ivar(device_t dev, device_t child, int which, uintptr_t value)
{

	return (EINVAL);
}

static void
siba_probe_nomatch(device_t dev, device_t child)
{

	/*
	 * Announce devices which weren't attached after we probed the bus.
	 */
	if (siba_debug) {
		struct siba_devid *sd;

		sd = siba_dev_match(siba_get_vendor(child),
		    siba_get_device(child), SIBA_REV_ANY);
		if (sd != NULL && sd->sd_desc != NULL) {
			device_printf(dev, "<%s> "
			    "at device %d (no driver attached)\n",
			    sd->sd_desc, siba_get_core_index(child));
		} else {
			device_printf(dev, "<0x%04x, 0x%04x> "
			    "at device %d (no driver attached)\n",
			    siba_get_vendor(child), siba_get_device(child),
			    siba_get_core_index(child));
		}
	}

}

static int
siba_print_all_resources(device_t dev)
{
	struct siba_devinfo *sdi = device_get_ivars(dev);
	struct resource_list *rl = &sdi->sdi_rl;
	int retval = 0;

	if (STAILQ_FIRST(rl))
		retval += printf(" at");

	retval += resource_list_print_type(rl, "mem", SYS_RES_MEMORY, "%#jx");
	retval += resource_list_print_type(rl, "irq", SYS_RES_IRQ, "%jd");

	return (retval);
}

static struct resource_list *
siba_get_reslist(device_t dev, device_t child)
{
	struct siba_devinfo *sdi = device_get_ivars(child);

	return (&sdi->sdi_rl);
}



static device_method_t siba_methods[] = {
	/* Device interface */
	DEVMETHOD(device_attach,	siba_attach),
	DEVMETHOD(device_detach,	siba_detach),
	DEVMETHOD(device_probe,		siba_probe),
	DEVMETHOD(device_resume,	bus_generic_resume),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD(device_suspend,	bus_generic_suspend),

	/* Bus interface */
	DEVMETHOD(bus_activate_resource,siba_activate_resource),
	DEVMETHOD(bus_add_child,	siba_add_child),
	DEVMETHOD(bus_alloc_resource,	siba_alloc_resource),
	DEVMETHOD(bus_get_resource_list,siba_get_reslist),
	DEVMETHOD(bus_print_child,	siba_print_child),
	DEVMETHOD(bus_probe_nomatch,	siba_probe_nomatch),
	DEVMETHOD(bus_read_ivar,	siba_read_ivar),
	DEVMETHOD(bus_setup_intr,	bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,	bus_generic_teardown_intr),
	DEVMETHOD(bus_write_ivar,	siba_write_ivar),

	DEVMETHOD_END
};

static driver_t siba_driver = {
	"siba",
	siba_methods,
	sizeof(struct siba_softc),
};
static devclass_t siba_devclass;

DRIVER_MODULE(siba, nexus, siba_driver, siba_devclass, 0, 0);
