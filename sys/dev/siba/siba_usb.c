/*-
 * Copyright (c) 2010, Aleksandr Rybalko <ray@ddteam.net>
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

/*
 * SIBA USB 2.0 core driver.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/malloc.h>
#include <sys/interrupt.h>

#include <machine/bus.h>
#include <machine/intr_machdep.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/pmap.h>
#include <machine/resource.h>
#include <machine/vmparam.h>



#include <dev/siba/sibavar.h>
#include <dev/siba/sibareg.h>
#include <dev/siba/siba_ids.h>
#include <dev/siba/sibautils.h>



struct siba_usb_softc {
	bus_space_tag_t		 sc_bt;
	bus_space_handle_t	 sc_bh;
	bus_addr_t		 sc_maddr;
	bus_size_t		 sc_msize;
	bus_addr_t		 sc_irqn;
	struct intr_event	*sc_events; /* IRQ events structs */

	struct resource *sc_mem;
	struct resource *sc_irq;
	struct rman 		 mem_rman;
	struct rman 		 irq_rman;
	int 			devid;

};


struct siba_usb_devinfo {
	struct resource_list	sdi_rl;
	uint8_t			sdi_unit;	/* core index on bus */
	uint8_t			sdi_irq;
	char 			sdi_name[8];
	bus_addr_t 		sdi_maddr;
	bus_size_t 		sdi_msize;
};


static int	siba_usb_attach(device_t);
static int	siba_usb_probe(device_t);
static device_t	siba_usb_add_child(device_t dev, u_int order, const char *name, 
		    int unit);



void mips_mask_irq(void *source);
void mips_unmask_irq(void *source);






static int
siba_usb_probe(device_t dev)
{

	if (siba_get_vendor(dev) == SIBA_VID_BROADCOM &&
	    (siba_get_device(dev) == SIBA_DEVID_USB ||
	    siba_get_device(dev) == SIBA_DEVID_USB20H)) {
		device_set_desc(dev, "USB core");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
siba_usb_attach(device_t dev)
{
	struct siba_usb_softc *sc = device_get_softc(dev);
	int rid;
	uint32_t tmp;

	/*
	 * Allocate the resources which the parent bus has already
	 * determined for us.
	 */
	rid = 0;
	sc->sc_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		device_printf(dev, "unable to allocate memory\n");
		return (ENXIO);
	}

	sc->sc_bt = rman_get_bustag(sc->sc_mem);
	sc->sc_bh = rman_get_bushandle(sc->sc_mem);
	sc->sc_maddr = rman_get_start(sc->sc_mem);
	sc->sc_msize = rman_get_size(sc->sc_mem);


	rid = 0;
	sc->sc_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, 
		RF_SHAREABLE | RF_ACTIVE);
	if (sc->sc_irq == NULL) {
		device_printf(dev, "unable to allocate irq\n");
		return (ENXIO);
	}
	sc->sc_irqn = rman_get_start(sc->sc_irq);

	sc->mem_rman.rm_start = sc->sc_maddr;
	sc->mem_rman.rm_end = sc->sc_maddr + sc->sc_msize - 1;
	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "SiBa USB core I/O memory addresses";
	if (rman_init(&sc->mem_rman) != 0 ||
	    rman_manage_region(&sc->mem_rman, sc->mem_rman.rm_start, sc->mem_rman.rm_end) != 0) {
		panic("%s: sc->mem_rman", __func__);
	}

	sc->irq_rman.rm_start = sc->sc_irqn;
	sc->irq_rman.rm_end = sc->sc_irqn;// + sc->sc_irqc - 1;
	sc->irq_rman.rm_type = RMAN_ARRAY;
	sc->irq_rman.rm_descr = "SIBA USB core IRQ";
	/* 
	 * Siba USB share one IRQ between OHCI and EHCI
	 */
	if (rman_init(&sc->irq_rman) != 0 ||
	    rman_manage_region(&sc->irq_rman, sc->irq_rman.rm_start, sc->irq_rman.rm_end) != 0)
		panic("%s: failed to set up IRQ rman", __func__);

	bus_write_4(sc->sc_mem, 0x200, 0x7ff); 
	DELAY(100); 

#define	OHCI_CONTROL		0x04
	bus_write_4(sc->sc_mem, OHCI_CONTROL, 0);

	if ( siba_get_device(dev) == SIBA_DEVID_USB20H) { 

    		device_printf(dev, "USB HOST 2.0 setup\n"); 
		if (siba_get_device(dev) == SIBA_DEVID_USB20H && 
		    ( siba_get_revid(dev) == 1/* || siba_get_revid(dev) == 2 */)) {
		    /* Change Flush control reg */ 
		    tmp = bus_read_4(sc->sc_mem, 0x400) & ~0x8;
    		    bus_write_4(sc->sc_mem, 0x400, tmp); 
    		    tmp = bus_read_4(sc->sc_mem, 0x400); 
        	    if ( bootverbose )
        		    device_printf(dev, "USB20H fcr: 0x%x\n", tmp); 

		    /* Change Shim control reg */ 
    		    tmp = bus_read_4(sc->sc_mem, 0x304) & ~0x100; 
        	    bus_write_4(sc->sc_mem, 0x304, tmp); 
        	    tmp = bus_read_4(sc->sc_mem, 0x304); 
        	    if ( bootverbose )
        		    device_printf(dev, "USB20H shim: 0x%x\n", tmp); 

		}
		if (0x5354) /* XXX must check chipid there */
		{
		    /* syn01 */ 
		    tmp = 0x00fe00fe; 
        	    bus_write_4(sc->sc_mem, 0x894, tmp); 

		    /* syn03 */ 
		    tmp = 0x1; 
        	    bus_write_4(sc->sc_mem, 0x89c, tmp); 
        	}
#define SIBA_HCI_TMSTATELOW_HOSTMODE        (1 << 29)

		tmp = bus_read_4(sc->sc_mem, SIBA_TMSLOW); 
		tmp |= SIBA_HCI_TMSTATELOW_HOSTMODE;
        	bus_write_4(sc->sc_mem, SIBA_TMSLOW, tmp); 
        	tmp = bus_read_4(sc->sc_mem, SIBA_TMSLOW); 
		DELAY(1);
        	tmp = bus_read_4(sc->sc_mem, SIBA_TMSLOW); 
        	if ( bootverbose )
        		device_printf(dev,"Host mode: %s\n", 
        			(tmp & SIBA_HCI_TMSTATELOW_HOSTMODE)?
        			 "enabled":"disabled" ); 

	}


	bus_generic_probe(dev);

	if (siba_get_device(dev) == SIBA_DEVID_USB20H && 
	    ( siba_get_revid(dev) == 1 || siba_get_revid(dev) == 2 )) 
		siba_usb_add_child(dev, 0, "ehci", -1);
	siba_usb_add_child(dev, 1, "ohci", -1);

	bus_generic_attach(dev);

	return (0);
}




static struct resource *
siba_usb_alloc_resource(device_t bus, device_t child, int type, int *rid,
    u_long start, u_long end, u_long count, u_int flags)
{
	struct resource			*rv;
	struct resource_list		*rl;
	struct resource_list_entry	*rle;
	int				 isdefault, needactivate;
	struct siba_usb_softc		*sc = device_get_softc(bus);

	isdefault = (start == 0UL && end == ~0UL);
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
	if (type == SYS_RES_MEMORY) {

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

	if (type == SYS_RES_IRQ) {

		rv = rman_reserve_resource(&sc->irq_rman, start, end, count,
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
	 * Pass the request to the parent.
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
siba_usb_activate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *r)
{
	return (BUS_ACTIVATE_RESOURCE(device_get_parent(bus), child, type,
	    rid, r));
}

static int
siba_usb_deactivate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *r)
{
	return (BUS_DEACTIVATE_RESOURCE(device_get_parent(bus), child, type,
	    rid, r));
}




static struct resource_list *
siba_usb_get_reslist(device_t dev, device_t child)
{
	struct siba_usb_devinfo *sdi = device_get_ivars(child);

	return (&sdi->sdi_rl);
}


static int
siba_usb_release_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	struct resource_list *rl;
	struct resource_list_entry *rle;

	rl = siba_usb_get_reslist(dev, child);
	if (rl == NULL)
		return (EINVAL);
	rle = resource_list_find(rl, type, rid);
	if (rle == NULL)
		return (EINVAL);
	rman_release_resource(r);
	rle->res = NULL;

	return (0);
}




static int
siba_usb_print_all_resources(device_t dev)
{
	struct siba_usb_devinfo *sdi = device_get_ivars(dev);
	struct resource_list *rl = &sdi->sdi_rl;
	int retval = 0;

	if (STAILQ_FIRST(rl))
		retval += printf(" at");

	retval += resource_list_print_type(rl, "mem", SYS_RES_MEMORY, "%#lx");
	retval += resource_list_print_type(rl, "irq", SYS_RES_IRQ, "%ld");

	return (retval);
}

static int
siba_usb_print_child(device_t bus, device_t child)
{
	int retval = 0;

	retval += bus_print_child_header(bus, child);
	retval += siba_usb_print_all_resources(child);
	if (device_get_flags(child))
		retval += printf(" flags %#x", device_get_flags(child));
	retval += printf(" on %s\n", device_get_nameunit(bus));

	return (retval);
}

static device_t
siba_usb_add_child(device_t dev, u_int order, const char *name, int unit)
{
	struct siba_usb_softc	*sc = device_get_softc(dev);
	device_t 		child;
	struct siba_usb_devinfo 	*sdi;

	child = device_add_child_ordered(dev, order, name, unit);
	if (child == NULL)
		return (NULL);

	sdi = malloc(sizeof(struct siba_usb_devinfo), M_DEVBUF, M_NOWAIT|M_ZERO);
	if (sdi == NULL)
		return (NULL);

	if (strncmp(name, "ohci", 4) == 0) 
	{
		sdi->sdi_maddr = sc->sc_maddr + 0x000;
		sdi->sdi_msize = 0x200;
		sdi->sdi_irq   = sc->sc_irqn;
	}
	else if (strncmp(name, "ehci", 4) == 0) 
	{
		sdi->sdi_maddr = sc->sc_maddr + 0x800;
		sdi->sdi_msize = 0x100;
		sdi->sdi_irq   = sc->sc_irqn;
	}
	else
	{
	    /* Unknown subdevice */
	    sdi->sdi_maddr = 1;
	    sdi->sdi_msize = 1;
	    sdi->sdi_irq   = 1;

	}

	resource_list_init(&sdi->sdi_rl);

	/*
	 * Determine memory window on bus and irq if one is needed.
	 */
	resource_list_add(&sdi->sdi_rl, SYS_RES_MEMORY, 0,
	    sdi->sdi_maddr, sdi->sdi_maddr + sdi->sdi_msize - 1, sdi->sdi_msize);

	resource_list_add(&sdi->sdi_rl, SYS_RES_IRQ, 0,
	    sdi->sdi_irq, sdi->sdi_irq, 1);

	device_set_ivars(child, sdi);
	return (child);

}

static device_method_t siba_usb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_attach,	siba_usb_attach),
	DEVMETHOD(device_probe,		siba_usb_probe),

	/* Bus interface */

	DEVMETHOD(bus_activate_resource,	siba_usb_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	siba_usb_deactivate_resource),
	DEVMETHOD(bus_alloc_resource,		siba_usb_alloc_resource),
	DEVMETHOD(bus_release_resource,		siba_usb_release_resource),
	DEVMETHOD(bus_get_resource_list,	siba_usb_get_reslist),
	DEVMETHOD(bus_add_child,		siba_usb_add_child),
	DEVMETHOD(bus_print_child,		siba_usb_print_child),
//	DEVMETHOD(bus_setup_intr,		siba_usb_setup_intr),
//	DEVMETHOD(bus_teardown_intr,		siba_usb_teardown_intr),
	DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),



	{0, 0},
};

static driver_t siba_usb_driver = {
	"siba_usb",
	siba_usb_methods,
	sizeof(struct siba_usb_softc),
};
static devclass_t siba_usb_devclass;

DRIVER_MODULE(siba_usb, siba, siba_usb_driver, siba_usb_devclass, 0, 0);
