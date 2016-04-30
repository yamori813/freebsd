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
 *  SIBA ChipCommon core driver.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: head/sys/dev/siba/siba_cc.c 227848 2011-11-22 21:55:40Z marius $");

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
#include <dev/siba/siba_cc.h>
#include <dev/siba/siba_cc_gpio.h>



struct siba_cc_softc {
	bus_space_tag_t		 sc_bt;
	bus_space_handle_t	 sc_bh;
	bus_addr_t		 sc_maddr;
	bus_size_t		 sc_msize;

	struct intr_event	*sc_eventstab; /* IRQ events structs */
	uint32_t		 sc_eventsflag[10]; /* IRQ events structs */

	struct resource 	*sc_mem;
	struct resource 	*sc_irq;
	struct rman 		 mem_rman;
	struct rman 		 irq_rman;
	int 			 devid;

};


struct siba_cc_devinfo {
	struct resource_list	sdi_rl;
	uint8_t			sdi_unit;	/* core index on bus */
	uint8_t			sdi_irq;
	char 			sdi_name[8];
	bus_addr_t 		sdi_maddr;
	bus_size_t 		sdi_msize;
};


int 	siba_cc_get_int_mask  (device_t);
void 	siba_cc_set_int_mask  (device_t, uint32_t);
int 	siba_cc_get_int_status(device_t);
void 	siba_cc_set_int_status(device_t, uint32_t);

static int	siba_cc_attach(device_t);
static int	siba_cc_probe(device_t);
static device_t
siba_cc_add_child(device_t dev, u_int order, const char *name, int unit);

static int
siba_cc_probe(device_t dev)
{

	if (siba_get_vendor(dev) == SIBA_VID_BROADCOM &&
	    siba_get_device(dev) == SIBA_DEVID_CHIPCOMMON) {
		device_set_desc(dev, "ChipCommon core");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
siba_cc_attach(device_t dev)
{
	struct siba_cc_softc *sc = device_get_softc(dev);
	int rid;
	u_int32_t cap;
	u_int32_t idhi;
	u_int32_t corerev;
	int uarts = 0;
	int i;

	sc->devid = SIBA_DEVID_CHIPCOMMON;
	/*
	 * Allocate the resources which the parent bus has already
	 * determined for us.
	 * TODO: interrupt routing
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
	sc->sc_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_SHAREABLE | RF_ACTIVE);
	if (sc->sc_irq == NULL) {
		device_printf(dev, "unable to allocate irq\n");
		return (ENXIO);
	}

	sc->mem_rman.rm_start = sc->sc_maddr;
	sc->mem_rman.rm_end = sc->sc_maddr + sc->sc_msize - 1;
	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "SiBa ChipCommon core I/O memory addresses";
	if (rman_init(&sc->mem_rman) != 0 ||
	    rman_manage_region(&sc->mem_rman, sc->mem_rman.rm_start, 
		sc->mem_rman.rm_end) != 0)
		    panic("%s: failed to setup MEM rman", __func__);

	sc->irq_rman.rm_start = rman_get_start(sc->sc_irq);
	sc->irq_rman.rm_end = rman_get_start(sc->sc_irq) + 
	    rman_get_size(sc->sc_irq) - 1;
	sc->irq_rman.rm_type = RMAN_ARRAY;
	sc->irq_rman.rm_descr = "SIBA ChipCommon core IRQ";
	/* 
	 * Siba CC IRQ 4, share between UART, GPIO, PLL etc.
	 */
	if (rman_init(&sc->irq_rman) != 0 ||
	    rman_manage_region(&sc->irq_rman, sc->irq_rman.rm_start, 
		sc->irq_rman.rm_end) != 0)
		    panic("%s: failed to setup IRQ rman", __func__);

	idhi = siba_cc_read_4(sc, SIBA_CORE_IDHI);
	corerev = (idhi & SIBA_IDHIGH_RCLO) | 
	    ((idhi & SIBA_IDHIGH_RCHI) >> SIBA_IDHIGH_RCHI_SHIFT);

	device_printf(dev,"Core Revision %d\n", corerev);

	cap = siba_cc_read_4(sc, SIBA_CC_CAPABILITIES);
	uarts = cap & SIBA_CC_CAP_UARTS_MASK;
	device_printf(dev,"Number of UARTs %d\n", uarts);
	if (cap & SIBA_CC_CAP_MIPSEB) 	
		device_printf(dev,"MIPS is in big-endian mode\n");
	device_printf(dev,"UARTs clock select 0x%02x\n", 
	    cap & SIBA_CC_CAP_UCLKSEL);
	if (cap & SIBA_CC_CAP_UINTCLK) {
	 	device_printf(dev,"UART use internal divided clock\n");
	 	device_printf(dev,"UART clock divider %d\n",
	 	    siba_cc_read_4(sc, SIBA_CC_CLOCKDEV) & 
	 	    SIBA_CC_CLOCKDEV_UART );
	}
	if (cap & SIBA_CC_CAP_UARTGPIO) 
		device_printf(dev,"UART use GPIO pins 15:12\n");
	switch ( cap & SIBA_CC_CAP_EXTBUS_MASK )
	{
	    case SIBA_CC_CAP_EXTBUS_NONE:
		device_printf(dev,"No ExtBus present\n");
		break;
	    case SIBA_CC_CAP_EXTBUS_FULL:
		device_printf(dev,"PCMCIA, IDE & Prog\n");
		break;
	    case SIBA_CC_CAP_EXTBUS_PROG:
		device_printf(dev,"ProgIf only\n");
		break;
	    default:
		device_printf(dev,"Unknown ExtBus type\n");
	}
	switch ( (cap & SIBA_CC_CAP_FLASH_MASK) >> SIBA_CC_CAP_FLASH_SHIFT )
	{
	    case SIBA_CC_FLASH_NONE:
		device_printf(dev,"No flash\n");
		break;
	    case SIBA_CC_SERIAL_FLASH_ST:
		device_printf(dev,"ST serial flash\n");
		break;
	    case SIBA_CC_SERIAL_FLASH_AT:
		device_printf(dev,"Atmel serial flash\n");
		break;
	    case SIBA_CC_PARALLEL_FLASH:
		device_printf(dev,"Parallel flash\n");
		break;
	    default:
		device_printf(dev,"Unknown flash type\n");
	}

	device_printf(dev,"Type of PLL 0x%02x\n", 
	    (cap & SIBA_CC_CAP_PLL_MASK) >> SIBA_CC_CAP_PLL_SHIFT);

	if (cap & SIBA_CC_CAP_PWR_CTL) 	
		device_printf(dev,"Have a Power control\n");
	if (cap & SIBA_CC_CAP_JTAGP) 	
		device_printf(dev,"JTAG Master Present\n");
	if (cap & SIBA_CC_CAP_ROM) 	
		device_printf(dev,"Internal boot rom active\n");
	if (cap & SIBA_CC_CAP_BKPLN64) 	
		device_printf(dev,"Backplane is 64 bits \n");
	if (corerev > 19 )
	{
	    if (cap & SIBA_CC_CAP_PMU) 	device_printf(dev,"PMU Present\n");
	    if (corerev > 20 )
		if (cap & SIBA_CC_CAP_ECI) 	
					device_printf(dev,"ECI Present\n");
	}

	/*
	 * XXX: We don`t know who is LED, who is Serial Bus on GPIO 
	 * so we can`t setup powersave PWM for GPIO
	 * TODO: relocate it to GPIO driver and use GPIO flags
	 */
	if (corerev >= 16) {
		siba_cc_write_4(sc, SIBA_CC_GPIO_BASE + SIBA_CC_GPIO_TIMER,
		    SIBA_CC_GPIO_TIMER_DEFAULT);
		siba_cc_write_4(sc, SIBA_CC_GPIO_BASE + 
		    0, 0);
		    /* XXX: BREAK THERE SIBA_CC_GPIO_TIMER_OUT_MASK, 0); */
	}

	bus_generic_probe(dev);
	for(i = 0; i < uarts; i++)   /* add all uarts found on CC  */
		siba_cc_add_child(dev, i+1, "uart", i);
	siba_cc_add_child(dev, 10, "cfi", 0);
	siba_cc_add_child(dev, 100, "gpio", 0);

	siba_cc_set_int_mask(dev, SIBA_CC_INT_UART);

	bus_generic_attach(dev);

	return (0);
}




static struct resource *
siba_cc_alloc_resource(device_t bus, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct resource			*rv;
	struct resource_list		*rl;
	struct resource_list_entry	*rle;
	int				 isdefault, needactivate;
	struct siba_cc_softc		*sc = device_get_softc(bus);

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
	if (type == SYS_RES_MEMORY &&
	    start >= sc->mem_rman.rm_start && end <= sc->mem_rman.rm_end) {

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
siba_cc_activate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *r)
{
	return (BUS_ACTIVATE_RESOURCE(device_get_parent(bus), child, type,
	    rid, r));
}

static int
siba_cc_deactivate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *r)
{
	return (BUS_DEACTIVATE_RESOURCE(device_get_parent(bus), child, type,
	    rid, r));
}

static struct resource_list *
siba_cc_get_reslist(device_t dev, device_t child)
{
	struct siba_cc_devinfo *sdi = device_get_ivars(child);

	return (&sdi->sdi_rl);
}

static int
siba_cc_release_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	struct resource_list *rl;
	struct resource_list_entry *rle;

	rl = siba_cc_get_reslist(dev, child);
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
siba_cc_print_all_resources(device_t dev)
{
	struct siba_cc_devinfo *sdi = device_get_ivars(dev);
	struct resource_list *rl = &sdi->sdi_rl;
	int retval = 0;

	if (STAILQ_FIRST(rl))
		retval += printf(" at");

	retval += resource_list_print_type(rl, "mem", SYS_RES_MEMORY, "%#lx");
	retval += resource_list_print_type(rl, "irq", SYS_RES_IRQ, "%#lx");

	return (retval);
}

static int
siba_cc_print_child(device_t bus, device_t child)
{
	int retval = 0;

	retval += bus_print_child_header(bus, child);
	retval += siba_cc_print_all_resources(child);
	if (device_get_flags(child))
		retval += printf(" flags %#x", device_get_flags(child));
	retval += printf(" on %s\n", device_get_nameunit(bus));

	return (retval);
}

static device_t
siba_cc_add_child(device_t dev, u_int order, const char *name, int unit)
{
	struct siba_cc_softc	*sc = device_get_softc(dev);
	device_t 		child;
	struct siba_cc_devinfo 	*sdi;

	child = device_add_child_ordered(dev, order, name, unit);
	if (child == NULL)
		return (NULL);

	sdi = malloc(sizeof(struct siba_cc_devinfo), M_DEVBUF, M_NOWAIT|M_ZERO);
	if (sdi == NULL)
		return (NULL);




	if (strncmp(name, "uart", 4) == 0) {
		if ( unit < 2 )	{
			sdi->sdi_maddr = sc->sc_maddr + SIBA_CC_UART0 + 
			    (unit * 0x100);
			sdi->sdi_msize   = 8;
			sdi->sdi_irq = sc->irq_rman.rm_start;
		} else {
			/* Not supported */
		}
	} else if (strncmp(name, "jtag", 4) == 0) {
		/* TODO: impement JTAG driver */
	} else if (strncmp(name, "gpio", 4) == 0) {
	    sdi->sdi_maddr = sc->sc_maddr + SIBA_CC_GPIO_BASE;
	    sdi->sdi_msize = SIBA_CC_GPIO_SIZE;
	    sdi->sdi_irq = sc->irq_rman.rm_start;
	} else if (strncmp(name, "cfi", 3) == 0)  {
	    if ( unit == 0 ) {
		BUS_READ_IVAR(device_get_parent(dev), dev, 
		    SIBA_IVAR_CORE_ADDRESS_SPACE2, &sdi->sdi_maddr);
		BUS_READ_IVAR(device_get_parent(dev), dev, 
		    SIBA_IVAR_CORE_ADDRESS_SPACE2_SIZE, &sdi->sdi_msize);
	    } else if ( unit == 1 ) {
		BUS_READ_IVAR(device_get_parent(dev), dev, 
		    SIBA_IVAR_CORE_ADDRESS_SPACE2, &sdi->sdi_maddr);
		BUS_READ_IVAR(device_get_parent(dev), dev, 
		    SIBA_IVAR_CORE_ADDRESS_SPACE2_SIZE, &sdi->sdi_msize);
	    } else {
		/* Not supported */
	    }
	} else{
	    /* Unknown subdevice */
	    sdi->sdi_maddr = 1;
	    sdi->sdi_msize = 1;
	}

	resource_list_init(&sdi->sdi_rl);

	/*
	 * Determine memory window on bus and irq if one is needed.
	 */
	resource_list_add(&sdi->sdi_rl, SYS_RES_MEMORY, 0, sdi->sdi_maddr, 
	    sdi->sdi_maddr + sdi->sdi_msize - 1, sdi->sdi_msize);

	resource_list_add(&sdi->sdi_rl, SYS_RES_IRQ, 0,
	    sdi->sdi_irq, sdi->sdi_irq, 1);

	device_set_ivars(child, sdi);
	return (child);

}

int
siba_cc_get_int_mask(device_t dev)
{
	struct siba_cc_softc *sc = device_get_softc(dev);
	return (siba_cc_read_4(sc, SIBA_CC_INTMASK));
}

void
siba_cc_set_int_mask(device_t dev, uint32_t mask)
{
	struct siba_cc_softc *sc = device_get_softc(dev);
	siba_cc_write_4(sc, SIBA_CC_INTMASK, mask);
	return;
}

int
siba_cc_get_int_status(device_t dev)
{
	struct siba_cc_softc *sc = device_get_softc(dev);
	return (siba_cc_read_4(sc, SIBA_CC_INTSTATUS));
}

void
siba_cc_set_int_status(device_t dev, uint32_t status)
{
	struct siba_cc_softc *sc = device_get_softc(dev);
	siba_cc_write_4(sc, SIBA_CC_INTSTATUS, status);
	return;
}

static int
siba_cc_read_ivar(device_t dev, device_t child, int which, uintptr_t *result)
{
//     struct siba_dev_softc *sd;
//     struct siba_softc *siba;

//     sd = device_get_ivars(child);
//     siba = (struct siba_softc *)sd->sd_bus;
	switch (which) {
	case SIBA_CC_IVAR_XTALFREQ:
		/* BCM5354 CC uart uses 25Mhz clock.
		 *  Slightly shift it to make uart divisor happy
		 */
		*result = 25804800;
		break;
	default:
		return (ENOENT);
	}
	
	return (0);
}

static device_method_t siba_cc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_attach,	siba_cc_attach),
	DEVMETHOD(device_probe,		siba_cc_probe),

	/* Bus interface */
	DEVMETHOD(bus_activate_resource,	siba_cc_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	siba_cc_deactivate_resource),
	DEVMETHOD(bus_alloc_resource,		siba_cc_alloc_resource),
	DEVMETHOD(bus_release_resource,		siba_cc_release_resource),
	DEVMETHOD(bus_get_resource_list,	siba_cc_get_reslist),
	DEVMETHOD(bus_add_child,		siba_cc_add_child),
	DEVMETHOD(bus_print_child,		siba_cc_print_child),
	DEVMETHOD(bus_read_ivar,                siba_cc_read_ivar),
	DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),

	DEVMETHOD_END
};

static driver_t siba_cc_driver = {
	"siba_cc",
	siba_cc_methods,
	sizeof(struct siba_cc_softc),
};
static devclass_t siba_cc_devclass;

DRIVER_MODULE(siba_cc, siba, siba_cc_driver, siba_cc_devclass, 0, 0);

