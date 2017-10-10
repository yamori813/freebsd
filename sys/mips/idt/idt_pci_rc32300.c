/*-
 * Copyright (c) 2007 David Young.
 * Copyright (c) 2007 Oleskandr Tymoshenko.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */
/*-
 * Copyright (c) 2006 Itronix Inc.
 * All rights reserved.
 *
 * Written by Garrett D'Amore for Itronix Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of Itronix Inc. may not be used to endorse
 *    or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ITRONIX INC. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL ITRONIX INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */ 

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>

#include <sys/bus.h>
#include <sys/interrupt.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <dev/pci/pcib_private.h>
#include <dev/fdt/fdt_common.h>

#include <mips/idt/idtreg_rc32300.h>
#include "pcib_if.h"

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_pci.h>

#ifdef IDTPCI_DEBUG
int idt_pci_debug = 1;
#define	IDTPCI_DPRINTF(__fmt, ...)		\
do {						\
	if (idt_pci_debug)			\
		printf((__fmt), __VA_ARGS__);	\
} while (/*CONSTCOND*/0)
#else /* !IDTPCI_DEBUG */
#define	IDTPCI_DPRINTF(__fmt, ...)	do { } while (/*CONSTCOND*/0)
#endif /* IDTPCI_DEBUG */

#define	IDTPCI_TAG_BUS_MASK		0x007f0000
#define	IDTPCI_TAG_DEVICE_MASK		0x00007800
#define	IDTPCI_TAG_FUNCTION_MASK	0x00000300
#define	IDTPCI_TAG_REGISTER_MASK	0x0000007c

#define	IDT_PCI_NIRQS			2

#define REG_READ(sc, o) bus_read_4((sc)->sc_res[0], (o))
#define REG_WRITE(sc, o, v) bus_write_4((sc)->sc_res[0], (o), (v))

static struct resource_spec idt_pci_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_MEMORY,	1,	RF_ACTIVE },
	{ SYS_RES_MEMORY,	2,	RF_ACTIVE },
	{ SYS_RES_MEMORY,	3,	RF_ACTIVE },
	{ SYS_RES_MEMORY,	4,	RF_ACTIVE },
/*	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		1,	RF_ACTIVE },
	{ SYS_RES_IRQ,		2,	RF_ACTIVE },*/
	{ -1, 0 }
};

struct idt_pci_softc {
	device_t		sc_dev;

	int			sc_busno;
	struct rman		sc_mem_rman[2];
	struct rman		sc_io_rman[2];
	struct rman		sc_irq_rman;
	struct resource		*sc_res[8];
	void *			pci_intrhand[IDT_PCI_NIRQS];
	struct intr_event	*sc_eventstab[IDT_PCI_NIRQS];
	struct ofw_bus_iinfo	pci_iinfo;
};

static void
idt_pci_write_config(device_t dev, u_int bus, u_int slot, u_int func, u_int reg,
    uint32_t data, int bytes);
//static int idt_pci_intr(void *);

static uint32_t
idt_pci_make_addr(int bus, int slot, int func, int reg)
{

	return 0x80000000 | (bus << 16) | (slot << 11) | (func << 8) | reg;
}

static int
idt_pci_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "idt,rc32300-pci"))
		return (ENXIO);

	device_set_desc(dev, "IDT RC32300 PCI Interface Controller");

	return (0);
}

static int
idt_pci_attach(device_t dev)
{
	int busno = 0;
	struct idt_pci_softc *sc = device_get_softc(dev);
	unsigned int force_endianess = 0;
//	int		i;
	uint32_t	startaddr;
	intptr_t xref;

	sc->sc_dev = dev;
	sc->sc_busno = busno;

	if (bus_alloc_resources(dev, idt_pci_spec, sc->sc_res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	xref = OF_xref_from_node(ofw_bus_get_node(dev));

	/* TODO: Check for host mode */

#if 0
	/* not work because of u-boot set */
	REG_WRITE(sc, IDT_PCI_ATTR, 0x04);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_DEVVENDOR,
	    0x111d | (0x0204 << 16), 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_REVID, 0x06800001, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_CACHELNSZ, 0x0000ff04, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_BAR(0), 0, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_BAR(1), 0, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_BAR(2), 0, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_BAR(3),
	    0 | PCIM_BAR_IO_SPACE, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_BAR(4), 0, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_BAR(5), 0, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_CIS, 0, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_BIOS, 0, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_CAP_PTR, 0, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_CAP_PTR + 4, 0, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_INTLINE, 0x38080101, 4);
	idt_pci_write_config(dev, 0, 0, 0, PCIR_INTLINE + 4, 0x00008080, 4);
	REG_WRITE(sc, IDT_PCI_CFG_ADDR, 0);
#endif

#ifdef	__MIPSEB__
	force_endianess = IDT_PCI_LBA_FE;
#endif

	/* PCI Memory Space 1 Base Register */
	startaddr = rman_get_start(sc->sc_res[2]);
	REG_WRITE(sc, IDT_PCI_MEMIOB1, startaddr | force_endianess);

	/* PCI Memory Space 2 Base Register */
	startaddr = rman_get_start(sc->sc_res[3]);
	REG_WRITE(sc, IDT_PCI_MEMIOB2, startaddr | force_endianess);

	/* PCI Memory Space 3 Base Register */
	startaddr = rman_get_start(sc->sc_res[4]);
	REG_WRITE(sc, IDT_PCI_MEMIOB3, startaddr | force_endianess);

	/* PCI I/O Space Base Register */
	startaddr = rman_get_start(sc->sc_res[1]);
	REG_WRITE(sc, IDT_PCI_MEMIOB4, startaddr | force_endianess);

	REG_WRITE(sc, IDT_PCI_ATTR, 0x0);

	REG_WRITE(sc, IDT_PCI_CPUB1, 0x0 | force_endianess);
	REG_WRITE(sc, IDT_PCI_CPUB3, 0x0 | force_endianess);

	__asm__ volatile ("sync");

	/* Use KSEG1 to access IO ports for it is uncached */
	sc->sc_io_rman[0].rm_type = RMAN_ARRAY;
	sc->sc_io_rman[0].rm_descr = "IDTPCI I/O Ports window 1";
	if (rman_init(&sc->sc_io_rman[0]) != 0 ||
	    rman_manage_region(&sc->sc_io_rman[0], 
	    rman_get_start(sc->sc_res[1]), rman_get_end(sc->sc_res[1])) != 0) {
//	    0x0, rman_get_size(sc->sc_res[1])) != 0) { // ???
		panic("idt_pci_attach: failed to set up I/O rman");
	}

	/* Use KSEG1 to access PCI memory for it is uncached */
	sc->sc_mem_rman[0].rm_type = RMAN_ARRAY;
	sc->sc_mem_rman[0].rm_descr = "IDTPCI PCI Memory window 1";
	if (rman_init(&sc->sc_mem_rman[0]) != 0 ||
	    rman_manage_region(&sc->sc_mem_rman[0], 
	    rman_get_start(sc->sc_res[4]), rman_get_end(sc->sc_res[4])) != 0) {
		panic("idt_pci_attach: failed to set up memory rman");
	}

	sc->sc_mem_rman[1].rm_type = RMAN_ARRAY;
	sc->sc_mem_rman[1].rm_descr = "IDTPCI PCI Memory window 2";
	if (rman_init(&sc->sc_mem_rman[1]) != 0 ||
	    rman_manage_region(&sc->sc_mem_rman[1], 
	    rman_get_start(sc->sc_res[3]), rman_get_end(sc->sc_res[3])) != 0) {
		panic("idt_pci_attach: failed to set up memory rman");
	}

	sc->sc_irq_rman.rm_type = RMAN_ARRAY;
	sc->sc_irq_rman.rm_descr = "IDTPCI PCI IRQs";
	if (rman_init(&sc->sc_irq_rman) != 0 ||
	    rman_manage_region(&sc->sc_irq_rman, 13, 13
	    ) != 0) {
		panic("idt_pci_attach: failed to set up IRQ rman");
	}

	/* Register ourselves as an interrupt controller */
/*
	if (intr_pic_register(dev, xref) == NULL) {
		device_printf(dev, "could not register PIC\n");
	}
*/
/*
	for (i = 0; i < 3; i++) {
		sc->pci_intrhand[i] = NULL;
		if (bus_setup_intr(dev, sc->sc_res[i+5], INTR_TYPE_MISC,
		    idt_pci_intr, NULL, sc, &sc->pci_intrhand[i])) {
			device_printf(dev, "could not setup intr handler %d\n",
			    i);
		}
	}
*/

	ofw_bus_setup_iinfo(ofw_bus_get_node(dev), &sc->pci_iinfo, 
	    sizeof(cell_t));

	device_add_child(dev, "pci", -1);
	return (bus_generic_attach(dev));
}

static int
idt_pci_maxslots(device_t dev)
{

	return (PCI_SLOTMAX);
}

static uint32_t
idt_pci_read_config(device_t dev, u_int bus, u_int slot, u_int func, u_int reg,
    int bytes)
{
	uint32_t data;
	uint32_t shift, mask;
	bus_addr_t addr;
	struct idt_pci_softc *sc = device_get_softc(dev);

	IDTPCI_DPRINTF("%s: tag (%x, %x, %x) reg %d(%d)\n", __func__, 
			bus, slot, func, reg, bytes);

	addr = idt_pci_make_addr(bus, slot, func, reg);

	REG_WRITE(sc, IDT_PCI_CFG_ADDR, addr);
	__asm__ volatile ("sync");
	data = REG_READ(sc, IDT_PCI_CFG_DATA);

	switch (reg % 4) {
	case 3:
		shift = 24;
		break;
	case 2:
		shift = 16;
		break;
	case 1:
		shift = 8;
		break;
	default:
		shift = 0;
		break;
	}	

	switch (bytes) {
	case 1:
		mask = 0xff;
		data = (data >> shift) & mask;
		break;
	case 2:
		mask = 0xffff;
		if (reg % 4 == 0)
			data = data & mask;
		else
			data = (data >> 16) & mask;
		break;
	case 4:
		break;
	default:
		panic("%s: wrong bytes count", __func__);
		break;
	}

	__asm__ volatile ("sync");
 	IDTPCI_DPRINTF("%s: read 0x%x\n", __func__, data);

	return (data);
}

static void
idt_pci_write_config(device_t dev, u_int bus, u_int slot, u_int func, u_int reg,
    uint32_t data, int bytes)
{
	bus_addr_t addr;
	uint32_t reg_data;
	uint32_t shift, mask;
	struct idt_pci_softc *sc = device_get_softc(dev);

	IDTPCI_DPRINTF("%s: tag (%x, %x, %x) reg %d(%d) data %08x\n", __func__, 
			bus, slot, func, reg, bytes, data);

	if (bytes != 4) {
		reg_data = idt_pci_read_config(dev, bus, slot, func, reg, 4);

		switch (reg % 4) {
		case 3:
			shift = 24;
			break;
		case 2:
			shift = 16;
			break;
		case 1:
			shift = 8;
			break;
		default:
			shift = 0;
			break;
		}	

		switch (bytes) {
		case 1:
			mask = 0xff;
			data = (reg_data & ~ (mask << shift)) | (data << shift);
			break;
		case 2:
			mask = 0xffff;
			if (reg % 4 == 0)
				data = (reg_data & ~mask) | data;
			else
				data = (reg_data & ~ (mask << shift)) | 
				    (data << shift);
			break;
		case 4:
			break;
		default:
			panic("%s: wrong bytes count", __func__);
			break;
		}
	}

	addr = idt_pci_make_addr(bus, slot, func, reg);

	REG_WRITE(sc, IDT_PCI_CFG_ADDR, addr);
	__asm__ volatile ("sync");
	REG_WRITE(sc, IDT_PCI_CFG_DATA, data);
	__asm__ volatile ("sync");

//	REG_WRITE(sc, IDT_PCI_CFG_ADDR, 0);
//	REG_WRITE(sc, IDT_PCI_CFG_DATA, 0);
}

static int
idt_pci_route_interrupt(device_t bus, device_t dev, int pin)
{
	struct idt_pci_softc *sc;
	struct ofw_pci_register reg;
	uint32_t pintr, mintr[4];
	phandle_t iparent;
	int intrcells;

	sc = device_get_softc(bus);
	pintr = pin;

	bzero(&reg, sizeof(reg));
	reg.phys_hi = (pci_get_bus(dev) << OFW_PCI_PHYS_HI_BUSSHIFT) |
	    (pci_get_slot(dev) << OFW_PCI_PHYS_HI_DEVICESHIFT) |
	    (pci_get_function(dev) << OFW_PCI_PHYS_HI_FUNCTIONSHIFT);

	intrcells = ofw_bus_lookup_imap(ofw_bus_get_node(dev),
	    &sc->pci_iinfo, &reg, sizeof(reg), &pintr, sizeof(pintr),
	    mintr, sizeof(mintr), &iparent);

	if (intrcells) {
		pintr = ofw_bus_map_intr(dev, iparent, intrcells, mintr);
		return (pintr);
	}

	device_printf(bus, "could not route pin %d for device %d.%d %x\n",
	    pin, pci_get_slot(dev), pci_get_function(dev), reg.phys_hi);

	return (PCI_INVALID_IRQ);
}

static int
idt_pci_read_ivar(device_t dev, device_t child, int which, uintptr_t *result)
{
	struct idt_pci_softc *sc;

	sc = device_get_softc(dev);

	switch (which) {
	case PCIB_IVAR_DOMAIN:
		*result = 0;
		return (0);
	case PCIB_IVAR_BUS:
		*result = sc->sc_busno;
		return (0);
	}

	return (ENOENT);
}

static int
idt_pci_write_ivar(device_t dev, device_t child, int which, uintptr_t result)
{
	struct idt_pci_softc * sc;

	sc = device_get_softc(dev);

	switch (which) {
	case PCIB_IVAR_BUS:
		sc->sc_busno = result;
		return (0);
	}
	return (ENOENT);
}

static struct resource *
idt_pci_alloc_resource(device_t dev, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{

	struct idt_pci_softc *sc;
	struct resource *rv = NULL;
	struct rman *rm1, *rm2;

	sc = device_get_softc(dev);

	switch (type) {
	case SYS_RES_IRQ:
//		rm1 = &sc->sc_irq_rman;
		rm1 = NULL;
		break;
	case SYS_RES_MEMORY:
		rm1 = &sc->sc_mem_rman[0];
		rm2 = &sc->sc_mem_rman[1];
		break;
	case SYS_RES_IOPORT:
		rm1 = &sc->sc_io_rman[0];
		rm2 = NULL;
		break;
	default:
		return (NULL);
	}

	if (rm1 == NULL)
		return (BUS_ALLOC_RESOURCE(device_get_parent(dev),
		    child, type, rid, start, end, count, flags));

	rv = rman_reserve_resource(rm1, start, end, count, flags, child);

	/* Try second window if it exists */
	if ((rv == NULL) && (rm2 != NULL))
		rv = rman_reserve_resource(rm2, start, end, count, flags, 
		    child);

	if (rv == NULL)
		return (NULL);

	rman_set_rid(rv, *rid);

	if (flags & RF_ACTIVE) {
		if (bus_activate_resource(child, type, *rid, rv)) {
			rman_release_resource(rv);
			return (NULL);
		}
	} 

	return (rv);
}
#if 0
static void
idt_pci_mask_irq(void *source)
{
}

static void
idt_pci_unmask_irq(void *source)
{
}
#endif

static int
idt_pci_setup_intr(device_t dev, device_t child, struct resource *ires,
    int flags, driver_filter_t *filt, driver_intr_t *handler,
    void *arg, void **cookiep)
{
#if 0
	struct idt_pci_softc *sc;
	struct intr_event *event;
	int irq, error, irqidx;

	sc = device_get_softc(dev);

	irq = rman_get_start(ires);

	irqidx = irq - IRQ_BASE;

	event = sc->sc_eventstab[irqidx];
	if (event == NULL) {
		error = intr_event_create(&event, (void *)irq, 0, irq,
		    idt_pci_mask_irq, idt_pci_unmask_irq, NULL, NULL,
		    "pci intr%d:", irq);

		if (error == 0) {
			sc->sc_eventstab[irqidx] = event;
		} else {
			return (error);
		}
	}

	intr_event_add_handler(event, device_get_nameunit(child), filt,
	    handler, arg, intr_priority(flags), flags, cookiep);

	return (0);
#endif
	return BUS_SETUP_INTR(device_get_parent(dev), dev, ires, flags,
	    filt, handler, arg, cookiep);
}

static int
idt_pci_teardown_intr(device_t dev, device_t child, struct resource *ires,
    void *cookie)
{

//	return (intr_event_remove_handler(cookie));
	return BUS_TEARDOWN_INTR(device_get_parent(dev), dev, ires, cookie);
}
/*
static int
idt_pci_intr(void *arg)
{
	return (FILTER_HANDLED);
}
*/

static device_method_t idt_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		idt_pci_probe),
	DEVMETHOD(device_attach,	idt_pci_attach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD(device_suspend,	bus_generic_suspend),
	DEVMETHOD(device_resume,	bus_generic_resume),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	idt_pci_read_ivar),
	DEVMETHOD(bus_write_ivar,	idt_pci_write_ivar),
	DEVMETHOD(bus_alloc_resource,	idt_pci_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_release_resource),
	DEVMETHOD(bus_activate_resource, bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_setup_intr,	idt_pci_setup_intr),
	DEVMETHOD(bus_teardown_intr,	idt_pci_teardown_intr),

	/* pcib interface */
	DEVMETHOD(pcib_maxslots,	idt_pci_maxslots),
	DEVMETHOD(pcib_read_config,	idt_pci_read_config),
	DEVMETHOD(pcib_write_config,	idt_pci_write_config),
	DEVMETHOD(pcib_route_interrupt,	idt_pci_route_interrupt),
	DEVMETHOD(pcib_request_feature,	pcib_request_feature_allow),

	DEVMETHOD_END
};

static driver_t idt_pci_driver = {
	"pcib",
	idt_pci_methods,
	sizeof(struct idt_pci_softc),
};

static devclass_t idt_pci_devclass;

//DRIVER_MODULE(idt_pci, ofwbus, idt_pci_driver, idt_pci_devclass, 0, 0);
DRIVER_MODULE(idt_pci, simplebus, idt_pci_driver, idt_pci_devclass, 0, 0);
