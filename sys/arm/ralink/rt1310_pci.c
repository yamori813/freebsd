/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2018 Hiroki Mori
 * Copyright (c) 2012-2017 Oleksandr Tymoshenko <gonzo@freebsd.org>
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
 * This code base on versatile/versatile_pci.c.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#define _ARM32_BUS_DMA_PRIVATE
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/watchdog.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <dev/pci/pcib_private.h>
#include "pcib_if.h"

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_pci.h>

#include <machine/bus.h>
#include <machine/fdt.h>

#define	RT1310ID			0x2005de31

#define	MSK(n)                    	((1 << (n)) - 1)

#define	PCI_ACCESS_READ			0
#define	PCI_ACCESS_WRITE		1

#define	PCI_CFGADDR_REGNUM_SHF		2
#define	PCI_CFGADDR_REGNUM_MSK		(MSK(6) << PCI_CFGADDR_REGNUM_SHF)
#define	PCI_CFGADDR_FUNCTNUM_SHF	8
#define	PCI_CFGADDR_FUNCTNUM_MSK	(MSK(3) << PCI_CFGADDR_FUNCTNUM_SHF)
#define	PCI_CFGADDR_DEVNUM_SHF		11
#define	PCI_CFGADDR_DEVNUM_MSK		(MSK(5) << PCI_CFGADDR_DEVNUM_SHF)
#define	PCI_CFGADDR_BUSNUM_SHF		16
#define	PCI_CFGADDR_BUSNUM_MSK		(MSK(8) << PCI_CFGADDR_BUSNUM_SHF)
#define	PCI_CFGADDR_CONFIGEN_SHF	31
#define	PCI_CFGADDR_CONFIGEN_MSK	(MSK(1) << PCI_CFGADDR_CONFIGEN_SHF)
#define	PCI_CFGADDR_CONFIGEN_BIT	PCI_CFGADDR_CONFIGEN_MSK

#define	PCI_A2P_BRGM_OFS		(0x00)
#define	PCI_A2P_BAR0_OFS		(0x04)
#define	PCI_A2P_UPATU_OFS		(0x14)
#define	PCI_A2P_DOWNATU_OFS		(0x18)
#define	PCI_A2P_T_RETRY_OFS		(0x1c)
#define	PCI_A2P_CFGADDR_OFS		(0x20)
#define	PCI_A2P_CFGDATA_OFS		(0x24)
#define	PCI_A2P_IER_OFS			(0x28)
#define	PCI_A2P_ISR_OFS			(0x2c)

#define	PCI_MEM_BASE			0
#define	PCI_IO_BASE			1
#define	PCI_BR_BASE			2
#define MEM_REGIONS			3

#define	PCI_NPREFETCH_WINDOW		0x19ce0000
#define	PCI_NPREFETCH_SIZE		0x04000000
#define	PCI_IO_WINDOW			0x1dce0000
#define	PCI_IO_SIZE			0x00800000

#define	RT1310_PCI_IRQ_START		18
#define	RT1310_PCI_IRQ_END		21

#ifdef DEBUG
#define dprintf(fmt, args...) do { printf("%s(): ", __func__);   \
    printf(fmt,##args); } while (0)
#else
#define dprintf(fmt, args...)
#endif

#define	rt1310_pci_mem_read_4(reg)	\
	bus_space_read_4(sc->bst_mem, sc->bsh_mem, (reg))
#define	rt1310_pci_mem_write_4(reg, val)	\
	bus_space_write_4(sc->bst_mem, sc->bsh_mem, (reg), (val))

#define	rt1310_pci_io_read_4(reg)	\
	bus_space_read_4(sc->bst_io, sc->bsh_io, (reg))
#define	rt1310_pci_io_write_4(reg, val)	\
	bus_space_write_4(sc->bst_io, sc->bsh_io, (reg), (val))

#define	rt1310_pci_br_read_4(reg)	\
	bus_space_read_4(sc->bst_br, sc->bsh_br, (reg))
#define	rt1310_pci_br_write_4(reg, val)	\
	bus_space_write_4(sc->bst_br, sc->bsh_br, (reg), (val))
#define	rt1310_pci_br_write_1(reg, val)	\
	bus_space_write_1(sc->bst_br, sc->bsh_br, (reg), (val))

struct rt1310_pci_softc {
	struct resource*	mem_res[MEM_REGIONS + 4];

	bus_space_tag_t		bst_mem;
	bus_space_handle_t	bsh_mem;
	bus_space_tag_t		bst_io;
	bus_space_handle_t	bsh_io;
	bus_space_tag_t		bst_br;
	bus_space_handle_t	bsh_br;

	/* Bus part */
	int			busno;
	struct rman		io_rman;
	struct rman		irq_rman;
	struct rman		mem_rman;

	struct mtx		mtx;
	struct ofw_bus_iinfo	pci_iinfo;
};

static struct resource_spec rt1310_pci_mem_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ SYS_RES_MEMORY, 1, RF_ACTIVE },
	{ SYS_RES_MEMORY, 2, RF_ACTIVE },
/*
	{ SYS_RES_IRQ, 0, RF_ACTIVE},
	{ SYS_RES_IRQ, 1, RF_ACTIVE},
	{ SYS_RES_IRQ, 2, RF_ACTIVE},
	{ SYS_RES_IRQ, 3, RF_ACTIVE},
*/
	{ -1, 0, 0 }
};

int get_cpuid(void);
static uint32_t rt1310_pci_read_config(device_t, u_int, u_int, u_int, u_int, int);
static void rt1310_pci_write_config(device_t, u_int, u_int, u_int, u_int, uint32_t, int);

static int
rt1310_pci_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_is_compatible(dev, "fv,rt1310-pci")) {
		device_set_desc(dev, "FV PCI controller");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
rt1310_pci_attach(device_t dev)
{
	struct rt1310_pci_softc *sc = device_get_softc(dev);
	int err;
	phandle_t node;

	node = ofw_bus_get_node(dev);

	/* Request memory resources */
	err = bus_alloc_resources(dev, rt1310_pci_mem_spec,
		sc->mem_res);
	if (err) {
		device_printf(dev, "Error: could not allocate memory resources\n");
		return (ENXIO);
	}

	sc->bst_mem = rman_get_bustag(sc->mem_res[PCI_MEM_BASE]);
	sc->bsh_mem = rman_get_bushandle(sc->mem_res[PCI_MEM_BASE]);
	sc->bst_io = rman_get_bustag(sc->mem_res[PCI_IO_BASE]);
	sc->bsh_io = rman_get_bushandle(sc->mem_res[PCI_IO_BASE]);
	sc->bst_br = rman_get_bustag(sc->mem_res[PCI_BR_BASE]);
	sc->bsh_br = rman_get_bushandle(sc->mem_res[PCI_BR_BASE]);

	/* Prepare resource managers */
	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "rt1310 PCI memory window";
	if (rman_init(&sc->mem_rman) != 0 || 
	    rman_manage_region(&sc->mem_rman, PCI_NPREFETCH_WINDOW, 
		PCI_NPREFETCH_WINDOW + PCI_NPREFETCH_SIZE - 1) != 0) {
		panic("rt1310_pci_attach: failed to set up memory rman");
	}

	sc->io_rman.rm_type = RMAN_ARRAY;
	sc->io_rman.rm_descr = "rt1310 PCI IO window";
	if (rman_init(&sc->io_rman) != 0 || 
	    rman_manage_region(&sc->io_rman, PCI_IO_WINDOW, 
		PCI_IO_WINDOW + PCI_IO_SIZE - 1) != 0) {
		panic("rt1310_pci_attach: failed to set up I/O rman");
	}

	sc->irq_rman.rm_type = RMAN_ARRAY;
	sc->irq_rman.rm_descr = "rt1310 PCI IRQs";
	if (rman_init(&sc->irq_rman) != 0 ||
	    rman_manage_region(&sc->irq_rman, RT1310_PCI_IRQ_START, 
	        RT1310_PCI_IRQ_END) != 0) {
		panic("rt1310_pci_attach: failed to set up IRQ rman");
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), "rt1310pci",
	    MTX_SPIN);

	if (get_cpuid() != RT1310ID) {
		device_printf(dev, "Error: not support PCI bus\n");
		return (ENXIO);
	}

	DELAY(1000*100);
	mtx_lock_spin(&sc->mtx);
	rt1310_pci_br_write_4(PCI_A2P_UPATU_OFS, 0x00000000);
	rt1310_pci_br_write_4(PCI_A2P_DOWNATU_OFS, 0x00000000);
	rt1310_pci_br_write_4(PCI_A2P_BAR0_OFS, 0x00000006);
	rt1310_pci_br_write_4(PCI_A2P_T_RETRY_OFS, 0x000000ff);
	rt1310_pci_br_write_4(PCI_A2P_BRGM_OFS, 0x00000001);
	DELAY(1000*100);
	mtx_unlock_spin(&sc->mtx);

	rt1310_pci_write_config(dev, 0, 0, 0, PCIR_BAR(0), 0x40000000, 4);
	rt1310_pci_write_config(dev, 0, 0, 0, PCIR_BAR(1), 0x0, 4);
	rt1310_pci_write_config(dev, 0, 0, 0, PCIR_BAR(2), 0x0, 4);

	rt1310_pci_write_config(dev, 0, 0, 0, PCIR_CACHELNSZ, 0x03, 1);
	rt1310_pci_write_config(dev, 0, 0, 0, PCIR_LATTIMER, 0x10, 1);
	/* Enable PCI bridge. */
	uint32_t val;
	val = rt1310_pci_read_config(dev, 0, 0, 0, PCIR_COMMAND, 2);
/*
	val |= PCIM_CMD_SERRESPEN | PCIM_CMD_BUSMASTEREN |
	    PCIM_CMD_MEMEN | PCIM_CMD_PORTEN;
*/
	val |= (PCIM_CMD_MEMEN | PCIM_CMD_PORTEN);
	rt1310_pci_write_config(dev, 0, 0, 0, PCIR_COMMAND, val, 2);
	rt1310_pci_write_config(dev, 0, 0, 0, PCIR_INTLINE, 0, 1);

	ofw_bus_setup_iinfo(node, &sc->pci_iinfo, sizeof(cell_t));

	device_add_child(dev, "pci", -1);
	return (bus_generic_attach(dev));
}

static int
rt1310_pci_read_ivar(device_t dev, device_t child, int which,
    uintptr_t *result)
{
	struct rt1310_pci_softc *sc = device_get_softc(dev);

	switch (which) {
	case PCIB_IVAR_DOMAIN:
		*result = 0;
		return (0);
	case PCIB_IVAR_BUS:
		*result = sc->busno;
		return (0);
	}

	return (ENOENT);
}

static int
rt1310_pci_write_ivar(device_t dev, device_t child, int which,
    uintptr_t result)
{
	struct rt1310_pci_softc * sc = device_get_softc(dev);

	switch (which) {
	case PCIB_IVAR_BUS:
		sc->busno = result;
		return (0);
	}

	return (ENOENT);
}

static struct resource *
rt1310_pci_alloc_resource(device_t bus, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{

	struct rt1310_pci_softc *sc = device_get_softc(bus);
	struct resource *rv;
	struct rman *rm;

	dprintf("Alloc resources %d, %08llx..%08llx, %llu\n",
	    type, start, end, count);

	switch (type) {
	case SYS_RES_IOPORT:
		rm = &sc->io_rman;
		break;
	case SYS_RES_IRQ:
		rm = NULL;
		break;
	case SYS_RES_MEMORY:
		rm = &sc->mem_rman;
		break;
	default:
		return (NULL);
	}

	if (rm == NULL)
		return (BUS_ALLOC_RESOURCE(device_get_parent(bus),
		    child, type, rid, start, end, count, flags));

	rv = rman_reserve_resource(rm, start, end, count, flags, child);
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

static int
rt1310_pci_activate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *r)
{
	vm_offset_t vaddr;
	int res;

	switch(type) {
	case SYS_RES_MEMORY:
	case SYS_RES_IOPORT:
		vaddr = (vm_offset_t)pmap_mapdev(rman_get_start(r),
				rman_get_size(r));
		rman_set_bushandle(r, vaddr);
		rman_set_bustag(r, fdtbus_bs_tag);
		res = rman_activate_resource(r);
		break;
	case SYS_RES_IRQ:
		res = (BUS_ACTIVATE_RESOURCE(device_get_parent(bus),
		    child, type, rid, r));
		break;
	default:
		res = ENXIO;
		break;
	}

	return (res);
}

static int
rt1310_pci_setup_intr(device_t bus, device_t child, struct resource *ires,
	    int flags, driver_filter_t *filt, driver_intr_t *handler,
	    void *arg, void **cookiep)
{

	return BUS_SETUP_INTR(device_get_parent(bus), bus, ires, flags,
	    filt, handler, arg, cookiep);
}

static int
rt1310_pci_teardown_intr(device_t dev, device_t child, struct resource *ires,
    void *cookie)
{

	return BUS_TEARDOWN_INTR(device_get_parent(dev), dev, ires, cookie);
}

static int
rt1310_pci_maxslots(device_t dev)
{

	return (PCI_SLOTMAX);
}

static int
rt1310_pci_route_interrupt(device_t bus, device_t dev, int pin)
{
	struct rt1310_pci_softc *sc;
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

	device_printf(bus, "could not route pin %d for device %d.%d\n",
	    pin, pci_get_slot(dev), pci_get_function(dev));
	return (PCI_INVALID_IRQ);
}

static uint32_t
rt1310_pci_config_access(device_t dev, char access_type, unsigned char bus_num,
    unsigned int dev_num, unsigned int fn_num, unsigned char where,
    uint32_t *data)
{
	struct rt1310_pci_softc *sc = device_get_softc(dev);

	rt1310_pci_br_write_4(PCI_A2P_CFGADDR_OFS,
	    (bus_num << PCI_CFGADDR_BUSNUM_SHF) |
	    (dev_num << PCI_CFGADDR_DEVNUM_SHF) |
	    (fn_num << PCI_CFGADDR_FUNCTNUM_SHF) |
	    ((where / 4) << PCI_CFGADDR_REGNUM_SHF) |
	    PCI_CFGADDR_CONFIGEN_BIT);
	DELAY(100);
	if (access_type == PCI_ACCESS_WRITE) {
		rt1310_pci_br_write_4(PCI_A2P_CFGDATA_OFS, *data);
	} else {
		*data = rt1310_pci_br_read_4(PCI_A2P_CFGDATA_OFS);
	}

	return 1;
}

static uint32_t
rt1310_pci_read_config(device_t dev, u_int bus, u_int slot, u_int func,
    u_int reg, int bytes)
{
	struct rt1310_pci_softc *sc = device_get_softc(dev);
	uint32_t data;
	uint32_t shift, mask;
	uint32_t addr;

	addr = (bus << 16) | (slot << 11) | (func << 8) | (reg & ~3);

	/* register access is 32-bit aligned */
	shift = (reg & 3) * 8;

	/* Create a mask based on the width, post-shift */
	if (bytes == 2)
		mask = 0xffff;
	else if (bytes == 1)
		mask = 0xff;
	else
		mask = 0xffffffff;

	dprintf("%s: tag (%x, %x, %x) reg %d(%d)\n", __func__, bus, slot, 
	    func, reg, bytes);

	mtx_lock_spin(&sc->mtx);
	rt1310_pci_config_access(dev, PCI_ACCESS_READ, bus, slot, func,
	    reg, &data);
	mtx_unlock_spin(&sc->mtx);

	/* get request bytes from 32-bit word */
	data = (data >> shift) & mask;

	dprintf("%s: read 0x%x\n", __func__, data);

	return (data);
}

static void
rt1310_pci_write_config(device_t dev, u_int bus, u_int slot, u_int func,
    u_int reg, uint32_t data, int bytes)
{

	struct rt1310_pci_softc *sc = device_get_softc(dev);
	uint32_t addr;
	uint32_t val;

	dprintf("%s: tag (%x, %x, %x) reg %d(%d)\n", __func__, bus, slot,
	    func, reg, bytes);

	addr = (bus << 16) | (slot << 11) | (func << 8) | reg;
	mtx_lock_spin(&sc->mtx);
	rt1310_pci_config_access(dev, PCI_ACCESS_READ, bus, slot, func,
	    reg, &val);
	switch (bytes) {
		case 4: 
			val = data;
			break;
		case 2:
			val = (val & ~(0xffff << ((reg & 3) << 3))) |
			    (data << ((reg & 3) << 3));
			break;
		case 1:
			val = (val & ~(0xff << ((reg & 3) << 3))) |
			    (data << ((reg & 3) << 3));
			break;
	}
	rt1310_pci_config_access(dev, PCI_ACCESS_WRITE, bus, slot, func,
	    reg, &val);
	mtx_unlock_spin(&sc->mtx);
}

static device_method_t rt1310_pci_methods[] = {
	DEVMETHOD(device_probe,		rt1310_pci_probe),
	DEVMETHOD(device_attach,	rt1310_pci_attach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	rt1310_pci_read_ivar),
	DEVMETHOD(bus_write_ivar,	rt1310_pci_write_ivar),
	DEVMETHOD(bus_alloc_resource,	rt1310_pci_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_release_resource),
	DEVMETHOD(bus_activate_resource, rt1310_pci_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_setup_intr,	rt1310_pci_setup_intr),
	DEVMETHOD(bus_teardown_intr,	rt1310_pci_teardown_intr),

	/* pcib interface */
	DEVMETHOD(pcib_maxslots,	rt1310_pci_maxslots),
	DEVMETHOD(pcib_read_config,	rt1310_pci_read_config),
	DEVMETHOD(pcib_write_config,	rt1310_pci_write_config),
	DEVMETHOD(pcib_route_interrupt,	rt1310_pci_route_interrupt),
	DEVMETHOD(pcib_request_feature,	pcib_request_feature_allow),

	DEVMETHOD_END
};

static driver_t rt1310_pci_driver = {
	"pcib",
	rt1310_pci_methods,
	sizeof(struct rt1310_pci_softc),
};

static devclass_t rt1310_pci_devclass;

//DRIVER_MODULE(rt1310_pci, simplebus, rt1310_pci_driver, rt1310_pci_devclass, 0, 0);
EARLY_DRIVER_MODULE(rt1310_pci, simplebus, rt1310_pci_driver,
rt1310_pci_devclass, 0, 0, BUS_PASS_BUS);
