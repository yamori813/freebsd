/*-
 * Copyright (c) 2009 Yohanes Nugroho <yohanes@gmail.com>
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
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/types.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <vm/vm.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>
#include <vm/vm_page.h>
#include <vm/vm_extern.h>

#define	_ARM32_BUS_DMA_PRIVATE
#include <machine/armreg.h>
#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "econa_reg.h"
#include "econa_var.h"

static struct resource_spec econa_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_MEMORY,	1,	RF_ACTIVE },
	{ -1, 0 }
};


static struct econa_softc *econa_softc;

unsigned int CPU_clock = 200000000;
unsigned int AHB_clock;
unsigned int APB_clock;

struct fdt_fixup_entry fdt_fixup_table[] = {
	{ NULL, NULL }
};

static int
econa_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "econa,pic"))
		return (ENXIO);

	device_set_desc(dev, "ECONA device bus");
	return (BUS_PROBE_DEFAULT);
}

static void
econa_identify(driver_t *drv, device_t parent)
{

	BUS_ADD_CHILD(parent, 0, "econaarm", 0);
}

struct arm32_dma_range *
bus_dma_get_range(void)
{

	return (NULL);
}

int
bus_dma_get_range_nb(void)
{

	return (0);
}

extern void irq_entry(void);

struct cpu_devs
{
	const char *name;
	int unit;
	bus_addr_t mem_base;
	bus_size_t mem_len;
	int irq0;
	int irq1;
	int irq2;
	int irq3;
	int irq4;
};

struct intc_trigger_t {
	int mode;
	int level;
};

static struct intc_trigger_t intc_trigger_table[] = {
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	{INTC_EDGE_TRIGGER, INTC_FALLING_EDGE},
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	{INTC_TRIGGER_UNKNOWN, INTC_TRIGGER_UNKNOWN},
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_LOW},
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_LOW},
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	{INTC_TRIGGER_UNKNOWN, INTC_TRIGGER_UNKNOWN},
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	{INTC_TRIGGER_UNKNOWN, INTC_TRIGGER_UNKNOWN},
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	{INTC_EDGE_TRIGGER, INTC_FALLING_EDGE},
	{INTC_TRIGGER_UNKNOWN, INTC_TRIGGER_UNKNOWN},
	{INTC_TRIGGER_UNKNOWN, INTC_TRIGGER_UNKNOWN},
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_LOW},
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_LOW},
};

static inline uint32_t
read_4(struct econa_softc *sc, bus_size_t off)
{

	return bus_space_read_4(sc->ec_sys_st, sc->ec_sys_sh, off);
}

static inline void
write_4(struct econa_softc *sc, bus_size_t off, uint32_t val)
{

	return bus_space_write_4(sc->ec_sys_st, sc->ec_sys_sh, off, val);
}

static inline uint32_t
system_read_4(struct econa_softc *sc, bus_size_t off)
{

	return bus_space_read_4(sc->ec_system_st, sc->ec_system_sh, off);
}

static inline void
system_write_4(struct econa_softc *sc, bus_size_t off, uint32_t val)
{

	return bus_space_write_4(sc->ec_system_st, sc->ec_system_sh, off, val);
}



static inline void
econa_set_irq_mode(struct econa_softc * sc, unsigned int irq,
		   unsigned int mode)
{
	unsigned int val;

	if ((mode != INTC_LEVEL_TRIGGER) && (mode != INTC_EDGE_TRIGGER))
		return;

	val =	read_4(sc, INTC_INTERRUPT_TRIGGER_MODE_REG_OFFSET);

	if (mode == INTC_LEVEL_TRIGGER) {
		if (val & (1UL << irq)) {
			val &= ~(1UL << irq);
			write_4(sc, INTC_INTERRUPT_TRIGGER_MODE_REG_OFFSET,
			    val);
		}
	} else {
		if (!(val & (1UL << irq))) {
			val |= (1UL << irq);
			write_4(sc, INTC_INTERRUPT_TRIGGER_MODE_REG_OFFSET,
			    val);
		}
	}
}

/*
 * Configure interrupt trigger level to be Active High/Low
 * or Rising/Falling Edge
 */
static inline void
econa_set_irq_level(struct econa_softc * sc,
    unsigned int irq, unsigned int level)
{
	unsigned int val;

	if ((level != INTC_ACTIVE_HIGH) &&
	    (level != INTC_ACTIVE_LOW) &&
	    (level != INTC_RISING_EDGE) &&
	    (level != INTC_FALLING_EDGE)) {
		return;
	}

	val = read_4(sc, INTC_INTERRUPT_TRIGGER_LEVEL_REG_OFFSET);

	if ((level == INTC_ACTIVE_HIGH) || (level == INTC_RISING_EDGE)) {
		if (val & (1UL << irq)) {
			val &= ~(1UL << irq);
			write_4(sc, INTC_INTERRUPT_TRIGGER_LEVEL_REG_OFFSET,
			    val);
		}
	} else {
		if (!(val & (1UL << irq))) {
			val |= (1UL << irq);
			write_4(sc, INTC_INTERRUPT_TRIGGER_LEVEL_REG_OFFSET,
			    val);
		}
	}
}

static void
get_system_clock(void)
{
	uint32_t sclock = system_read_4(econa_softc, SYSTEM_CLOCK);

	sclock = (sclock >> 6) & 0x03;

	switch (sclock) {
	case 0:
		CPU_clock = 175000000;
		break;
	case 1:
		CPU_clock = 200000000;
		break;
	case 2:
		CPU_clock = 225000000;
		break;
	case 3:
		CPU_clock = 250000000;
		break;
	}
	AHB_clock = CPU_clock >> 1;
	APB_clock = AHB_clock >> 1;
}

static int
econa_attach(device_t dev)
{
	struct econa_softc *sc = device_get_softc(dev);
	int i;

	econa_softc = sc;
	sc->dev = dev;

	if (bus_alloc_resources(dev, econa_spec, sc->ec_res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	sc->ec_sys_st = rman_get_bustag(sc->ec_res[0]);
	sc->ec_sys_sh = rman_get_bushandle(sc->ec_res[0]);
	sc->ec_system_st = rman_get_bustag(sc->ec_res[1]);
	sc->ec_system_sh = rman_get_bushandle(sc->ec_res[1]);

	write_4(sc, INTC_INTERRUPT_CLEAR_EDGE_TRIGGER_REG_OFFSET, 0xffffffff);

	write_4(sc, INTC_INTERRUPT_MASK_REG_OFFSET, 0xffffffff);

	write_4(sc, INTC_FIQ_MODE_SELECT_REG_OFFSET, 0);

	/*initialize irq*/
	for (i = 0; i < 32; i++) {
		if (intc_trigger_table[i].mode != INTC_TRIGGER_UNKNOWN) {
			econa_set_irq_mode(sc,i, intc_trigger_table[i].mode);
			econa_set_irq_level(sc, i, intc_trigger_table[i].level);
		}
	}

	get_system_clock();

	return (0);
}

static struct resource *
econa_alloc_resource(device_t dev, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct econa_softc *sc = device_get_softc(dev);
	struct resource_list_entry *rle;
	struct econa_ivar *ivar = device_get_ivars(child);
	struct resource_list *rl = &ivar->resources;

	if (device_get_parent(child) != dev)
		return (BUS_ALLOC_RESOURCE(device_get_parent(dev), child,
			   type, rid, start, end, count, flags));

	rle = resource_list_find(rl, type, *rid);
	if (rle == NULL) {
		return (NULL);
	}
	if (rle->res)
		panic("Resource rid %d type %d already in use", *rid, type);
	if (RMAN_IS_DEFAULT_RANGE(start, end)) {
		start = rle->start;
		count = ulmax(count, rle->count);
		end = ulmax(rle->end, start + count - 1);
	}
	switch (type)
	{
	case SYS_RES_IRQ:
		rle->res = rman_reserve_resource(&sc->ec_irq_rman,
		    start, end, count, flags, child);
		break;
	case SYS_RES_MEMORY:
		rle->res = rman_reserve_resource(&sc->ec_mem_rman,
		    start, end, count, flags, child);
		if (rle->res != NULL) {
			rman_set_bustag(rle->res, arm_base_bs_tag);
			rman_set_bushandle(rle->res, start);
		}
		break;
	}
	if (rle->res) {
		rle->start = rman_get_start(rle->res);
		rle->end = rman_get_end(rle->res);
		rle->count = count;
		rman_set_rid(rle->res, *rid);
	}
	return (rle->res);
}

static struct resource_list *
econa_get_resource_list(device_t dev, device_t child)
{
	struct econa_ivar *ivar;
	ivar = device_get_ivars(child);
	return (&(ivar->resources));
}

static int
econa_release_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	struct resource_list *rl;
	struct resource_list_entry *rle;

	rl = econa_get_resource_list(dev, child);
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
econa_setup_intr(device_t dev, device_t child,
    struct resource *ires, int flags, driver_filter_t *filt,
    driver_intr_t *intr, void *arg, void **cookiep)
{
	int error;

	if (rman_get_start(ires) == ECONA_IRQ_SYSTEM && filt == NULL)
		panic("All system interrupt ISRs must be FILTER");

	error = BUS_SETUP_INTR(device_get_parent(dev), child, ires, flags,
	    filt, intr, arg, cookiep);
	if (error)
		return (error);

	return (0);
}

static int
econa_teardown_intr(device_t dev, device_t child, struct resource *res,
    void *cookie)
{

	return (BUS_TEARDOWN_INTR(device_get_parent(dev), child, res, cookie));
}

static int
econa_activate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *r)
{

	return (rman_activate_resource(r));
}

static int
econa_print_child(device_t dev, device_t child)
{
	struct econa_ivar *ivars;
	struct resource_list *rl;
	int retval = 0;

	ivars = device_get_ivars(child);
	rl = &ivars->resources;

	retval += bus_print_child_header(dev, child);

	retval += resource_list_print_type(rl, "port", SYS_RES_IOPORT, "%#jx");
	retval += resource_list_print_type(rl, "mem", SYS_RES_MEMORY, "%#jx");
	retval += resource_list_print_type(rl, "irq", SYS_RES_IRQ, "%jd");
	if (device_get_flags(dev))
		retval += printf(" flags %#x", device_get_flags(dev));

	retval += bus_print_child_footer(dev, child);

	return (retval);
}

void
arm_mask_irq(uintptr_t nb)
{
	unsigned int value;

	value = read_4(econa_softc,INTC_INTERRUPT_MASK_REG_OFFSET) | 1<<nb;
	write_4(econa_softc, INTC_INTERRUPT_MASK_REG_OFFSET, value);
}

void
arm_unmask_irq(uintptr_t nb)
{
	unsigned int value;

	value = read_4(econa_softc,
	    INTC_INTERRUPT_CLEAR_EDGE_TRIGGER_REG_OFFSET) | (1 << nb);
	write_4(econa_softc,
	    INTC_INTERRUPT_CLEAR_EDGE_TRIGGER_REG_OFFSET, value);
	value = read_4(econa_softc, INTC_INTERRUPT_MASK_REG_OFFSET)& ~(1 << nb);
	write_4(econa_softc, INTC_INTERRUPT_MASK_REG_OFFSET, value);
}

int
arm_get_next_irq(int x)
{
	int irq;

	irq = read_4(econa_softc, INTC_INTERRUPT_STATUS_REG_OFFSET) &
	    ~(read_4(econa_softc, INTC_INTERRUPT_MASK_REG_OFFSET));

	if (irq!=0) {
		return (ffs(irq) - 1);
	}

	return (-1);
}

void
cpu_reset(void)
{
	uint32_t control;

	control = system_read_4(econa_softc, RESET_CONTROL);
	control |= GLOBAL_RESET;
	system_write_4(econa_softc, RESET_CONTROL, control);
	control = system_read_4(econa_softc, RESET_CONTROL);
	control &= (~(GLOBAL_RESET));
	system_write_4(econa_softc, RESET_CONTROL, control);
	while (1);
}



void
power_on_network_interface(void)
{
	uint32_t cfg_reg;
	int ii;

	cfg_reg =  system_read_4(econa_softc, RESET_CONTROL);
	cfg_reg |= NET_INTERFACE_RESET;
	/* set reset bit to HIGH active; */
	system_write_4(econa_softc, RESET_CONTROL, cfg_reg);

	/*pulse delay */
	for (ii = 0; ii < 0xFFF; ii++)
		DELAY(100);
	/* set reset bit to LOW active; */
	cfg_reg =  system_read_4(econa_softc, RESET_CONTROL);
	cfg_reg &= ~(NET_INTERFACE_RESET);
	system_write_4(econa_softc, RESET_CONTROL, cfg_reg);

	/*pulse delay */
	for (ii = 0; ii < 0xFFF; ii++)
		DELAY(100);
	cfg_reg = system_read_4(econa_softc, RESET_CONTROL);
	cfg_reg |= NET_INTERFACE_RESET;
	/* set reset bit to HIGH active; */
	system_write_4(econa_softc, RESET_CONTROL, cfg_reg);
}

unsigned int
get_tclk(void)
{

	return CPU_clock;
}

static int
fdt_pic_decode_ic(phandle_t node, pcell_t *intr, int *interrupt, int *trig,
    int *pol)
{
	if (!fdt_is_compatible(node, "str,pic"))
		return (ENXIO);

	*interrupt = fdt32_to_cpu(intr[0]);
	*trig = INTR_TRIGGER_CONFORM;
	*pol = INTR_POLARITY_CONFORM;
	return (0);
}

fdt_pic_decode_t fdt_pic_table[] = {
	&fdt_pic_decode_ic,
	NULL
};

static device_method_t econa_methods[] = {
	DEVMETHOD(device_probe,		econa_probe),
	DEVMETHOD(device_attach,		econa_attach),
	DEVMETHOD(device_identify,		econa_identify),
	DEVMETHOD(bus_alloc_resource,		econa_alloc_resource),
	DEVMETHOD(bus_setup_intr,		econa_setup_intr),
	DEVMETHOD(bus_teardown_intr,		econa_teardown_intr),
	DEVMETHOD(bus_activate_resource,	econa_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_get_resource_list,	econa_get_resource_list),
	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_release_resource,	econa_release_resource),
	DEVMETHOD(bus_print_child,		econa_print_child),
	{0, 0},
};

static driver_t econa_driver = {
	"econaarm",
	econa_methods,
	sizeof(struct econa_softc),
};
static devclass_t econa_devclass;

DRIVER_MODULE(econaarm, simplebus, econa_driver, econa_devclass, 0, 0);
