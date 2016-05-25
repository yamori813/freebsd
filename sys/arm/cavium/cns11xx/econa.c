/*-
 * Copyright (c) 2016 Hiroki Mori
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
get_system_clock(device_t dev)
{
	uint32_t sclock = system_read_4(econa_softc, SYSTEM_CLOCK);

	if (bootverbose) {
		device_printf(dev, "Reset Latch Configuration Register %x\n",
		    sclock);
	}

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

	get_system_clock(dev);

	return (0);
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
	{0, 0},
};

static driver_t econa_driver = {
	"econaarm",
	econa_methods,
	sizeof(struct econa_softc),
};
static devclass_t econa_devclass;

DRIVER_MODULE(econaarm, simplebus, econa_driver, econa_devclass, 0, 0);
