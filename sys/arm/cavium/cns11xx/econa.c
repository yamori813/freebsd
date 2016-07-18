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

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/types.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <vm/vm.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>
#include <vm/vm_page.h>
#include <vm/vm_extern.h>

#define	_ARM32_BUS_DMA_PRIVATE
#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "econa_reg.h"
#include "econa_var.h"

#ifndef INTRNG
static void ec_intc_eoi(void *);
#else
static int econa_pic_attach(struct econa_softc *sc);
#endif

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

struct intc_trigger_t {
	int mode;
	int level;
};

// Datasheet P42
static struct intc_trigger_t intc_trigger_table[] = {
	// Bit[0]: Timer#1
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	// Bit[1]: Timer#2
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	// Bit[2]: Clock and Power Management
	{INTC_EDGE_TRIGGER, INTC_FALLING_EDGE},
	// Bit[3]: Watch Dog Timer
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	// Bit[4]: GPIO
	{INTC_TRIGGER_UNKNOWN, INTC_TRIGGER_UNKNOWN},
	// Bit[5]: PCI External Interrupt 0
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_LOW},
	// Bit[6]: PCI External Interrupt 1
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_LOW},
	// Bit[7]: PCI External Interrupt 2
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	// Bit[8]: AHB-to-PCI Bridge Status
	{INTC_TRIGGER_UNKNOWN, INTC_TRIGGER_UNKNOWN},
	// Bit[9]: Reserved
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	// Bit[10]: UART
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	// Bit[11]: Generic DMA Terminal Counter
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	// Bit[12]: Generic DMA Error
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	// Bit[13]: PCMCIA
	{INTC_TRIGGER_UNKNOWN, INTC_TRIGGER_UNKNOWN},
	// Bit[14]: RTC
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	// Bit[15]: External Interrupt
	{INTC_EDGE_TRIGGER, INTC_FALLING_EDGE},
	// Bit[16]: Reserved
	{INTC_TRIGGER_UNKNOWN, INTC_TRIGGER_UNKNOWN},
	// Bit[17]: Reserved
	{INTC_TRIGGER_UNKNOWN, INTC_TRIGGER_UNKNOWN},
	// Bit[18]: Switch Controller
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_HIGH},
	// Bit[19]: Switch DMA TSTC (To-Switch-Tx-Complete)
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	// Bit[20]: Switch DMA FSRC (Fm-Switch-Rx-Complete)
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	// Bit[21]: Switch DMA TSQE(To-Switch-Queue-Empty)
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	// Bit[22]: Switch DMA FSQE (Fm-Switch-Queue-Empty)
	{INTC_EDGE_TRIGGER, INTC_RISING_EDGE},
	// Bit[23]: USB 1.1 host controller
	{INTC_LEVEL_TRIGGER, INTC_ACTIVE_LOW},
	// Bit[24]: USB 2.0 host controller
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
econa_set_irq_mode(struct econa_softc * sc)
{
	unsigned int val;
	int i;

	val = 0;
	for (i = 0; i < INTC_NIRQS; i++) {
		if(intc_trigger_table[i].mode == INTC_EDGE_TRIGGER) {
			val |= (1UL << i);
		}
	}
	write_4(sc, INTC_INTERRUPT_TRIGGER_MODE_REG_OFFSET, val);
}

/*
 * Configure interrupt trigger level to be Active High/Low
 * or Rising/Falling Edge
 */
static inline void
econa_set_irq_level(struct econa_softc * sc)
{
	unsigned int val;
	int i;

	val = 0;
	for (i = 0; i < INTC_NIRQS; i++) {
		if(intc_trigger_table[i].level == INTC_FALLING_EDGE ||
		    intc_trigger_table[i].level == INTC_ACTIVE_LOW) {
			val |= (1UL << i);
		}
	}
	write_4(sc, INTC_INTERRUPT_TRIGGER_LEVEL_REG_OFFSET, val);
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

#ifndef INTRNG
	arm_post_filter = ec_intc_eoi;
#else
	econa_pic_attach(sc);
#endif

	write_4(sc, INTC_INTERRUPT_CLEAR_EDGE_TRIGGER_REG_OFFSET, 0xffffffff);

	write_4(sc, INTC_INTERRUPT_MASK_REG_OFFSET, 0xffffffff);

	write_4(sc, INTC_FIQ_MODE_SELECT_REG_OFFSET, 0);

	/*initialize irq*/
	econa_set_irq_mode(sc);
	econa_set_irq_level(sc);

	get_system_clock(dev);

	return (0);
}

#ifndef INTRNG
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

	write_4(econa_softc,
	    INTC_INTERRUPT_CLEAR_EDGE_TRIGGER_REG_OFFSET, 1 << nb);
	value = read_4(econa_softc, INTC_INTERRUPT_MASK_REG_OFFSET)& ~(1 << nb);
	write_4(econa_softc, INTC_INTERRUPT_MASK_REG_OFFSET, value);
}

int
arm_get_next_irq(int x)
{
	int irq;

	irq = read_4(econa_softc, INTC_INTERRUPT_STATUS_REG_OFFSET);

	if (irq!=0) {
		return (ffs(irq) - 1);
	}

	return (-1);
}
#endif

#ifdef INTRNG
static void
econa_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	u_int irq = ((struct econa_irqsrc *)isrc)->ec_irq;
	unsigned int value;

	write_4(econa_softc,
	    INTC_INTERRUPT_CLEAR_EDGE_TRIGGER_REG_OFFSET, 1 << irq);

	value = read_4(econa_softc, INTC_INTERRUPT_MASK_REG_OFFSET)& ~(1 << irq);
	write_4(econa_softc, INTC_INTERRUPT_MASK_REG_OFFSET, value);
}

static void
econa_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	u_int irq = ((struct econa_irqsrc *)isrc)->ec_irq;
	unsigned int value;

	value = read_4(econa_softc,INTC_INTERRUPT_MASK_REG_OFFSET) | 1 << irq;
	write_4(econa_softc, INTC_INTERRUPT_MASK_REG_OFFSET, value);
}

static int
econa_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct intr_map_data_fdt *daf;
	struct econa_softc *sc;

	if (data->type != INTR_MAP_DATA_FDT)
		return (ENOTSUP);

	daf = (struct intr_map_data_fdt *)data;

	if (daf->ncells != 1 || daf->cells[0] >= INTC_NIRQS)
		return (EINVAL);

	sc = device_get_softc(dev);
	*isrcp = &sc->ec_isrcs[daf->cells[0]].ec_isrc;
	return (0);
}

static void
econa_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	arm_irq_memory_barrier(0);
	econa_disable_intr(dev, isrc);
}

static void
econa_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	arm_irq_memory_barrier(0);
	econa_enable_intr(dev, isrc);
}

static void
econa_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
	u_int irq = ((struct econa_irqsrc *)isrc)->ec_irq;
	
	arm_irq_memory_barrier(0);
	write_4(econa_softc, INTC_INTERRUPT_CLEAR_EDGE_TRIGGER_REG_OFFSET,
	    1 << irq);
}

static int
econa_intr(void *arg)
{
	uint32_t irq;
	struct econa_softc *sc = arg;
	unsigned int value;

	irq = ffs(read_4(econa_softc, INTC_INTERRUPT_STATUS_REG_OFFSET)) - 1;

	if(intr_isrc_dispatch(&sc->ec_isrcs[irq].ec_isrc,
	    curthread->td_intr_frame) != 0) {

		value = read_4(econa_softc,INTC_INTERRUPT_MASK_REG_OFFSET) | 1 << irq;
		write_4(econa_softc, INTC_INTERRUPT_MASK_REG_OFFSET, value);
		write_4(econa_softc, INTC_INTERRUPT_CLEAR_EDGE_TRIGGER_REG_OFFSET,
		    1 << irq);
		device_printf(sc->dev, "Stray irq %u disabled\n", irq);
	}

	arm_irq_memory_barrier(irq); /* XXX */

	return (FILTER_HANDLED);
}

static int
econa_pic_attach(struct econa_softc *sc)
{
	struct intr_pic *pic;
	int error;
	uint32_t irq;
	const char *name;
	intptr_t xref;

	name = device_get_nameunit(sc->dev);
	for (irq = 0; irq < INTC_NIRQS; irq++) {
		sc->ec_isrcs[irq].ec_irq = irq;

		error = intr_isrc_register(&sc->ec_isrcs[irq].ec_isrc,
		    sc->dev, 0, "%s,%u", name, irq);
		if (error != 0)
			return (error);
	}

	xref = OF_xref_from_node(ofw_bus_get_node(sc->dev));
	pic = intr_pic_register(sc->dev, xref);
	if (pic == NULL)
		return (ENXIO);

	return (intr_pic_claim_root(sc->dev, xref, econa_intr, sc, 0));
}

#else
static void
ec_intc_eoi(void *data)
{
	int nb = (int)data;

	write_4(econa_softc, INTC_INTERRUPT_CLEAR_EDGE_TRIGGER_REG_OFFSET,
	    1 << nb);

}
#endif

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

#ifndef INTRNG
static int
fdt_pic_decode_ic(phandle_t node, pcell_t *intr, int *interrupt, int *trig,
    int *pol)
{
	if (!fdt_is_compatible(node, "econa,pic"))
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
#endif

static device_method_t econa_methods[] = {
	DEVMETHOD(device_probe,		econa_probe),
	DEVMETHOD(device_attach,	econa_attach),
#ifdef INTRNG
	DEVMETHOD(pic_disable_intr,     econa_disable_intr),
	DEVMETHOD(pic_enable_intr,      econa_enable_intr),
	DEVMETHOD(pic_map_intr,         econa_map_intr),
	DEVMETHOD(pic_post_filter,      econa_post_filter),
	DEVMETHOD(pic_post_ithread,     econa_post_ithread),
	DEVMETHOD(pic_pre_ithread,      econa_pre_ithread),
#endif

	{0, 0},
};

static driver_t econa_driver = {
	"econaarm",
	econa_methods,
	sizeof(struct econa_softc),
};
static devclass_t econa_devclass;

DRIVER_MODULE(econaarm, simplebus, econa_driver, econa_devclass, 0, 0);
