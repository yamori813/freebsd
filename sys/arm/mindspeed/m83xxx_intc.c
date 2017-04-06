/*-
 * Copyright (c) 2015 Hiroki Mori
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
#include <sys/kdb.h>

#define _ARM32_BUS_DMA_PRIVATE
#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/mindspeed/m83xxxreg.h>

#define INTC_NIRQS	64

#include "pic_if.h"

struct m83xxx_irqsrc {
	struct intr_irqsrc      ci_isrc;
	u_int                   ci_irq;
};

struct m83xxx_intc_softc {
	device_t dev;
	struct resource *	ci_res;
	bus_space_tag_t		ci_bst;
	bus_space_handle_t	ci_bsh;
	struct m83xxx_irqsrc	ci_isrcs[INTC_NIRQS];
};

static int m83xxx_intc_probe(device_t);
static int m83xxx_intc_attach(device_t);
static int m83xxx_pic_attach(struct m83xxx_intc_softc *sc);

static struct m83xxx_intc_softc *intc_softc = NULL;

#define	intc_read_4(_sc, _reg)		\
    bus_space_read_4((_sc)->ci_bst, (_sc)->ci_bsh, (_reg))
#define	intc_write_4(_sc, _reg, _val)		\
    bus_space_write_4((_sc)->ci_bst, (_sc)->ci_bsh, (_reg), (_val))

struct m83xxx_irqdef {
	u_int                   ci_trig;
	u_int                   ci_prio;
};

static int
m83xxx_intc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "comcerto,intc"))
		return (ENXIO);

	device_set_desc(dev, "Comcerto Interrupt Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
m83xxx_intc_attach(device_t dev)
{
	struct m83xxx_intc_softc *sc = device_get_softc(dev);
	int rid = 0;

	if (intc_softc)
		return (ENXIO);

	sc->dev = dev;

	sc->ci_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, 
	    RF_ACTIVE);
	if (!sc->ci_res) {
		device_printf(dev, "could not alloc resources\n");
		return (ENXIO);
	}

	sc->ci_bst = rman_get_bustag(sc->ci_res);
	sc->ci_bsh = rman_get_bushandle(sc->ci_res);
	intc_softc = sc;
	m83xxx_pic_attach(sc);

	/* Clear interrupt status registers and disable all interrupts */
	intc_write_4(sc, COMCERTO_INTC_ARM0_IRQMASK_0, 0);
	intc_write_4(sc, COMCERTO_INTC_ARM0_IRQMASK_1, 0);
	intc_write_4(sc, COMCERTO_INTC_ARM1_IRQMASK_0, 0);
	intc_write_4(sc, COMCERTO_INTC_ARM1_IRQMASK_1, 0);
	return (0);
}

static void
m83xxx_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	u_int irq;
	unsigned int value;
	struct m83xxx_intc_softc *sc;
	int reg;

	sc = intc_softc;
	irq = ((struct m83xxx_irqsrc *)isrc)->ci_irq;

	if (irq < 32)
		reg = COMCERTO_INTC_ARM0_IRQMASK_0;
	else
		reg = COMCERTO_INTC_ARM0_IRQMASK_1;

	value = intc_read_4(sc, reg);
	value |= (1 << (irq % 32));
	intc_write_4(sc, reg, value);
}

static void
m83xxx_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	u_int irq;
	unsigned int value;
	struct m83xxx_intc_softc *sc;
	int reg;

	sc = intc_softc;
	irq = ((struct m83xxx_irqsrc *)isrc)->ci_irq;

	if (irq < 32)
		reg = COMCERTO_INTC_ARM0_IRQMASK_0;
	else
		reg = COMCERTO_INTC_ARM0_IRQMASK_1;

	value = intc_read_4(sc, reg);
	value &= ~(1 << (irq % 32));
	intc_write_4(sc, reg, value);
}

static int
m83xxx_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	u_int irq;
	struct intr_map_data_fdt *daf;
	struct m83xxx_intc_softc *sc;

	if (data->type != INTR_MAP_DATA_FDT)
		return (ENOTSUP);

	daf = (struct intr_map_data_fdt *)data;

	if (daf->ncells != 1 || daf->cells[0] >= INTC_NIRQS)
		return (EINVAL);

	irq = daf->cells[0];

	sc = device_get_softc(dev);
	*isrcp = &sc->ci_isrcs[irq].ci_isrc;
	return (0);
}

static void
m83xxx_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
//	arm_irq_memory_barrier(0);
	m83xxx_disable_intr(dev, isrc);
}

static void
m83xxx_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
//	arm_irq_memory_barrier(0);
	m83xxx_enable_intr(dev, isrc);
}

static void
m83xxx_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
	u_int irq;
	struct m83xxx_intc_softc *sc;
	int reg;

	arm_irq_memory_barrier(0);
	sc = intc_softc;
	irq = ((struct m83xxx_irqsrc *)isrc)->ci_irq;
	
	if (irq < 32)
		reg = COMCERTO_INTC_STATUS_REG_0;
	else
		reg = COMCERTO_INTC_STATUS_REG_1;

   	intc_write_4(sc, reg, 1 << (irq % 32));
}

static int
m83xxx_intr(void *arg)
{
	uint32_t irq, stat, mask;
	struct m83xxx_intc_softc *sc = arg;
	int reg;

	stat = intc_read_4(sc, COMCERTO_INTC_STATUS_REG_0);
	mask = intc_read_4(sc, COMCERTO_INTC_ARM0_IRQMASK_0);
	if (stat & 0x01) {
		stat = intc_read_4(sc, COMCERTO_INTC_STATUS_REG_1);
		mask = intc_read_4(sc, COMCERTO_INTC_ARM0_IRQMASK_1);
		irq = ffs(stat & mask) - 1 + 32;
	} else {
		irq = ffs(stat & mask) - 1;
	}

	if (intr_isrc_dispatch(&sc->ci_isrcs[irq].ci_isrc,
	    curthread->td_intr_frame) != 0) {
		kdb_break();
		if (irq < 32)
			reg = COMCERTO_INTC_STATUS_REG_0;
		else
			reg = COMCERTO_INTC_STATUS_REG_1;

   		intc_write_4(sc, reg, 1 << (irq % 32));

		device_printf(sc->dev, "Stray irq %u disabled\n", irq);
	}

	arm_irq_memory_barrier(0);

	return (FILTER_HANDLED);
}

#if 0
static int
m83xxx_intr(void *arg)
{
	uint32_t i;
	uint32_t stat0, mask0, stat1, mask1;
	struct m83xxx_intc_softc *sc = arg;

	stat0 = intc_read_4(sc, COMCERTO_INTC_STATUS_REG_0);
	mask0 = intc_read_4(sc, COMCERTO_INTC_ARM0_IRQMASK_0);
	stat1 = intc_read_4(sc, COMCERTO_INTC_STATUS_REG_1);
	mask1 = intc_read_4(sc, COMCERTO_INTC_ARM0_IRQMASK_1);

	for (i = 0; i < 32; ++i) {
		if (stat0 & mask0 & (1 << i)) {
			if (intr_isrc_dispatch(&sc->ci_isrcs[i].ci_isrc,
			    curthread->td_intr_frame) != 0) {
				device_printf(sc->dev, "Stray irq %u disabled\n", i);
			}
			arm_irq_memory_barrier(0);
		}
	}

	for (i = 0; i < 32; ++i) {
		if (stat1 & mask1 & (1 << i)) {
			if (intr_isrc_dispatch(&sc->ci_isrcs[i + 32].ci_isrc,
			    curthread->td_intr_frame) != 0) {
				device_printf(sc->dev, "Stray irq %u disabled\n", i + 32);
			}
			arm_irq_memory_barrier(0);
		}
	}

	return (FILTER_HANDLED);
}
#endif

static int
m83xxx_pic_attach(struct m83xxx_intc_softc *sc)
{
	struct intr_pic *pic;
	int error;
	uint32_t irq;
	const char *name;
	intptr_t xref;

	name = device_get_nameunit(sc->dev);
	for (irq = 0; irq < INTC_NIRQS; irq++) {
		sc->ci_isrcs[irq].ci_irq = irq;

		error = intr_isrc_register(&sc->ci_isrcs[irq].ci_isrc,
		    sc->dev, 0, "%s,%u", name, irq);
		if (error != 0)
			return (error);
	}

	xref = OF_xref_from_node(ofw_bus_get_node(sc->dev));
	pic = intr_pic_register(sc->dev, xref);
	if (pic == NULL)
		return (ENXIO);

	return (intr_pic_claim_root(sc->dev, xref, m83xxx_intr, sc, 0));
}
/*
#ifdef SMP
static int
m83xxx_init_bind(device_t dev, struct intr_irqsrc *isrc)
{
	return (EOPNOTSUPP);
}
#endif
*/

struct fdt_fixup_entry fdt_fixup_table[] = {
	{ NULL, NULL }
};

static device_method_t m83xxx_intc_methods[] = {
	DEVMETHOD(device_probe,		m83xxx_intc_probe),
	DEVMETHOD(device_attach,	m83xxx_intc_attach),
	DEVMETHOD(pic_disable_intr,	m83xxx_disable_intr),
	DEVMETHOD(pic_enable_intr,	m83xxx_enable_intr),
	DEVMETHOD(pic_map_intr,		m83xxx_map_intr),
	DEVMETHOD(pic_post_filter,	m83xxx_post_filter),
	DEVMETHOD(pic_post_ithread,	m83xxx_post_ithread),
	DEVMETHOD(pic_pre_ithread,	m83xxx_pre_ithread),
#ifdef SMP
//	DEVMETHOD(pic_bind_intr,	m83xxx_init_bind),
#endif
	{ 0, 0 }
};

static driver_t m83xxx_intc_driver = {
	"pic",
	m83xxx_intc_methods,
	sizeof(struct m83xxx_intc_softc),
};

static devclass_t m83xxx_intc_devclass;

EARLY_DRIVER_MODULE(pic, simplebus, m83xxx_intc_driver, m83xxx_intc_devclass, 0, 0, BUS_PASS_INTERRUPT);
