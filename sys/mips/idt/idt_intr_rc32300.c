/*-
 * Copyright (c) 2007, Oleksandr Tymoshenko <gonzo@freebsd.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed for the NetBSD Project by
 *	Wasabi Systems, Inc.
 * 4. The name of Wasabi Systems, Inc. may not be used to endorse
 *    or promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY WASABI SYSTEMS, INC. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL WASABI SYSTEMS, INC
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
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
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/malloc.h>
#include <sys/proc.h>

#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/idt/idtreg_rc32300.h>
#include <mips/idt/idt_intr_rc32300.h>

#include "pic_if.h"

#define	PIC_INTR_ISRC(sc, irq)	(&(sc)->pic_irqs[(irq-1)].isrc)

#define PIC_READ_REG(_sc, _reg) \
	bus_read_4((_sc)->sc_res[0], _reg)
#define PIC_WRITE_REG(_sc, _reg, _val) \
	bus_write_4((_sc)->sc_res[0], _reg, _val)

static int	idt_intr_probe(device_t);
static int	idt_intr_attach(device_t);
static int	idt_intc_intr(void *arg);

static struct resource_spec idt_pic_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

static int
idt_intr_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "idt,rc32300-intc"))
		return (ENXIO);

	device_set_desc(dev, "IDT RC32300 Expansion Interrupt Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
idt_intr_attach(device_t dev)
{
	struct idt_intr_softc *sc;
	intptr_t xref;
	struct intr_irqsrc *isrc;
	int err;
	int i;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	if (bus_alloc_resources(dev, idt_pic_spec, sc->sc_res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	xref = OF_xref_from_node(ofw_bus_get_node(dev));

	for (i = 0; i < NIRQS; i++) {
		sc->pic_irqs[i].irq = i + 1;
		isrc = &sc->pic_irqs[i].isrc;
		err = intr_isrc_register(isrc, sc->sc_dev,
			0, "pic-grp%d", i + 1);
	}

	if (intr_pic_register(dev, xref) == NULL) {
		device_printf(dev, "could not register PIC\n");
		return (ENXIO);
	}

	if ((bus_setup_intr(dev, sc->sc_res[1], INTR_TYPE_MISC, 
		idt_intc_intr, NULL, sc, &sc->sc_ih[0]))) {
		device_printf(dev, "could not setup irq handler\n");
		intr_pic_deregister(dev, xref);
		return (ENXIO);
	}


	return (0);
}

static int
idt_intc_intr(void *arg)
{
	struct idt_intr_softc *sc;
	uint32_t intr;
	int i;

	sc = arg;

	intr = PIC_READ_REG(sc, 0);

	while ((i = fls(intr)) != 0) {
		i--;
		intr &= ~(1u << i);

		if (intr_isrc_dispatch(PIC_INTR_ISRC(sc, i),
		    curthread->td_intr_frame) != 0) {
			device_printf(sc->sc_dev,
				"Stray interrupt %u detected\n", i);
			continue;
		}
	}

	return (FILTER_HANDLED);
}

static void
idt_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct idt_intr_softc *sc;
	u_int irq;
	uint32_t reg;

	sc = device_get_softc(dev);
	irq = ((struct idt_pic_irqsrc *)isrc)->irq;
	reg = PIC_READ_REG(sc, 4);
	reg &= ~(1 << irq);
	PIC_WRITE_REG(sc, 4, reg);
}

static void
idt_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct idt_intr_softc *sc;
	u_int irq;
	uint32_t reg;

	sc = device_get_softc(dev);
	irq = ((struct idt_pic_irqsrc *)isrc)->irq;
	reg = PIC_READ_REG(sc, 4);
	reg |= (1 << irq);
	PIC_WRITE_REG(sc, 4, reg);
}

static void
idt_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	idt_pic_disable_intr(dev, isrc);
}

static void
idt_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	idt_pic_enable_intr(dev, isrc);
}

#if 0
static void
idt_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
	uint32_t irq;

	irq = ((struct idt_pic_irqsrc *)isrc)->irq;
}
#endif

static int
idt_pic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct idt_intr_softc *sc;
	uint32_t irq;
	struct intr_map_data_fdt *daf;

	sc = device_get_softc(dev);
	daf = (struct intr_map_data_fdt *)data;

	if (data == NULL || data->type != INTR_MAP_DATA_FDT ||
	    daf->ncells != 1)
		return (EINVAL);

	irq = daf->cells[0];

	*isrcp = &sc->pic_irqs[irq].isrc;

	return (0);
}

static device_method_t idt_intr_methods[] = {
	DEVMETHOD(device_attach,		idt_intr_attach),
	DEVMETHOD(device_probe,			idt_intr_probe),

	DEVMETHOD(pic_disable_intr,		idt_pic_disable_intr),
	DEVMETHOD(pic_enable_intr,		idt_pic_enable_intr),
	DEVMETHOD(pic_map_intr,			idt_pic_map_intr),
//	DEVMETHOD(pic_post_filter,		idt_pic_post_filter),
	DEVMETHOD(pic_post_ithread,		idt_pic_post_ithread),
	DEVMETHOD(pic_pre_ithread,		idt_pic_pre_ithread),

	{0, 0},
};

static driver_t idt_intr_driver = {
	"pic",
	idt_intr_methods,
	sizeof(struct idt_intr_softc),
};
static devclass_t idt_intr_devclass;

EARLY_DRIVER_MODULE(idt_intr, simplebus, idt_intr_driver, idt_intr_devclass,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
