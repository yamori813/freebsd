/*-
 * Copyright (c) 2010 Jakub Wojciech Klama <jceel@FreeBSD.org>
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/timetc.h>
#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/ralink/rt1310reg.h>

struct rt1310_intc_softc {
	struct resource *	li_res;
	bus_space_tag_t		li_bst;
	bus_space_handle_t	li_bsh;
};

static int rt1310_intc_probe(device_t);
static int rt1310_intc_attach(device_t);
static void rt1310_intc_eoi(void *);

static struct rt1310_intc_softc *intc_softc = NULL;

#define	intc_read_4(_sc, _reg)		\
    bus_space_read_4((_sc)->li_bst, (_sc)->li_bsh, (_reg))
#define	intc_write_4(_sc, _reg, _val)		\
    bus_space_write_4((_sc)->li_bst, (_sc)->li_bsh, (_reg), (_val))

int irqprio[] = {3, 3, 3, 4, 4, 4, 2, 2, 2, 2};

static int
rt1310_intc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "rt,pic"))
		return (ENXIO);

	device_set_desc(dev, "RT1310 Interrupt Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
rt1310_intc_attach(device_t dev)
{
	struct rt1310_intc_softc *sc = device_get_softc(dev);
	int rid = 0;
	int i;

	if (intc_softc)
		return (ENXIO);

	sc->li_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, 
	    RF_ACTIVE);
	if (!sc->li_res) {
		device_printf(dev, "could not alloc resources\n");
		return (ENXIO);
	}

	sc->li_bst = rman_get_bustag(sc->li_res);
	sc->li_bsh = rman_get_bushandle(sc->li_res);
	intc_softc = sc;
	arm_post_filter = rt1310_intc_eoi;

	intc_write_4(sc, RT_INTC_IECR, 0);
	intc_write_4(sc, RT_INTC_ICCR, ~0);

	for(i = 0; i <= 9; ++i) {
		intc_write_4(sc, RT_INTC_SCR0+i*4, 
			(RT_INTC_TRIG_HIGH_LVL << RT_INTC_TRIG_SHIF) | 
			irqprio[i]);
		intc_write_4(sc, RT_INTC_SVR0+i*4, i);
	}

	/* Clear interrupt status registers and disable all interrupts */
	intc_write_4(sc, RT_INTC_ICCR, ~0);
	intc_write_4(sc, RT_INTC_IMR, 0);
	return (0);
}

static device_method_t rt1310_intc_methods[] = {
	DEVMETHOD(device_probe,		rt1310_intc_probe),
	DEVMETHOD(device_attach,	rt1310_intc_attach),
	{ 0, 0 }
};

static driver_t rt1310_intc_driver = {
	"pic",
	rt1310_intc_methods,
	sizeof(struct rt1310_intc_softc),
};

static devclass_t rt1310_intc_devclass;

//DRIVER_MODULE(pic, simplebus, rt1310_intc_driver, rt1310_intc_devclass, 0, 0);
EARLY_DRIVER_MODULE(pic, simplebus, rt1310_intc_driver, rt1310_intc_devclass, 0, 0, BUS_PASS_INTERRUPT);

int
arm_get_next_irq(int last)
{
	struct rt1310_intc_softc *sc = intc_softc;
	uint32_t value;
	int i;
	value = intc_read_4(sc, RT_INTC_IPR);
	for (i = 0; i < 32; i++) {
		if (value & (1 << i))
			return (i);
	}

	return (-1);
}

void
arm_mask_irq(uintptr_t nb)
{
	struct rt1310_intc_softc *sc = intc_softc;
	uint32_t value;
	
	/* Make sure that interrupt isn't active already */
	rt1310_intc_eoi((void *)nb);

	/* Clear bit in ER register */
	value = intc_read_4(sc, RT_INTC_IECR);
	value &= ~(1 << nb);
	intc_write_4(sc, RT_INTC_IECR, value);
	intc_write_4(sc, RT_INTC_IMR, value);

	intc_write_4(sc, RT_INTC_ICCR, 1 << nb);
}

void
arm_unmask_irq(uintptr_t nb)
{
	struct rt1310_intc_softc *sc = intc_softc;
	uint32_t value;

	value = intc_read_4(sc, RT_INTC_IECR);

	value |= (1 << nb);

	intc_write_4(sc, RT_INTC_IMR, value);
	intc_write_4(sc, RT_INTC_IECR, value);
}

static void
rt1310_intc_eoi(void *data)
{
	struct rt1310_intc_softc *sc = intc_softc;
	int nb = (int)data;

	intc_write_4(sc, RT_INTC_ICCR, 1 << nb);
	if(nb == 0) {
	uint32_t value;
	value = intc_read_4(sc, RT_INTC_IECR);
	value &= ~(1 << nb);
	intc_write_4(sc, RT_INTC_IECR, value);
	intc_write_4(sc, RT_INTC_IMR, value);
	}
}

struct fdt_fixup_entry fdt_fixup_table[] = {
	{ NULL, NULL }
};

static int
fdt_pic_decode_ic(phandle_t node, pcell_t *intr, int *interrupt, int *trig,
    int *pol)
{
	if (!fdt_is_compatible(node, "lpc,pic"))
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
