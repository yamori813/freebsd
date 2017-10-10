/*-
 * Copyright (c) 2017 Hiroki Mori
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

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/idt/idtreg_rc32300.h>

#define MEMCTL_READ_REG(_sc, _reg) \
	bus_read_4((_sc)->sc_res[0], _reg)
#define MEMCTL_WRITE_REG(_sc, _reg, _val) \
	bus_write_4((_sc)->sc_res[0], _reg, _val)

static struct resource_spec idt_memctl_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

struct idt_memctl_softc {
	device_t		sc_dev;
	struct resource		*sc_res[1];
};

static int
idt_memctl_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "idt,rc32300-memctl"))
		return (ENXIO);

	device_set_desc(dev, "IDT RC32300 Memory Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
idt_memctl_attach(device_t dev)
{
	struct idt_memctl_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	if (bus_alloc_resources(dev, idt_memctl_spec, sc->sc_res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	reg = MEMCTL_READ_REG(sc, 0);
	reg &= ~((1 << 12) | (3 << 10));
	reg |= (1 << 10);
	MEMCTL_WRITE_REG(sc, 0, reg);

	return (0);
}

static device_method_t idt_memctl_methods[] = {
	DEVMETHOD(device_attach,	idt_memctl_attach),
	DEVMETHOD(device_probe,		idt_memctl_probe),

	{0, 0},
};

static driver_t idt_memctl_driver = {
	"memctl",
	idt_memctl_methods,
	sizeof(struct idt_memctl_softc),
};
static devclass_t idt_memctl_devclass;

EARLY_DRIVER_MODULE(idt_memctl, simplebus, idt_memctl_driver, idt_memctl_devclass,
    0, 0, BUS_PASS_ORDER_EARLY);
