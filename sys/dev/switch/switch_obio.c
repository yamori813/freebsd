/*-
 * Copyright (c) 2010-2012 Aleksandr Rybalko.
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
 */

/*
 * switch control attached to OBIO bus
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

#include <machine/bus.h>

#include <dev/switch/switchvar.h>

#include "switch_if.h"
#include "switchb_if.h"

#define READ4(_sc, _reg) \
	bus_space_read_4(_sc->sc_bst, _sc->sc_bsh, _reg)

#define WRITE4(_sc, _reg, _val) \
	bus_space_write_4(_sc->sc_bst, _sc->sc_bsh, _reg, _val)

static int	switch_obio_probe(device_t);
static int	switch_obio_attach(device_t);
static int	switch_obio_detach(device_t);
static int	switch_obio_intr(void *);

static int
switch_obio_probe(device_t dev)
{
	device_set_desc(dev, "ethernet switch bus");
	return (0);
}

static int
switch_obio_attach(device_t dev)
{
	struct switch_softc *sc;
	int error = 0;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	/* Map control registers. */
	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->mem_rid, RF_ACTIVE);

	if (sc->mem_res == NULL) {
		device_printf(dev, "couldn't map memory\n");
		error = ENXIO;
		switch_obio_detach(dev);
		return(error);
	}

	sc->sc_bst = rman_get_bustag(sc->mem_res);
	sc->sc_bsh = rman_get_bushandle(sc->mem_res);

	sc->irq_rid = 0;
	if ((sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, 
	    &sc->irq_rid, RF_SHAREABLE | RF_ACTIVE)) == NULL) {
		device_printf(dev, "unable to allocate IRQ resource\n");
		return (ENXIO);
	}

	if ((bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC, 
	    switch_obio_intr, NULL, sc, &sc->ihandl))) {
		device_printf(dev,
		    "WARNING: unable to register interrupt handler\n");
		return (ENXIO);
	}
	SWITCH_LOCK_INIT(sc);
	error = switch_init(sc);
	if (error)
		return (error);

	return (bus_generic_attach(dev));
}

static int
switch_obio_detach(device_t dev)
{
	struct switch_softc *sc;

	sc = device_get_softc(dev);

	SWITCH_LOCK_DESTROY(sc);

	if (sc->irq_res)
		bus_teardown_intr(dev, sc->irq_res, &sc->ihandl);

	switch_deinit(sc);

	bus_generic_detach(dev);

	if (sc->irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid,
		    sc->irq_res);

	if (sc->mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
		    sc->mem_res);

	return(0);
}

static int
switch_obio_intr(void *arg)
{
	struct switch_softc *sc;

	sc = (struct switch_softc *)arg;

	if (sc->sc_isr)
		return (*sc->sc_isr)(sc->sc_cookie);

	return (FILTER_HANDLED);
}

static int
switch_obio_register_isr(device_t dev, driver_filter_t isr, device_t child)
{
	struct switch_softc *sc;
	void *child_sc;

	sc = device_get_softc(dev);

	if (!sc->sc_isr) {
		child_sc = device_get_softc(child);
		sc->sc_isr = isr;
		sc->sc_cookie = child_sc;
	}

	return (0);
}

static void
switch_obio_unregister_isr(device_t dev, device_t child)
{
	struct switch_softc *sc;
	void *child_sc;

	sc = device_get_softc(dev);
	child_sc = device_get_softc(child);

	/* Only registered child can unregister */
	if (sc->sc_cookie == child_sc) {
		sc->sc_isr = NULL;
		sc->sc_cookie = NULL;
	}

	return;
}

static uint32_t
switch_obio_read4(device_t dev, uint32_t reg)
{
	struct switch_softc *sc;

	sc = device_get_softc(dev);

	return (READ4(sc, reg));
}

static void
switch_obio_write4(device_t dev, uint32_t reg, uint32_t val)
{
	struct switch_softc *sc;

	sc = device_get_softc(dev);

	WRITE4(sc, reg, val);
}

static device_method_t switch_obio_methods[] = {
	DEVMETHOD(device_probe,		switch_obio_probe),
	DEVMETHOD(device_attach,	switch_obio_attach),
	DEVMETHOD(device_detach,	switch_obio_detach),

	DEVMETHOD(switchb_read4,	switch_obio_read4),
	DEVMETHOD(switchb_write4,	switch_obio_write4),
	DEVMETHOD(switchb_register_isr,	switch_obio_register_isr),
	DEVMETHOD(switchb_unregister_isr, switch_obio_unregister_isr),
	{0, 0},
};

static driver_t switch_obio_driver = {
	"switch",
	switch_obio_methods,
	sizeof(struct switch_softc),
};
static devclass_t switch_obio_devclass;

DRIVER_MODULE(switch, obio, switch_obio_driver, switch_obio_devclass, 0, 0);
