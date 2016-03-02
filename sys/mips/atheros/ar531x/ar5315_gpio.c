/*-
 * Copyright (c) 2009, Oleksandr Tymoshenko <gonzo@FreeBSD.org>
 * Copyright (c) 2009, Luiz Otavio O Souza. 
 * Copyright (c) 2016, Hiroki Mori
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
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
 * GPIO driver for AR5315 
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <mips/atheros/ar531x/ar5315reg.h>
#include <mips/atheros/ar531x/ar5315_cpudef.h>
#include <mips/atheros/ar531x/ar5315_gpiovar.h>
#include <dev/gpio/gpiobusvar.h>

#include "gpio_if.h"

#define	DEFAULT_CAPS	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)

/*
 * Helpers
 */
static void ar5315_gpio_pin_configure(struct ar5315_gpio_softc *sc, 
    struct gpio_pin *pin, uint32_t flags);

/*
 * Driver stuff
 */
static int ar5315_gpio_probe(device_t dev);
static int ar5315_gpio_attach(device_t dev);
static int ar5315_gpio_detach(device_t dev);
#ifdef NOTUSE
static int ar5315_gpio_filter(void *arg);
#endif
static void ar5315_gpio_intr(void *arg);
static void ar5315_gpio_attach_sysctl(device_t dev);

/*
 * GPIO interface
 */
static device_t ar5315_gpio_get_bus(device_t);
static int ar5315_gpio_pin_max(device_t dev, int *maxpin);
static int ar5315_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps);
static int ar5315_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t
    *flags);
static int ar5315_gpio_pin_getname(device_t dev, uint32_t pin, char *name);
static int ar5315_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags);
static int ar5315_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value);
static int ar5315_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val);
static int ar5315_gpio_pin_toggle(device_t dev, uint32_t pin);

static void
ar5315_gpio_attach_sysctl(device_t dev)
{
	struct ar5315_gpio_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;

	sc = device_get_softc(dev);
	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);

#ifdef notyet
	// only support ar5315
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		"ppsenable", CTLFLAG_RW, &sc->gpio_ppsenable, 0,
		"ar5315_gpio pps enable flags");
#endif
}

static void
ar5315_gpio_pin_configure(struct ar5315_gpio_softc *sc, struct gpio_pin *pin,
    unsigned int flags)
{
	uint32_t mask;

	mask = 1 << pin->gp_pin;

	/*
	 * Manage input/output
	 */
	if (flags & (GPIO_PIN_INPUT|GPIO_PIN_OUTPUT)) {
		pin->gp_flags &= ~(GPIO_PIN_INPUT|GPIO_PIN_OUTPUT);
		if (flags & GPIO_PIN_OUTPUT) {
			pin->gp_flags |= GPIO_PIN_OUTPUT;
			GPIO_SET_BITS(sc, ar531x_gpio_cr(), mask);
		}
		else {
			pin->gp_flags |= GPIO_PIN_INPUT;
			GPIO_CLEAR_BITS(sc, ar531x_gpio_cr(), mask);
		}
	}
}

static device_t
ar5315_gpio_get_bus(device_t dev)
{
	struct ar5315_gpio_softc *sc;

	sc = device_get_softc(dev);

	return (sc->busdev);
}

static int
ar5315_gpio_pin_max(device_t dev, int *maxpin)
{

	*maxpin = ar531x_gpio_pins() - 1;
	return (0);
}

static int
ar5315_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct ar5315_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	GPIO_LOCK(sc);
	*caps = sc->gpio_pins[i].gp_caps;
	GPIO_UNLOCK(sc);

	return (0);
}

static int
ar5315_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct ar5315_gpio_softc *sc = device_get_softc(dev);
	int i;
	int dir;

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	dir = GPIO_READ(sc, ar531x_gpio_cr()) & (1 << pin);

	*flags = dir ? GPIO_PIN_OUTPUT : GPIO_PIN_INPUT;

/*
	GPIO_LOCK(sc);
	*flags = sc->gpio_pins[i].gp_flags;
	GPIO_UNLOCK(sc);
*/

	return (0);
}

static int
ar5315_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct ar5315_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	GPIO_LOCK(sc);
	memcpy(name, sc->gpio_pins[i].gp_name, GPIOMAXNAME);
	GPIO_UNLOCK(sc);

	return (0);
}

static int
ar5315_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	int i;
	struct ar5315_gpio_softc *sc = device_get_softc(dev);

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	ar5315_gpio_pin_configure(sc, &sc->gpio_pins[i], flags);

	return (0);
}

static int
ar5315_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct ar5315_gpio_softc *sc = device_get_softc(dev);
	uint32_t state;

	state = GPIO_READ(sc, ar531x_gpio_do());

	if(value == 1) {
		state |= (1 << pin);
	} else {
		state &= ~(1 << pin);
	}

	GPIO_WRITE(sc, ar531x_gpio_do(), state);

	return (0);
}

static int
ar5315_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct ar5315_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	*val = (GPIO_READ(sc, ar531x_gpio_di()) & (1 << pin)) ? 1 : 0;

	return (0);
}

static int
ar5315_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	int res, i;
	struct ar5315_gpio_softc *sc = device_get_softc(dev);

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	res = (GPIO_READ(sc, ar531x_gpio_do()) & (1 << pin)) ? 1 : 0;
	if (res)
		GPIO_CLEAR_BITS(sc, ar531x_gpio_do(), pin);
	else
		GPIO_SET_BITS(sc, ar531x_gpio_do(), pin);

	return (0);
}

#ifdef NOTUSE
static int
ar5315_gpio_filter(void *arg)
{

	/* TODO: something useful */
	return (FILTER_STRAY);
}
#endif

static void
ar5315_gpio_intr(void *arg)
{
	int val;
	struct ar5315_gpio_softc *sc = arg;
	GPIO_LOCK(sc);
	val = (GPIO_READ(sc, ar531x_gpio_di()) & (1 << sc->gpio_ppspin))
		? 1 : 0;
	if(val == 0) {
#ifdef notyet
		pps_event(&sc->gpio_pps, PPS_CAPTUREASSERT);
#endif
	}
	GPIO_UNLOCK(sc);
}

static int
ar5315_gpio_probe(device_t dev)
{

	device_set_desc(dev, "Atheros AR5315 GPIO driver");
	return (0);
}

static int
ar5315_gpio_attach(device_t dev)
{
	struct ar5315_gpio_softc *sc = device_get_softc(dev);
	int i, j, maxpin;
	int mask, pinon;
	int ppspin;
	uint32_t oe;

	KASSERT((device_get_unit(dev) == 0),
	    ("ar5315_gpio: Only one gpio module supported"));

	mtx_init(&sc->gpio_mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Map control/status registers. */
	sc->gpio_mem_rid = 0;
	sc->gpio_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->gpio_mem_rid, RF_ACTIVE);

	if (sc->gpio_mem_res == NULL) {
		device_printf(dev, "couldn't map memory\n");
		ar5315_gpio_detach(dev);
		return (ENXIO);
	}

	if ((sc->gpio_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, 
	    &sc->gpio_irq_rid, RF_SHAREABLE | RF_ACTIVE)) == NULL) {
		device_printf(dev, "unable to allocate IRQ resource\n");
		ar5315_gpio_detach(dev);
		return (ENXIO);
	}

	if ((bus_setup_intr(dev, sc->gpio_irq_res, INTR_TYPE_MISC, 
//	    ar5315_gpio_filter, ar5315_gpio_intr, sc, &sc->gpio_ih))) {
	    NULL, ar5315_gpio_intr, sc, &sc->gpio_ih))) {
		device_printf(dev,
		    "WARNING: unable to register interrupt handler\n");
		ar5315_gpio_detach(dev);
		return (ENXIO);
	}

	sc->dev = dev;

	if (resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "ppspin", &ppspin) != 0)
		ppspin = -1;

	/* Disable interrupts for all pins. */
	if(ppspin != -1) {
#ifdef notyet
		sc->gpio_ppspin = ppspin;
		GPIO_WRITE(sc, AR5315_SYSREG_GPIO_INT, 3 << 6 | ppspin);
		device_printf(dev, "gpio ppspin=0x%x\n", ppspin);
		sc->gpio_pps.ppscap = PPS_CAPTUREASSERT | PPS_ECHOASSERT;
		pps_init(&sc->gpio_pps);
#endif
	} else {
		GPIO_WRITE(sc, AR5315_SYSREG_GPIO_INT, 0);
	}

	/* Initialise all pins specified in the mask, up to the pin count */
	(void) ar5315_gpio_pin_max(dev, &maxpin);
	if (resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "pinmask", &mask) != 0)
		mask = 0;
	if (resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "pinon", &pinon) != 0)
		pinon = 0;
	device_printf(dev, "gpio pinmask=0x%x\n", mask);
	for (j = 0; j <= maxpin; j++) {
		if ((mask & (1 << j)) == 0)
			continue;
		sc->gpio_npins++;
	}

	/* Iniatilize the GPIO pins, keep the loader settings. */
	oe = GPIO_READ(sc, ar531x_gpio_cr());
	sc->gpio_pins = malloc(sizeof(struct gpio_pin) * sc->gpio_npins,
	    M_DEVBUF, M_WAITOK | M_ZERO);
	for (i = 0, j = 0; j <= maxpin; j++) {
		if ((mask & (1 << j)) == 0)
			continue;
		snprintf(sc->gpio_pins[i].gp_name, GPIOMAXNAME,
		    "pin %d", j);
		sc->gpio_pins[i].gp_pin = j;
		sc->gpio_pins[i].gp_caps = DEFAULT_CAPS;
		if (oe & (1 << j))
			sc->gpio_pins[i].gp_flags = GPIO_PIN_OUTPUT;
		else
			sc->gpio_pins[i].gp_flags = GPIO_PIN_INPUT;
		i++;
	}

	/* Turn on the hinted pins. */
	for (i = 0; i < sc->gpio_npins; i++) {
		j = sc->gpio_pins[i].gp_pin;
		if ((pinon & (1 << j)) != 0) {
			ar5315_gpio_pin_setflags(dev, j, GPIO_PIN_OUTPUT);
			ar5315_gpio_pin_set(dev, j, 1);
		}
	}

	sc->busdev = gpiobus_attach_bus(dev);
	if (sc->busdev == NULL) {
		ar5315_gpio_detach(dev);
		return (ENXIO);
	}

	ar5315_gpio_attach_sysctl(dev);

	return (0);
}

static int
ar5315_gpio_detach(device_t dev)
{
	struct ar5315_gpio_softc *sc = device_get_softc(dev);

	KASSERT(mtx_initialized(&sc->gpio_mtx), ("gpio mutex not initialized"));

	gpiobus_detach_bus(dev);
	if (sc->gpio_ih)
		bus_teardown_intr(dev, sc->gpio_irq_res, sc->gpio_ih);
	if (sc->gpio_irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, sc->gpio_irq_rid,
		    sc->gpio_irq_res);
	if (sc->gpio_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->gpio_mem_rid,
		    sc->gpio_mem_res);
	if (sc->gpio_pins)
		free(sc->gpio_pins, M_DEVBUF);
	mtx_destroy(&sc->gpio_mtx);

	return(0);
}

static device_method_t ar5315_gpio_methods[] = {
	DEVMETHOD(device_probe, ar5315_gpio_probe),
	DEVMETHOD(device_attach, ar5315_gpio_attach),
	DEVMETHOD(device_detach, ar5315_gpio_detach),

	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus, ar5315_gpio_get_bus),
	DEVMETHOD(gpio_pin_max, ar5315_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname, ar5315_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getflags, ar5315_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps, ar5315_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags, ar5315_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get, ar5315_gpio_pin_get),
	DEVMETHOD(gpio_pin_set, ar5315_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle, ar5315_gpio_pin_toggle),
	{0, 0},
};

static driver_t ar5315_gpio_driver = {
	"gpio",
	ar5315_gpio_methods,
	sizeof(struct ar5315_gpio_softc),
};
static devclass_t ar5315_gpio_devclass;

DRIVER_MODULE(ar5315_gpio, apb, ar5315_gpio_driver, ar5315_gpio_devclass, 0, 0);
