/*-
 * Copyright (c) 2010, Aleksandr Rybalko <ray@ddteam.net>
 * Copyright (c) 2009, Oleksandr Tymoshenko <gonzo@FreeBSD.org>
 * Copyright (c) 2009, Luiz Otavio O Souza. 
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
 * GPIO driver for SIBA ChipCommon core.
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
#include <sys/mutex.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <dev/siba/sibareg.h>
#include <dev/siba/siba_cc.h>
#include <dev/siba/siba_cc_gpio.h>
#include <dev/siba/siba_cc_gpiovar.h>

#include "gpio_if.h"

#define	DEFAULT_CAPS	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT | GPIO_PIN_PULLUP | \
 		         GPIO_PIN_PULLDOWN | GPIO_PIN_INVIN |                 \
 		         GPIO_PIN_INVOUT | GPIO_PIN_PULSATE | GPIO_PIN_REPORT )

/*
 * Helpers
 */
static void siba_cc_gpio_pin_configure(struct siba_cc_gpio_softc *sc, 
    struct gpio_pin *pin, uint32_t flags);

/*
 * Driver stuff
 */
static int siba_cc_gpio_probe(device_t dev);
static int siba_cc_gpio_attach(device_t dev);
static int siba_cc_gpio_detach(device_t dev);
static int siba_cc_gpio_intr(void *arg);

int 	siba_cc_get_int_mask  (device_t);
void 	siba_cc_set_int_mask  (device_t, uint32_t);
int 	siba_cc_get_int_status(device_t);
void 	siba_cc_set_int_status(device_t, uint32_t);

/*
 * GPIO interface
 */
static int siba_cc_gpio_pin_max(device_t dev, int *maxpin);
static int siba_cc_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps);
static int siba_cc_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t
    *flags);
static int siba_cc_gpio_pin_getname(device_t dev, uint32_t pin, char *name);
static int siba_cc_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags);
static int siba_cc_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value);
static int siba_cc_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val);
static int siba_cc_gpio_pin_toggle(device_t dev, uint32_t pin);

static void
siba_cc_gpio_pin_configure(struct siba_cc_gpio_softc *sc, struct gpio_pin *pin,
    unsigned int flags)
{
	uint32_t mask;

	mask = 1 << pin->gp_pin;
	GPIO_LOCK(sc);

	/*
	 * Manage input/output
	 */
	if (flags & (GPIO_PIN_INPUT|GPIO_PIN_OUTPUT)) {
		pin->gp_flags &= ~(GPIO_PIN_INPUT|GPIO_PIN_OUTPUT);
		if (flags & GPIO_PIN_OUTPUT) {
			pin->gp_flags |= GPIO_PIN_OUTPUT;
			GPIO_SET_BITS(sc, SIBA_CC_GPIO_OUTEN, mask);
		}
		else {
			pin->gp_flags |= GPIO_PIN_INPUT;
			GPIO_CLEAR_BITS(sc, SIBA_CC_GPIO_OUTEN, mask);
		}
	}
	if (flags & GPIO_PIN_PULLUP) {
		pin->gp_flags |= GPIO_PIN_PULLUP;
		GPIO_SET_BITS(sc, SIBA_CC_GPIO_PULLUP, mask);
	}
	else {
		pin->gp_flags &= ~GPIO_PIN_PULLUP;
		GPIO_CLEAR_BITS(sc, SIBA_CC_GPIO_PULLUP, mask);
	}

	if (flags & GPIO_PIN_PULLDOWN) {
		pin->gp_flags |= GPIO_PIN_PULLDOWN;
		GPIO_SET_BITS(sc, SIBA_CC_GPIO_PULLDOWN, mask);
	}
	else {
		pin->gp_flags &= ~GPIO_PIN_PULLDOWN;
		GPIO_CLEAR_BITS(sc, SIBA_CC_GPIO_PULLDOWN, mask);
	}

	if (flags & GPIO_PIN_PULSATE) {
		pin->gp_flags |= GPIO_PIN_PULSATE;
		GPIO_SET_BITS(sc, SIBA_CC_GPIO_TIMER_OUT_MASK, mask);
	}
	else {
		pin->gp_flags &= ~GPIO_PIN_PULSATE;
		GPIO_CLEAR_BITS(sc, SIBA_CC_GPIO_TIMER_OUT_MASK, mask);
	}

	if (flags & GPIO_PIN_INVOUT) {
		pin->gp_flags |= GPIO_PIN_INVOUT;
	}
	else {
		pin->gp_flags &= ~GPIO_PIN_INVOUT;
	}

	if (flags & GPIO_PIN_INVIN) {
		pin->gp_flags |= GPIO_PIN_INVIN;
	}
	else {
		pin->gp_flags &= ~GPIO_PIN_INVIN;
	}


	if (flags & GPIO_PIN_REPORT) {
		pin->gp_flags |= GPIO_PIN_REPORT;
		GPIO_SET_BITS(sc, SIBA_CC_GPIO_INT_MASK, mask);
		device_printf(sc->dev, "Will report interrupt on pin %#x\n", 
		    mask);

		/* If pin inverted, invert int polarity default */
		if ( flags & GPIO_PIN_INVIN ) {
			GPIO_SET_BITS(sc, SIBA_CC_GPIO_INT_POLARITY, mask);
		}
	}
	else {
		pin->gp_flags &= ~GPIO_PIN_REPORT;
		GPIO_CLEAR_BITS(sc, SIBA_CC_GPIO_INT_MASK, mask);
	}

	GPIO_UNLOCK(sc);
}

static int
siba_cc_gpio_pin_max(device_t dev, int *maxpin)
{

	*maxpin = SIBA_CC_GPIO_PINS - 1;
	return (0);
}

static int
siba_cc_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct siba_cc_gpio_softc *sc = device_get_softc(dev);
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
siba_cc_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct siba_cc_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	GPIO_LOCK(sc);
	*flags = sc->gpio_pins[i].gp_flags;
	GPIO_UNLOCK(sc);

	return (0);
}

static int
siba_cc_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct siba_cc_gpio_softc *sc = device_get_softc(dev);
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
siba_cc_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	int i;
	struct siba_cc_gpio_softc *sc = device_get_softc(dev);

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	/* Filter out unwanted flags */
	if ((flags &= sc->gpio_pins[i].gp_caps) != flags)
		return (EINVAL);

	/* Can't mix input/output together */
	if ((flags & (GPIO_PIN_INPUT|GPIO_PIN_OUTPUT)) ==
	    (GPIO_PIN_INPUT|GPIO_PIN_OUTPUT))
		return (EINVAL);

	siba_cc_gpio_pin_configure(sc, &sc->gpio_pins[i], flags);


	return (0);
}

static int
siba_cc_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct siba_cc_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	if ( sc->gpio_pins[i].gp_flags & GPIO_PIN_INVOUT )
		value = value?0:1;

	GPIO_LOCK(sc);
	if (value) GPIO_SET_BITS  (sc, SIBA_CC_GPIO_OUTPUT, (1 << pin));
	else       GPIO_CLEAR_BITS(sc, SIBA_CC_GPIO_OUTPUT, (1 << pin));
	GPIO_UNLOCK(sc);

	return (0);
}

static int
siba_cc_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct siba_cc_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	GPIO_LOCK(sc);
	*val = (GPIO_READ(sc, SIBA_CC_GPIO_INPUT) & (1 << pin)) ? 1 : 0;
	GPIO_UNLOCK(sc);

	if ( sc->gpio_pins[i].gp_flags & GPIO_PIN_INVIN )
		*val = *val?0:1;

	return (0);
}

static int
siba_cc_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	int res, i;
	struct siba_cc_gpio_softc *sc = device_get_softc(dev);

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	GPIO_LOCK(sc);
	res = (GPIO_READ(sc, SIBA_CC_GPIO_INPUT) & (1 << pin)) ? 1 : 0;
	if (res) GPIO_CLEAR_BITS(sc, SIBA_CC_GPIO_OUTEN, (1 << pin));
	else     GPIO_SET_BITS(sc, SIBA_CC_GPIO_OUTEN, (1 << pin));
	GPIO_UNLOCK(sc);

	return (0);
}

static int
siba_cc_gpio_intr(void *arg)
{
	struct siba_cc_gpio_softc *sc = arg;
	uint32_t input, status, mask, value, i;
	char notify[16];
	char pinname[6];

	mask = siba_cc_get_int_mask(device_get_parent(sc->dev));
	siba_cc_set_int_mask(device_get_parent(sc->dev),
	    mask & ~SIBA_CC_INT_GPIO);

	status = siba_cc_get_int_status(device_get_parent(sc->dev));
	if (!( status & SIBA_CC_INT_GPIO)) {
		siba_cc_set_int_mask(device_get_parent(sc->dev), mask);
		return (FILTER_HANDLED);
	}

	status &= ~SIBA_CC_INT_GPIO;
	siba_cc_set_int_status(device_get_parent(sc->dev), status);

	GPIO_LOCK(sc);
	/* 
	 * input ^ int_polarity & int_mask
	   call GPIOBUS for every changed pin
	 */
	value  = GPIO_READ(sc, SIBA_CC_GPIO_INPUT);
	input  = value ^ GPIO_READ(sc, SIBA_CC_GPIO_INT_POLARITY);
	input &= GPIO_READ(sc, SIBA_CC_GPIO_INT_MASK);

	if (!input) goto intr_done;
	/*
	 * Invert polarity of pin interrup to get new value 
	 * When gpio down, next they up, so invert falling/rising edge
	 */
	GPIO_WRITE(sc, SIBA_CC_GPIO_INT_POLARITY, 
		    GPIO_READ(sc, SIBA_CC_GPIO_INT_POLARITY) ^ input);

	for ( i = 0; i < SIBA_CC_GPIO_PINS; i ++ )
	{
		if ( (( input >> i) & 1) &&
		     (((value & input) >> i) & 1) != sc->gpio_pins[i].gp_last )
		{
			/* !system=GPIO subsystem=pin7 type=PIN_HIGH period=3 */
			snprintf(notify , sizeof(notify ), "period=%d", 
			    (uint32_t)time_uptime - sc->gpio_pins[i].gp_time);
			snprintf(pinname, sizeof(pinname), "pin%02d", i);
			devctl_notify("GPIO", pinname, 
			    (((value & input) >> i) & 1)?"PIN_HIGH":"PIN_LOW", 
			    notify);
			sc->gpio_pins[i].gp_last = ((value & input) >> i) & 1;
			sc->gpio_pins[i].gp_time = time_uptime;
		}
	}

intr_done:
	GPIO_UNLOCK(sc);
	siba_cc_set_int_mask(device_get_parent(sc->dev), mask);
	return (FILTER_HANDLED);
}

static int
siba_cc_gpio_probe(device_t dev)
{
	device_set_desc(dev, "SIBA CC GPIO driver");
	return (0);
}

static int
siba_cc_gpio_attach(device_t dev)
{
	struct siba_cc_gpio_softc *sc = device_get_softc(dev);
	int error = 0, i;

	KASSERT((device_get_unit(dev) == 0),
	    ("siba_cc_gpio_gpio: Only one gpio module supported"));

	mtx_init(&sc->gpio_mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK,
	    MTX_DEF);

	/* Map control/status registers. */
	sc->gpio_mem_rid = 0;
	sc->gpio_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->gpio_mem_rid, RF_ACTIVE);

	if (sc->gpio_mem_res == NULL) {
		device_printf(dev, "couldn't map memory\n");
		error = ENXIO;
		siba_cc_gpio_detach(dev);
		return(error);
	}

	if ((sc->gpio_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, 
	    &sc->gpio_irq_rid, RF_SHAREABLE | RF_ACTIVE)) == NULL) {
		device_printf(dev, "unable to allocate IRQ resource\n");
		return (ENXIO);
	}

	if ((bus_setup_intr(dev, sc->gpio_irq_res, INTR_TYPE_MISC, 
	    /* siba_cc_gpio_filter, */
	    siba_cc_gpio_intr, NULL, sc, &sc->gpio_ih))) {
		device_printf(dev,
		    "WARNING: unable to register interrupt handler\n");
		return (ENXIO);
	}

	sc->dev = dev;
	/* Configure all pins as input */
	/* disable interrupts for all pins */
	GPIO_WRITE(sc, SIBA_CC_GPIO_INT_MASK, 0);
	GPIO_WRITE(sc, SIBA_CC_GPIO_OUTEN, 0);
	GPIO_WRITE(sc, SIBA_CC_GPIO_PULLDOWN, 0xffffffff );
	printf("\tSIBA_CC_GPIO_CONTROL = %#x\n", 
	    GPIO_READ(sc, SIBA_CC_GPIO_CONTROL));
	GPIO_WRITE(sc, SIBA_CC_GPIO_CONTROL, 0 );

	sc->gpio_npins = SIBA_CC_GPIO_PINS;
	resource_int_value(device_get_name(dev), device_get_unit(dev), 
	    "pins", &sc->gpio_npins);

	for (i = 0; i < sc->gpio_npins; i++) {
 		sc->gpio_pins[i].gp_pin = i;
 		sc->gpio_pins[i].gp_caps = DEFAULT_CAPS;
 		sc->gpio_pins[i].gp_flags = 0;
	}

	device_add_child(dev, "gpiobus", device_get_unit(dev));

	siba_cc_set_int_mask(device_get_parent(sc->dev),
	    siba_cc_get_int_mask(device_get_parent(sc->dev)) |
	    SIBA_CC_INT_GPIO);

	return (bus_generic_attach(dev));
}

static int
siba_cc_gpio_detach(device_t dev)
{
	struct siba_cc_gpio_softc *sc = device_get_softc(dev);

	KASSERT(mtx_initialized(&sc->gpio_mtx), ("gpio mutex not initialized"));

	bus_generic_detach(dev);

	if (sc->gpio_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->gpio_mem_rid,
		    sc->gpio_mem_res);

	mtx_destroy(&sc->gpio_mtx);

	return(0);
}

static device_method_t siba_cc_gpio_methods[] = {
	DEVMETHOD(device_probe, siba_cc_gpio_probe),
	DEVMETHOD(device_attach, siba_cc_gpio_attach),
	DEVMETHOD(device_detach, siba_cc_gpio_detach),

	/* GPIO protocol */
	DEVMETHOD(gpio_pin_max, siba_cc_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname, siba_cc_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getflags, siba_cc_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps, siba_cc_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags, siba_cc_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get, siba_cc_gpio_pin_get),
	DEVMETHOD(gpio_pin_set, siba_cc_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle, siba_cc_gpio_pin_toggle),
	{0, 0},
};

static driver_t siba_cc_gpio_driver = {
	"gpio",
	siba_cc_gpio_methods,
	sizeof(struct siba_cc_gpio_softc),
};
static devclass_t siba_cc_gpio_devclass;

DRIVER_MODULE(siba_cc_gpio, siba_cc, siba_cc_gpio_driver, 
    siba_cc_gpio_devclass, 0, 0);
