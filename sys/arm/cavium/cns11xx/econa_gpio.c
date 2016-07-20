/*-
 * Copyright (c) 2016 Hiroki Mori
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
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/time.h>
#include <sys/timetc.h>
#include <sys/watchdog.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>
#include <machine/intr.h>
#include <machine/fdt.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/cavium/cns11xx/econa_reg.h>
#include <arm/cavium/cns11xx/econa_var.h>

#define	EC_GPIO_NPINS	21

#define EC_GPIO_OUT       (0x0)
#define EC_GPIO_IN        (0x4)
#define EC_GPIO_DIR       (0x8)

#define GPIO_LOCK(_sc)		mtx_lock(&(_sc)->gpio_mtx)
#define GPIO_UNLOCK(_sc)	mtx_unlock(&(_sc)->gpio_mtx)

#include "gpio_if.h"

struct econa_gpio_softc
{
	device_t		lg_dev;
	device_t		lg_busdev;
	struct resource *	lg_res;
	bus_space_tag_t		lg_bst;
	bus_space_handle_t	lg_bsh;
	struct mtx		gpio_mtx;
	struct gpio_pin		*gpio_pins;
};

static int econa_gpio_probe(device_t);
static int econa_gpio_attach(device_t);
static int econa_gpio_detach(device_t);

static device_t econa_gpio_get_bus(device_t);
static int econa_gpio_pin_max(device_t, int *);
static int econa_gpio_pin_getcaps(device_t, uint32_t, uint32_t *);
static int econa_gpio_pin_getflags(device_t, uint32_t, uint32_t *);
static int econa_gpio_pin_setflags(device_t, uint32_t, uint32_t);
static int econa_gpio_pin_getname(device_t, uint32_t, char *);
static int econa_gpio_pin_get(device_t, uint32_t, uint32_t *);
static int econa_gpio_pin_set(device_t, uint32_t, uint32_t);
static int econa_gpio_pin_toggle(device_t, uint32_t);

int econa_gpio_set_flags(device_t, int, int);
int econa_gpio_set_state(device_t, int, int);
int econa_gpio_get_state(device_t, int, int *);

static struct econa_gpio_softc *econa_gpio_sc = NULL;

#define	econa_gpio_read_4(_sc, _reg) \
    bus_space_read_4(_sc->lg_bst, _sc->lg_bsh, _reg)
#define	econa_gpio_write_4(_sc, _reg, _val) \
    bus_space_write_4(_sc->lg_bst, _sc->lg_bsh, _reg, _val)

static int
econa_gpio_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "gpio"))
		return (ENXIO);

	device_set_desc(dev, "Econa GPIO");
	return (BUS_PROBE_DEFAULT);
}

static int
econa_gpio_attach(device_t dev)
{
	struct econa_gpio_softc *sc = device_get_softc(dev);
	int rid;

	sc->lg_dev = dev;

	mtx_init(&sc->gpio_mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	rid = 0;
	sc->lg_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->lg_res) {
		device_printf(dev, "cannot allocate memory window\n");
		return (ENXIO);
	}

	sc->lg_bst = rman_get_bustag(sc->lg_res);
	sc->lg_bsh = rman_get_bushandle(sc->lg_res);

	econa_gpio_sc = sc;

	sc->lg_busdev = gpiobus_attach_bus(dev);
	if (sc->lg_busdev == NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->lg_res);
		return (ENXIO);
	}

	sc->gpio_pins = malloc(sizeof(struct gpio_pin) * EC_GPIO_NPINS,
	    M_DEVBUF, M_WAITOK | M_ZERO);

	return (0);
}

static int
econa_gpio_detach(device_t dev)
{
	struct econa_gpio_softc *sc = device_get_softc(dev);

	mtx_destroy(&sc->gpio_mtx);

	return (EBUSY);
}

static device_t
econa_gpio_get_bus(device_t dev)
{
	struct econa_gpio_softc *sc;

	sc = device_get_softc(dev);

	return (sc->lg_busdev);
}

static int
econa_gpio_pin_max(device_t dev, int *npins)
{
	*npins = EC_GPIO_NPINS - 1;
	return (0);
}

static int
econa_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct econa_gpio_softc *sc = device_get_softc(dev);
	
	if (pin > EC_GPIO_NPINS)
		return (ENODEV);

	GPIO_LOCK(sc);
	*caps = sc->gpio_pins[pin].gp_caps;
	GPIO_UNLOCK(sc);

	return (0);
}

static int
econa_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct econa_gpio_softc *sc = device_get_softc(dev);
	int dir;

	if (pin > EC_GPIO_NPINS)
		return (ENODEV);

	dir = econa_gpio_read_4(sc, EC_GPIO_DIR) & (1 << pin);

	*flags = dir ? GPIO_PIN_OUTPUT : GPIO_PIN_INPUT;

	return (0);
}

static int
econa_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct econa_gpio_softc *sc = device_get_softc(dev);
	uint32_t dir, state;

	if (pin > EC_GPIO_NPINS)
		return (ENODEV);

	if (flags & GPIO_PIN_INPUT)
		dir = 0;

	if (flags & GPIO_PIN_OUTPUT)
		dir = 1;

	state = econa_gpio_read_4(sc, EC_GPIO_DIR);
	if (flags & GPIO_PIN_INPUT) {
		state &= ~(1 << pin);
	} else {
		state |= (1 << pin);
	}
	econa_gpio_write_4(sc, EC_GPIO_DIR, state);

	return (0);
}

static int
econa_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct econa_gpio_softc *sc = device_get_softc(dev);

	if (pin > EC_GPIO_NPINS)
		return (ENODEV);

	GPIO_LOCK(sc);
	memcpy(name, sc->gpio_pins[pin].gp_name, GPIOMAXNAME);
	GPIO_UNLOCK(sc);

	return (0);
}

static int
econa_gpio_pin_get(device_t dev, uint32_t pin, uint32_t *value)
{
	struct econa_gpio_softc *sc = device_get_softc(dev);
	uint32_t flags;

	if (econa_gpio_pin_getflags(dev, pin, &flags))
		return (ENXIO);

	*value = (econa_gpio_read_4(sc, EC_GPIO_IN) & (1 << pin)) ? 1 : 0;

	return (0);
}

static int
econa_gpio_pin_set(device_t dev, uint32_t pin, uint32_t value)
{
	struct econa_gpio_softc *sc = device_get_softc(dev);
	uint32_t state, flags;

	if (econa_gpio_pin_getflags(dev, pin, &flags))
		return (ENXIO);

	if ((flags & GPIO_PIN_OUTPUT) == 0)
		return (EINVAL);

	state = econa_gpio_read_4(sc, EC_GPIO_OUT);
	if(value == 1) {
		state |= (1 << pin);
	} else {
		state &= ~(1 << pin);
	}
	econa_gpio_write_4(sc, EC_GPIO_OUT, state);

	return (0);
}

static int
econa_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	uint32_t flags;

	if (econa_gpio_pin_getflags(dev, pin, &flags))
		return (ENXIO);

	if ((flags & GPIO_PIN_OUTPUT) == 0)
		return (EINVAL);
	
	panic("not implemented yet");

	return (0);

}

int
econa_gpio_set_flags(device_t dev, int pin, int flags)
{
	if (econa_gpio_sc == NULL)
		return (ENXIO);

	return econa_gpio_pin_setflags(econa_gpio_sc->lg_dev, pin, flags);
}

int
econa_gpio_set_state(device_t dev, int pin, int state)
{
	if (econa_gpio_sc == NULL)
		return (ENXIO);

	return econa_gpio_pin_set(econa_gpio_sc->lg_dev, pin, state); 
}

int
econa_gpio_get_state(device_t dev, int pin, int *state)
{
	if (econa_gpio_sc == NULL)
		return (ENXIO);

	return econa_gpio_pin_get(econa_gpio_sc->lg_dev, pin, state);
}

static phandle_t
econa_gpio_get_node(device_t bus, device_t dev)
{
	/* We only have one child, the GPIO bus, which needs our own node. */
	return (ofw_bus_get_node(bus));
}

static device_method_t econa_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		econa_gpio_probe),
	DEVMETHOD(device_attach,	econa_gpio_attach),
	DEVMETHOD(device_detach,	econa_gpio_detach),

	/* GPIO interface */
	DEVMETHOD(gpio_get_bus,		econa_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		econa_gpio_pin_max),
	DEVMETHOD(gpio_pin_getcaps,	econa_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_getflags,	econa_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,	econa_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_getname,	econa_gpio_pin_getname),
	DEVMETHOD(gpio_pin_set,		econa_gpio_pin_set),
	DEVMETHOD(gpio_pin_get,		econa_gpio_pin_get),
	DEVMETHOD(gpio_pin_toggle,	econa_gpio_pin_toggle),

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node,     econa_gpio_get_node),

	{ 0, 0 }
};

static devclass_t econa_gpio_devclass;

static driver_t econa_gpio_driver = {
	"gpio",
	econa_gpio_methods,
	sizeof(struct econa_gpio_softc),
};

EARLY_DRIVER_MODULE(econagpio, simplebus, econa_gpio_driver, econa_gpio_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
MODULE_VERSION(econagpio, 1);
