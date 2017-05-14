/*-
 * Copyright (c) 2017 Hiroki Mori
 * Copyright (c) 2016 Ganbold Tsagaankhuu <ganbold@freebsd.org>
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
 * This is 74HC153 input evdev driver on gpio bus
 * Developed on Buffalo WZR-HP-G300NH
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <machine/bus.h>

#include <sys/gpio.h>
#include <dev/gpio/gpiobusvar.h>

#include "gpiobus_if.h"

#include <dev/evdev/input.h>
#include <dev/evdev/evdev.h>

struct hc153key_softc {
	device_t		sc_dev;
	device_t		sc_busdev;
	int			pina;
	int			pinb;
	int			pin1y;
	int			pin2y;
	int			lastpins;
	int			swmask;
	struct callout		scan_callout;
	struct evdev_dev	*sc_evdev;
};

static int
getpins(struct hc153key_softc *sc)
{
	int pins;
	int i;
	int pin1, pin2;

	pins = 0;

	for (i = 0; i < 4; ++i) {
		GPIOBUS_PIN_SET(sc->sc_busdev, sc->sc_dev,
		    sc->pina, i % 2);
		GPIOBUS_PIN_SET(sc->sc_busdev, sc->sc_dev,
		    sc->pinb, i / 2);
		DELAY(10);
		GPIOBUS_PIN_GET(sc->sc_busdev, sc->sc_dev,
		    sc->pin1y, &pin1);
		GPIOBUS_PIN_GET(sc->sc_busdev, sc->sc_dev,
		    sc->pin2y, &pin2);
		pins |= ((pin1 ? 1 : 0) << (i * 2 + 1)) |
		    ((pin2 ? 1 : 0) << (i * 2));
	}

	return (pins);
}

static void
key_scan(void *arg)
{
	struct hc153key_softc *sc;
	int pins;
	int bitmask;
	int i, j;

	sc = arg;

	pins = getpins(sc);

	/* push button check */
	bitmask = ~sc->swmask & 0xff;
	if (bitmask != 0 && (sc->lastpins & bitmask) != (pins & bitmask)) {
		evdev_push_event(sc->sc_evdev,
		    EV_MSC, MSC_SCAN, ~pins & bitmask);
		evdev_sync(sc->sc_evdev);
	}
	/* slide switch check */
	bitmask = sc->swmask;
	if (bitmask != 0 && (sc->lastpins & bitmask) != (pins & bitmask)) {
		j = 0;
		for (i = 0; i < 8; ++i) {
			if (bitmask & (1 << i)) {
				evdev_push_sw(sc->sc_evdev, j,
				    (~pins & (1 << i)) ? 1 : 0);
				++j;
			}
		}
		evdev_sync(sc->sc_evdev);
	}
	sc->lastpins = pins;

	callout_reset(&sc->scan_callout, hz, key_scan, sc);
}

static int
hc153key_probe(device_t dev)
{

	device_set_desc(dev, "GPIO 74HC153 evdev controller");
	return (BUS_PROBE_DEFAULT);
}

static int
hc153key_attach(device_t dev)
{
	struct hc153key_softc *sc;
	int err;
	int value;
	int i, j;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_busdev = device_get_parent(dev);

	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "pina", &value))
		return (ENXIO);
	sc->pina = value & 0xff;

	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "pinb", &value))
		return (ENXIO);
	sc->pinb = value & 0xff;

	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "pin1y", &value))
		return (ENXIO);
	sc->pin1y = value & 0xff;

	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "pin2y", &value))
		return (ENXIO);
	sc->pin2y = value & 0xff;

	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "swmask", &value))
		return (ENXIO);
	sc->swmask = value & 0xff;

	GPIOBUS_PIN_SETFLAGS(sc->sc_busdev, sc->sc_dev, sc->pina,
	    GPIO_PIN_OUTPUT);
	GPIOBUS_PIN_SETFLAGS(sc->sc_busdev, sc->sc_dev, sc->pinb,
	    GPIO_PIN_OUTPUT);
	GPIOBUS_PIN_SETFLAGS(sc->sc_busdev, sc->sc_dev, sc->pin1y,
	    GPIO_PIN_INPUT);
	GPIOBUS_PIN_SETFLAGS(sc->sc_busdev, sc->sc_dev, sc->pin2y,
	    GPIO_PIN_INPUT);

	sc->sc_evdev = evdev_alloc();
	evdev_set_name(sc->sc_evdev, device_get_desc(sc->sc_dev));
	evdev_set_phys(sc->sc_evdev, device_get_nameunit(sc->sc_dev));
	evdev_set_id(sc->sc_evdev, BUS_HOST, 0, 0, 0);

	/* key regist */
	if (sc->swmask != 0xff) {
		evdev_support_event(sc->sc_evdev, EV_SYN);
		evdev_support_event(sc->sc_evdev, EV_MSC);
		evdev_support_msc(sc->sc_evdev, MSC_SCAN);
	}

	/* switch regist */
	if (sc->swmask != 0x00) {
		j = 0;
		evdev_support_event(sc->sc_evdev, EV_SW);
		for (i = 0; i < 8; ++i) {
			if (sc->swmask & (1 << i)) {
				evdev_support_sw(sc->sc_evdev, j);
				++j;
			}
		}
	}

	err = evdev_register(sc->sc_evdev);
	if (err) {
		device_printf(dev,
		    "failed to register evdev: error=%d\n", err);
		goto error;
	}

	sc->lastpins = getpins(sc);

	/* push initial value */
	if (sc->swmask != 0x00) {
		j = 0;
		for (i = 0; i < 8; ++i) {
			if (sc->swmask & (1 << i)) {
				evdev_push_sw(sc->sc_evdev,
				    j, (~sc->lastpins & (1 << i)) ? 1 : 0);
				++j;
			}
		}
		evdev_sync(sc->sc_evdev);
	}

	callout_init(&sc->scan_callout, 0);

	callout_reset(&sc->scan_callout, hz, key_scan, sc);

	return (0);
error:
	return (ENXIO);
}

static device_method_t hc153key_methods[] = {
	DEVMETHOD(device_probe, hc153key_probe),
	DEVMETHOD(device_attach, hc153key_attach),

	DEVMETHOD_END
};

static driver_t hc153key_driver = {
	"hc153key",
	hc153key_methods,
	sizeof(struct hc153key_softc),
};
static devclass_t hc153key_devclass;

DRIVER_MODULE(hc153key, gpiobus, hc153key_driver, hc153key_devclass, 0, 0);
MODULE_DEPEND(hc153key, evdev, 1, 1, 1);
