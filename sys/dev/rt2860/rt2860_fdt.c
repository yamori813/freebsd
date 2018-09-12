
/*-
 * Copyright (c) 2018 Hiroki Mori
 * Copyright (c) 2009-2010 Alexander Egorenkov <egorenar@gmail.com>
 * Copyright (c) 2009 Damien Bergamini <damien.bergamini@free.fr>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/rt2860/rt2860_softc.h>
#include <dev/rt2860/rt2860_reg.h>
#include <dev/rt2860/rt2860_eeprom.h>
#include <dev/rt2860/rt2860_txwi.h>
#include <dev/rt2860/rt2860_rxwi.h>
#include <dev/rt2860/rt2860_io.h>
#include <dev/rt2860/rt2860_read_eeprom.h>
#include <dev/rt2860/rt2860_led.h>
#include <dev/rt2860/rt2860_rf.h>
#include <dev/rt2860/rt2860_debug.h>


static const struct ofw_compat_data rt_compat_data[] = {
	{ "ralink,rt2880-wmac",		RT_CHIPID_RT2880 },
	{ "ralink,rt3050-wmac",		RT_CHIPID_RT3050 },
	{ "ralink,rt3052-wmac",		RT_CHIPID_RT3052 },
	{ NULL,				0 }
};

/*
 * Static function prototypes
 */

static int rt2860_fdt_probe(device_t dev);

static int rt2860_fdt_attach(device_t dev);

int rt2860_attach(device_t dev, int id);

int rt2860_detach(device_t dev);

int rt2860_shutdown(device_t dev);

int rt2860_suspend(device_t dev);

int rt2860_resume(device_t dev);

/*
 * rt2860_fdt_probe
 */
static int rt2860_fdt_probe(device_t dev)
{
	const struct ofw_compat_data * cd;


	cd = ofw_bus_search_compatible(dev, rt_compat_data);
	if (cd->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Ralink RT2860 802.11n MAC/BPP");
	return 0;
}

/*
 * rt2860_fdt_attach
 */
static int rt2860_fdt_attach(device_t dev)
{
	struct rt2860_softc *sc;
	const struct ofw_compat_data * cd;

	sc = device_get_softc(dev);
	sc->mem_rid = 0;
	sc->bustype = RT2860_RBUS;

	cd = ofw_bus_search_compatible(dev, rt_compat_data);

	return (rt2860_attach(dev, cd->ocd_data));
}

static device_method_t rt2860_fdt_dev_methods[] =
{
	DEVMETHOD(device_probe, rt2860_fdt_probe),
	DEVMETHOD(device_attach, rt2860_fdt_attach),
	DEVMETHOD(device_detach, rt2860_detach),
	DEVMETHOD(device_shutdown, rt2860_shutdown),
	DEVMETHOD(device_suspend, rt2860_suspend),
	DEVMETHOD(device_resume, rt2860_resume),
	{ 0, 0 }
};

static driver_t rt2860_fdt_driver =
{
	"rt2860",
	rt2860_fdt_dev_methods,
	sizeof(struct rt2860_softc)
};

static devclass_t rt2860_fdt_dev_class;

DRIVER_MODULE(rt2860, simplebus, rt2860_fdt_driver, rt2860_fdt_dev_class, 0, 0);
MODULE_DEPEND(rt2860, wlan, 1, 1, 1);

