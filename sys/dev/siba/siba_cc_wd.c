/*-
 * Copyright (c) 2006 Sam Leffler.  All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * SiBa Common Chip Core Watchdog Timer Support.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/time.h>
#include <sys/bus.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/watchdog.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/frame.h>
#include <machine/resource.h>
#include <machine/intr.h>


struct siba_cc_wdog_softc {
	device_t		sc_dev;
};


static void
siba_cc_wdog_fn(void *arg, u_int cmd, int *error)
{
	struct siba_cc_wdog_softc *sc = arg;
	u_int u = cmd & WD_INTERVAL;

}

static int
siba_cc_wdog_probe(device_t dev)
{
	device_set_desc(dev, "SiBa Common Chip Core Watchdog Timer");
	return (0);
}

static int
siba_cc_wdog_attach(device_t dev)
{
	struct siba_cc_wdog_softc *sc = device_get_softc(dev);

	sc->sc_dev = dev;

	EVENTHANDLER_REGISTER(watchdog_list, siba_cc_wdog_fn, sc, 0);
	return (0);
}

static device_method_t siba_cc_wdog_methods[] = {
	DEVMETHOD(device_probe,		siba_cc_wdog_probe),
	DEVMETHOD(device_attach,	siba_cc_wdog_attach),
	{0, 0},
};

static driver_t siba_cc_wdog_driver = {
	"siba_cc_wdog",
	siba_cc_wdog_methods,
	sizeof(struct siba_cc_wdog_softc),
};
static devclass_t siba_cc_wdog_devclass;
DRIVER_MODULE(siba_cc_wdog, siba_cc, siba_cc_wdog_driver, siba_cc_wdog_devclass, 0, 0);
