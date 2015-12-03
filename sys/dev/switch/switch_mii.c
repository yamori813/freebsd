/*-
 * Copyright (c) 2011,2012 Aleksandr Rybalko
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
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/bus.h>

/* Required for struct switch_softc */
#include <machine/bus.h>
/* MIIBUS_READREG/MIIBUS_WRITEREG macro */
#include "miibus_if.h"

/* Switch interface */
#include <dev/switch/switchvar.h>
#include "switch_if.h"
#include "switchb_if.h"
#include "switchpub_if.h"

static int 	switch_mii_probe(device_t);
static int 	switch_mii_attach(device_t);
static uint32_t	switch_mii_read4(device_t, uint32_t);
static void	switch_mii_write4(device_t, uint32_t, uint32_t);

static device_method_t switch_mii_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe,		switch_mii_probe),
	DEVMETHOD(device_attach,	switch_mii_attach),
	DEVMETHOD(device_detach,	mii_phy_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),

	/* switch bus interface */
	DEVMETHOD(switchb_read4,	switch_mii_read4),
	DEVMETHOD(switchb_write4,	switch_mii_write4),

	/* switch public interface (used by floatphy) */
	DEVMETHOD(switchpub_get_reg,	switchpub_get_reg),
	DEVMETHOD(switchpub_set_reg,	switchpub_set_reg),

	{ 0, 0 }
};

static devclass_t switch_mii_devclass;

static driver_t switch_mii_driver = {
	"switch",
	switch_mii_methods,
	sizeof(struct switch_softc)
};

DRIVER_MODULE(switch, miibus, switch_mii_driver, switch_mii_devclass, 0, 0);
DRIVER_MODULE(switch, mii, switch_mii_driver, switch_mii_devclass, 0, 0);

static int
switch_mii_probe(device_t dev)
{

	return (BUS_PROBE_NOWILDCARD);
}

static int
switch_mii_attach(device_t dev)
{
	struct switch_softc	*ssc;
	int err;

	ssc = device_get_softc(dev);
	ssc->sc_dev = dev;
	ssc->sc_miidev = device_get_parent(dev);
	/* Satisfy all MDIO attached switch drivers */
	ssc->phys = 0xffffffff;

	SWITCH_LOCK_INIT(ssc);
	err = switch_init(ssc);
	if (err)
		return (err);

	return (bus_generic_attach(dev));
}

static uint32_t
switch_mii_read4(device_t dev, uint32_t r)
{
	struct switch_softc *ssc;
	int phy, reg;

	ssc = device_get_softc(dev);
	phy = (r >> 8) & 0xff;
	reg = r & 0xff;

	return (MIIBUS_READREG(ssc->sc_miidev, phy, reg) & 0xffff);
}

static void
switch_mii_write4(device_t dev, uint32_t r, uint32_t val)
{
	struct switch_softc *ssc;
	int phy, reg;

	ssc = device_get_softc(dev);
	phy = (r >> 8) & 0xff;
	reg = r & 0xff;

	MIIBUS_WRITEREG(ssc->sc_miidev, phy, reg, (val & 0xffff));
}

