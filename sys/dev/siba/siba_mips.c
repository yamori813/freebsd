/*-
 * Copyright (c) 2007 Bruce M. Simpson.
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
 * Child driver for MIPS 3302 core.
 * Interrupt controller registers live here. Interrupts may not be routed
 * to the MIPS core if they are masked out.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/malloc.h>

#include <machine/bus.h>

#include <dev/siba/sibavar.h>
#include <dev/siba/sibareg.h>
#include <dev/siba/siba_ids.h>


#define siba_mips_read_1(sc, reg)				\
	bus_space_read_1((sc)->sc_bt, (sc)->sc_bh,(reg))

#define siba_mips_read_2(sc, reg)				\
	bus_space_read_2((sc)->sc_bt, (sc)->sc_bh,(reg))

#define siba_mips_read_4(sc, reg)				\
	bus_space_read_4((sc)->sc_bt, (sc)->sc_bh,(reg))

#define siba_mips_write_1(sc, reg, val)			\
	bus_space_write_1((sc)->sc_bt, (sc)->sc_bh,	\
			 (reg), (val))

#define siba_mips_write_2(sc, reg, val)			\
	bus_space_write_2((sc)->sc_bt, (sc)->sc_bh,	\
			 (reg), (val))

#define siba_mips_write_4(sc, reg, val)			\
	bus_space_write_4((sc)->sc_bt, (sc)->sc_bh,	\
			 (reg), (val))



static int	siba_mips_attach(device_t);
static int	siba_mips_probe(device_t);

static int
siba_mips_probe(device_t dev)
{

	if (siba_get_vendor(dev) == SIBA_VID_BROADCOM &&
	    siba_get_device(dev) == SIBA_DEVID_MIPS_3302) {
		device_set_desc(dev, "MIPS 3302 processor");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

struct siba_mips_softc {
	bus_space_tag_t		 sc_bt;
	bus_space_handle_t	 sc_bh;
	bus_addr_t		 sc_maddr;
	bus_size_t		 sc_msize;

	struct resource *sc_mem;
};

static int
siba_mips_attach(device_t dev)
{
	struct siba_mips_softc *sc = device_get_softc(dev);
	int rid;

	/*
	 * Allocate the resources which the parent bus has already
	 * determined for us.
	 * TODO: interrupt routing
	 */
	rid = 0;
	sc->sc_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		device_printf(dev, "unable to allocate memory\n");
		return (ENXIO);
	}

	sc->sc_bt = rman_get_bustag(sc->sc_mem);
	sc->sc_bh = rman_get_bushandle(sc->sc_mem);
	sc->sc_maddr = rman_get_start(sc->sc_mem);
	sc->sc_msize = rman_get_size(sc->sc_mem);

	device_printf(dev, "SBIPSFlag=%08x\n"     , 
	    siba_mips_read_4(sc, SIBA_IPSFLAG));
	device_printf(dev, "SBIPSFlagHigh=%08x\n" , 
	    siba_mips_read_4(sc, SIBA_IPSFLAGH));
	device_printf(dev, "SBIntVec=%08x\n"      , 
	    siba_mips_read_4(sc, SIBA_INTVEC));

	return (0);
}

static device_method_t siba_mips_methods[] = {
	/* Device interface */
	DEVMETHOD(device_attach,	siba_mips_attach),
	DEVMETHOD(device_probe,		siba_mips_probe),

	{0, 0},
};

static driver_t siba_mips_driver = {
	"siba_mips",
	siba_mips_methods,
	sizeof(struct siba_softc),
};
static devclass_t siba_mips_devclass;

DRIVER_MODULE(siba_mips, siba, siba_mips_driver, siba_mips_devclass, 0, 0);
