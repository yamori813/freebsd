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
 * Child driver for SDRAM/DDR controller core.
 * Generally the OS should not need to access this device unless the
 * firmware has not configured the SDRAM controller.
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

static int	siba_sdram_attach(device_t);
static int	siba_sdram_probe(device_t);

static int
siba_sdram_probe(device_t dev)
{

	if (siba_get_vendor(dev) == SIBA_VID_BROADCOM &&
	    siba_get_device(dev) == SIBA_DEVID_SDRAMDDR) {
		device_set_desc(dev, "SDRAM/DDR core");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

struct siba_sdram_softc {
	bus_space_tag_t		 sc_bt;
	bus_space_handle_t	 sc_bh;
	bus_addr_t		 sc_maddr;
	bus_size_t		 sc_msize;
	struct resource *sc_mem;
	struct resource *sc_irq;
};

static int
siba_sdram_attach(device_t dev)
{
	struct siba_sdram_softc *sc = device_get_softc(dev);
	int rid;
	uint32_t	corerev;
	uint32_t	coreinfo;
	uint32_t	memsize;
	BUS_READ_IVAR(device_get_parent(dev), dev, SIBA_IVAR_CORE_ADDRESS_SPACE1, (uintptr_t *)&corerev);

	/*
	 * Allocate the resources which the parent bus has already
	 * determined for us.
	 * TODO: interrupt routing
	 */
	rid = 0;
	sc->sc_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		device_printf(dev, "unable to allocate memory\n");
		return (ENXIO);
	}

	sc->sc_bt = rman_get_bustag(sc->sc_mem);
	sc->sc_bh = rman_get_bushandle(sc->sc_mem);
	sc->sc_maddr = rman_get_start(sc->sc_mem);
	sc->sc_msize = rman_get_size(sc->sc_mem);


#define	SRCI_MS0_MASK		0xf
#define SR_MS0_BASE		16
#define	SRCI_SRNB_MASK		0xf0
#define	SRCI_SRNB_SHIFT		4
#define	SRCI_SRBSZ_MASK		0xf
#define	SRCI_SRBSZ_SHIFT	0
#define SR_BSZ_BASE		14
#define SRCI_LSS_MASK		0x00f00000
#define SRCI_LSS_SHIFT		20


#define SDRAM_CORE_INFO		0


	coreinfo = bus_space_read_4((sc)->sc_bt, (sc)->sc_bh,SDRAM_CORE_INFO);


	/* Calculate size from coreinfo based on rev */
	if (corerev == 0) {
		memsize = 1 << (SR_MS0_BASE + (coreinfo & SRCI_MS0_MASK));
	}
	else if (corerev < 3) {
		memsize = 1 << (SR_BSZ_BASE + ((coreinfo & SRCI_SRBSZ_MASK) >> SRCI_SRBSZ_SHIFT));
		memsize *= (coreinfo & SRCI_SRNB_MASK) >> SRCI_SRNB_SHIFT;
	}
	else {
		uint32_t nb = (coreinfo & SRCI_SRNB_MASK) >> SRCI_SRNB_SHIFT;
		uint32_t bsz = (coreinfo & SRCI_SRBSZ_MASK);
		uint32_t lss = (coreinfo & SRCI_LSS_MASK) >> SRCI_LSS_SHIFT;
		if (lss != 0)
			nb --;
		memsize = nb * (1 << (bsz + SR_BSZ_BASE));
		if (lss != 0)
			memsize += (1 << ((lss - 1) + SR_BSZ_BASE));
	}


	/* No one say, but if two different devices 2x16M and 1x32M say 0x00020000, 
	 * so we shift it by 8 to get 32M
	 */
	memsize <<= 8;
	device_printf(dev, "Rev = %d, Total Memsize = %d MB\n", corerev,memsize>>20);



#if 0
	device_printf(dev, "start %08lx size %04lx\n",
	    rman_get_start(mem), rman_get_size(mem));
#endif

	return (0);
}

static device_method_t siba_sdram_methods[] = {
	/* Device interface */
	DEVMETHOD(device_attach,	siba_sdram_attach),
	DEVMETHOD(device_probe,		siba_sdram_probe),

	{0, 0},
};

static driver_t siba_sdram_driver = {
	"siba_sdram",
	siba_sdram_methods,
	sizeof(struct siba_softc),
};
static devclass_t siba_sdram_devclass;

DRIVER_MODULE(siba_sdram, siba, siba_sdram_driver, siba_sdram_devclass, 0, 0);
