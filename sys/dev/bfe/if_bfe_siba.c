/*-
 * Copyright (c) 2010 Rybalko Aleksandr<ray@dlink.ua>
 * aka Alex RAY <ray@ddteam.net>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS 'AS IS' AND
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

#define BFE_MDIO


#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_vlan_var.h>

#include <machine/bus.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

/* "device miibus" required.  See GENERIC if you get errors here. */
#include "miibus_if.h"

#if defined(BFE_MDIO)
#include <dev/mdio/mdio.h>
#include <dev/etherswitch/miiproxy.h>
#include "mdio_if.h"
#endif

#include <dev/siba/siba_ids.h>
#include <dev/siba/sibareg.h>
#include <dev/siba/sibavar.h>

#include <dev/bfe/if_bfereg.h>
#include <dev/bfe/if_bfevar.h>


static int  bfe_siba_probe	(device_t);
static int  bfe_siba_attach	(device_t);
static void bfe_siba_setup 	(struct bfe_softc *, u_int32_t);

/*
 * Probe for a Broadcom 4401 chip.
 */
static int
bfe_siba_probe(device_t dev)
{
	if (siba_get_vendor(dev) == SIBA_VID_BROADCOM &&
	    siba_get_device(dev) == SIBA_DEVID_ETHERNET) {
		device_set_desc(dev, "Broadcom 44xx Ethernet Chip");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}



static void
bfe_siba_setup(struct bfe_softc *sc, u_int32_t cores)
{
	u_int32_t val;

	val = CSR_READ_4(sc, BFE_SBINTVEC);
	val |= cores;
	CSR_WRITE_4(sc, BFE_SBINTVEC, val);
}

static int
bfe_siba_attach(device_t dev)
{
	struct bfe_softc *sc = device_get_softc(dev);

	sc->bfe_setup = bfe_siba_setup;
	sc->bfe_dev = dev;
	sc->bfe_memrid = 0;
	sc->bfe_irqrid = 0;
	sc->bfe_dma_offset = 0;

	return (bfe_attach(dev));
}

static device_method_t bfe_siba_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bfe_siba_probe),
	DEVMETHOD(device_attach,	bfe_siba_attach),
	DEVMETHOD(device_detach,	bfe_detach),
	DEVMETHOD(device_shutdown,	bfe_shutdown),
	DEVMETHOD(device_suspend,	bfe_suspend),
	DEVMETHOD(device_resume,	bfe_resume),

	/* bus interface */
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD(bus_driver_added,	bus_generic_driver_added),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	bfe_miibus_readreg),
	DEVMETHOD(miibus_writereg,	bfe_miibus_writereg),
	DEVMETHOD(miibus_statchg,	bfe_miibus_statchg),

	{ 0, 0 }
};

static driver_t bfe_driver = {
	"bfe",
	bfe_siba_methods,
	sizeof(struct bfe_softc)
};

static devclass_t bfe_devclass;

MODULE_DEPEND(bfe, siba, 1, 1, 1);
MODULE_DEPEND(bfe, ether, 1, 1, 1);
MODULE_DEPEND(bfe, miibus, 1, 1, 1);

DRIVER_MODULE(bfe, siba, bfe_driver, bfe_devclass, 0, 0);
DRIVER_MODULE(miibus, bfe, miibus_driver, miibus_devclass, 0, 0);

#if defined(BFE_MDIO)
static int bfemdio_probe(device_t);
static int bfemdio_attach(device_t);
static int bfemdio_detach(device_t);

/*
 * Declare an additional, separate driver for accessing the MDIO bus.
 */
static device_method_t bfemdio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,         bfemdio_probe),
	DEVMETHOD(device_attach,        bfemdio_attach),
	DEVMETHOD(device_detach,        bfemdio_detach),

	/* bus interface */
	DEVMETHOD(bus_add_child,        device_add_child_ordered),
        
	/* MDIO access */
//	DEVMETHOD(mdio_readreg,         bfe_miibus_readreg),
//	DEVMETHOD(mdio_writereg,        bfe_miibus_writereg),
};

DEFINE_CLASS_0(bfemdio, bfemdio_driver, bfemdio_methods,
    sizeof(struct bfe_softc));
static devclass_t bfemdio_devclass;

DRIVER_MODULE(miiproxy, bfe, miiproxy_driver, miiproxy_devclass, 0, 0);
DRIVER_MODULE(bfemdio, nexus, bfemdio_driver, bfemdio_devclass, 0, 0);
DRIVER_MODULE(mdio, bfemdio, mdio_driver, mdio_devclass, 0, 0);

static int bfemdio_probe(device_t dev)
{
	device_set_desc(dev, "Broadcom 44xx Ethernet Chip ethernet interface, MDIO controller");
	return (0);
}

static int bfemdio_attach(device_t dev)
{
	return (0);
}

static int bfemdio_detach(device_t dev)
{
	return (0);
}
#endif


