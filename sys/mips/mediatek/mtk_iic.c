/*-
 * Copyright (c) 2017 Hiroki Mori
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
 * 3. Neither the name of RMI Corporation, nor the names of its contributors,
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/*
 * This code do not test 
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/cpu.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include "iicbus_if.h"

#include "opt_platform.h"

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/mediatek/mtk_iic.h>

/*
 * register space access macros
 */
#define	I2C_WRITE(sc, reg, val) do {	\
		bus_write_4(sc->sc_mem_res, (reg), (val));	\
	} while (0)

#define	I2C_READ(sc, reg)	bus_read_4(sc->sc_mem_res, (reg))

/*
 * Device methods
 */
static int mtk_iic_probe(device_t);
static int mtk_iic_attach(device_t);
static int mtk_iic_detach(device_t);

static int mtk_iic_start(device_t dev, u_char slave, int timeout);
static int mtk_iic_stop(device_t dev);
static int mtk_iic_read(device_t dev, char *buf, int len, int *read, int last, int delay);
static int mtk_iic_write(device_t dev, const char *buf, int len, int *sent, int timeout);
static int mtk_iic_callback(device_t dev, int index, caddr_t data);
static int mtk_iic_repeated_start(device_t dev, u_char slave, int timeout);
static int mtk_iic_transfer(device_t bus, struct iic_msg *msgs, uint32_t nmsgs);

static struct ofw_compat_data compat_data[] = {
	{ "ralink,rt3050-i2c",  1 },
	{ NULL,                 0 }
};

struct mtk_iic_softc {
	device_t sc_dev;
	struct resource *sc_mem_res;
	device_t iicbus;
	unsigned int clkdiv;
	int sc_started;
	uint8_t i2cdev_addr;
	struct mtx sc_mtx;
};

static int
mtk_iic_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return(ENXIO);

	device_set_desc(dev, "MTK I2C Controller");

	return (0);
}

static int
mtk_iic_attach(device_t dev)
{
	int rid;
	struct mtk_iic_softc *sc;

	sc = device_get_softc(dev);

	mtx_init(&sc->sc_mtx, "mtk_iic", "mtk_iic", MTX_DEF);

	sc->sc_dev = dev;
	rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "Could not map memory\n");
		return (ENXIO);
	}

	sc->clkdiv = CLKDIV_VALUE;

	sc->iicbus = device_add_child(dev, "iicbus", -1);
	if (sc->iicbus == NULL) {
		device_printf(dev, "cannot add iicbus child device\n");
		return (ENXIO);
	}
	return (bus_generic_attach(dev));
}

static int
mtk_iic_detach(device_t dev)
{
	struct mtk_iic_softc *sc;

	sc = device_get_softc(dev);

	if (sc->sc_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);

	return (0);
}

static int 
mtk_iic_start(device_t dev, u_char slave, int timeout)
{
	int error = 0;
	struct mtk_iic_softc *sc;
	uint32_t r;

	sc = device_get_softc(dev);
        mtx_lock(&sc->sc_mtx);
	sc->sc_started = 1;
	sc->i2cdev_addr = (slave >> 1);

	r = I2C_CONFIG_ADDRLEN(I2C_CONFIG_ADDRLEN_8) |
	    I2C_CONFIG_DEVADLEN(I2C_CONFIG_DEVADLEN_7) |
	    I2C_CONFIG_ADDRDIS;
	I2C_WRITE(sc, RA_I2C_CONFIG, r);

	I2C_WRITE(sc, RA_I2C_CLKDIV, sc->clkdiv);

	return error;

}

static int 
mtk_iic_stop(device_t dev)
{
	int error = 0;
	struct mtk_iic_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->sc_mtx);
	return error;

}

static inline void
mtk_iic_busy_wait(struct mtk_iic_softc *sc)
{
	int i;
	uint32_t r;

	for (i = 0; i < i2c_busy_loop; ++i) {
		r = I2C_READ(sc, RA_I2C_STATUS);
		if ((r & I2C_STATUS_BUSY) == 0)
			break;
	}
}

static int 
mtk_iic_read(device_t dev, char *buf, int len, int *read, int last,
    int delay)
{
	struct mtk_iic_softc *sc;
	int i, j;
	uint32_t r;

	sc = device_get_softc(dev);

	I2C_WRITE(sc, RA_I2C_DEVADDR, sc->i2cdev_addr);
	I2C_WRITE(sc, RA_I2C_BYTECNT, len - 1);
	I2C_WRITE(sc, RA_I2C_STARTXFR, I2C_OP_READ);

	for (i=0; i < len; i++) {
		for (j=0; j < max_ee_busy_loop; j++) {
			r = I2C_READ(sc, RA_I2C_STATUS);
			if ((r & I2C_STATUS_DATARDY) != 0) {
				buf[i] = I2C_READ(sc, RA_I2C_DATAIN);
				break;
			}
		}
		if (j == max_ee_busy_loop) {
			return -1;
		}
	}

	*read =  i;

	mtk_iic_busy_wait(sc);
	return 0;

}

static int 
mtk_iic_write(device_t dev, const char *buf, int len, int *sent,
    int timeout /* us */ )
{
	struct mtk_iic_softc *sc;
	int i, j;
	uint32_t r;

	sc = device_get_softc(dev);

	I2C_WRITE(sc, RA_I2C_DEVADDR, sc->i2cdev_addr);
	I2C_WRITE(sc, RA_I2C_BYTECNT, len - 1);
	I2C_WRITE(sc, RA_I2C_STARTXFR, I2C_OP_WRITE);

	for (i=0; i < len; i++) {
		for (j=0; j < max_ee_busy_loop; j++) {
			r = I2C_READ(sc, RA_I2C_STATUS);
			if ((r & I2C_STATUS_SDOEMPTY) != 0) {
				I2C_WRITE(sc, RA_I2C_DATAOUT, buf[i]);
				break;
			}
		}
		if (j == max_ee_busy_loop) {
			return -1;
		}
	}

	*sent = i;

	mtk_iic_busy_wait(sc);
	return 0;
}

static int
mtk_iic_callback(device_t dev, int index, caddr_t data)
{
	return 0;
}

static int
mtk_iic_repeated_start(device_t dev, u_char slave, int timeout)
{
	return 0;
}

int
mtk_iic_transfer(device_t bus, struct iic_msg *msgs, uint32_t nmsgs)
{       
        int i, error, lenread, lenwrote;
        u_char addr;
 
	addr = msgs[0].slave | LSB;
	error = mtk_iic_start(bus, addr, 0);
        for (i = 0, error = 0; i < nmsgs && error == 0; i++) {
                if (msgs[i].flags & IIC_M_RD) {
		        error = mtk_iic_read((bus), msgs[i].buf, msgs[i].len,
			    &lenread, IIC_LAST_READ, 0);
		}
                else {    
		        error = mtk_iic_write((bus), msgs[i].buf, msgs[i].len,
			    &lenwrote, 0);
		}
        }	
	error = mtk_iic_stop(bus);
        return (error);
}

static int
mtk_iic_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	return (IIC_ENOADDR);
}

static phandle_t
mtk_iic_get_node(device_t bus, device_t dev)
{

	/* We only have one child, the I2C bus, which needs our own node. */
	return (ofw_bus_get_node(bus));
}


static device_method_t mtk_iic_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe, mtk_iic_probe),
	DEVMETHOD(device_attach, mtk_iic_attach),
	DEVMETHOD(device_detach, mtk_iic_detach),

	/* Bus interface */
	DEVMETHOD(bus_setup_intr,	bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,	bus_generic_teardown_intr),
	DEVMETHOD(bus_alloc_resource,	bus_generic_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_release_resource),
	DEVMETHOD(bus_activate_resource, bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_adjust_resource,	bus_generic_adjust_resource),
	DEVMETHOD(bus_set_resource,	bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,	bus_generic_rl_get_resource),

	/* iicbus interface */
	DEVMETHOD(iicbus_reset, mtk_iic_reset),
	DEVMETHOD(iicbus_callback, mtk_iic_callback),
	DEVMETHOD(iicbus_repeated_start, mtk_iic_repeated_start),
	DEVMETHOD(iicbus_start, mtk_iic_start),
	DEVMETHOD(iicbus_stop, mtk_iic_stop),
	DEVMETHOD(iicbus_write, mtk_iic_write),
	DEVMETHOD(iicbus_read, mtk_iic_read),
	DEVMETHOD(iicbus_transfer, mtk_iic_transfer),

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node, mtk_iic_get_node),

	DEVMETHOD_END
};

static driver_t mtk_iic_driver = {
	"mtkiic",
	mtk_iic_methods,
	sizeof(struct mtk_iic_softc),
};

static devclass_t mtk_iic_devclass;

DRIVER_MODULE(mtk_iic, simplebus, mtk_iic_driver, mtk_iic_devclass, 0, 0);
DRIVER_MODULE(iicbus, mtk_iic, iicbus_driver, iicbus_devclass, 0, 0);

MODULE_DEPEND(mtk_iic, iicbus, 1, 1, 1);

MODULE_VERSION(mtk_iic, 1);
