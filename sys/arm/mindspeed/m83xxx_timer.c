/*-
 * Copyright (c) 2015 Hiroki Mori
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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/timetc.h>
#include <sys/timeet.h>
#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/mindspeed/m83xxxreg.h>

struct m83xxx_timer_softc {
	device_t		ct_dev;
	struct eventtimer	ct_et;
	struct resource	*	ct_res[8];
	bus_space_tag_t		ct_bst0;
	bus_space_handle_t	ct_bsh0;
};

static struct resource_spec m83xxx_timer_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		1,	RF_ACTIVE },
	{ SYS_RES_IRQ,		2,	RF_ACTIVE },
	{ SYS_RES_IRQ,		3,	RF_ACTIVE },
	{ SYS_RES_IRQ,		4,	RF_ACTIVE },
	{ SYS_RES_IRQ,		5,	RF_ACTIVE },
	{ -1, 0 }
};

static struct m83xxx_timer_softc *timer_softc = NULL;
static int m83xxx_timer_initialized = 0;
static int m83xxx_timer_probe(device_t);
static int m83xxx_timer_attach(device_t);
static int m83xxx_timer_start(struct eventtimer *,
    sbintime_t first, sbintime_t period);
static int m83xxx_timer_stop(struct eventtimer *et);
static unsigned m83xxx_get_timecount(struct timecounter *);
static int m83xxx_hardclock(void *);

#define	timer_read_4(sc, reg)			\
    bus_space_read_4(sc->ct_bst0, sc->ct_bsh0, reg)
#define	timer_write_4(sc, reg, val)		\
    bus_space_write_4(sc->ct_bst0, sc->ct_bsh0, reg, val)

static struct timecounter m83xxx_timecounter = {
	.tc_get_timecount = m83xxx_get_timecount,
	.tc_name = "Comcerto Timer1",
	.tc_frequency = 0, /* will be filled later */
	.tc_counter_mask = ~0u,
	.tc_quality = 1000,
};

static int
m83xxx_timer_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "comcerto,timer"))
		return (ENXIO);

	device_set_desc(dev, "Comcerto Timer");
	return (BUS_PROBE_DEFAULT);
}

static int
m83xxx_timer_attach(device_t dev)
{
	void *intrcookie;
	struct m83xxx_timer_softc *sc = device_get_softc(dev);
	phandle_t node;
	uint32_t freq;

	if (timer_softc)
		return (ENXIO);

	timer_softc = sc;

	if (bus_alloc_resources(dev, m83xxx_timer_spec, sc->ct_res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	sc->ct_bst0 = rman_get_bustag(sc->ct_res[0]);
	sc->ct_bsh0 = rman_get_bushandle(sc->ct_res[0]);

	/* Timer0 interrupt */
	if (bus_setup_intr(dev, sc->ct_res[1], INTR_TYPE_CLK,
	    m83xxx_hardclock, NULL, sc, &intrcookie)) {
		device_printf(dev, "could not setup interrupt handler\n");
		bus_release_resources(dev, m83xxx_timer_spec, sc->ct_res);
		return (ENXIO);
	}

	/* Get PERIPH_CLK encoded in parent bus 'bus-frequency' property */

	node = ofw_bus_get_node(dev);
	if (OF_getprop(OF_parent(node), "bus-frequency", &freq,
	    sizeof(pcell_t)) <= 0) {
		bus_release_resources(dev, m83xxx_timer_spec, sc->ct_res);
		bus_teardown_intr(dev, sc->ct_res[2], intrcookie);
		device_printf(dev, "could not obtain base clock frequency\n");
		return (ENXIO);
	}

	freq = fdt32_to_cpu(freq);

	/* Set desired frequency in event timer and timecounter */
	sc->ct_et.et_frequency = (uint64_t)freq;
	m83xxx_timecounter.tc_frequency = (uint64_t)freq;

	sc->ct_et.et_name = "Comcerto Timer0";
	sc->ct_et.et_flags = ET_FLAGS_ONESHOT;
	sc->ct_et.et_quality = 1000;
	sc->ct_et.et_min_period = (0x00000002LLU << 32) / sc->ct_et.et_frequency;
	sc->ct_et.et_max_period = (0xfffffffeLLU << 32) / sc->ct_et.et_frequency;
	sc->ct_et.et_start = m83xxx_timer_start;
	sc->ct_et.et_stop = m83xxx_timer_stop;
	sc->ct_et.et_priv = sc;

	et_register(&sc->ct_et);
	tc_init(&m83xxx_timecounter);

	/* Reset and enable timecounter */

	timer_write_4(sc, COMCERTO_TIMER1_HIGH_BOUND, 0xffffffff);
	timer_write_4(sc, COMCERTO_TIMER_IRQ_MASK, 0);

	/* DELAY() now can work properly */
	m83xxx_timer_initialized = 1;

	return (0);
}

static int
m83xxx_timer_start(struct eventtimer *et, sbintime_t first, sbintime_t period)
{
	struct m83xxx_timer_softc *sc = (struct m83xxx_timer_softc *)et->et_priv;
	uint32_t ticks;

	ticks = ((uint32_t)et->et_frequency * first) >> 32;

	/* Start timer */
	timer_write_4(sc, COMCERTO_TIMER0_HIGH_BOUND, ticks);
	timer_write_4(sc, COMCERTO_TIMER0_CURRENT_COUNT, 0);
	timer_write_4(sc, COMCERTO_TIMER_IRQ_MASK, 1);

	return (0);
}

static int
m83xxx_timer_stop(struct eventtimer *et)
{
	struct m83xxx_timer_softc *sc = (struct m83xxx_timer_softc *)et->et_priv;

	timer_write_4(sc, COMCERTO_TIMER0_HIGH_BOUND, 0);
	timer_write_4(sc, COMCERTO_TIMER0_CURRENT_COUNT, 0);
	timer_write_4(sc, COMCERTO_TIMER_IRQ_MASK, 0);

	return (0);
}

static device_method_t m83xxx_timer_methods[] = {
	DEVMETHOD(device_probe,		m83xxx_timer_probe),
	DEVMETHOD(device_attach,	m83xxx_timer_attach),
	{ 0, 0 }
};

static driver_t m83xxx_timer_driver = {
	"timer",
	m83xxx_timer_methods,
	sizeof(struct m83xxx_timer_softc),
};

static devclass_t m83xxx_timer_devclass;

EARLY_DRIVER_MODULE(timer, simplebus, m83xxx_timer_driver, m83xxx_timer_devclass, 0, 0, BUS_PASS_TIMER);

static int
m83xxx_hardclock(void *arg)
{
	struct m83xxx_timer_softc *sc = (struct m83xxx_timer_softc *)arg;

	/* Reset pending interrupt */
	timer_write_4(sc, COMCERTO_TIMER_STATUS, 0x01);
	timer_write_4(sc, COMCERTO_TIMER0_CURRENT_COUNT, 0);
	timer_write_4(sc, COMCERTO_TIMER0_HIGH_BOUND, 0);
	timer_write_4(sc, COMCERTO_TIMER_IRQ_MASK, 0);
	timer_write_4(sc, COMCERTO_TIMER_STATUS_CLR, 0x01);

	if (sc->ct_et.et_active)
		sc->ct_et.et_event_cb(&sc->ct_et, sc->ct_et.et_arg);

	return (FILTER_HANDLED);
}

static unsigned
m83xxx_get_timecount(struct timecounter *tc)
{

	return timer_read_4(timer_softc, COMCERTO_TIMER1_CURRENT_COUNT);
}

void
DELAY(int usec)
{
	uint32_t counter;
	uint32_t first, last;
	int val;

	val = (m83xxx_timecounter.tc_frequency / 1000000 + 1) * usec;

	/* Timer is not initialized yet */
	if (!m83xxx_timer_initialized) {
		for (; usec > 0; usec--)
			for (counter = 100; counter > 0; counter--)
				;
		return;
	}

	first = m83xxx_get_timecount(&m83xxx_timecounter);
	while (val > 0) {
		last = m83xxx_get_timecount(&m83xxx_timecounter);
		if (last < first) {
			/* Timer rolled over */
			last = first;
		}
		
		val -= (last - first);
		first = last;
	}
}
