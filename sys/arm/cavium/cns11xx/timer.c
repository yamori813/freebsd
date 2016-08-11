/*-
 * Copyright (c) 2016 Hiroki Mori
 * Copyright (c) 2009 Yohanes Nugroho <yohanes@gmail.com>.
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
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
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

#include "opt_timer.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/timetc.h>
#include <sys/timeet.h>
#include <sys/watchdog.h>
#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>
/*
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
*/

#include "econa_reg.h"
#include "econa_var.h"

static int timers_initialized = 0;

extern unsigned int CPU_clock;
extern unsigned int AHB_clock;
extern unsigned int APB_clock;

struct ec_timer_softc {
	struct resource	*	timer_res[3];
	bus_space_tag_t		timer_bst;
	bus_space_handle_t	timer_bsh;
	struct mtx		timer_mtx;
	struct eventtimer	ec_et;
	int			ec_start;
	int			ec_oneshot;
	uint32_t		ec_period;
};

static struct resource_spec ec_timer_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		1,	RF_ACTIVE },
	{ -1, 0 }
};

#define TIMER_LOCK(_sc)		mtx_lock(&(_sc)->timer_mtx)
#define TIMER_UNLOCK(_sc)	mtx_unlock(&(_sc)->timer_mtx)

static unsigned ec_timer_get_timecount(struct timecounter *);
#ifndef NO_EVENTTIMERS
static int ec_timer_start(struct eventtimer *,
    sbintime_t first, sbintime_t period);
static int ec_timer_stop(struct eventtimer *et);
#endif

static struct timecounter ec_timecounter = {
	.tc_get_timecount = ec_timer_get_timecount,
	.tc_name = "Timer1",
	/* This is assigned on the fly in the init sequence */
	.tc_frequency = 0,
	.tc_counter_mask = ~0u,
	.tc_quality = 1000,
};

static struct ec_timer_softc *timer_softc = NULL;

static inline
void write_4(unsigned int val, unsigned int addr)
{
	bus_space_write_4(timer_softc->timer_bst,
			  timer_softc->timer_bsh, addr, val);

}

static inline
unsigned int read_4(unsigned int addr)
{

	return bus_space_read_4(timer_softc->timer_bst,
	    timer_softc->timer_bsh, addr);
}

static inline unsigned int
read_second_timer_counter(void)
{

	return read_4(TIMER_TM1_COUNTER_REG);
}

void
DELAY(int usec)
{
	uint32_t val, val_temp;
	int nticks;

	if (!timers_initialized) {
		for (; usec > 0; usec--)
			for (val = 100; val > 0; val--)
				;
		return;
	}

	// timer2 is count up
	val = read_second_timer_counter();
	nticks = (ec_timecounter.tc_frequency / 1000000 + 1) * usec;

	while (nticks > 0) {
		val_temp = read_second_timer_counter();
		if (val < val_temp)
			nticks -= (val_temp - val);
		else
			nticks -= ((~0u - val) + val_temp);

		val = val_temp;
	}

}

/*
 * Setup timer
 */
static inline void
setup_timer(void)
{
	unsigned int control_value;
	unsigned int mask_value;

	control_value = read_4(TIMER_TM_CR_REG);

	// Timer2 is eventtimer (count down)
	control_value &= ~(TIMER2_CLOCK_SOURCE);
	control_value |= TIMER2_UP_DOWN_COUNT;

	// Timer1 is timecounter (count up)
	// 0 -> 0xffffffff -> 0 -> ...
	control_value &= ~(TIMER1_CLOCK_SOURCE);
	control_value &= ~(TIMER1_UP_DOWN_COUNT);

	write_4(control_value, TIMER_TM_CR_REG);

	// only use timer2 interrupt
	mask_value = 0x07;   // 000000111

	write_4(mask_value, TIMER_TM_INTR_MASK_REG);
}

/*
 * Get timer interrupt status
 */
static inline unsigned int
read_timer_interrupt_status(void)
{

	return read_4(TIMER_TM_INTR_STATUS_REG);
}

/*
 * Clear timer interrupt status
 */
static inline void
clear_timer_interrupt_status(unsigned int irq)
{
	unsigned int interrupt_status;

	interrupt_status =   read_4(TIMER_TM_INTR_STATUS_REG);
	if (irq == 0) {
		if (interrupt_status & (TIMER1_MATCH1_INTR))
			interrupt_status &= ~(TIMER1_MATCH1_INTR);
		if (interrupt_status & (TIMER1_MATCH2_INTR))
			interrupt_status &= ~(TIMER1_MATCH2_INTR);
		if (interrupt_status & (TIMER1_OVERFLOW_INTR))
			interrupt_status &= ~(TIMER1_OVERFLOW_INTR);
	}
	if (irq == 1) {
		if (interrupt_status & (TIMER2_MATCH1_INTR))
			interrupt_status &= ~(TIMER2_MATCH1_INTR);
		if (interrupt_status & (TIMER2_MATCH2_INTR))
			interrupt_status &= ~(TIMER2_MATCH2_INTR);
		if (interrupt_status & (TIMER2_OVERFLOW_INTR))
			interrupt_status &= ~(TIMER2_OVERFLOW_INTR);
	}

	write_4(interrupt_status, TIMER_TM_INTR_STATUS_REG);
}

static unsigned
ec_timer_get_timecount(struct timecounter *a)
{
	return read_second_timer_counter();
}

static int
ec_timer_probe(device_t dev)
{
/*
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "econa,timer"))
		return (ENXIO);
*/

	device_set_desc(dev, "Econa CPU Timer");
	return (0);
}

static int
ec_hardclock(void *arg)
{
	struct  ec_timer_softc *sc = (struct ec_timer_softc *)arg;
	unsigned int control_value;

	clear_timer_interrupt_status(1);

	control_value = read_4(TIMER_TM_CR_REG);
	control_value &= ~TIMER2_ENABLE;
	write_4(control_value, TIMER_TM_CR_REG);

	if(sc->ec_start == 0)
		return (FILTER_HANDLED);

	/* Start timer again */
	if (!sc->ec_oneshot) {
		write_4(sc->ec_period, TIMER_TM2_COUNTER_REG);
		write_4(sc->ec_period, TIMER_TM2_LOAD_REG);
		control_value = read_4(TIMER_TM_CR_REG);
		control_value |= TIMER2_OVERFLOW_ENABLE;
		control_value |= TIMER2_ENABLE;
		write_4(control_value, TIMER_TM_CR_REG);
	} else {
		sc->ec_start = 0;
	}

	if (sc->ec_et.et_active)
		sc->ec_et.et_event_cb(&sc->ec_et, sc->ec_et.et_arg);

	return (FILTER_HANDLED);
}

static int
ec_timer_attach(device_t dev)
{
	struct	ec_timer_softc *sc;
	int	error;
	void	*ihl;
	unsigned int control_value;


	if (timer_softc != NULL)
		return (ENXIO);

	sc = (struct ec_timer_softc *)device_get_softc(dev);

	mtx_init(&sc->timer_mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	timer_softc = sc;

	error = bus_alloc_resources(dev, ec_timer_spec, sc->timer_res);
	if (error) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	sc->timer_bst = rman_get_bustag(sc->timer_res[0]);
	sc->timer_bsh = rman_get_bushandle(sc->timer_res[0]);

	setup_timer();

	if (bus_setup_intr(dev, sc->timer_res[2], INTR_TYPE_CLK,
	    ec_hardclock, NULL, sc, &ihl) != 0) {
		bus_release_resources(dev, ec_timer_spec, sc->timer_res);
		device_printf(dev, "could not setup hardclock interrupt\n");
		return (ENXIO);
	}

	ec_timecounter.tc_frequency = (uint64_t)APB_clock;

#ifndef NO_EVENTTIMERS
	sc->ec_et.et_frequency = (uint64_t)APB_clock;
	sc->ec_et.et_name = "Timer2";
	sc->ec_et.et_flags = ET_FLAGS_PERIODIC | ET_FLAGS_ONESHOT;
	sc->ec_et.et_quality = 1000;
	sc->ec_et.et_min_period = (0x00000002LLU << 32) / sc->ec_et.et_frequency;
	sc->ec_et.et_max_period = (0xfffffffeLLU << 32) / sc->ec_et.et_frequency;
	sc->ec_et.et_start = ec_timer_start;
	sc->ec_et.et_stop = ec_timer_stop;
	sc->ec_et.et_priv = sc;
	et_register(&sc->ec_et);
	sc->ec_start = 0;
#endif
	tc_init(&ec_timecounter);

	/* enable timecounter */
	// Timer1 is timecounter (count up)
	write_4(0, TIMER_TM1_COUNTER_REG);
	write_4(0, TIMER_TM1_LOAD_REG);
	write_4(~0u, TIMER_TM1_MATCH1_REG);
	write_4(~0u, TIMER_TM1_MATCH2_REG);
	control_value = read_4(TIMER_TM_CR_REG);
	control_value |= TIMER1_ENABLE;
	control_value &= ~TIMER2_ENABLE;
	write_4(control_value, TIMER_TM_CR_REG);

	/* DELAY() now can work properly */
	timers_initialized = 1;

	return (0);
}

#ifndef NO_EVENTTIMERS
static int
ec_timer_start(struct eventtimer *et, sbintime_t first, sbintime_t period)
{
	struct ec_timer_softc *sc = (struct ec_timer_softc *)et->et_priv;

	uint32_t ticks;

	if (period == 0) {
		sc->ec_oneshot = 1;
		sc->ec_period = 0;
	} else {
		sc->ec_oneshot = 0;
		sc->ec_period = ((uint32_t)et->et_frequency * period) >> 32;
	}

	if (first == 0)
		ticks = sc->ec_period;
	else
		ticks = ((uint32_t)et->et_frequency * first) >> 32;
	
	// Timer2 is eventtimer (count down)
	// ticks -> 0
	write_4(ticks, TIMER_TM2_COUNTER_REG);
	write_4(ticks, TIMER_TM2_LOAD_REG);
	write_4(0, TIMER_TM2_MATCH1_REG);
	write_4(0, TIMER_TM2_MATCH2_REG);

	unsigned int control_value;

	control_value = read_4(TIMER_TM_CR_REG);

	control_value |= TIMER2_OVERFLOW_ENABLE;
	control_value |= TIMER2_ENABLE;

	write_4(control_value, TIMER_TM_CR_REG);

	sc->ec_start = 1;

	return (0);
}

static int
ec_timer_stop(struct eventtimer *et)
{
	struct ec_timer_softc *sc = (struct ec_timer_softc *)et->et_priv;

	unsigned int control_value;

	control_value = read_4(TIMER_TM_CR_REG);

	control_value &= ~TIMER2_ENABLE;

	write_4(control_value, TIMER_TM_CR_REG);

	sc->ec_start = 0;

	return (0);
}

#else
void
cpu_startprofclock(void)
{}

void
cpu_initclocks(void)
{}

void
cpu_stopprofclock(void)
{}
#endif

static device_method_t ec_timer_methods[] = {
	DEVMETHOD(device_probe, ec_timer_probe),
	DEVMETHOD(device_attach, ec_timer_attach),
	{ 0, 0 }
};

static driver_t ec_timer_driver = {
	"timer",
	ec_timer_methods,
	sizeof(struct ec_timer_softc),
};

static devclass_t ec_timer_devclass;

DRIVER_MODULE(timer, econaarm, ec_timer_driver, ec_timer_devclass, 0, 0);
