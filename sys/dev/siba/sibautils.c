/*-
 * Copyright (c) 2010, Aleksandr Rybalko <ray@ddteam.net>
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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/malloc.h>
#include <sys/interrupt.h>

#include <machine/bus.h>

#include <dev/siba/sibavar.h>
#include <dev/siba/sibareg.h>
#include <dev/siba/siba_ids.h>
#include <dev/siba/siba_cc.h>

int	siba_dev_isenabled(device_t, uint8_t);
void	siba_dev_enable   (device_t, uint8_t, uint32_t);
void	siba_dev_disable  (device_t, uint8_t, uint32_t);

static uint32_t siba_tmslow_reject_bitmask(struct siba_softc *, uint8_t);



int
siba_dev_isenabled(device_t dev, uint8_t idx)
{
	uint32_t reject, val;
	device_t siba;
	struct siba_softc *sc;

	if (strncmp(device_get_name(dev), "siba", 4) == 0) 
		siba = dev;
	else
		siba = device_get_parent(dev);
	if (strncmp(device_get_name(siba), "siba", 4) != 0) 
		return 0;

	sc = device_get_softc(siba);

	if (idx > sc->sc_ncores)
		return (0);

	reject = siba_tmslow_reject_bitmask(sc, idx);
	val = siba_read_4(sc, idx, SIBA_TGSLOW);
	val &= SIBA_TGSLOW_CLOCK | SIBA_TGSLOW_RESET | reject;

	return (val == SIBA_TGSLOW_CLOCK);
}

void
siba_dev_enable(device_t dev, uint8_t idx, uint32_t flags)
{
	uint32_t val;
	device_t siba;
	struct siba_softc *sc;

	if (strncmp(device_get_name(dev), "siba", 4) == 0) 
		siba = dev;
	else
		siba = device_get_parent(dev);
	if (strncmp(device_get_name(siba), "siba", 4) != 0) 
		return;

	sc = device_get_softc(siba);

	if (idx > sc->sc_ncores)
		return;

	siba_dev_disable(dev, idx, flags);
	siba_write_4(sc, idx, SIBA_TGSLOW, SIBA_TGSLOW_RESET | SIBA_TGSLOW_CLOCK |
	    SIBA_TGSLOW_FGC | flags);
	siba_read_4(sc, idx, SIBA_TGSLOW);
	DELAY(1);

	if (siba_read_4(sc, idx, SIBA_TGSHIGH) & SIBA_TGSHIGH_SERR)
		siba_write_4(sc, idx, SIBA_TGSHIGH, 0);

	val = siba_read_4(sc, idx, SIBA_IAS);
	if (val & (SIBA_IAS_INBAND_ERR | SIBA_IAS_TIMEOUT)) {
		val &= ~(SIBA_IAS_INBAND_ERR | SIBA_IAS_TIMEOUT);
		siba_write_4(sc, idx, SIBA_IAS, val);
	}

	siba_write_4(sc, idx, SIBA_TGSLOW,
	    SIBA_TGSLOW_CLOCK | SIBA_TGSLOW_FGC | flags);
	siba_read_4(sc, idx, SIBA_TGSLOW);
	DELAY(1);

	siba_write_4(sc, idx, SIBA_TGSLOW, SIBA_TGSLOW_CLOCK | flags);
	siba_read_4(sc, idx, SIBA_TGSLOW);
	DELAY(1);
}


static uint32_t
siba_tmslow_reject_bitmask(struct siba_softc *sc, uint8_t idx)
{
	uint32_t rev = siba_read_4(sc, idx, SIBA_IDLOW) & SIBA_IDLOW_SSBREV;

	switch (rev) {
	case SIBA_IDLOW_SSBREV_22:
		return (SIBA_TGSLOW_REJECT_22);
	case SIBA_IDLOW_SSBREV_23:
		return (SIBA_TGSLOW_REJECT_23);
	case SIBA_IDLOW_SSBREV_24:
	case SIBA_IDLOW_SSBREV_25:
	case SIBA_IDLOW_SSBREV_26:
	case SIBA_IDLOW_SSBREV_27:
		return (SIBA_TGSLOW_REJECT_23);
	default:
		KASSERT(0 == 1,
		    ("%s:%d: unknown backplane rev %#x\n",
			__func__, __LINE__, rev));
	}
	return (SIBA_TGSLOW_REJECT_22 | SIBA_TGSLOW_REJECT_23);
}

void
siba_dev_disable(device_t dev, uint8_t idx, uint32_t flags)
{
	uint32_t reject, val;
	int i;
	device_t siba;
	struct siba_softc *sc;

	if (strncmp(device_get_name(dev), "siba", 4) == 0) 
		siba = dev;
	else
		siba = device_get_parent(dev);
	if (strncmp(device_get_name(siba), "siba", 4) != 0) 
		return;

	sc = device_get_softc(siba);

	if (idx > sc->sc_ncores)
		return;

	if (siba_read_4(sc, idx, SIBA_TGSLOW) & SIBA_TGSLOW_RESET)
		return;

	reject = siba_tmslow_reject_bitmask(sc, idx);
	siba_write_4(sc, idx, SIBA_TGSLOW, reject | SIBA_TGSLOW_CLOCK);

	for (i = 0; i < 1000; i++) {
		val = siba_read_4(sc, idx, SIBA_TGSLOW);
		if (val & reject)
			break;
		DELAY(10);
	}
	if ((val & reject) == 0) {
		device_printf(dev, "timeout (bit %#x reg %#x)\n",
		    reject, SIBA_TGSLOW);
	}
	for (i = 0; i < 1000; i++) {
		val = siba_read_4(sc, idx, SIBA_TGSHIGH);
		if (!(val & SIBA_TGSHIGH_BUSY))
			break;
		DELAY(10);
	}
	if ((val & SIBA_TGSHIGH_BUSY) != 0) {
		device_printf(dev, "timeout (bit %#x reg %#x)\n",
		    SIBA_TGSHIGH_BUSY, SIBA_TGSHIGH);
	}

	siba_write_4(sc, idx, SIBA_TGSLOW, SIBA_TGSLOW_FGC | SIBA_TGSLOW_CLOCK |
	    reject | SIBA_TGSLOW_RESET | flags);
	siba_read_4(sc, idx, SIBA_TGSLOW);
	DELAY(1);
	siba_write_4(sc, idx, SIBA_TGSLOW, reject | SIBA_TGSLOW_RESET | flags);
	siba_read_4(sc, idx, SIBA_TGSLOW);
	DELAY(1);
}




