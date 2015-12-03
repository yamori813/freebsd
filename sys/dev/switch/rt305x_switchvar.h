/*-
 * Copyright (c) 2010-2012 Aleksandr Rybalko.
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
 * $FreeBSD$
 */

#ifndef _RT305X_SWITCHVAR_H_
#define _RT305X_SWITCHVAR_H_

struct rt305x_switch_softc {
	device_t		sc_dev;
	device_t		parent;
	int 			ports;
	int 			vlans;
	bus_space_tag_t 	sc_bst;
	bus_space_handle_t 	sc_bsh;
	struct switch_capability *caps;
};

#define READ4(_sc, _reg) \
	SWITCHB_READ4(_sc->parent, _reg)

#define WRITE4(_sc, _reg, _val) \
	SWITCHB_WRITE4(_sc->parent, _reg, _val)

#define WAIT4(sc, reg, field, value, timeout_usec)	\
    ({int result;					\
    do {						\
	uint32_t c, timeout = timeout_usec;		\
	while (1) {					\
	    c = (READ4(sc, reg) & field);		\
	    if (c == (value)) {				\
		result = 0;				\
		break;					\
	    } else if (!timeout) {			\
		result = -1;				\
		break;					\
	    } else {					\
		DELAY(1); timeout--;			\
	    }						\
	}						\
    } while (0);					\
    result;})

#endif /* _RT305X_SWITCHVAR_H_ */

