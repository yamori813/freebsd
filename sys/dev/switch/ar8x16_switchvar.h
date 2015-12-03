/*-
 * Copyright (c) 2011 Aleksandr Rybalko.
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

#ifndef _AR8X16_SWITCHVAR_H_
#define _AR8X16_SWITCHVAR_H_

struct ar8x16_switch_softc {
	device_t		sc_dev;
	device_t		parent;
	int 			ports;
	int 			vlans;
	struct switch_capability *caps;
	int			page_reg;
	int			page_phy;
	int			curpage;
	int			devid;
	int			revid;
	uint32_t		sc_mii_mode;
	uint16_t		*vlan_idx;
};

/* Ask switch bus to read/write device registers */
#define MII_SW_READ4(_sc, _reg)				\
	SWITCHB_READ4(_sc->parent, _reg)

#define MII_SW_WRITE4(_sc, _reg, _val)			\
	SWITCHB_WRITE4(_sc->parent, _reg, _val)

/* Read/write switch registers */
#define	READ4(_sc, _addr)				\
    ar8x16_reg_read((_sc), (_addr))
#define	WRITE4(_sc, _addr, _val)			\
    ar8x16_reg_write((_sc), (_addr), (_val))

/* Ask switch bus to read/write phy registers */
#define MII_READ(sc, phy, reg)				\
	MII_SW_READ4(sc, (((phy) << 8) | (reg)))
#define MII_WRITE(sc, phy, reg, val)			\
	MII_SW_WRITE4(sc, (((phy) << 8) | (reg)), val)

#define	REG_REG(reg)		(((reg) >> 1) & 0x01e)
#define	REG_PHY(reg)		(((reg) >> 6) & 0x007)
#define	REG_PAGE(reg)		(((reg) >> 9) & 0x1ff)
#define	VLAN_IDX_VALID		0x8000

#define SET4(sc, reg, mask, val)			\
	WRITE4(sc, reg, (READ4(sc, reg) & ~mask) | val)

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

#endif /* _AR8X16_SWITCHVAR_H_ */
