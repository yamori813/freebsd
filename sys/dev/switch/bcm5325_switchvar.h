/*-
 * Copyright (c) 2011,2012 Aleksandr Rybalko.
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

#ifndef _BCM5325_SWITCHVAR_H_
#define _BCM5325_SWITCHVAR_H_

struct bcm5325_switch_softc {
	device_t		sc_dev;
	device_t		parent;
	device_t		new_mii; /* Unused */
	int 			ports;
	int 			vlans;
	uint16_t		base_vlan;
	struct switch_capability *caps;
	int			page_reg;
	int			page_phy;
	int			devid;
	int			mode; /* Unused */
	uint32_t		sc_mii_mode;
	int (*vlan_write)	(struct bcm5325_switch_softc *, uint16_t,
	    uint32_t, uint32_t);
	int (*vlan_read)	(struct bcm5325_switch_softc *, uint16_t,
	    uint32_t *, uint32_t *);
};

#define MII_SW_READ4(_sc, _reg) \
	SWITCHB_READ4(_sc->parent, _reg)

#define MII_SW_WRITE4(_sc, _reg, _val) \
	SWITCHB_WRITE4(_sc->parent, _reg, _val)

#endif /* _BCM5325_SWITCHVAR_H_ */
