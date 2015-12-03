/*-
 * Copyright (c) 2010-2012 Aleksandr Rybalko <ray@ddteam.net>
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
 *
 */

#ifndef _SWITCHVAR_H_
#define _SWITCHVAR_H_

#include <sys/param.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_media.h>

#include <dev/mii/miivar.h>

/* Switch capability list */
struct switch_capability {

	uint32_t ports;		/* Number of ports */
	uint32_t main;
	uint32_t vlan;
	uint32_t qos;
	uint32_t lacp;
	uint32_t stp;
	uint32_t acl;
	uint32_t stat;
};

struct child_res_avl {
	size_t		memres_size;
	uint32_t	phys;
	uint8_t		irqs;
};

struct switch_softc {
	struct mii_softc	sc_mii;
	struct mtx		sc_mtx;		/* bus mutex */
	struct resource		*mem_res;
	int			mem_rid;
	size_t			mem_size;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	struct resource		*irq_res;
	int			irq_rid;
	uint32_t		phys;
	uint32_t		isol_phys;
	void			*ihandl;
	driver_filter_t		*sc_isr;
	void 			*sc_cookie;

	device_t		sc_dev;
	device_t		sc_miidev;
	device_t		child;
	struct cdev		*sc_cdev;
	device_t		child_miibus;
	struct ifnet		*ifp;
	struct callout		tick_callout;

	int			enable;
	struct switch_capability *caps;
	struct child_res_avl	*args;
};

int switch_init(struct switch_softc *sc);
int switch_deinit(struct switch_softc *sc);
int switchpub_get_reg(device_t dev, uint32_t reg, uint32_t *value);
int switchpub_set_reg(device_t dev, uint32_t reg, uint32_t *value);

#define	SWITCH_REG_TYPE_MASK	0xc0000000
#define	SWITCH_REG_TYPE_SHIFT	30
#define	SWITCH_REG_TYPE_SWITCH	(0)
#define	SWITCH_REG_TYPE_PHY	(1<<SWITCH_REG_TYPE_SHIFT)
#define	SWITCH_REG_TYPE_RAW	(2<<SWITCH_REG_TYPE_SHIFT)

/* Child MDIO access */
int	switch_miibus_writereg(device_t dev, int phy, int reg, int value);
int	switch_miibus_readreg(device_t dev, int phy, int reg);
void	switch_miibus_statchg(device_t dev);

#define	SWITCH_LOCK(_sc) mtx_lock(&(_sc)->sc_mtx)
#define	SWITCH_UNLOCK(_sc) mtx_unlock(&(_sc)->sc_mtx)
#define	SWITCH_LOCK_INIT(_sc) \
	mtx_init(&(_sc)->sc_mtx, device_get_nameunit((_sc)->sc_dev), \
	    "etherswitch", MTX_DEF)
#define	SWITCH_LOCK_DESTROY(_sc) mtx_destroy(&(_sc)->sc_mtx);
#define	SWITCH_ASSERT_LOCKED(_sc) mtx_assert(&(_sc)->sc_mtx, MA_OWNED);
#define	SWITCH_ASSERT_UNLOCKED(_sc) mtx_assert(&(_sc)->sc_mtx, MA_NOTOWNED);

#endif /* _SWITCHVAR_H_ */

