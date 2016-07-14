/*-
 * Copyright (c) 2009 Yohanes Nugroho <yohanes@gmail.com>
 * Copyright (c) 1994-1998 Mark Brinicombe.
 * Copyright (c) 1994 Brini.
 * All rights reserved.
 *
 * This code is derived from software written for Brini by Mark Brinicombe
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Brini.
 * 4. The name of the company nor the name of the author may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY BRINI ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL BRINI OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * from: FreeBSD: //depot/projects/arm/src/sys/arm/at91/kb920x_machdep.c, rev 45
 */

#include "opt_ddb.h"
#include "opt_global.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#define _ARM32_BUS_DMA_PRIVATE
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/reboot.h>
#include <sys/devmap.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/fdt.h>
#include <machine/machdep.h>
#include <machine/platform.h> 
#include <machine/cpu.h>

#include <arm/cavium/cns11xx/econa_reg.h>

#include <dev/fdt/fdt_common.h>

vm_offset_t
platform_lastaddr(void)
{

	return (devmap_lastaddr());
}

void
platform_probe_and_attach(void)
{
}

void
platform_gpio_init(void)
{

	/*
	 * Set initial values of GPIO output ports
	 */
}

void
platform_late_init(void)
{
	bootverbose = 1;
	boothowto |= (RB_SINGLE);
}

/*
 * Add a single static device mapping.
 * The values used were taken from the ranges property of the SoC node in the
 * dts file when this code was converted to arm_devmap_add_entry().
 */
int
platform_devmap_init(void)
{
	devmap_add_entry(0x10000000, 0x800000);
	devmap_add_entry(0x70000000, 0x10000000);
	devmap_add_entry(0xc0000000, 0x10000000);
	return (0);
}

/* Physical and virtual addresses for some global pages */

struct pv_addr msgbufpv;
struct pv_addr kernelstack;
