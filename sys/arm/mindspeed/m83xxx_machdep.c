/*-
 * Copyright (c) 2016 Hiroki Mori
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

#include "opt_ddb.h"
#include "opt_platform.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/devmap.h>
#include <sys/reboot.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/armreg.h>
#include <machine/bus.h>
#include <machine/machdep.h>
#include <machine/platform.h> 
#include <machine/fdt.h>

#include <arm/mindspeed/m83xxxreg.h>

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
}

void
platform_late_init(void)
{
	bus_space_handle_t bsh;
	uint32_t reg;

        boothowto |= (RB_SINGLE);

	/* Enable cache */
/*
	cpufunc_control(CPU_CONTROL_DC_ENABLE|CPU_CONTROL_IC_ENABLE,
	    CPU_CONTROL_DC_ENABLE|CPU_CONTROL_IC_ENABLE);
*/
#ifdef NOTWORK
	/* CFI A22 Hi (GPIO22) */
	bus_space_map(fdtbus_bs_tag, COMCERTO_APB_GPIO_BASE, 0x20000, 0, &bsh);
	reg = bus_space_read_4(fdtbus_bs_tag, bsh, COMCERTO_GPIO_OUTPUT_REG);
	bus_space_write_4(fdtbus_bs_tag, bsh, COMCERTO_GPIO_OUTPUT_REG,
		reg | (1 << 22));
	bus_space_unmap(fdtbus_bs_tag, bsh, 0x20000);
#endif
}

/*
 * Set up static device mappings.
 */
int
platform_devmap_init(void)
{

	devmap_add_entry(0x10000000, 0x02000000);
	devmap_add_entry(0x20000000, 0x02000000);
	
	return (0);
}

void
cpu_reset()
{

	for (;;)
		continue;
}
