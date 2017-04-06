/*-
 * Copyright (c) 2016 Hiroki Mori
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/smp.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/cpu.h>
#include <machine/smp.h>
#include <machine/fdt.h>
#include <machine/intr.h>

#include <arm/mindspeed/m83xxxreg.h>

void
platform_mp_setmaxid(void)
{

	mp_maxid = 1;
	mp_ncpus = 2;
}

void    
platform_mp_start_ap(void)
{
	bus_addr_t clock_addr;
	bus_addr_t ddrmem;
	int val;

	if (bus_space_map(fdtbus_bs_tag, COMCERTO_APB_CLK_BASE, 0x1000,
	    0, &clock_addr) != 0)
		panic("Couldn't map the RESET\n");
	if (bus_space_map(fdtbus_bs_tag, COMCERTO_AHB_DDR_BASE, 0x1000,
	    0, &ddrmem) != 0)
		panic("Couldn't map the DDR MEMORY\n");

	bus_space_write_4(fdtbus_bs_tag, ddrmem, 0, 0xE59FF018);
	bus_space_write_4(fdtbus_bs_tag, ddrmem, 0x20,
	    pmap_kextract((vm_offset_t)mpentry));

	dcache_wbinv_poc_all();

	val = bus_space_read_4(fdtbus_bs_tag, clock_addr,
	    COMCERTO_BLOCK_RESET_REG);
	val |= (1 << 1);
	bus_space_write_4(fdtbus_bs_tag, clock_addr,
	    COMCERTO_BLOCK_RESET_REG, val);

	dsb();
	sev();

	bus_space_unmap(fdtbus_bs_tag, clock_addr, 0x1000);
	bus_space_unmap(fdtbus_bs_tag, ddrmem, 0x1000);
}
