/*-
 * Copyright (c) 2007 Bruce M. Simpson.
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <machine/cpuregs.h>

#include <mips/broadcom/bcm47xx/bcm47xx_reg.h>

#include "opt_ddb.h"

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/imgact.h>
#include <sys/bio.h>
#include <sys/buf.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/cons.h>
#include <sys/exec.h>
#include <sys/ucontext.h>
#include <sys/proc.h>
#include <sys/kdb.h>
#include <sys/ptrace.h>
#include <sys/reboot.h>
#include <sys/signalvar.h>
#include <sys/sysent.h>
#include <sys/sysproto.h>
#include <sys/user.h>

#include <vm/vm.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>

#include <machine/cache.h>
#include <machine/clock.h>
#include <machine/cpu.h>
#include <machine/cpuinfo.h>
#include <machine/cpufunc.h>
#include <machine/cpuregs.h>
#include <machine/hwfunc.h>
#include <machine/intr_machdep.h>
#include <machine/locore.h>
#include <machine/md_var.h>
#include <machine/pte.h>
#include <machine/sigframe.h>
#include <machine/trap.h>
#include <machine/vmparam.h>


#ifdef CFE
#include <dev/cfe/cfe_api.h>
#include <dev/cfe/cfe_ioctl.h>
#endif
#ifdef CFE_ENV
void cfe_env_init(void);
#endif


extern int *edata;
extern int *end;

/* Interrupt control */
#define SIBA_CC_INTSTATUS	0x20
#define SIBA_CC_INTMASK		0x24
#define SIBA_CC_CHIPCONTROL 	0x28 /* rev >= 11 */
#define SIBA_CC_CHIPSTATUS	0x2c /* rev >= 11 */

void mips_mask_irq(void *source);
void mips_unmask_irq(void *source);

void
mips_mask_irq(void *source)
{
}

void
mips_unmask_irq(void *source)
{
}


void
platform_cpu_init()
{
	/* Nothing special */
}

static void
mips_init(void)
{
	int i;

	printf("entry: mips_init()\n");

#ifdef CFE_ENV
	cfe_env_init();
#endif

	TUNABLE_INT_FETCH("boothowto", &boothowto);

	if (boothowto & RB_VERBOSE)
		bootverbose++;
	boothowto |= (RB_SERIAL | RB_MULTIPLE);	/* Use multiple consoles */

	/* Initialize pcpu stuff */
	mips_pcpu0_init();


#ifdef CFE_CONSOLE
	/*
	 * Query DRAM memory map from CFE.
	 */
	physmem = 0;
	for (i = 0; i < 10; i += 2) {
		int result;
		uint64_t addr, len, type;

		result = cfe_enummem(i, CFE_FLG_FULL_ARENA, &addr, &len, 
		    &type);
		if (result < 0) {
			phys_avail[i] = phys_avail[i + 1] = 0;
			break;
		}
		if (type != CFE_MI_AVAILABLE)
			continue;

		phys_avail[i] = addr;
		if (i == 0 && addr == 0) {
			/*
			 * If this is the first physical memory segment probed
			 * from CFE, omit the region at the start of physical
			 * memory where the kernel has been loaded.
			 */
			phys_avail[i] += MIPS_KSEG0_TO_PHYS((vm_offset_t)&end);
		}
		phys_avail[i + 1] = addr + len;
		physmem += len;
	}
#else

		#define addr 0x00000000
//		#define len  0x02000000
		#define len  0x01000000
		phys_avail[0] = MIPS_KSEG0_TO_PHYS(kernel_kseg0_end);
		phys_avail[1] = len;
		physmem += len;
		printf("phys_avail[0] = 0x%08x, len = 0x%08x\n",
		    phys_avail[0], phys_avail[1]);

		phys_avail[2] = phys_avail[3] = 0;
		phys_avail[4] = phys_avail[5] = 0;
		phys_avail[6] = phys_avail[7] = 0;
		phys_avail[8] = phys_avail[9] = 0;

#endif
	for (i = 0; i < 10; i++)
		dump_avail[i] = phys_avail[i];


	physmem = btoc(physmem);
	realmem = physmem;

	init_param1();
	init_param2(physmem);
	mips_cpu_init();
	pmap_bootstrap();
	mips_proc0_init();
	mutex_init();
	kdb_init();
#ifdef KDB
	if (boothowto & RB_KDB)
		kdb_enter(KDB_WHY_BOOTFLAGS, "Boot flags requested debugger");

#endif
}

void
platform_reset(void)
{

	/* Software Hard Reset and Board Reset bits */
	s5_wr_reset(0x3);

	/* Set WatchDog timer to 1 ms */
	*((volatile uint32_t *)MIPS_PHYS_TO_KSEG1(0x18000080)) = 1;
	DELAY(100);
}

static void
kseg0_map_coherent(void)
{
}

void
platform_start(__register_t a0 __unused, __register_t a1 __unused, 
    __register_t a2 __unused, __register_t a3 __unused)
{
	uint64_t platform_counter_freq;
	uint32_t core_id;


	/*
	 * Make sure that kseg0 is mapped cacheable-coherent
	 */
	kseg0_map_coherent();

	/* clear the BSS and SBSS segments */
	memset(&edata, 0, (vm_offset_t)&end - (vm_offset_t)&edata);
	mips_postboot_fixup();


	/* Initialize pcpu stuff */
	mips_pcpu0_init();


	/* initialize console so that we have printf */
	boothowto |= (RB_SERIAL | RB_MULTIPLE);	/* Use multiple consoles */
	boothowto |= (RB_VERBOSE);
	boothowto |= (RB_SINGLE);

#ifdef CFE
	/*
	 * Initialize CFE firmware trampolines before
	 * we initialize the low-level console.
	 *
	 * CFE passes the following values in registers:
	 * a0: firmware handle
	 * a2: firmware entry point
	 * a3: entry point seal
	 */
	nvram_info_t nvram_info;
	if (a3 == CFE_EPTSEAL) {
		cfe_init(a0, a2);
		cfe_ioctl(a0, IOCTL_NVRAM_GETINFO,
		    (unsigned char *)&nvram_info, sizeof(nvram_info), 0, 0);
	}
#endif

	cninit();
	mips_init();
#ifdef CFE
	printf("nvram_info offset=%08x, size=%08x, eraseflg=%d\n",
	nvram_info.nvram_offset,	/* offset of environment area */
	nvram_info.nvram_size,		/* size of environment area */
	nvram_info.nvram_eraseflg);	/* true if we need to erase first */
#endif

	core_id = (*((volatile uint32_t *)
	    MIPS_PHYS_TO_KSEG1(BCM47XX_EXTIFADR)) & 0xffff);

	switch(core_id)
	{
	    /* BCM5836 is 264MHz */
	    case 0x00004704:
		platform_counter_freq = 264 * 1000 * 1000;
		break;
	    /* BCM5354 is 240MHz */
	    case 0x00005354:
		platform_counter_freq = 240 * 1000 * 1000; 
		break;
	    default:
		platform_counter_freq = 240 * 1000 * 1000; 
		break;
	}

	mips_timer_init_params(platform_counter_freq, 1);
}
