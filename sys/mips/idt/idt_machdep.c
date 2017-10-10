/*-
 * Copyright (C) 2015-2016 by Stanislav Galabov. All rights reserved.
 * Copyright (C) 2010-2011 by Aleksandr Rybalko. All rights reserved.
 * Copyright (C) 2007 by Oleksandr Tymoshenko. All rights reserved.
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
 * IN NO EVENT SHALL THE AUTHOR OR HIS RELATIVES BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF MIND, USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: $
 * 
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_ddb.h"
#include "opt_platform.h"

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

#ifdef FDT
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#endif

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

extern int	*edata;
extern int	*end;

void
platform_cpu_init()
{
	/* Nothing special */
}

void
platform_reset(void)
{
	volatile unsigned int * p = (void *)0xb8008000;
	/* 
	 * TODO: we should take care of TLB stuff here. Otherwise
	 * board does not boots properly next time
	 */

	/* Write 0x8000_0001 to the Reset register */
	*p = 0x80000001;

	__asm __volatile("li	$25, 0xbfc00000");
	__asm __volatile("j	$25");
}

#ifndef FDT
void
platform_start(__register_t a0, __register_t a1,
    __register_t a2 __unused, __register_t a3 __unused)
{
	uint64_t platform_counter_freq;
	vm_offset_t kernend;
	int argc = a0;
	char **argv = (char **)a1;
	int i, mem;


	/* clear the BSS and SBSS segments */
	kernend = (vm_offset_t)&end;
	memset(&edata, 0, kernend - (vm_offset_t)(&edata));

	mips_postboot_fixup();

	/* Initialize pcpu stuff */
	mips_pcpu0_init();

	/*
	 * Looking for mem=XXM argument
	 */
	mem = 0; /* Just something to start with */
	for (i=0; i < argc; i++) {
		if (strncmp(argv[i], "mem=", 4) == 0) {
			mem = strtol(argv[i] + 4, NULL, 0);
			break;
		}
	}

	bootverbose = 1;
	if (mem > 0)
		realmem = btoc(mem << 20);
	else
		realmem = btoc(32 << 20);

	for (i = 0; i < 10; i++) {
		phys_avail[i] = 0;
	}

	/* phys_avail regions are in bytes */
	phys_avail[0] = MIPS_KSEG0_TO_PHYS(kernel_kseg0_end);
	phys_avail[1] = ctob(realmem);

	dump_avail[0] = phys_avail[0];
	dump_avail[1] = phys_avail[1];

	physmem = realmem;

	/* 
	 * ns8250 uart code uses DELAY so ticker should be inititalized 
	 * before cninit. And tick_init_params refers to hz, so * init_param1 
	 * should be called first.
	 */
	init_param1();
	/* TODO: parse argc,argv */
	platform_counter_freq = 330000000UL;
	mips_timer_init_params(platform_counter_freq, 1);
	cninit();
	/* Panic here, after cninit */ 
	if (mem == 0)
		panic("No mem=XX parameter in arguments");

	printf("cmd line: ");
	for (i=0; i < argc; i++)
		printf("%s ", argv[i]);
	printf("\n");

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

#else

/* Todo: move prototype to sys/mips/include/hwfunc.h */

uint32_t platform_get_timerclk(void);
uint32_t platform_get_cpuclk(void);
uint32_t platform_get_uartclk(void);

uint32_t platform_get_timerclk()
{ 
	return 150 * 1000 * 1000;
}

uint32_t platform_get_cpuclk()
{
	return 150 * 1000 * 1000;
}

uint32_t platform_get_uartclk()
{
	return 150 * 1000 * 1000;
}

/* Todo: move to sys/mips/mips/machdep.c */

static char     boot1_env[0x1000];

static void
mips_init(void)
{
	struct mem_region mr[FDT_MEM_REGIONS];
	uint64_t val;
	int i, j, mr_cnt;

	printf("entry: mips_init()\n");

	bootverbose = 1;

	for (i = 0; i < 10; i++)
		phys_avail[i] = 0;

	dump_avail[0] = phys_avail[0] = MIPS_KSEG0_TO_PHYS(kernel_kseg0_end);

	/*
	 * The most low memory MT7621 can have. Currently MT7621 is the chip
	 * that supports the most memory, so that seems reasonable.
	 */
	realmem = btoc(448 * 1024 * 1024);

	if (fdt_get_mem_regions(mr, &mr_cnt, &val) == 0) {
		physmem = btoc(val);

		printf("RAM size: %ldMB (from FDT)\n",
		    ctob(physmem) / (1024 * 1024));

		KASSERT((phys_avail[0] >= mr[0].mr_start) && \
			(phys_avail[0] < (mr[0].mr_start + mr[0].mr_size)),
			("First region is not within FDT memory range"));

		/* Limit size of the first region */
		phys_avail[1] = (mr[0].mr_start +
		    MIN(mr[0].mr_size, ctob(realmem)));
		dump_avail[1] = phys_avail[1];

		/* Add the rest of the regions */
		for (i = 1, j = 2; i < mr_cnt; i++, j+=2) {
			phys_avail[j] = mr[i].mr_start;
			phys_avail[j+1] = (mr[i].mr_start + mr[i].mr_size);
			dump_avail[j] = phys_avail[j];
			dump_avail[j+1] = phys_avail[j+1];
		}
	} else {
		/* platform default memory setup */
		
	}

	if (physmem < realmem)
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

static void
_parse_bootarg(char *v)
{
	char *n;

	if (*v == '-') {
		while (*v != '\0') {
			v++;
			switch (*v) {
			case 'a': boothowto |= RB_ASKNAME; break;
			/* Someone should simulate that ;-) */
			case 'C': boothowto |= RB_CDROM; break;
			case 'd': boothowto |= RB_KDB; break;
			case 'D': boothowto |= RB_MULTIPLE; break;
			case 'm': boothowto |= RB_MUTE; break;
			case 'g': boothowto |= RB_GDB; break;
			case 'h': boothowto |= RB_SERIAL; break;
			case 'p': boothowto |= RB_PAUSE; break;
			case 'r': boothowto |= RB_DFLTROOT; break;
			case 's': boothowto |= RB_SINGLE; break;
			case 'v': boothowto |= RB_VERBOSE; break;
			}
		}
	} else {
		n = strsep(&v, "=");
		if (v == NULL)
			kern_setenv(n, "1");
		else
			kern_setenv(n, v);
	}
}

/* Parse cmd line args as env - copied from xlp_machdep. */
/* XXX-BZ this should really be centrally provided for all (boot) code. */
static void
_parse_bootargs(char *cmdline)
{
	char *v;

	while ((v = strsep(&cmdline, " \n")) != NULL) {
		if (*v == '\0')
			continue;
		_parse_bootarg(v);
	}
}

void
platform_start(__register_t a0 __unused, __register_t a1 __unused, 
    __register_t a2 __unused, __register_t a3 __unused)
{
	vm_offset_t kernend;
	int argc = a0, i;//, res;
	uint32_t timer_clk;
	char **argv = (char **)MIPS_PHYS_TO_KSEG0(a1);
	char **envp = (char **)MIPS_PHYS_TO_KSEG0(a2);
	void *dtbp;
	phandle_t chosen;
	char buf[2048];

	/* clear the BSS and SBSS segments */
	kernend = (vm_offset_t)&end;
	memset(&edata, 0, kernend - (vm_offset_t)(&edata));

	mips_postboot_fixup();

	/* Initialize pcpu stuff */
	mips_pcpu0_init();

	dtbp = &fdt_static_dtb;
	if (OF_install(OFW_FDT, 0) == FALSE)
		while (1);
	if (OF_init((void *)dtbp) != 0)
		while (1);

	if ((timer_clk = platform_get_timerclk()) == 0)
		timer_clk = 1000000000; /* no such speed yet */

	mips_timer_early_init(timer_clk);

	/* initialize console so that we have printf */
	boothowto |= (RB_SERIAL | RB_MULTIPLE);	/* Use multiple consoles */
	boothowto |= (RB_VERBOSE);
	cninit();

	init_static_kenv(boot1_env, sizeof(boot1_env));

	/*
	 * Get bsdbootargs from FDT if specified.
	 */
	chosen = OF_finddevice("/chosen");
	if (OF_getprop(chosen, "bsdbootargs", buf, sizeof(buf)) != -1)
		_parse_bootargs(buf);

	printf("FDT DTB  at: 0x%08x\n", (uint32_t)dtbp);

	printf("CPU   clock: %4dMHz\n", platform_get_cpuclk()/(1000*1000));
	printf("Timer clock: %4dMHz\n", timer_clk/(1000*1000));
	printf("UART  clock: %4dMHz\n\n", platform_get_uartclk()/(1000*1000));

	printf("U-Boot args (from %d args):\n", argc - 1);

	if (argc == 1)
		printf("\tNone\n");

	for (i = 1; i < argc; i++) {
		char *n = "argv  ", *arg;

		if (i > 99)
			break;

		if (argv[i])
		{
			arg = (char *)(intptr_t)MIPS_PHYS_TO_KSEG0(argv[i]);
			printf("\targv[%d] = %s\n", i, arg);
			sprintf(n, "argv%d", i);
			kern_setenv(n, arg);
		}
	}

	printf("Environment:\n");

	for (i = 0; envp[i] && MIPS_IS_VALID_PTR(envp[i]); i++) {
		char *n, *arg;

		arg = (char *)(intptr_t)MIPS_PHYS_TO_KSEG0(envp[i]);
		if (! MIPS_IS_VALID_PTR(arg))
			continue;
		printf("\t%s\n", arg);
		n = strsep(&arg, "=");
		if (arg == NULL)
			kern_setenv(n, "1");
		else
			kern_setenv(n, arg);
	}


	mips_init();
	mips_timer_init_params(timer_clk, 0);
}
#endif
