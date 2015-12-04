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
 *
 * $FreeBSD$
 */

#ifndef _SIBA_PCIBVAR_H_
#define _SIBA_PCIBVAR_H_

#include <sys/rman.h>

struct siba_pcib_softc {
	device_t		 sc_dev;	/* Device ID */
	u_int			 sc_bus;	/* PCI bus number */
	struct resource		*sc_mem;	/* siba memory window */
	struct resource		*sc_csr;	/* config space */

	bus_space_tag_t		 sc_bt;
	bus_space_handle_t	 sc_bh;
#if 0
	bus_addr_t		 sc_maddr;
	bus_size_t		 sc_msize;

	struct bus_space	 sc_pci_memt;
	struct bus_space	 sc_pci_iot;
	bus_dma_tag_t		 sc_dmat;
#endif
};

#define SIBA_PCI_RESET0		0x0F98 /* 00030001 PCI */
#define SIBA_PCI_RESET1		0x0F98 /* 00030000 PCI */
#define SIBA_PCI_RESET2		0x0F98 /* 00010000 PCI */
#define SIBA_PCI_PCICONTROL	0x0000 /* 00000000 PCI /no_in */
#define SIBA_PCI_BISTSTATUS	0x000C /* 00000000 PCI /no_in */
#define SIBA_PCI_PCIARBCONTROL	0x0010 /* 00000000 PCI /no_in */
#define SIBA_PCI_INTSTATUS	0x0020 /* 00000000 PCI /no_in */
#define SIBA_PCI_INTMASK	0x0024 /* 00000000 PCI /no_in */
#define SIBA_PCI_SBTOPCIMAILBOX 0x0028 /* 00000000 PCI */
#define SIBA_PCI_BROADCASTADDR	0x0050 /* 00000000 PCI /no_in */
#define SIBA_PCI_BROADCASTDATA	0x0054 /* 00000000 PCI /no_in */
#define SIBA_PCI_SBTOPCIXLATE0	0x0100 /* 00000000 PCI /no_in */
#define SIBA_PCI_SBTOPCIXLATE1	0x0104 /* 00000000 PCI /no_in */
#define SIBA_PCI_SBTOPCIXLATE2	0x0108 /* 00000000 PCI /no_in */



#endif /* _SIBA_PCIBVAR_H_ */
