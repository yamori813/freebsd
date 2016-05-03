/*-
 * Copyright (c) 2003 Marcel Moolenaar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
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
#include <sys/conf.h>
#include <sys/cons.h>
#include <sys/tty.h>
#include <machine/bus.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_bus.h>
#include <mips/broadcom/bcm47xx/uart_dev_siba.h>

#include "uart_if.h"

//#define      DEFAULT_RCLK    3686400
#define      DEFAULT_RCLK    25804800  /* BCM5354 CC */


#include <sys/types.h>
#include <sys/kernel.h>
#include <sys/time.h>

#include <sys/cons.h>
#include <sys/consio.h>

/*
 * Low-level UART interface.
 */
static int siba_probe(struct uart_bas *bas);
static void siba_init(struct uart_bas *bas, int, int, int, int);
static void siba_term(struct uart_bas *bas);
static void siba_putc(struct uart_bas *bas, int);
static int siba_rxready(struct uart_bas *bas);
static int siba_getc(struct uart_bas *bas, struct mtx *mtx);

extern SLIST_HEAD(uart_devinfo_list, uart_devinfo) uart_sysdevs;

static struct uart_ops uart_siba_ops = {
	.probe = siba_probe,
	.init = siba_init,
	.term = siba_term,
	.putc = siba_putc,
	.rxready = siba_rxready,
	.getc = siba_getc,
};

static int
siba_probe(struct uart_bas *bas)
{
	return (0);
}

static void
siba_init(struct uart_bas *bas, int baudrate, int databits, int stopbits,
    int parity)
{
	unsigned int baud_base = DEFAULT_RCLK;

	if (bas->rclk == 0)
		bas->rclk = baud_base;


	/* Set baud and 8N1 */
	unsigned int quot = baud_base / 16 / 115200;
	uart_setreg(bas, UART_LCR, UART_LCR_DLAB);
	uart_setreg(bas, UART_DLL, quot & 0xff);
	uart_setreg(bas, UART_DLM, quot >> 8);
	uart_setreg(bas, UART_LCR, UART_LCR_WLEN8);
	uart_getreg(bas, UART_LCR);

	DELAY(10000);


}

static void
siba_term(struct uart_bas *bas)
{
	/* XXX */
}

static void
siba_putc(struct uart_bas *bas, int c)
{
	DELAY(1000);
//	while (!(uart_getreg(bas, UART_LSR) & UART_LSR_THRE));
	uart_setreg(bas, UART_TX, c);
	// Commit
	uart_getreg(bas, UART_LSR);

}

static int
siba_rxready(struct uart_bas *bas)
{
	return (1);

//	return ((uart_getreg(bas, UART_LSR) & UART_LSR_RXRDY) != 0 ? 1 : 0);
}

static int
siba_getc(struct uart_bas *bas, struct mtx *mtx)
{
	int c;

	uart_lock(mtx);

	DELAY(1000);
//	while ((uart_getreg(bas, UART_LSR) & UART_LSR_RXRDY) == 0) {
//		uart_unlock(mtx);
//		DELAY(4);
//		uart_lock(mtx);
//	}

	c = uart_getreg(bas, UART_DATA);

	uart_unlock(mtx);
	c &= 0xff;

	return (c);

}

static int siba_bus_probe(struct uart_softc *sc);
static int siba_bus_attach(struct uart_softc *sc);
static int siba_bus_flush(struct uart_softc *, int);
static int siba_bus_getsig(struct uart_softc *);
static int siba_bus_ioctl(struct uart_softc *, int, intptr_t);
static int siba_bus_ipend(struct uart_softc *);
static int siba_bus_param(struct uart_softc *, int, int, int, int);
static int siba_bus_receive(struct uart_softc *);
static int siba_bus_setsig(struct uart_softc *, int);
static int siba_bus_transmit(struct uart_softc *);

static kobj_method_t siba_methods[] = {
	KOBJMETHOD(uart_probe,		siba_bus_probe),
	KOBJMETHOD(uart_attach, 	siba_bus_attach),
	KOBJMETHOD(uart_flush,		siba_bus_flush),
	KOBJMETHOD(uart_getsig,		siba_bus_getsig),
	KOBJMETHOD(uart_ioctl,		siba_bus_ioctl),
	KOBJMETHOD(uart_ipend,		siba_bus_ipend),
	KOBJMETHOD(uart_param,		siba_bus_param),
	KOBJMETHOD(uart_receive,	siba_bus_receive),
	KOBJMETHOD(uart_setsig,		siba_bus_setsig),
	KOBJMETHOD(uart_transmit,	siba_bus_transmit),
	
	{0, 0 }
};

int
siba_bus_probe(struct uart_softc *sc)
{
	return (0);
}

static int
siba_bus_attach(struct uart_softc *sc)
{
//	 bcopy(&sc->sc_sysdev->bas, &sc->sc_bas, sizeof(sc->sc_bas));

//	 sc->sc_txfifosz = 3;
//	 sc->sc_rxfifosz = 1;
//	 sc->sc_hwiflow = 0;
//	 uart_setreg(&sc->sc_bas, SACOM_CR3, CR3_RXE | CR3_TXE | CR3_RIE | CR3_TIE);
	return (0);
}
static int
siba_bus_transmit(struct uart_softc *sc)
{
//	int i;
//
//	sc->sc_txbusy = 1;
//	uart_setreg(&sc->sc_bas, SACOM_CR3, uart_getreg(&sc->sc_bas, SACOM_CR3)
//	    | CR3_TIE);    
//	for (i = 0; i < sc->sc_txdatasz; i++) {
//		while (!(uart_getreg(&sc->sc_bas, SACOM_SR1) & SR1_TNF));
//
//		uart_setreg(&sc->sc_bas, SACOM_DR, sc->sc_txbuf[i]);
//		uart_barrier(&sc->sc_bas);
//	}

	return (0);
}
static int
siba_bus_setsig(struct uart_softc *sc, int sig)
{
	return (0);
}
static int
siba_bus_receive(struct uart_softc *sc)
{
	
	
//	uart_setreg(&sc->sc_bas, SACOM_CR3, uart_getreg(&sc->sc_bas, SACOM_CR3)
//	    | CR3_RIE);
//	uart_rx_put(sc, uart_getreg(&sc->sc_bas, SACOM_DR));
	return (0);
}
static int
siba_bus_param(struct uart_softc *sc, int baudrate, int databits,
    int stopbits, int parity)
{
//	int brd;
//	
//	if (baudrate > 0) {
//		brd = SACOMSPEED(baudrate);
//		uart_setreg(&sc->sc_bas, SACOM_CR1, brd >> 8);
//		uart_setreg(&sc->sc_bas, SACOM_CR2, brd & 0xff);
//	}
	return (0);
}
static int
siba_bus_ipend(struct uart_softc *sc)
{
//	int sr = uart_getreg(&sc->sc_bas, SACOM_SR0);
//	int ipend = 0;
//	int mask = CR3_RIE | CR3_TIE;
//	if (sr & 1) {
//		if (uart_getreg(&sc->sc_bas, SACOM_CR3) & CR3_TIE)
//			ipend |= SER_INT_TXIDLE;
//		mask &= ~CR3_TIE;
//	}
//	if (sr & 4) {
//		if (uart_getreg(&sc->sc_bas, SACOM_CR3) & CR3_RIE)
//			ipend |= SER_INT_RXREADY;
//		mask &= ~CR3_RIE;
//	}
//	uart_setreg(&sc->sc_bas, SACOM_CR3, CR3_RXE | mask); 
//	return (ipend);
	return (0);
}
static int
siba_bus_flush(struct uart_softc *sc, int what)
{
	return (0);
}

static int
siba_bus_getsig(struct uart_softc *sc)
{
	return (0);
}

static int
siba_bus_ioctl(struct uart_softc *sc, int request, intptr_t data)
{
	return (EINVAL);
}

struct uart_class uart_siba_class = {
	"siba",
	siba_methods,
	1,
	.uc_ops = &uart_siba_ops,
	.uc_range = 8,
	.uc_rclk = DEFAULT_RCLK
};
