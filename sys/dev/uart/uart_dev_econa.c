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

/* delete MCR_DTR, MCR_DRS, MCR_IE, IER_ETXRDY */

#include "opt_platform.h"
#include "opt_uart.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>
#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_cpu_fdt.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_dev_econa.h>
#include <dev/uart/uart_ppstypes.h>

#include <dev/ic/ns16550.h>

#include "uart_if.h"

#define	DEFAULT_RCLK	1843200

/*
 * Set the default baudrate tolerance to 3.0%.
 *
 * Some embedded boards have odd reference clocks (eg 25MHz)
 * and we need to handle higher variances in the target baud rate.
 */
#ifndef	UART_DEV_TOLERANCE_PCT
#define	UART_DEV_TOLERANCE_PCT	30
#endif	/* UART_DEV_TOLERANCE_PCT */

static int broken_txfifo = 0;
SYSCTL_INT(_hw, OID_AUTO, broken_txfifo, CTLFLAG_RWTUN,
	&broken_txfifo, 0, "UART FIFO has QEMU emulation bug");

/*
 * Clear pending interrupts. THRE is cleared by reading IIR. Data
 * that may have been received gets lost here.
 */
static void
econa_clrint(struct uart_bas *bas)
{
//	uint8_t iir, lsr;
	uint8_t iir;

	iir = uart_getreg(bas, REG_IIR);
	while ((iir & IIR_NOPEND) == 0) {
		iir &= IIR_IMASK;
/*
		if (iir == IIR_RLS) {
			lsr = uart_getreg(bas, REG_LSR);
			if (lsr & (LSR_BI|LSR_FE|LSR_PE))
				(void)uart_getreg(bas, REG_DATA);
		} else */
		if (iir == IIR_RXRDY || iir == IIR_RXTOUT)
			(void)uart_getreg(bas, REG_DATA);
		else if (iir == IIR_MLSC)
			(void)uart_getreg(bas, REG_MSR);
		uart_barrier(bas);
		iir = uart_getreg(bas, REG_IIR);
	}
}

static int
econa_delay(struct uart_bas *bas)
{
	int divisor;
	u_char lcr;

	lcr = uart_getreg(bas, REG_LCR);
	uart_setreg(bas, REG_LCR, lcr | LCR_DLAB);
	uart_barrier(bas);
	divisor = uart_getreg(bas, REG_DLL) | (uart_getreg(bas, REG_DLH) << 8);
	uart_barrier(bas);
	uart_setreg(bas, REG_LCR, lcr);
	uart_barrier(bas);

	/* 1/10th the time to transmit 1 character (estimate). */
	if (divisor <= 134)
		return (16000000 * divisor / bas->rclk);
	return (16000 * divisor / (bas->rclk / 1000));
}

static int
econa_divisor(int rclk, int baudrate)
{
	int actual_baud, divisor;
	int error;

	if (baudrate == 0)
		return (0);

	divisor = (rclk / (baudrate << 3) + 1) >> 1;
	if (divisor == 0 || divisor >= 65536)
		return (0);
	actual_baud = rclk / (divisor << 4);

	/* 10 times error in percent: */
	error = ((actual_baud - baudrate) * 2000 / baudrate + 1) >> 1;

	/* enforce maximum error tolerance: */
	if (error < -UART_DEV_TOLERANCE_PCT || error > UART_DEV_TOLERANCE_PCT)
		return (0);

	return (divisor);
}

static int
econa_drain(struct uart_bas *bas, int what)
{
	int delay, limit;

	delay = econa_delay(bas);

	if (what & UART_DRAIN_TRANSMITTER) {
		/*
		 * Pick an arbitrary high limit to avoid getting stuck in
		 * an infinite loop when the hardware is broken. Make the
		 * limit high enough to handle large FIFOs.
		 */
		limit = 10*1024;
		while ((uart_getreg(bas, REG_LSR) & LSR_TEMT) == 0 && --limit)
			DELAY(delay);
		if (limit == 0) {
			/* printf("econa: transmitter appears stuck... "); */
			return (EIO);
		}
	}

	if (what & UART_DRAIN_RECEIVER) {
printf("MORI MORI rec\n");
		/*
		 * Pick an arbitrary high limit to avoid getting stuck in
		 * an infinite loop when the hardware is broken. Make the
		 * limit high enough to handle large FIFOs and integrated
		 * UARTs. The HP rx2600 for example has 3 UARTs on the
		 * management board that tend to get a lot of data send
		 * to it when the UART is first activated.
		 */
		limit=10*4096;
		while ((uart_getreg(bas, REG_LSR) & LSR_RXRDY) && --limit) {
			(void)uart_getreg(bas, REG_DATA);
			uart_barrier(bas);
			DELAY(delay << 2);
		}
		if (limit == 0) {
			/* printf("econa: receiver appears broken... "); */
			return (EIO);
		}
	}

	return (0);
}

/*
 * We can only flush UARTs with FIFOs. UARTs without FIFOs should be
 * drained. WARNING: this function clobbers the FIFO setting!
 */
static void
econa_flush(struct uart_bas *bas, int what)
{
	uint8_t fcr;

	fcr = FCR_ENABLE;
	if (what & UART_FLUSH_TRANSMITTER)
		fcr |= FCR_XMT_RST;
	if (what & UART_FLUSH_RECEIVER)
		fcr |= FCR_RCV_RST;
	uart_setreg(bas, REG_FCR, fcr);
	uart_barrier(bas);
}

static int
econa_param(struct uart_bas *bas, int baudrate, int databits, int stopbits,
    int parity)
{
	int divisor;
	uint8_t lcr;

	lcr = 0;
	if (databits >= 8)
		lcr |= LCR_8BITS;
	else if (databits == 7)
		lcr |= LCR_7BITS;
	else if (databits == 6)
		lcr |= LCR_6BITS;
	else
		lcr |= LCR_5BITS;
	if (stopbits > 1)
		lcr |= LCR_STOPB;
	lcr |= parity << 3;

	/* Set baudrate. */
	if (baudrate > 0) {
		divisor = econa_divisor(bas->rclk, baudrate);
		if (divisor == 0)
			return (EINVAL);
		uart_setreg(bas, REG_LCR, lcr | LCR_DLAB);
		uart_barrier(bas);
		uart_setreg(bas, REG_DLL, divisor & 0xff);
		uart_setreg(bas, REG_DLH, (divisor >> 8) & 0xff);
		uart_barrier(bas);
	}

	/* Set LCR and clear DLAB. */
	uart_setreg(bas, REG_LCR, lcr);
	uart_barrier(bas);
	return (0);
}

/*
 * Low-level UART interface.
 */
static int econa_probe(struct uart_bas *bas);
static void econa_init(struct uart_bas *bas, int, int, int, int);
static void econa_term(struct uart_bas *bas);
static void econa_putc(struct uart_bas *bas, int);
static int econa_rxready(struct uart_bas *bas);
static int econa_getc(struct uart_bas *bas, struct mtx *);

struct uart_ops uart_econa_ops = {
	.probe = econa_probe,
	.init = econa_init,
	.term = econa_term,
	.putc = econa_putc,
	.rxready = econa_rxready,
	.getc = econa_getc,
};

static int
econa_probe(struct uart_bas *bas)
{
	u_char val;

	/* Check known 0 bits that don't depend on DLAB. */
	val = uart_getreg(bas, REG_IIR);
	if (val & 0x30)
		return (ENXIO);
	/*
	 * Bit 6 of the MCR (= 0x40) appears to be 1 for the Sun1699
	 * chip, but otherwise doesn't seem to have a function. In
	 * other words, uart(4) works regardless. Ignore that bit so
	 * the probe succeeds.
	 */
	val = uart_getreg(bas, REG_MCR);
	if (val & 0xa0)
		return (ENXIO);

	return (0);
}

static void
econa_init(struct uart_bas *bas, int baudrate, int databits, int stopbits,
    int parity)
{
	u_char	ier;

	if (bas->rclk == 0)
		bas->rclk = DEFAULT_RCLK;
	econa_param(bas, baudrate, databits, stopbits, parity);

	/* Disable all interrupt sources. */
	/*
	 * We use 0xe0 instead of 0xf0 as the mask because the XScale PXA
	 * UARTs split the receive time-out interrupt bit out separately as
	 * 0x10.  This gets handled by ier_mask and ier_rxbits below.
	 */
	ier = uart_getreg(bas, REG_IER) & 0xe0;
	uart_setreg(bas, REG_IER, ier);
	uart_barrier(bas);

	/* Disable the FIFO (if present). */
	uart_setreg(bas, REG_FCR, 0);
	uart_barrier(bas);

	/* Set RTS & DTR. */
	uart_setreg(bas, REG_MCR, MCR_RTS);
	uart_barrier(bas);

	econa_clrint(bas);
}

static void
econa_term(struct uart_bas *bas)
{
	/* Clear RTS */
	uart_setreg(bas, REG_MCR, 0);
	uart_barrier(bas);
}

static void
econa_putc(struct uart_bas *bas, int c)
{
	int limit;

	limit = 250000;
	while ((uart_getreg(bas, REG_LSR) & LSR_THRE) == 0 && --limit)
		DELAY(4);
	uart_setreg(bas, REG_DATA, c);
	uart_barrier(bas);
	limit = 250000;
	while ((uart_getreg(bas, REG_LSR) & LSR_TEMT) == 0 && --limit)
		DELAY(4);
}

static int
econa_rxready(struct uart_bas *bas)
{

	return ((uart_getreg(bas, REG_LSR) & LSR_RXRDY) != 0 ? 1 : 0);
}

static int
econa_getc(struct uart_bas *bas, struct mtx *hwmtx)
{
	int c;

	uart_lock(hwmtx);

	while ((uart_getreg(bas, REG_LSR) & LSR_RXRDY) == 0) {
		uart_unlock(hwmtx);
		DELAY(4);
		uart_lock(hwmtx);
	}

	c = uart_getreg(bas, REG_DATA);

	uart_unlock(hwmtx);

	return (c);
}

static kobj_method_t econa_methods[] = {
	KOBJMETHOD(uart_attach,		econa_bus_attach),
	KOBJMETHOD(uart_detach,		econa_bus_detach),
	KOBJMETHOD(uart_flush,		econa_bus_flush),
	KOBJMETHOD(uart_getsig,		econa_bus_getsig),
	KOBJMETHOD(uart_ioctl,		econa_bus_ioctl),
	KOBJMETHOD(uart_ipend,		econa_bus_ipend),
	KOBJMETHOD(uart_param,		econa_bus_param),
	KOBJMETHOD(uart_probe,		econa_bus_probe),
	KOBJMETHOD(uart_receive,	econa_bus_receive),
	KOBJMETHOD(uart_setsig,		econa_bus_setsig),
	KOBJMETHOD(uart_transmit,	econa_bus_transmit),
	KOBJMETHOD(uart_grab,		econa_bus_grab),
	KOBJMETHOD(uart_ungrab,		econa_bus_ungrab),
	{ 0, 0 }
};

struct uart_class uart_econa_class = {
	"econa",
	econa_methods,
	sizeof(struct econa_softc),
	.uc_ops = &uart_econa_ops,
	.uc_range = 8,
	.uc_rclk = DEFAULT_RCLK,
	.uc_rshift = 0
};

static struct ofw_compat_data compat_data[] = {
	{"econa-uart",		(uintptr_t)&uart_econa_class},
	{NULL,			(uintptr_t)NULL},
};
UART_FDT_CLASS_AND_DEVICE(compat_data);

/* Use token-pasting to form SER_ and MSR_ named constants. */
#define	SER(sig)	SER_##sig
#define	SERD(sig)	SER_D##sig
#define	MSR(sig)	MSR_##sig
#define	MSRD(sig)	MSR_D##sig

/*
 * Detect signal changes using software delta detection.  The previous state of
 * the signals is in 'var' the new hardware state is in 'msr', and 'sig' is the
 * short name (DCD, CTS, etc) of the signal bit being processed; 'var' gets the
 * new state of both the signal and the delta bits.
 */
#define SIGCHGSW(var, msr, sig)					\
	if ((msr) & MSR(sig)) {					\
		if ((var & SER(sig)) == 0)			\
			var |= SERD(sig) | SER(sig);		\
	} else {						\
		if ((var & SER(sig)) != 0)			\
			var = SERD(sig) | (var & ~SER(sig));	\
	}

/*
 * Detect signal changes using the hardware msr delta bits.  This is currently
 * used only when PPS timing information is being captured using the "narrow
 * pulse" option.  With a narrow PPS pulse the signal may not still be asserted
 * by time the interrupt handler is invoked.  The hardware will latch the fact
 * that it changed in the delta bits.
 */
#define SIGCHGHW(var, msr, sig)					\
	if ((msr) & MSRD(sig)) {				\
		if (((msr) & MSR(sig)) != 0)			\
			var |= SERD(sig) | SER(sig);		\
		else						\
			var = SERD(sig) | (var & ~SER(sig));	\
	}

int
econa_bus_attach(struct uart_softc *sc)
{
	struct econa_softc *econa = (struct econa_softc*)sc;
	struct uart_bas *bas;
	unsigned int ivar;
	phandle_t node;
	pcell_t cell;

	/* Check whether uart has a broken txfifo. */
	node = ofw_bus_get_node(sc->sc_dev);
	if ((OF_getencprop(node, "broken-txfifo", &cell, sizeof(cell))) > 0)
		broken_txfifo =  cell ? 1 : 0;

	bas = &sc->sc_bas;

	econa->mcr = uart_getreg(bas, REG_MCR);
	econa->fcr = FCR_ENABLE;
#if 0
	if (!resource_int_value("uart", device_get_unit(sc->sc_dev), "flags",
	    &ivar)) {
		if (UART_FLAGS_FCR_RX_LOW(ivar)) 
			econa->fcr |= FCR_RX_LOW;
		else if (UART_FLAGS_FCR_RX_MEDL(ivar)) 
			econa->fcr |= FCR_RX_MEDL;
		else if (UART_FLAGS_FCR_RX_HIGH(ivar)) 
			econa->fcr |= FCR_RX_HIGH;
		else
			econa->fcr |= FCR_RX_MEDH;
	} else 
		econa->fcr |= FCR_RX_MEDH;
#else
	econa->fcr |= FCR_RX_LOW;
#endif
	
	/* Get IER mask */
	ivar = 0xf0;
	resource_int_value("uart", device_get_unit(sc->sc_dev), "ier_mask",
	    &ivar);
	econa->ier_mask = (uint8_t)(ivar & 0xff);
	
	uart_setreg(bas, REG_FCR, econa->fcr);
	uart_barrier(bas);
	econa_bus_flush(sc, UART_FLUSH_RECEIVER|UART_FLUSH_TRANSMITTER);

	if (econa->mcr & MCR_RTS)
		sc->sc_hwsig |= SER_RTS;
	econa_bus_getsig(sc);

	econa_clrint(bas);
//	econa->ier = uart_getreg(bas, REG_IER) & econa->ier_mask;
	econa->ier = IER_ERXRDY;
	uart_setreg(bas, REG_IER, econa->ier);
	uart_barrier(bas);

	/*
	 * Timing of the H/W access was changed with r253161 of uart_core.c
	 * It has been observed that an ITE IT8513E would signal a break
	 * condition with pretty much every character it received, unless
	 * it had enough time to settle between econa_bus_attach() and
	 * econa_bus_ipend() -- which it accidentally had before r253161.
	 * It's not understood why the UART chip behaves this way and it
	 * could very well be that the DELAY make the H/W work in the same
	 * accidental manner as before. More analysis is warranted, but
	 * at least now we fixed a known regression.
	 */
	DELAY(200);
	return (0);
}

int
econa_bus_detach(struct uart_softc *sc)
{
	struct econa_softc *econa;
	struct uart_bas *bas;
	u_char ier;

	econa = (struct econa_softc *)sc;
	bas = &sc->sc_bas;
	ier = uart_getreg(bas, REG_IER) & econa->ier_mask;
	uart_setreg(bas, REG_IER, ier);
	uart_barrier(bas);
	econa_clrint(bas);
	return (0);
}

int
econa_bus_flush(struct uart_softc *sc, int what)
{
	struct econa_softc *econa = (struct econa_softc*)sc;
	struct uart_bas *bas;
	int error;

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);
	if (sc->sc_rxfifosz > 1) {
		econa_flush(bas, what);
		uart_setreg(bas, REG_FCR, econa->fcr);
		uart_barrier(bas);
		error = 0;
	} else
		error = econa_drain(bas, what);
	uart_unlock(sc->sc_hwmtx);
	return (error);
}

int
econa_bus_getsig(struct uart_softc *sc)
{
	uint32_t old, sig;
	uint8_t msr;

	/*
	 * The delta bits are reputed to be broken on some hardware, so use
	 * software delta detection by default.  Use the hardware delta bits
	 * when capturing PPS pulses which are too narrow for software detection
	 * to see the edges.  Hardware delta for RI doesn't work like the
	 * others, so always use software for it.  Other threads may be changing
	 * other (non-MSR) bits in sc_hwsig, so loop until it can successfully
	 * update without other changes happening.  Note that the SIGCHGxx()
	 * macros carefully preserve the delta bits when we have to loop several
	 * times and a signal transitions between iterations.
	 */
	do {
		old = sc->sc_hwsig;
		sig = old;
		uart_lock(sc->sc_hwmtx);
		msr = uart_getreg(&sc->sc_bas, REG_MSR);
		uart_unlock(sc->sc_hwmtx);
		if (sc->sc_pps_mode & UART_PPS_NARROW_PULSE) {
			SIGCHGHW(sig, msr, DSR);
			SIGCHGHW(sig, msr, CTS);
			SIGCHGHW(sig, msr, DCD);
		} else {
			SIGCHGSW(sig, msr, DSR);
			SIGCHGSW(sig, msr, CTS);
			SIGCHGSW(sig, msr, DCD);
		}
		SIGCHGSW(sig, msr, RI);
	} while (!atomic_cmpset_32(&sc->sc_hwsig, old, sig & ~SER_MASK_DELTA));
	return (sig);
}

int
econa_bus_ioctl(struct uart_softc *sc, int request, intptr_t data)
{
	struct uart_bas *bas;
	int baudrate, divisor, error;
	uint8_t efr, lcr;

	bas = &sc->sc_bas;
	error = 0;
	uart_lock(sc->sc_hwmtx);
	switch (request) {
	case UART_IOCTL_BREAK:
		lcr = uart_getreg(bas, REG_LCR);
		if (data)
			lcr |= LCR_SBREAK;
		else
			lcr &= ~LCR_SBREAK;
		uart_setreg(bas, REG_LCR, lcr);
		uart_barrier(bas);
		break;
	case UART_IOCTL_IFLOW:
		lcr = uart_getreg(bas, REG_LCR);
		uart_barrier(bas);
		uart_setreg(bas, REG_LCR, 0xbf);
		uart_barrier(bas);
		efr = uart_getreg(bas, REG_EFR);
		if (data)
			efr |= EFR_RTS;
		else
			efr &= ~EFR_RTS;
		uart_setreg(bas, REG_EFR, efr);
		uart_barrier(bas);
		uart_setreg(bas, REG_LCR, lcr);
		uart_barrier(bas);
		break;
	case UART_IOCTL_OFLOW:
		lcr = uart_getreg(bas, REG_LCR);
		uart_barrier(bas);
		uart_setreg(bas, REG_LCR, 0xbf);
		uart_barrier(bas);
		efr = uart_getreg(bas, REG_EFR);
		if (data)
			efr |= EFR_CTS;
		else
			efr &= ~EFR_CTS;
		uart_setreg(bas, REG_EFR, efr);
		uart_barrier(bas);
		uart_setreg(bas, REG_LCR, lcr);
		uart_barrier(bas);
		break;
	case UART_IOCTL_BAUD:
		lcr = uart_getreg(bas, REG_LCR);
		uart_setreg(bas, REG_LCR, lcr | LCR_DLAB);
		uart_barrier(bas);
		divisor = uart_getreg(bas, REG_DLL) |
		    (uart_getreg(bas, REG_DLH) << 8);
		uart_barrier(bas);
		uart_setreg(bas, REG_LCR, lcr);
		uart_barrier(bas);
		baudrate = (divisor > 0) ? bas->rclk / divisor / 16 : 0;
		if (baudrate > 0)
			*(int*)data = baudrate;
		else
			error = ENXIO;
		break;
	default:
		error = EINVAL;
		break;
	}
	uart_unlock(sc->sc_hwmtx);
	return (error);
}

int
econa_bus_ipend(struct uart_softc *sc)
{
	struct uart_bas *bas;
	struct econa_softc *econa;
	int ipend;
	uint8_t iir, lsr;

	econa = (struct econa_softc *)sc;
	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);
	iir = uart_getreg(bas, REG_IIR);

	if (econa->busy_detect && (iir & IIR_BUSY) == IIR_BUSY) {
		(void)uart_getreg(bas, DW_REG_USR);
		uart_unlock(sc->sc_hwmtx);
		return (0);
	}
	if (iir & IIR_NOPEND) {
		uart_unlock(sc->sc_hwmtx);
		return (0);
	}
	ipend = 0;
	if (iir & IIR_RXRDY) {
		lsr = uart_getreg(bas, REG_LSR);
		if (lsr & LSR_OE)
			ipend |= SER_INT_OVERRUN;
		if (lsr & LSR_BI)
			ipend |= SER_INT_BREAK;
		if (lsr & LSR_RXRDY)
			ipend |= SER_INT_RXREADY;
	} else {
		if (iir & IIR_TXRDY) {
			ipend |= SER_INT_TXIDLE;
			uart_setreg(bas, REG_IER, econa->ier);
			uart_barrier(bas);
		} else
			ipend |= SER_INT_SIGCHG;
	}
	if (ipend == 0)
		econa_clrint(bas);
	uart_unlock(sc->sc_hwmtx);
	return (ipend);
}

int
econa_bus_param(struct uart_softc *sc, int baudrate, int databits,
    int stopbits, int parity)
{
	struct econa_softc *econa;
	struct uart_bas *bas;
	int error, limit;

	econa = (struct econa_softc*)sc;
	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);
	/*
	 * When using DW UART with BUSY detection it is necessary to wait
	 * until all serial transfers are finished before manipulating the
	 * line control. LCR will not be affected when UART is busy.
	 */
	if (econa->busy_detect != 0) {
		/*
		 * Pick an arbitrary high limit to avoid getting stuck in
		 * an infinite loop in case when the hardware is broken.
		 */
		limit = 10 * 1024;
		while (((uart_getreg(bas, DW_REG_USR) & USR_BUSY) != 0) &&
		    --limit)
			DELAY(4);

		if (limit <= 0) {
			/* UART appears to be stuck */
			uart_unlock(sc->sc_hwmtx);
			return (EIO);
		}
	}

	error = econa_param(bas, baudrate, databits, stopbits, parity);
	uart_unlock(sc->sc_hwmtx);
	return (error);
}

int
econa_bus_probe(struct uart_softc *sc)
{
	struct econa_softc *econa;
	struct uart_bas *bas;
	int error;
	uint8_t mcr;

	econa = (struct econa_softc *)sc;
	bas = &sc->sc_bas;

	error = econa_probe(bas);
	if (error)
		return (error);

	mcr = 0;
	if (sc->sc_sysdev == NULL) {
		/* By using econa_init() we also set DTR and RTS. */
		econa_init(bas, 115200, 8, 1, UART_PARITY_NONE);
	} else
		mcr |= MCR_RTS;

	error = econa_drain(bas, UART_DRAIN_TRANSMITTER);
	if (error)
		return (error);

	sc->sc_rxfifosz = 16;
	device_set_desc(sc->sc_dev, "econa class UART with FIFOs");

	/*
	 * Force the Tx FIFO size to 16 bytes for now. We don't program the
	 * Tx trigger. Also, we assume that all data has been sent when the
	 * interrupt happens.
	 */
	sc->sc_txfifosz = 16;

	return (0);
}

int
econa_bus_receive(struct uart_softc *sc)
{
	struct uart_bas *bas;
	int xc;
	uint8_t lsr;

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);
	lsr = uart_getreg(bas, REG_LSR);
	while (lsr & LSR_RXRDY) {
		if (uart_rx_full(sc)) {
			sc->sc_rxbuf[sc->sc_rxput] = UART_STAT_OVERRUN;
			break;
		}
		xc = uart_getreg(bas, REG_DATA);
		if (lsr & LSR_FE)
			xc |= UART_STAT_FRAMERR;
		if (lsr & LSR_PE)
			xc |= UART_STAT_PARERR;
		uart_rx_put(sc, xc);
		lsr = uart_getreg(bas, REG_LSR);
	}
	/* Discard everything left in the Rx FIFO. */
	while (lsr & LSR_RXRDY) {
		(void)uart_getreg(bas, REG_DATA);
		uart_barrier(bas);
		lsr = uart_getreg(bas, REG_LSR);
	}
	uart_unlock(sc->sc_hwmtx);
 	return (0);
}

int
econa_bus_setsig(struct uart_softc *sc, int sig)
{
	struct econa_softc *econa = (struct econa_softc*)sc;
	struct uart_bas *bas;
	uint32_t new, old;

	bas = &sc->sc_bas;
	do {
		old = sc->sc_hwsig;
		new = old;
		if (sig & SER_DDTR) {
			new = (new & ~SER_DTR) | (sig & (SER_DTR | SER_DDTR));
		}
		if (sig & SER_DRTS) {
			new = (new & ~SER_RTS) | (sig & (SER_RTS | SER_DRTS));
		}
	} while (!atomic_cmpset_32(&sc->sc_hwsig, old, new));
	uart_lock(sc->sc_hwmtx);
	econa->mcr &= ~MCR_RTS;
	if (new & SER_RTS)
		econa->mcr |= MCR_RTS;
	uart_setreg(bas, REG_MCR, econa->mcr);
	uart_barrier(bas);
	uart_unlock(sc->sc_hwmtx);
	return (0);
}

int
econa_bus_transmit(struct uart_softc *sc)
{
	struct econa_softc *econa = (struct econa_softc*)sc;
	struct uart_bas *bas;
	int i;

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);
	while ((uart_getreg(bas, REG_LSR) & LSR_THRE) == 0)
		;
	for (i = 0; i < sc->sc_txdatasz; i++) {
		uart_setreg(bas, REG_DATA, sc->sc_txbuf[i]);
		uart_barrier(bas);
	}
	uart_setreg(bas, REG_IER, econa->ier | IER_ETXRDY);
	uart_barrier(bas);
	if (broken_txfifo)
		econa_drain(bas, UART_DRAIN_TRANSMITTER);
	else
		sc->sc_txbusy = 1;
	uart_unlock(sc->sc_hwmtx);
	if (broken_txfifo)
		uart_sched_softih(sc, SER_INT_TXIDLE);
	return (0);
}

void
econa_bus_grab(struct uart_softc *sc)
{
	struct uart_bas *bas = &sc->sc_bas;
	struct econa_softc *econa = (struct econa_softc*)sc;
	u_char ier;

	/*
	 * turn off all interrupts to enter polling mode. Leave the
	 * saved mask alone. We'll restore whatever it was in ungrab.
	 * All pending interrupt signals are reset when IER is set to 0.
	 */
	uart_lock(sc->sc_hwmtx);
	ier = uart_getreg(bas, REG_IER);
	uart_setreg(bas, REG_IER, ier & econa->ier_mask);
	uart_barrier(bas);
	uart_unlock(sc->sc_hwmtx);
}

void
econa_bus_ungrab(struct uart_softc *sc)
{
	struct econa_softc *econa = (struct econa_softc*)sc;
	struct uart_bas *bas = &sc->sc_bas;

	/*
	 * Restore previous interrupt mask
	 */
	uart_lock(sc->sc_hwmtx);
	uart_setreg(bas, REG_IER, econa->ier);
	uart_barrier(bas);
	uart_unlock(sc->sc_hwmtx);
}
