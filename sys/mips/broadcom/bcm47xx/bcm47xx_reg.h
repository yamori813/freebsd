/* $FreeBSD: head/sys/mips/sentry5/s5reg.h 178173 2008-04-13 07:44:55Z imp $ */

#ifndef _MIPS32_BCM47XX_BCM47XXREG_H_
#define _MIPS32_BCM47XX_BCM47XXREG_H_

#define	BCM47XX_UART0ADR	0x18000300
#define	BCM47XX_UART1ADR	0x18000400

/* Reset register implemented here in a PLD device. */
#define	BCM47XX_EXTIFADR	0x18000000
#define	BCM47XX_DORESET		0x80





/*
 * Custom CP0 register macros.
 * XXX: This really needs the mips cpuregs.h file for the barrier.
 */
#define BCM47XX_RW32_C0P0_CUST22(n,r)				\
static __inline u_int32_t					\
s5_rd_ ## n (void)						\
{								\
	int v0;							\
	__asm __volatile ("mfc0 %[v0], $22, "__XSTRING(r)" ;"	\
			  : [v0] "=&r"(v0));			\
	/*mips_barrier();*/					\
	return (v0);						\
}								\
static __inline void						\
s5_wr_ ## n (u_int32_t a0)					\
{								\
	__asm __volatile ("mtc0 %[a0], $22, "__XSTRING(r)" ;"	\
			 __XSTRING(COP0_SYNC)";"		\
			 "nop;"					\
			 "nop;"					\
			 :					\
			 : [a0] "r"(a0));			\
	/*mips_barrier();*/					\
} struct __hack

/*
 * All 5 of these sub-registers are used by Linux.
 * There is a further custom register at 25 which is not used.
 */
#define	BCM47XX_CP0_DIAG	0
#define	BCM47XX_CP0_CLKCFG1	1
#define	BCM47XX_CP0_CLKCFG2	2
#define	BCM47XX_CP0_SYNC	3
#define	BCM47XX_CP0_CLKCFG3	4
#define	BCM47XX_CP0_RESET	5

/* s5_[rd|wr]_xxx() */
BCM47XX_RW32_C0P0_CUST22(diag, BCM47XX_CP0_DIAG);
BCM47XX_RW32_C0P0_CUST22(clkcfg1, BCM47XX_CP0_CLKCFG1);
BCM47XX_RW32_C0P0_CUST22(clkcfg2, BCM47XX_CP0_CLKCFG2);
BCM47XX_RW32_C0P0_CUST22(sync, BCM47XX_CP0_SYNC);
BCM47XX_RW32_C0P0_CUST22(clkcfg3, BCM47XX_CP0_CLKCFG3);
BCM47XX_RW32_C0P0_CUST22(reset, BCM47XX_CP0_RESET);

#endif /* _MIPS32_BCM47XX_BCM47XXREG_H_ */
