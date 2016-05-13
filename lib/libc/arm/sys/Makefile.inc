# $FreeBSD$

.if ${MACHINE_ARCH} == "arm"
SRCS+=  trivial-vdso_tc.c
#CFLAGS+=-march=armv5te
#SRCS+=  __vdso_gettc.c
.else
SRCS+=  __vdso_gettc.c
.endif


MDASM= Ovfork.S brk.S cerror.S pipe.S ptrace.S sbrk.S shmat.S sigreturn.S syscall.S

# Don't generate default code for these syscalls:
NOASM=	break.o exit.o getlogin.o openbsd_poll.o sstk.o vfork.o yield.o

PSEUDO= _exit.o _getlogin.o
