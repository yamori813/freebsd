/*-
 * Copyright (c) 2010, Aleksandr Rybalko <ray@ddteam.net>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
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

/*
 * GPIO driver for SIBA ChipCommon core.
 */

#ifndef _SIBA_CC_GPIO_H_
#define _SIBA_CC_GPIO_H_

#define SIBA_CC_GPIO_BASE		0x058
#define SIBA_CC_GPIO_SIZE		0x038

#define SIBA_CC_GPIO_PULLUP		0x000 /* 0x58, corerev >= 20 */
#define SIBA_CC_GPIO_PULLDOWN		0x004 /* 0x5c, corerev >= 20 */
#define SIBA_CC_GPIO_INPUT		0x008 /* R */
#define SIBA_CC_GPIO_OUTPUT		0x00c /* R/W */
#define SIBA_CC_GPIO_OUTEN		0x010 /* R/W */
#define SIBA_CC_GPIO_CONTROL		0x014 /* R/W */
#define SIBA_CC_GPIO_INT_POLARITY	0x018 /* W */
#define SIBA_CC_GPIO_INT_MASK		0x01c /* W */

/* GPIO events corerev >= 11 */
#define SIBA_CC_GPIO_EVENT		0x020
#define SIBA_CC_GPIO_EVENTMASK		0x024

/* Watchdog timer 0x028 */

/* GPIO events corerev >= 11 */
#define SIBA_CC_GPIO_EVENT_POLARITY	0x02c

/* LED powersave (corerev >= 16) */
/* 20% on/80% off = 20 << 16 | 80*/
#define SIBA_CC_GPIO_TIMER		0x030
#define SIBA_CC_GPIO_TIMER_ONTIME_SHIFT	16
#define SIBA_CC_GPIO_TIMER_DEFAULT	\
	    ((20 << SIBA_CC_GPIO_TIMER_ONTIME_SHIFT) | 80)
#define SIBA_CC_GPIO_TIMER_OUT_MASK	0x034

#endif /* _SIBA_CC_GPIO_H_ */

