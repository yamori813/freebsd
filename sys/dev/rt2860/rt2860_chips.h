
/*-
 * Copyright (c) 2009-2010 Alexander Egorenkov <egorenar@gmail.com>
 * Copyright (c) 2009 Damien Bergamini <damien.bergamini@free.fr>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <dev/rt2860/rt2860_io.h>
#include <dev/rt2860/rt2860_reg.h>

#if defined(RT2860_RT2880)
#if !defined(EEPROM_IN_FLASH)
static const uint8_t rt2880_eeprom[] = {
	0x80,0x28,0x03,0x01,0x00,0x0C,0x43,0x28,0x60,0x20,0x01,0x08,0x14,0x18,0x01,0x80,
	0x00,0x00,0x80,0x28,0x14,0x18,0x00,0x00,0x01,0x00,0x6A,0xFF,0x0C,0x00,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0x23,0x02,0x2C,0x00,0xFF,0xFF,0x1D,0x00,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0x0A,0x10,0x00,0x00,0x00,0x0C,0x00,0x00,0x00,0x08,0xFF,0xFF,
	0xFF,0xFF,0x0F,0x0F,0x0F,0x0F,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
	0x13,0x13,0x12,0x12,0x11,0x11,0x10,0x10,0x10,0x10,0x0F,0x0F,0x0F,0x0F,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,

	0x06,0x06,0x06,0x06,0x04,0x04,0x04,0x04,0x04,0x04,0x05,0x05,0x05,0x05,0x05,0x05,
	0x05,0x05,0x06,0x06,0x06,0x07,0x07,0x07,0x08,0x08,0x08,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x05,0x05,0x05,0x05,0x05,0x05,0x06,0x06,0x06,0x04,
	0x05,0x05,0x05,0x05,0x05,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,
	0x06,0x06,0x06,0x06,0x07,0x07,0x08,0x09,0x09,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x66,0x66,
	0xCC,0xAA,0x88,0x66,0xCC,0xAA,0x88,0x66,0xCC,0xAA,0x88,0x66,0xCC,0xAA,0x88,0x66,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,

	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,

	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	};
#endif

#endif

#if defined(RT2860_RT305X)
#if !defined(EEPROM_IN_FLASH)
/* Default EEPROM value for RT3050 */
static const uint8_t rt3050_eeprom[] = {
	0x50, 0x30, 0x01, 0x01, 0x00, 0x0c, 0x43, 0x30, 0x52, 0x88, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0c, 
	0x43, 0x30, 0x52, 0x77, 0x00, 0x0c, 0x43, 0x30, 0x52, 0x66, 0x11, 0x05, 0x20, 0x00, 
	0xff, 0xff, 0x2f, 0x01, 0x55, 0x77, 0xa8, 0xaa, 0x8c, 0x88, 0xff, 0xff, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x0d, 0x0d, 
	0x0d, 0x0d, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x10, 0x10, 
	0x11, 0x11, 0x11, 0x12, 0x12, 0x12, 0x13, 0x13, 0x13, 0x14, 0x14, 0x14, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x66, 0x66, 
	0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa, 
	0x88, 0x66, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
	};
#endif

static const REG_PAIR rt3052_rf_default[] = {
	{0,		0x50}, //
	{1,		0x01}, //
	{2,		0xF7}, //
	{3,		0x75}, //0x02}, //
	{4,		0x40},
	{5,		0x03}, //0x00},
	{6,		0x42}, //pass the crystal auto-tuned
	{7,		0x50}, //
	{8,		0x39}, //
	{9,		0x0F}, //0x04},
	{10,		0x60}, //0x71}, //0x72},
	{11,		0x21}, //0x11},
	{12,		0x75}, //0x7B}
	{13,		0x75}, //
	{14,		0x90}, //0xC0},
	{15,		0x58}, //0x78},
	{16,		0xB3}, //0xB2},
	{17,		0x92}, //Mason Hsu 20080624
	{18,		0x2C},
	{19,		0x02},
	{20,		0xBA},
	{21,		0xDB},
	{22,		0x00}, //
	{23,		0x31}, //
	{24,		0x08}, //
	{25,		0x01},
	{26, 		0x25}, //Core Power: 0x25=1.25v
	{27, 		0x23}, //RF: 1.35v (Mason Hsu 20081023)
	{28, 		0x13}, //ADC: must consist with R27
	{29,		0x83}, //0x18},
	{30,		0x00}, //
	{31,		0x00}, //
};

static const REG_PAIR rt305x_def_bbp[] =
{
	{ 31,	0x08 },
	{ 78,	0x0e },
	{ 80,	0x08 },
	{ 103,	0xc0 }
};
#endif

#if defined(RT2860_RT3350)
static const REG_PAIR rt3350_rf_default[] = {
	{0,		0xA1},
	{1,		0xC1},
	{2,		0xF1},
	{3,		0x72},
	{4,		0x40},
	{5,		0xCF},
	{6,		0x7E},
	{7,		0xD0},
	{8,		0x53},
	{9,		0x01},
	{10,		0x10},
	{11,		0x21},
	{12,		0x26},// QA : 0x2D(TX0_ALC)
	{13,		0xB0},
	{14,		0x00},
	{15,		0x78},
	{16,		0x44},
	{17,		0x92},
	{18,		0x6C},
	{19,		0xCC},
	{20,		0xBB},
	{21,		0x6F},
	{22,		0x00},
	{23,		0x12},
	{24,		0x08},
	{25,		0x3F},
	{26,	 	0x0D},
	{27, 		0x75},
   	{28, 		0x10},
	{29,		0x07},
	{30,		0x00},
	{31,		0x08},
};
#endif

#if defined(RT2860_RT3352)
static const REG_PAIR rt3352_rf_default[] = {
	{0, 0xF0},
	{1, 0x23}, /* R1 bit<1,0>=11 Path setting By EEPROM */
	{2, 0x50}, /* Fix Power DAC Variation.(20110928) */
	{3, 0x18},
	{4, 0x00},
	{5, 0x00}, /* Read only */
	{6, 0x33},
	{7, 0x00},
	{08, 0xF1}, /* By Channel Plan */
	{09, 0x02}, /* By Channel Plan */
	{10, 0xD2},
	{11, 0x42},
	{12, 0x1C},
	{13, 0x00},
	{14, 0x5A},
	{15, 0x00},
	{16, 0x01},
/*	{17, 0x1A}, By EEPROM Frequency offset */
	{18, 0x45},
	{19, 0x02},
	{20, 0x00},
	{21, 0x00},
	{22, 0x00},
	{23, 0x00},
	{24, 0x00},
	{25, 0x80},
	{26, 0x00},
	{27, 0x03},
	{28, 0x03},
	{29, 0x00},
	{30, 0x10}, /* 20MBW=0x10	40MBW=0x13 */
	{31, 0x80},
	{32, 0x80},
	{33, 0x00},
	{34, 0x01},
	{35, 0x03},
	{36, 0xBD},
	{37, 0x3C},
	{38, 0x5F},
	{39, 0xC5},
	{40, 0x33},
	{41, 0x5B},
	{42, 0x5B},
	{43, 0xDB},
	{44, 0xDB}, 
	{45, 0xDB},
	{46, 0xDD},
	{47, 0x0D},
	{48, 0x14},
	{49, 0x00},
	{50, 0x2D},
	{51, 0x7F},
	{52, 0x00},
	{53, 0x52},
	{54, 0x1B},
	{55, 0x7F},
	{56, 0x00},
	{57, 0x52},
	{58, 0x1B},
	{59, 0x00},
	{60, 0x00},
	{61, 0x00},
	{62, 0x00},
	{63, 0x00},
};
#endif

#if defined(RT2860_RT3883)
static const uint8_t rt3883_eeprom[] = {
	0x83, 0x38, 0x01, 0x00, 0x00, 0x0c, 0x43, 0x28, 0x83, 0x00, 0x83, 0x28, 0x14, 0x18, 0xff, 0xff,
	0xff, 0xff, 0x83, 0x28, 0x14, 0x18, 0x00, 0x00, 0x01, 0x00, 0x6a, 0xff, 0x00, 0x02, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0c, 0x43, 0x28, 0x83, 0x01, 0x00, 0x0c,
	0x43, 0x28, 0x83, 0x02, 0x33, 0x0a, 0xec, 0x00, 0x33, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0x20, 0x01, 0x55, 0x77, 0xa8, 0xaa, 0x8c, 0x88, 0xff, 0xff, 0x0a, 0x08, 0x08, 0x06,
	0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x05, 0x05,
	0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x06, 0x06, 0x06, 0x06,
	0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
	0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
	0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
	0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
	0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x06, 0x06,
	0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
	0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
	0x06, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0x66, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa,
	0x88, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xaa, 0xaa, 0x88, 0x66, 0xaa, 0xaa,
	0x88, 0x66, 0xaa, 0xaa, 0x88, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa,
	0x88, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xaa, 0xaa, 0x88, 0x66, 0xaa, 0xaa, 0x88, 0x66, 0xaa, 0xaa,
	0x88, 0x66, 0xaa, 0xaa, 0x88, 0x66, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	} ;

static const REG_PAIR rt3352_rf_default[] = {
	{00, 0xE0},
	{01, 0x03},
	{02, 0x50},
	{03, 0x20},
	{04, 0x00},
	{05, 0x00},
	{06, 0x40},
	{07, 0x00},
	{08, 0x5B},
	{09, 0x08},
	{10, 0xD3}, /* Gary, 2010-02-12 */
	{11, 0x48},
	{12, 0x1A},	/* Gary, 2011-03-10 */
	{13, 0x12},
	{14, 0x00},
	{15, 0x00},
	{16, 0x00},
/*	{17, 0x26}, */
	{18, 0x40},
	{19, 0x00},
	{20, 0x00},
	{21, 0x00},
	{22, 0x20},
	{23, 0xC0},
	{24, 0x00},
	{25, 0x00},
	{26, 0x00},
	{27, 0x00},
	{28, 0x00},
	{29, 0x00},
	{30, 0x10},
	{31, 0x80},
	{32, 0x80},
	{33, 0x00},
	{34, 0x20},
	{35, 0x00},
	{36, 0x00},
	{37, 0x00},
	{38, 0x86},	/* Gary, 2010-02-12 */
	{39, 0x23},
	{40, 0x00},
	{41, 0x00},
	{42, 0x00},
	{43, 0x00},
	{44, 0x93},	/* Gary, 2009-12-08 */
	{45, 0xBB},
	{46, 0x60},
	{47, 0x00},
	{48, 0x00},
	{49, 0x8E},
	{50, 0x86},
	{51, 0x51},
	{52, 0x05},
	{53, 0x76},
	{54, 0x76},
	{55, 0x76},
	{56, 0xDB},
	{57, 0x3E},
	{58, 0x00},
	{59, 0x00},
	{60, 0x00},
	{61, 0x00},
	{62, 0x00},
	{63, 0x00},
};
#endif

#if defined(RT2860_RT5350)
static const uint8_t rt5350_eeprom[] = {
	0x52, 0x33, 0x01, 0x01, 0x00, 0x0c, 0x43, 0x30, 0x52, 0x88, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0c, 
	0x43, 0x33, 0x52, 0x77, 0x00, 0x0c, 0x43, 0x33, 0x52, 0x66, 0x22, 0x0c, 0x20, 0x00, 
	0xff, 0xff, 0x2f, 0x01, 0x55, 0x77, 0xa8, 0xaa, 0x8c, 0x88, 0xff, 0xff, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x0d, 0x0d, 
	0x0d, 0x0d, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x10, 0x10, 
	0x11, 0x11, 0x11, 0x12, 0x12, 0x12, 0x13, 0x13, 0x13, 0x14, 0x14, 0x14, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x66, 0x66, 
	0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa, 0x88, 0x66, 0xcc, 0xaa, 
	0x88, 0x66, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
	} ;

static const REG_PAIR rt5350_rf_default[] = {
	{0, 0xF0},
	{1, 0x23},
	{2, 0x50},
	{3, 0x08},
	{4, 0x49},
	{5, 0x10},
	{6, 0xE0},
	{7, 0x00},
	{8, 0xF1},
	{9, 0x02},
	{10, 0x53},
	{11, 0x4A},
	{12, 0x46},
	{13, 0x9F},
	{14, 0x00},
	{15, 0x00},
	{16, 0xC0}, /* for 40M spur reduction */
/*	{17, 0x1A}, */
	{18, 0x03},
	{19, 0x00},
	{20, 0x00},
	{21, 0x00},
	{22, 0x20},
	{23, 0x00},
	{24, 0x00},
	{25, 0x80},
	{26, 0x00},
	{27, 0x03},
	{28, 0x00},
	{29, 0xD0}, /* optimize for H/W diversity, do not use 0x10 */
	{30, 0x10},
	{31, 0x80},
	{32, 0x80},
	{33, 0x00},
	{34, 0x07},
	{35, 0x12},
	{36, 0x00},
	{37, 0x08},
	{38, 0x85},
	{39, 0x1B},
	{40, 0x0B},
	{41, 0xBB},
	{42, 0xD5},
	{43, 0x9B},
	{44, 0x0C},
	{45, 0xA6},
	{46, 0x73},
	{47, 0x00},
	{48, 0x10},
	{49, 0x80},
	{50, 0x00},
	{51, 0x00},
	{52, 0x38},
	{53, 0x00},
	{54, 0x38},
	{55, 0x43},
	{56, 0x82},
	{57, 0x00},
	{58, 0x39},
	{59, 0x0B},
	{60, 0x45},
	{61, 0xD1},
	{62, 0x00},
	{63, 0x00},
};

static const RT2860_REG_PAIR rt5350_def_mac[] =
{
	{ RT2860_REG_TX_SW_CFG0, 	0x404 },
};
#endif

#if defined(RT6352)
	/* xxx */
#endif
