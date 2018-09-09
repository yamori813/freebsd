
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

/*
 * rt2860_io_mac_read
 */
uint32_t rt2860_io_mac_read(struct rt2860_softc *sc, uint16_t reg)
{

	return bus_space_read_4(sc->bst, sc->bsh, reg);
}

/*
 * rt2860_io_mac_read_multi
 */
void rt2860_io_mac_read_multi(struct rt2860_softc *sc,
	uint16_t reg, void *buf, size_t len)
{

	bus_space_read_region_1(sc->bst, sc->bsh, reg, buf, len);
}

/*
 * rt2860_io_mac_write
 */
void rt2860_io_mac_write(struct rt2860_softc *sc,
	uint16_t reg, uint32_t val)
{

	bus_space_write_4(sc->bst, sc->bsh, reg, val);
}

/*
 * rt2860_io_mac_write_multi
 */
void rt2860_io_mac_write_multi(struct rt2860_softc *sc,
	uint16_t reg, const void *buf, size_t len)
{
	int i;
	const uint8_t *p;

	p = buf;
	for (i = 0; i < len; i ++)
		bus_space_write_1(sc->bst, sc->bsh, reg + i, *(p+i));

#ifdef notyet
	bus_space_write_region_1(sc->bst, sc->bsh, reg, buf, len);
#endif
}

/*
 * rt2860_io_mac_set_region_4
 */
void rt2860_io_mac_set_region_4(struct rt2860_softc *sc,
	uint16_t reg, uint32_t val, size_t len)
{
	int i;

	for (i = 0; i < len; i += sizeof(uint32_t))
		rt2860_io_mac_write(sc, reg + i, val);
}

