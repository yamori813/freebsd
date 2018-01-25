/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2012 Semihalf
 * Copyright (c) 2009 Jakub Klama <jakub.klama@uj.edu.pl>
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

#include "opt_geom.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/slicer.h>

#include <geom/geom.h>
#include <geom/geom_slice.h>
#include <geom/geom_disk.h>

#include <dev/nand/nand_dev.h>

#define	FLASHMAP_CLASS_NAME "Flashmap"

#ifdef GEOM_FLASHMAP_ADJUST_BORDER
#define	FLASHMAP_MARKER_STR	".!/bin/sh"
#define	FLASHMAP_MAX_MARKER_LEN	64
#define	FLASHMAP_SEARCH_STEP	(64 * 1024)
#endif

struct g_flashmap_slice {
	off_t		sl_start;
	off_t		sl_end;
	const char	*sl_name;

	STAILQ_ENTRY(g_flashmap_slice) sl_link;
};

STAILQ_HEAD(g_flashmap_head, g_flashmap_slice);

static struct {
	const char	*type;
	flash_slicer_t	slicer;
} g_flashmap_slicers[] = {
	{ "NAND::device",	NULL },
	{ "CFI::device",	NULL },
	{ "SPI::device",	NULL },
	{ "MMC::device",	NULL }
};

static g_ioctl_t g_flashmap_ioctl;
static g_taste_t g_flashmap_taste;

static int g_flashmap_load(device_t dev, struct g_provider *pp,
    struct g_consumer *cp, flash_slicer_t slicer, struct g_flashmap_head *head);
static int g_flashmap_modify(struct g_geom *gp, const char *devname,
    int secsize, struct g_flashmap_head *slices);
static void g_flashmap_print(struct g_flashmap_slice *slice);

MALLOC_DECLARE(M_FLASHMAP);
MALLOC_DEFINE(M_FLASHMAP, "geom_flashmap", "GEOM flash memory slicer class");

static void
g_flashmap_print(struct g_flashmap_slice *slice)
{

	printf("%08jx-%08jx: %s (%juKB)\n", (uintmax_t)slice->sl_start,
	    (uintmax_t)slice->sl_end, slice->sl_name,
	    (uintmax_t)(slice->sl_end - slice->sl_start) / 1024);
}

static int
g_flashmap_modify(struct g_geom *gp, const char *devname, int secsize,
    struct g_flashmap_head *slices)
{
	struct g_flashmap_slice *slice;
	int i, error;

	g_topology_assert();

	i = 0;
	STAILQ_FOREACH(slice, slices, sl_link) {
		if (bootverbose) {
			printf("%s: slice ", devname);
			g_flashmap_print(slice);
		}

		error = g_slice_config(gp, i++, G_SLICE_CONFIG_CHECK,
		    slice->sl_start,
		    slice->sl_end - slice->sl_start + 1,
		    secsize, FLASH_SLICES_FMT, gp->name, slice->sl_name);

		if (error)
			return (error);
	}

	i = 0;
	STAILQ_FOREACH(slice, slices, sl_link) {
		error = g_slice_config(gp, i++, G_SLICE_CONFIG_SET,
		    slice->sl_start,
		    slice->sl_end - slice->sl_start + 1,
		    secsize, "%ss.%s", gp->name, slice->sl_name);

		if (error)
			return (error);
	}

	return (0);
}

static int
g_flashmap_ioctl(struct g_provider *pp, u_long cmd, void *data, int fflag,
    struct thread *td)
{
	struct g_consumer *cp;
	struct g_geom *gp;

	if (cmd != NAND_IO_GET_CHIP_PARAM)
		return (ENOIOCTL);

	cp = LIST_FIRST(&pp->geom->consumer);
	if (cp == NULL)
		return (ENOIOCTL);
	gp = cp->provider->geom;
	if (gp->ioctl == NULL)
		return (ENOIOCTL);

	return (gp->ioctl(cp->provider, cmd, data, fflag, td));
}

static struct g_geom *
g_flashmap_taste(struct g_class *mp, struct g_provider *pp, int flags)
{
	struct g_geom *gp;
	struct g_consumer *cp;
	struct g_flashmap_head head;
	struct g_flashmap_slice *slice, *slice_temp;
	flash_slicer_t slicer;
	device_t dev;
	int i, size;

	g_trace(G_T_TOPOLOGY, "flashmap_taste(%s,%s)", mp->name, pp->name);
	g_topology_assert();

	if (flags == G_TF_NORMAL &&
	    strcmp(pp->geom->class->name, G_DISK_CLASS_NAME) != 0)
		return (NULL);

	gp = g_slice_new(mp, FLASH_SLICES_MAX_NUM, pp, &cp, NULL, 0, NULL);
	if (gp == NULL)
		return (NULL);

	STAILQ_INIT(&head);

	do {
		slicer = NULL;
		for (i = 0; i < nitems(g_flashmap_slicers); i++) {
			size = sizeof(device_t);
			if (g_io_getattr(g_flashmap_slicers[i].type, cp,
			    &size, &dev) == 0) {
				slicer = g_flashmap_slicers[i].slicer;
				break;
			}
		}
		if (slicer == NULL)
			break;

		if (g_flashmap_load(dev, pp, cp, slicer, &head) == 0)
			break;

		g_flashmap_modify(gp, cp->provider->name,
		    cp->provider->sectorsize, &head);
	} while (0);

	g_access(cp, -1, 0, 0);

	STAILQ_FOREACH_SAFE(slice, &head, sl_link, slice_temp)
		free(slice, M_FLASHMAP);

	if (LIST_EMPTY(&gp->provider)) {
		g_slice_spoiled(cp);
		return (NULL);
	}
	return (gp);
}

#ifdef GEOM_FLASHMAP_ADJUST_BORDER
static int
find_marker(struct g_consumer *cp, off_t *offset)
{
	off_t search_start, search_offset, search_step;
	uint8_t *buf;
	size_t sectorsize;
	char key[FLASHMAP_MAX_MARKER_LEN], search_key[FLASHMAP_MAX_MARKER_LEN];
	int c;

	search_start = *offset;
	sectorsize = cp->provider->sectorsize;
	search_step = FLASHMAP_SEARCH_STEP;
	strcpy(search_key, FLASHMAP_MARKER_STR);

	for (search_offset = search_start;
	     search_offset < cp->provider->mediasize;
	     search_offset += search_step) {

		g_topology_unlock();
		buf = g_read_data(cp, search_offset, sectorsize, NULL);
		g_topology_lock();

		strncpy(key, search_key, FLASHMAP_MAX_MARKER_LEN);

		for (c = 0; c < FLASHMAP_MAX_MARKER_LEN && key[c]; c++) {
			if (key[c] == '.') {
				key[c] = buf[c];
			}
		}

		/* Assume buf != NULL here */
		if (memcmp(buf, key, strlen(search_key)) == 0) {
			g_free(buf);
			/* Marker found, so return their offset */
			if (*offset == search_offset)
				return (0);
			*offset = search_offset;
			return (1);
		}
		g_free(buf);
	}

	printf("geom_flash map not found marker\n");
	return (0);
}

static void
adjust_print(char *str, off_t from, off_t to)
{

	printf("adjust %s %08jx to %08jx\n", str, (uintmax_t)from,
	    (uintmax_t)to);
}
#endif

static int
g_flashmap_load(device_t dev, struct g_provider *pp, struct g_consumer *cp,
    flash_slicer_t slicer, struct g_flashmap_head *head)
{
	struct flash_slice *slices;
	struct g_flashmap_slice *slice;
	int i, nslices = 0;

	slices = malloc(sizeof(struct flash_slice) * FLASH_SLICES_MAX_NUM,
	    M_FLASHMAP, M_WAITOK | M_ZERO);
	if (slicer(dev, pp->name, slices, &nslices) == 0) {
		for (i = 0; i < nslices; i++) {
			slice = malloc(sizeof(struct g_flashmap_slice),
			    M_FLASHMAP, M_WAITOK);

			slice->sl_name = slices[i].label;
			slice->sl_start = slices[i].base;
			slice->sl_end = slices[i].base + slices[i].size - 1;
#ifdef GEOM_FLASHMAP_ADJUST_BORDER
			if(strcmp("kernel", slice->sl_name) == 0) {
				++slice->sl_end;
				if (find_marker(cp, &slice->sl_end) ) {
					adjust_print("kernel end",
					    slices[i].base + slices[i].size - 1,
					    slice->sl_end - 1);
				}
				--slice->sl_end;
			}
			if(strcmp("rootfs", slice->sl_name) == 0) {
				if( find_marker(cp, &slice->sl_start) ) {
					adjust_print("rootfs start",
					    slices[i].base,
					    slice->sl_start);
					slice->sl_end -= slice->sl_start - 
					    slices[i].base;
				}
			}
#endif

			STAILQ_INSERT_TAIL(head, slice, sl_link);
		}
	}

	free(slices, M_FLASHMAP);
	return (nslices);
}

void flash_register_slicer(flash_slicer_t slicer, u_int type, bool force)
{

	g_topology_lock();
	if (g_flashmap_slicers[type].slicer == NULL || force == TRUE)
		g_flashmap_slicers[type].slicer = slicer;
	g_topology_unlock();
}

static struct g_class g_flashmap_class = {
	.name = FLASHMAP_CLASS_NAME,
	.version = G_VERSION,
	.taste = g_flashmap_taste,
	.ioctl = g_flashmap_ioctl,
};

DECLARE_GEOM_CLASS(g_flashmap_class, g_flashmap);
MODULE_VERSION(g_flashmap, 0);
