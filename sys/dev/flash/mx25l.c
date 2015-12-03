/*-
 * Copyright (c) 2006 M. Warner Losh.  All rights reserved.
 * Copyright (c) 2009 Oleksandr Tymoshenko.  All rights reserved.
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
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: head/sys/dev/flash/mx25l.c 239794 2012-08-28 22:17:22Z adrian $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <geom/geom_disk.h>

#include <dev/spibus/spi.h>
#include "spibus_if.h"

#define MX25L_DEBUG

#include <dev/flash/mx25lreg.h>

#define	FL_NONE			0x00
#define	FL_ERASE_4K		0x01
#define	FL_ERASE_32K		0x02
#define	FL_2BYTE_ADDR		0x04

/*
 * Define the sectorsize to be a smaller size rather than the flash
 * sector size. Trying to run FFS off of a 64k flash sector size
 * results in a completely un-usable system.
 */
#define	MX25L_SECTORSIZE	512

struct mx25l_flash_ident
{
	const char	*name;
	uint8_t		manufacturer_id;
	uint16_t	device_id;
	unsigned int	sectorsize;
	unsigned int	sectorcount;
	unsigned int	pagesize;
	unsigned int	flags;
};

struct mx25l_softc 
{
	device_t	sc_dev;
	uint8_t		sc_manufacturer_id;
	uint32_t	sc_pagesize;
	uint16_t	sc_device_id;
	uint32_t	sc_debug;
	unsigned int	sc_sectorsize;
	struct mtx	sc_mtx;
	struct disk	*sc_disk;
	struct proc	*sc_p;
	struct bio_queue_head sc_bio_queue;
	unsigned int	sc_flags;
};

typedef enum {
	MX25L_DBG_READ 	=	0x00000001,
	MX25L_DBG_WRITE	=	0x00000002,
	MX25L_DBG_ERASE	=	0x00000004,
	MX25L_DBG_EACHWRITE=	0x00000008,
	MX25L_DBG_INFO	=	0x00000010,
} mx25l_debug_flags;

#ifdef MX25L_DEBUG
#define	MX25LDEBUG(_sc, _m, ...) 					\
	do {								\
		if ((_m) & (_sc)->sc_debug)				\
			device_printf((_sc)->sc_dev, __VA_ARGS__);	\
	} while (0)
#else
#define	MX25LDEBUG(_sc, _m, ...)
#endif

#define M25PXX_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	M25PXX_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define M25PXX_LOCK_INIT(_sc) \
	mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->sc_dev), \
	    "mx25l", MTX_DEF)
#define M25PXX_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);
#define M25PXX_ASSERT_LOCKED(_sc)	mtx_assert(&_sc->sc_mtx, MA_OWNED);
#define M25PXX_ASSERT_UNLOCKED(_sc) mtx_assert(&_sc->sc_mtx, MA_NOTOWNED);

/* disk routines */
static int mx25l_open(struct disk *dp);
static int mx25l_close(struct disk *dp);
static int mx25l_ioctl(struct disk *, u_long, void *, int, struct thread *);
static void mx25l_strategy(struct bio *bp);
static void mx25l_task(void *arg);

struct mx25l_flash_ident flash_devices[] = {
	{ "mx25ll32",    0xc2, 0x2016, 64 * 1024,  64, 256, FL_NONE },
	{ "m25p64",      0x20, 0x2017, 64 * 1024, 128, 256, FL_NONE },
	{ "mx25ll64",    0xc2, 0x2017, 64 * 1024, 128, 256, FL_NONE },
	{ "mx25ll128",   0xc2, 0x2018, 64 * 1024, 256, 256, FL_ERASE_4K | FL_ERASE_32K },
	{ "s25fl128",    0x01, 0x2018, 64 * 1024, 256, 256, FL_NONE },
	{ "s25s1032",    0x01, 0x0215, 64 * 1024, 64,  256, FL_NONE },
	{ "s25sl064a",   0x01, 0x0216, 64 * 1024, 128, 256, FL_NONE },
	/* EON -- en25pxx */
	{ "en25p32",     0x1c, 0x2016, 64 * 1024,  64, 256, FL_NONE },
	{ "en25f32",     0x1c, 0x3116, 64 * 1024,  64, 256, FL_NONE },
	{ "en25p64",     0x1c, 0x2017, 64 * 1024, 128, 256, FL_NONE },
	{ "en25q64",     0x1c, 0x3017, 64 * 1024, 128, 256, FL_ERASE_4K },
	/* EEPROMs, ID cmd not supported, can only be hinted */
	{ "at25128",        0,      0,        64, 256,  64, FL_NONE },
	{ "at25256",        0,      0,        64, 512,  64, FL_NONE },
	{ "w25q64", 	 0xef, 0x4017, 64 * 1024, 128, 256, FL_ERASE_4K },
	{ "w25q64bv",    0xef, 0x4017, 64 * 1024, 128, 256, FL_ERASE_4K },
	{ "SST25VF032B", 0xbf, 0x254a, 64 * 1024,  64, 256, FL_ERASE_4K | FL_ERASE_32K },
	{ "w25x32",      0xef, 0x3016, 64 * 1024,  64, 256, FL_ERASE_4K },
};

#define MX25L_IDENT_SIZE (sizeof(flash_devices)/sizeof(struct mx25l_flash_ident))

static uint8_t
mx25l_get_status(device_t dev)
{
	struct mx25l_softc *sc;
	uint8_t txBuf[4], rxBuf[4];
	struct spi_command cmd;
	int err;

	sc = device_get_softc(dev);

	memset(&cmd, 0, sizeof(cmd));
	memset(txBuf, 0, sizeof(txBuf));
	memset(rxBuf, 0, sizeof(rxBuf));

	txBuf[0] = CMD_READ_STATUS;
	cmd.tx_cmd = txBuf;
	cmd.tx_cmd_sz = 1;
	cmd.rx_data = rxBuf;
	cmd.rx_data_sz = 1;

	err = SPIBUS_TRANSFER(device_get_parent(dev), dev, &cmd);
	if (err)
		return (0);

	MX25LDEBUG(sc, MX25L_DBG_INFO, "STATUS=%02x\n", rxBuf[0]);

	return (rxBuf[0]);
}

static void
mx25l_wait_for_device_ready(device_t dev)
{
	while ((mx25l_get_status(dev) & STATUS_WIP))
		continue;
}

static struct mx25l_flash_ident*
mx25l_get_device_ident(struct mx25l_softc *sc)
{
	device_t dev = sc->sc_dev;
	uint8_t txBuf[8], rxBuf[8];
	struct spi_command cmd;
	uint8_t manufacturer_id;
	uint16_t dev_id;
	int err, i;

	memset(&cmd, 0, sizeof(cmd));
	memset(txBuf, 0, sizeof(txBuf));
	memset(rxBuf, 0, sizeof(rxBuf));

	txBuf[0] = CMD_READ_IDENT;
	cmd.tx_cmd = &txBuf;
	cmd.tx_cmd_sz = 1;
	cmd.rx_data = &rxBuf;
	cmd.rx_data_sz = 5;
	/*
	 * Some compatible devices has extended two-bytes ID
	 * We'll use only manufacturer/deviceid atm
	 */
	err = SPIBUS_TRANSFER(device_get_parent(dev), dev, &cmd);
	if (err)
		return (NULL);

	MX25LDEBUG(sc, MX25L_DBG_INFO, "DEVID=%02x %02x %02x %02x %02x\n",
	    rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4]);

	manufacturer_id = rxBuf[0];
	dev_id = (rxBuf[1] << 8) | (rxBuf[2]);

	for (i = 0; i < MX25L_IDENT_SIZE; i++) {
		if ((flash_devices[i].manufacturer_id == manufacturer_id) &&
		    (flash_devices[i].device_id == dev_id))
			return &flash_devices[i];
	}

	printf("Unknown SPI flash device. Vendor: %02x, device id: %04x\n",
	    manufacturer_id, dev_id);
	return (NULL);
}

static void
mx25l_set_writable(device_t dev, int writable)
{
	struct mx25l_softc *sc;
	uint8_t txBuf[4];//, rxBuf[4];
	uint8_t status;
	struct spi_command cmd;
	int err;

	sc = device_get_softc(dev);

	MX25LDEBUG(sc, MX25L_DBG_WRITE, "%s(dev, writable=%d)\n", __func__,
	    writable);

	memset(&cmd, 0, sizeof(cmd));
	memset(txBuf, 0, sizeof(txBuf));

	txBuf[0] = writable ? CMD_WRITE_ENABLE : CMD_WRITE_DISABLE;
	cmd.tx_cmd = txBuf;
	cmd.tx_cmd_sz = 1;
	err = SPIBUS_TRANSFER(device_get_parent(dev), dev, &cmd);

	status = mx25l_get_status(dev);

	if (writable && !(status & STATUS_WEL))
		device_printf(dev, "%s - fail\n", __func__);
}

static void
mx25l_erase_cmd(device_t dev, off_t sector, uint8_t ecmd)
{
	struct mx25l_softc *sc;
	uint8_t txBuf[4], rxBuf[4];
	struct spi_command cmd;
	int err;

	sc = device_get_softc(dev);

	MX25LDEBUG(sc, MX25L_DBG_ERASE,
	    "%s(dev, sector=0x%08x, ecmd=0x%08x)\n",
	    __func__, (uint32_t)sector, (uint8_t)ecmd);

	mx25l_wait_for_device_ready(dev);
	mx25l_set_writable(dev, 1);

	memset(&cmd, 0, sizeof(cmd));
	memset(txBuf, 0, sizeof(txBuf));
	memset(rxBuf, 0, sizeof(rxBuf));

	txBuf[0] = ecmd;
	txBuf[1] = ((sector >> 16) & 0xff);
	txBuf[2] = ((sector >> 8) & 0xff);
	txBuf[3] = (sector & 0xff);
	cmd.tx_cmd = txBuf;
	cmd.tx_cmd_sz = 4;
	cmd.rx_cmd = rxBuf; /* XXX check it */
	cmd.rx_cmd_sz = 4;  /* XXX check it */
	err = SPIBUS_TRANSFER(device_get_parent(dev), dev, &cmd);
}

static int
mx25l_write(device_t dev, off_t offset, caddr_t data, off_t count)
{
	struct mx25l_softc *sc;
	uint8_t txBuf[8];
	struct spi_command cmd;
	off_t write_offset;
	long bytes_to_write, bytes_writen;
	device_t pdev;
	int err = 0;

	pdev = device_get_parent(dev);
	sc = device_get_softc(dev);

	MX25LDEBUG(sc, MX25L_DBG_WRITE,
	    "%s(dev, offset=0x%08x, data, count=0x%08x)\n",
	    __func__, (uint32_t)offset, (uint32_t)count);

	bytes_writen = 0;
	write_offset = offset;

	/*
	 * Use the erase sectorsize here since blocks are fully erased
	 * first before they're written to.
	 */
	if (count % sc->sc_sectorsize != 0 || offset % sc->sc_sectorsize != 0)
		return (EIO);

	/*
	 * Assume here that we write per-sector only 
	 * and sector size should be 256 bytes aligned
	 */
	KASSERT(write_offset % sc->sc_pagesize == 0,
	    ("offset for BIO_WRITE is not page size (%d bytes) aligned",
		sc->sc_pagesize));

	/*
	 * Maximum write size for CMD_PAGE_PROGRAM is 
	 * sc->sc_pagesize, so split data to chunks 
	 * sc->sc_pagesize bytes eash and write them
	 * one by one
	 */
	while (bytes_writen < count) {
		/*
		 * If we crossed sector boundary - erase next sector
		 */
		if (((offset + bytes_writen) % sc->sc_sectorsize) == 0)
			mx25l_erase_cmd(dev, offset + bytes_writen, CMD_SECTOR_ERASE);

		txBuf[0] = CMD_PAGE_PROGRAM;
		if (sc->sc_flags & FL_2BYTE_ADDR) {
			txBuf[1] = ((write_offset >> 8) & 0xff);
			txBuf[2] = (write_offset & 0xff);
			cmd.tx_cmd_sz = 3;
		} else {
			txBuf[1] = ((write_offset >> 16) & 0xff);
			txBuf[2] = ((write_offset >> 8) & 0xff);
			txBuf[3] = (write_offset & 0xff);
			cmd.tx_cmd_sz = 4;
		}

		bytes_to_write = MIN(sc->sc_pagesize,
		    count - bytes_writen);
		cmd.tx_cmd = txBuf;
		cmd.rx_cmd_sz = 0;
		cmd.tx_data = data + bytes_writen;
		cmd.tx_data_sz = bytes_to_write;
		cmd.rx_data_sz = 0;

		/*
		 * Eash completed write operation resets WEL 
		 * (write enable latch) to disabled state,
		 * so we re-enable it here 
		 */
		mx25l_wait_for_device_ready(dev);
		mx25l_set_writable(dev, 1);

		err = SPIBUS_TRANSFER(pdev, dev, &cmd);
		if (err) {
			MX25LDEBUG(sc, MX25L_DBG_EACHWRITE,
			    "%s: error writing %ld bytes\n",
			    __func__, bytes_to_write);

			break;
		}
		MX25LDEBUG(sc, MX25L_DBG_EACHWRITE,
		    "%s: write %ld bytes - successful\n",
		    __func__, bytes_to_write);

		bytes_writen += bytes_to_write;
		write_offset += bytes_to_write;
	}

	return (err);
}

static int
mx25l_read(device_t dev, off_t offset, caddr_t data, off_t count)
{
	struct mx25l_softc *sc;
	uint8_t txBuf[8];
	struct spi_command cmd;
	device_t pdev;
	int err = 0;

	pdev = device_get_parent(dev);
	sc = device_get_softc(dev);

	MX25LDEBUG(sc, MX25L_DBG_READ,
	    "%s(dev, offset=0x%08x, data, count=0x%08x)\n",
	    __func__, (uint32_t)offset, (uint32_t)count);

	err = SPIBUS_GET_BLOCK(pdev, dev, offset, data, count);
	if (!err)
		return (0);

	/*
	 * Enforce the disk read sectorsize not the erase sectorsize.
	 * In this way, smaller read IO is possible,dramatically
	 * speeding up filesystem/geom_compress access.
	 */
	if (count % sc->sc_disk->d_sectorsize != 0
	    || offset % sc->sc_disk->d_sectorsize != 0)
		return (EIO);

	txBuf[0] = CMD_FAST_READ;
	if (sc->sc_flags & FL_2BYTE_ADDR) {
		txBuf[1] = ((offset >> 8) & 0xff);
		txBuf[2] = (offset & 0xff);
		cmd.tx_cmd_sz = 3;
	} else {
		txBuf[1] = ((offset >> 16) & 0xff);
		txBuf[2] = ((offset >> 8) & 0xff);
		txBuf[3] = (offset & 0xff);
		/* Dummy byte */
		txBuf[4] = 0;
		cmd.tx_cmd_sz = 5;
	}

	cmd.tx_cmd = txBuf;
	cmd.rx_cmd_sz = 0;

	cmd.rx_data = data;
	cmd.rx_data_sz = count;
	cmd.tx_data_sz = 0;

	err = SPIBUS_TRANSFER(pdev, dev, &cmd);

	return (err);
}

#ifdef	MX25L_DEBUG
#include <sys/sysctl.h>
static void
mx25l_attach_sysctl(device_t dev)
{
	struct mx25l_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;

	sc = device_get_softc(dev);
	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);

	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		"debug", CTLFLAG_RW, &sc->sc_debug, 0,
		"mx25l debugging flags");
}
#endif

static int
mx25l_probe(device_t dev)
{
	device_set_desc(dev, "M25Pxx Flash Family");
	return (0);
}

static int
mx25l_attach(device_t dev)
{
	int i;
	const char *chipname;
	struct mx25l_softc *sc;
	struct mx25l_flash_ident *ident = NULL;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_debug = 0;
	M25PXX_LOCK_INIT(sc);

	if (resource_string_value(device_get_name(dev),
	    device_get_unit(dev), "chipname", &chipname))
		 chipname = NULL;

	/* If chip name hinted */
	if (chipname) {
		for (i = 0; i < MX25L_IDENT_SIZE; i++) {
			if (strcmp(flash_devices[i].name, chipname) == 0) {
				ident = &flash_devices[i];
				break;
			}
		}
	}

	/* if not hinted or if hinted not found */
	if (ident == NULL)
		ident = mx25l_get_device_ident(sc);

	if (ident == NULL)
		return (ENXIO);

	mx25l_wait_for_device_ready(sc->sc_dev);

	sc->sc_disk = disk_alloc();
	sc->sc_disk->d_open = mx25l_open;
	sc->sc_disk->d_close = mx25l_close;
	sc->sc_disk->d_strategy = mx25l_strategy;
	sc->sc_disk->d_ioctl = mx25l_ioctl;
	sc->sc_disk->d_name = "flash/spi";
	sc->sc_disk->d_drv1 = sc;
	sc->sc_disk->d_maxsize = ident->sectorsize;
	sc->sc_disk->d_sectorsize = 4;
	sc->sc_disk->d_mediasize = ident->sectorsize * ident->sectorcount;
	sc->sc_disk->d_unit = device_get_unit(sc->sc_dev);
	sc->sc_disk->d_dump = NULL;		/* NB: no dumps */
	/* Sectorsize for erase operations */
	sc->sc_sectorsize =  ident->sectorsize;
	sc->sc_flags = ident->flags;
	sc->sc_flags |= (sc->sc_disk->d_mediasize <= (64 * 1024)) ?
	    FL_2BYTE_ADDR : 0;
	sc->sc_pagesize =  ident->pagesize;

        /* NB: use stripesize to hold the erase/region size for RedBoot */
	sc->sc_disk->d_stripesize = ident->sectorsize;

	disk_create(sc->sc_disk, DISK_VERSION);
	bioq_init(&sc->sc_bio_queue);

	kproc_create(&mx25l_task, sc, &sc->sc_p, 0, 0, "task: mx25l flash");
	device_printf(sc->sc_dev, "%s, sector %d bytes, %d sectors\n", 
	    ident->name, ident->sectorsize, ident->sectorcount);

#ifdef	MX25L_DEBUG
	/* setup sysctl variables */
	mx25l_attach_sysctl(dev);
#endif

	return (0);
}

static int
mx25l_detach(device_t dev)
{

	return (EIO);
}

static int
mx25l_open(struct disk *dp)
{
	return (0);
}

static int
mx25l_close(struct disk *dp)
{

	return (0);
}

static int
mx25l_ioctl(struct disk *dp, u_long cmd, void *data, int fflag,
	struct thread *td)
{

	return (EINVAL);
}

static void
mx25l_strategy(struct bio *bp)
{
	struct mx25l_softc *sc;

	sc = (struct mx25l_softc *)bp->bio_disk->d_drv1;
	M25PXX_LOCK(sc);
	bioq_disksort(&sc->sc_bio_queue, bp);
	wakeup(sc);
	M25PXX_UNLOCK(sc);
}

static void
mx25l_task(void *arg)
{
	struct mx25l_softc *sc = (struct mx25l_softc*)arg;
	struct bio *bp;
	device_t dev;

	for (;;) {
		dev = sc->sc_dev;
		M25PXX_LOCK(sc);
		do {
			bp = bioq_first(&sc->sc_bio_queue);
			if (bp == NULL)
				msleep(sc, &sc->sc_mtx, PRIBIO, "jobqueue", 0);
		} while (bp == NULL);
		bioq_remove(&sc->sc_bio_queue, bp);
		M25PXX_UNLOCK(sc);

		switch (bp->bio_cmd) {
		case BIO_READ:
			bp->bio_error = mx25l_read(dev, bp->bio_offset, 
			    bp->bio_data, bp->bio_bcount);
			break;
		case BIO_WRITE:
			bp->bio_error = mx25l_write(dev, bp->bio_offset, 
			    bp->bio_data, bp->bio_bcount);
			break;
		default:
			bp->bio_error = EINVAL;
		}


		biodone(bp);
	}
}

static devclass_t mx25l_devclass;

static device_method_t mx25l_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mx25l_probe),
	DEVMETHOD(device_attach,	mx25l_attach),
	DEVMETHOD(device_detach,	mx25l_detach),

	{ 0, 0 }
};

static driver_t mx25l_driver = {
	"mx25l",
	mx25l_methods,
	sizeof(struct mx25l_softc),
};

DRIVER_MODULE(mx25l, spibus, mx25l_driver, mx25l_devclass, 0, 0);
