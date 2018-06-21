
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

#ifndef _RT2860_SOFTC_H_
#define _RT2860_SOFTC_H_

#include <sys/param.h>
#include <sys/sysctl.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/taskqueue.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/malloc.h>
#include <sys/firmware.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_input.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_regdomain.h>

//#include "opt_rt2860.h"
#define RT2860_DEBUG
//#define RT2860_HW_CRYPTO

#include <dev/rt2860/rt2860_rxdesc.h>
#include <dev/rt2860/rt2860_txdesc.h>
#include <dev/rt2860/rt2860_txwi.h>
#include <dev/rt2860/rt2860_amrr.h>

#define	RT_CHIPID_RT3050				0x3050
#define	RT_CHIPID_RT3052				0x3052

#define RT2860_SOFTC_LOCK(sc)				mtx_lock(&(sc)->lock)
#define RT2860_SOFTC_UNLOCK(sc)				\
		mtx_unlock(&(sc)->lock)
#define	RT2860_SOFTC_ASSERT_LOCKED(sc)			\
		mtx_assert(&(sc)->lock, MA_OWNED)

#define RT2860_SOFTC_TX_RING_LOCK(ring)			mtx_lock(&(ring)->lock)
#define RT2860_SOFTC_TX_RING_UNLOCK(ring)		\
		mtx_unlock(&(ring)->lock)
#define	RT2860_SOFTC_TX_RING_ASSERT_LOCKED(ring)	\
		mtx_assert(&(ring)->lock, MA_OWNED)

#define RT2860_SOFTC_FLAGS_UCODE_LOADED			(1 << 0)

#define RT2860_SOFTC_LED_OFF_COUNT			3

#define RT2860_SOFTC_RSSI_OFF_COUNT			3

#define RT2860_SOFTC_LNA_GAIN_COUNT			4

#define RT2860_SOFTC_TXPOW_COUNT			50

#define RT2860_SOFTC_TXPOW_RATE_COUNT			5

#define RT2860_SOFTC_TSSI_COUNT				9

#define RT2860_SOFTC_BBP_EEPROM_COUNT			8

#define RT2860_SOFTC_RSSI_COUNT				3

#define RT2860_SOFTC_STAID_COUNT			64

#define RT2860_SOFTC_TX_RING_COUNT			6

#define RT2860_SOFTC_RX_RING_DATA_COUNT			128

#define RT2860_SOFTC_MAX_SCATTER			10

#define RT2860_SOFTC_TX_RING_DATA_COUNT			256
#define RT2860_SOFTC_TX_RING_DESC_COUNT					\
	(RT2860_SOFTC_TX_RING_DATA_COUNT * RT2860_SOFTC_MAX_SCATTER)

#define RT2860_SOFTC_RX_RADIOTAP_PRESENT				\
	((1 << IEEE80211_RADIOTAP_FLAGS) |				\
	 (1 << IEEE80211_RADIOTAP_RATE) |				\
	 (1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL) |			\
	 (1 << IEEE80211_RADIOTAP_DBM_ANTNOISE) |			\
	 (1 << IEEE80211_RADIOTAP_ANTENNA) |				\
	 (1 << IEEE80211_RADIOTAP_DB_ANTSIGNAL) |			\
	 (1 << IEEE80211_RADIOTAP_XCHANNEL))

#define RT2860_SOFTC_TX_RADIOTAP_PRESENT				\
	((1 << IEEE80211_RADIOTAP_FLAGS) |				\
	 (1 << IEEE80211_RADIOTAP_RATE) |				\
	 (1 << IEEE80211_RADIOTAP_XCHANNEL))

struct rt2860_softc_rx_data
{
	bus_dmamap_t dma_map;
	struct mbuf *m;
};

struct rt2860_softc_rx_ring
{
	bus_dma_tag_t desc_dma_tag;
	bus_dmamap_t desc_dma_map;
	bus_addr_t desc_phys_addr;
	struct rt2860_rxdesc *desc;
	bus_dma_tag_t data_dma_tag;
	bus_dmamap_t spare_dma_map;
	struct rt2860_softc_rx_data data[RT2860_SOFTC_RX_RING_DATA_COUNT];
	int cur;
};

struct rt2860_softc_tx_data
{
	bus_dmamap_t dma_map;
	struct ieee80211_node *ni;
	struct mbuf *m;
};

struct rt2860_softc_tx_ring
{
	struct mtx lock;
	bus_dma_tag_t desc_dma_tag;
	bus_dmamap_t desc_dma_map;
	bus_addr_t desc_phys_addr;
	struct rt2860_txdesc *desc;
	int desc_queued;
	int desc_cur;
	int desc_next;
	bus_dma_tag_t seg0_dma_tag;
	bus_dmamap_t seg0_dma_map;
	bus_addr_t seg0_phys_addr;
	uint8_t *seg0;
	bus_dma_tag_t data_dma_tag;
	struct rt2860_softc_tx_data data[RT2860_SOFTC_TX_RING_DATA_COUNT];
	int data_queued;
	int data_cur;
	int data_next;
	int qid;
};

struct rt2860_softc_node
{
	struct ieee80211_node ni;

	uint8_t staid;

	uint8_t last_rssi[RT2860_SOFTC_RSSI_COUNT];
	int8_t last_rssi_dbm[RT2860_SOFTC_RSSI_COUNT];
};

struct rt2860_softc_vap
{
	struct ieee80211vap	vap;

	struct ieee80211_beacon_offsets beacon_offsets;
	struct mbuf *beacon_mbuf;
	struct rt2860_txwi beacon_txwi;

	struct rt2860_amrr amrr;

	int	(*newstate)(struct ieee80211vap *vap,
		enum ieee80211_state nstate, int arg);
};

struct rt2860_softc_rx_radiotap_header
{
	struct ieee80211_radiotap_header ihdr;
	uint8_t	flags;
	uint8_t	rate;
	int8_t dbm_antsignal;
	int8_t dbm_antnoise;
	uint8_t	antenna;
	uint8_t	antsignal;
	uint8_t pad[2];
	uint32_t chan_flags;
	uint16_t chan_freq;
	uint8_t chan_ieee;
	int8_t chan_maxpow;
} __packed;

struct rt2860_softc_tx_radiotap_header
{
	struct ieee80211_radiotap_header ihdr;
	uint8_t flags;
	uint8_t	rate;
	uint8_t pad[2];
	uint32_t chan_flags;
	uint16_t chan_freq;
	uint8_t chan_ieee;
	int8_t chan_maxpow;
} __packed;

struct rt2860_softc
{
	struct ieee80211com sc_ic;

	struct mtx lock;

	struct mbufq sc_snd;

	uint32_t flags;
	uint32_t sc_flags;
#define RT2860_ENABLED          (1 << 0)
#define RT2860_ADVANCED_PS      (1 << 1)
#define RT2860_PCIE             (1 << 2)
#define RT2860_RUNNING          (1 << 3)

	device_t dev;

	int pid;

	int	mem_rid;
	struct resource	*mem;

	int	irq_rid;
	struct resource *irq;
	void *irqh;

	bus_space_tag_t bst;
	bus_space_handle_t bsh;

	int nvaps;
	int napvaps;
	int nadhocvaps;
	int nstavaps;
	int nwdsvaps;

	void (*node_cleanup)(struct ieee80211_node *ni);

	int (*recv_action)(struct ieee80211_node *ni,
		const struct ieee80211_frame *wh,
		const uint8_t *frm, const uint8_t *efrm);

	int (*send_action)(struct ieee80211_node *ni,
		int cat, int act, void *sa);

	int (*addba_response)(struct ieee80211_node *ni,
		struct ieee80211_tx_ampdu *tap,
		int status, int baparamset, int batimeout);

	void (*addba_stop)(struct ieee80211_node *ni,
		struct ieee80211_tx_ampdu *tap);

	int (*ampdu_rx_start)(struct ieee80211_node *ni,
		struct ieee80211_rx_ampdu *rap,
		int baparamset, int batimeout, int baseqctl);

	void (*ampdu_rx_stop)(struct ieee80211_node *ni,
		struct ieee80211_rx_ampdu *rap);

	struct rt2860_amrr_node amrr_node[RT2860_SOFTC_STAID_COUNT];

	uint32_t mac_rev;
	uint8_t eeprom_addr_num;
	uint16_t eeprom_rev;
	uint8_t rf_rev;

	uint8_t mac_addr[IEEE80211_ADDR_LEN];

	uint8_t ntxpath;
	uint8_t nrxpath;

	int hw_radio_cntl;
	int tx_agc_cntl;
	int ext_lna_2ghz;
	int ext_lna_5ghz;

	uint8_t country_2ghz;
	uint8_t country_5ghz;

	uint8_t rf_freq_off;

	uint8_t led_cntl;
	uint16_t led_off[RT2860_SOFTC_LED_OFF_COUNT];

	int8_t rssi_off_2ghz[RT2860_SOFTC_RSSI_OFF_COUNT];
	int8_t rssi_off_5ghz[RT2860_SOFTC_RSSI_OFF_COUNT];

	int8_t lna_gain[RT2860_SOFTC_LNA_GAIN_COUNT];

	int8_t txpow1[RT2860_SOFTC_TXPOW_COUNT];
	int8_t txpow2[RT2860_SOFTC_TXPOW_COUNT];

	int8_t txpow_rate_delta_2ghz;
	int8_t txpow_rate_delta_5ghz;
	uint32_t txpow_rate_20mhz[RT2860_SOFTC_TXPOW_RATE_COUNT];
	uint32_t txpow_rate_40mhz_2ghz[RT2860_SOFTC_TXPOW_RATE_COUNT];
	uint32_t txpow_rate_40mhz_5ghz[RT2860_SOFTC_TXPOW_RATE_COUNT];

	int tx_agc_cntl_2ghz;
	int tx_agc_cntl_5ghz;

	uint8_t tssi_2ghz[RT2860_SOFTC_TSSI_COUNT];
	uint8_t tssi_step_2ghz;
	uint8_t tssi_5ghz[RT2860_SOFTC_TSSI_COUNT];
	uint8_t tssi_step_5ghz;

	struct
	{
		uint8_t	val;
		uint8_t	reg;
	} __packed bbp_eeprom[RT2860_SOFTC_BBP_EEPROM_COUNT], rf[10];

	uint16_t powersave_level;

	uint8_t staid_mask[RT2860_SOFTC_STAID_COUNT / NBBY];

	uint32_t intr_enable_mask;
	uint32_t intr_disable_mask;
	uint32_t intr_pending_mask;

	struct task rx_done_task;
	int rx_process_limit;

	struct task tx_done_task;

	struct task fifo_sta_full_task;

	struct task periodic_task;
	struct callout periodic_ch;
	unsigned long periodic_round;

	struct taskqueue *taskqueue;

	struct rt2860_softc_rx_ring rx_ring;

	struct rt2860_softc_tx_ring tx_ring[RT2860_SOFTC_TX_RING_COUNT];
	int tx_ring_mgtqid;

	struct callout tx_watchdog_ch;
	int tx_timer;

	struct rt2860_softc_rx_radiotap_header rxtap;
	struct rt2860_softc_tx_radiotap_header txtap;

	/* statistic counters */

	int interrupts;
	int tx_coherent_interrupts;
	int rx_coherent_interrupts;
	int txrx_coherent_interrupts;
	int fifo_sta_full_interrupts;
	int rx_interrupts;
	int rx_delay_interrupts;
	int tx_interrupts[RT2860_SOFTC_TX_RING_COUNT];
	int tx_delay_interrupts;
	int pre_tbtt_interrupts;
	int tbtt_interrupts;
	int mcu_cmd_interrupts;
	int auto_wakeup_interrupts;
	int gp_timer_interrupts;

	int tx_data_queue_full[RT2860_SOFTC_TX_RING_COUNT];

	int tx_watchdog_timeouts;

	int tx_defrag_packets;

	int no_tx_desc_avail;

	int rx_mbuf_alloc_errors;
	int rx_mbuf_dmamap_errors;

	int tx_queue_not_empty[2];

	int tx_beacons;
	int tx_noretryok;
	int tx_retryok;
	int tx_failed;
	int tx_underflows;
	int tx_zerolen;
	int tx_nonagg;
	int tx_agg;
	int tx_ampdu;
	int tx_mpdu_zero_density;
	int tx_ampdu_sessions;

	int rx_packets;
	int rx_ampdu;
	int rx_ampdu_retries;
	int rx_mpdu_zero_density;
	int rx_ampdu_sessions;
	int rx_amsdu;
	int rx_crc_errors;
	int rx_phy_errors;
	int rx_false_ccas;
	int rx_plcp_errors;
	int rx_dup_packets;
	int rx_fifo_overflows;
	int rx_cipher_no_errors;
	int rx_cipher_icv_errors;
	int rx_cipher_mic_errors;
	int rx_cipher_invalid_key_errors;

	int tx_stbc;

	uint8_t rf24_20mhz;
	uint8_t	rf24_40mhz;
	uint8_t	patch_dac;
	uint8_t	txmixgain_2ghz;
	uint8_t	txmixgain_5ghz;

#ifdef RT2860_DEBUG
	int debug;
#endif
};

#endif /* #ifndef _RT2860_SOFTC_H_ */
