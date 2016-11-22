/*
 * Common define of private data for XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef XRADIO_H
#define XRADIO_H

#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include <net/mac80211.h>

//Macroses for Driver parameters.
#define XRWL_MAX_QUEUE_SZ    (128)
#define AC_QUEUE_NUM           4

#ifdef P2P_MULTIVIF
#define XRWL_MAX_VIFS        (3)
#else
#define XRWL_MAX_VIFS        (2)
#endif
#define XRWL_GENERIC_IF_ID   (2)
#define XRWL_HOST_VIF0_11N_THROTTLE   (58)  //(XRWL_MAX_QUEUE_SZ/(XRWL_MAX_VIFS-1))*0.9
#define XRWL_HOST_VIF1_11N_THROTTLE   (58)  //(XRWL_MAX_QUEUE_SZ/(XRWL_MAX_VIFS-1))*0.9
#define XRWL_HOST_VIF0_11BG_THROTTLE  (35)  //XRWL_HOST_VIF0_11N_THROTTLE*0.6 = 35
#define XRWL_HOST_VIF1_11BG_THROTTLE  (35)  //XRWL_HOST_VIF0_11N_THROTTLE*0.6 = 35
#if 0
#define XRWL_FW_VIF0_THROTTLE         (15)
#define XRWL_FW_VIF1_THROTTLE         (15)
#endif

#define IEEE80211_FCTL_WEP      0x4000
#define IEEE80211_QOS_DATAGRP   0x0080
#define WSM_KEY_MAX_IDX         20

#include "common.h"
#include "queue.h"
#include "wsm.h"
#include "scan.h"
#include "txrx.h"
#include "ht.h"
#include "pm.h"
#include "fwio.h"
#ifdef CONFIG_XRADIO_TESTMODE
#include "nl80211_testmode_msg_copy.h"
#endif /*CONFIG_XRADIO_TESTMODE*/

/* #define ROC_DEBUG */
/* hidden ssid is only supported when separate probe resp IE
   configuration is supported */
#ifdef PROBE_RESP_EXTRA_IE
#define HIDDEN_SSID   1
#endif

#define XRADIO_MAX_CTRL_FRAME_LEN  (0x1000)


#define WLAN_LINK_ID_MAX           (MAX_STA_IN_AP_MODE + 3)

#define XRADIO_MAX_STA_IN_AP_MODE   (5)
#define XRADIO_MAX_REQUEUE_ATTEMPTS (5)
#define XRADIO_LINK_ID_UNMAPPED     (15)

#define XRADIO_TX_BLOCK_ACK_ENABLED_FOR_ALL_TID    (0x3F)
#define XRADIO_RX_BLOCK_ACK_ENABLED_FOR_ALL_TID    (0x3F)
#define XRADIO_RX_BLOCK_ACK_ENABLED_FOR_BE_TID \
       (XRADIO_TX_BLOCK_ACK_ENABLED_FOR_ALL_TID & 0x01)
#define XRADIO_TX_BLOCK_ACK_DISABLED_FOR_ALL_TID   (0)
#define XRADIO_RX_BLOCK_ACK_DISABLED_FOR_ALL_TID   (0)

#define XRADIO_BLOCK_ACK_CNT    (30)
#define XRADIO_BLOCK_ACK_THLD   (800)
#define XRADIO_BLOCK_ACK_HIST   (3)
#define XRADIO_BLOCK_ACK_INTERVAL	(1 * HZ / XRADIO_BLOCK_ACK_HIST)
#define XRWL_ALL_IFS           (-1)

#ifdef ROAM_OFFLOAD
#define XRADIO_SCAN_TYPE_ACTIVE 0x1000
#define XRADIO_SCAN_BAND_5G     0x2000
#endif /*ROAM_OFFLOAD*/

#define IEEE80211_FCTL_WEP      0x4000
#define IEEE80211_QOS_DATAGRP   0x0080
#ifdef CONFIG_XRADIO_TESTMODE
#define XRADIO_SCAN_MEASUREMENT_PASSIVE (0)
#define XRADIO_SCAN_MEASUREMENT_ACTIVE  (1)
#endif



#define MAX_RATES_STAGE   8   //
#define MAX_RATES_RETRY   15

#define XRADIO_WORKQUEUE   "xradio_wq"
#define WIFI_CONF_PATH    "/data/xr_wifi.conf"

//
extern char *drv_version;
#define DRV_VERSION    drv_version

/* extern */ struct sbus_ops;
/* extern */ struct task_struct;
/* extern */ struct xradio_debug_priv;
/* extern */ struct xradio_debug_common;
/* extern */ struct firmware;

/* Please keep order */




enum xradio_bss_loss_status {
	XRADIO_BSS_LOSS_NONE,
	XRADIO_BSS_LOSS_CHECKING,
	XRADIO_BSS_LOSS_CONFIRMING,
	XRADIO_BSS_LOSS_CONFIRMED,
};



#if defined(ROAM_OFFLOAD) || defined(CONFIG_XRADIO_TESTMODE)
struct xradio_testframe {
	u8 len;
	u8 *data;
};
#endif
#ifdef CONFIG_XRADIO_TESTMODE
struct advance_scan_elems {
	u8 scanMode;
	u16 duration;
};
/**
 * xradio_tsm_info - Keeps information about ongoing TSM collection
 * @ac: Access category for which metrics to be collected
 * @use_rx_roaming: Use received voice packets to compute roam delay
 * @sta_associated: Set to 1 after association
 * @sta_roamed: Set to 1 after successful roaming
 * @roam_delay: Roam delay
 * @rx_timestamp_vo: Timestamp of received voice packet
 * @txconf_timestamp_vo: Timestamp of received tx confirmation for
 * successfully transmitted VO packet
 * @sum_pkt_q_delay: Sum of packet queue delay
 * @sum_media_delay: Sum of media delay
 *
 */
struct xradio_tsm_info {
	u8 ac;
	u8 use_rx_roaming;
	u8 sta_associated;
	u8 sta_roamed;
	u16 roam_delay;
	u32 rx_timestamp_vo;
	u32 txconf_timestamp_vo;
	u32 sum_pkt_q_delay;
	u32 sum_media_delay;
};

/**
 * xradio_start_stop_tsm - To start or stop collecting TSM metrics in
 * xradio driver
 * @start: To start or stop collecting TSM metrics
 * @up: up for which metrics to be collected
 * @packetization_delay: Packetization delay for this TID
 *
 */
struct xradio_start_stop_tsm {
	u8 start;       /*1: To start, 0: To stop*/
	u8 up;
	u16 packetization_delay;
};

#endif /* CONFIG_XRADIO_TESTMODE */
struct xradio_common {
	struct xradio_debug_common	*debug;
	struct xradio_queue		tx_queue[AC_QUEUE_NUM];
	struct xradio_queue_stats	tx_queue_stats;

	struct ieee80211_hw		*hw;
	struct mac_address		addresses[XRWL_MAX_VIFS];

	/*Will be a pointer to a list of VIFs - Dynamically allocated */
	struct ieee80211_vif		*vif_list[XRWL_MAX_VIFS];
	atomic_t			num_vifs;
	spinlock_t			vif_list_lock;
	u32				if_id_slot;
	struct device			*pdev;
	struct workqueue_struct		*workqueue;

	struct mutex			conf_mutex;

	const struct sbus_ops		*sbus_ops;
	struct sbus_priv		*sbus_priv;
	int 			driver_ready;

	/* firmware/hardware info */
	unsigned int tx_hdr_len;

	/* Radio data */
	int output_power;
	int noise;

	/* calibration, output power limit and rssi<->dBm conversation data */

	/* BBP/MAC state */

	struct ieee80211_rate		*rates;
	struct ieee80211_rate		*mcs_rates;
	u8 mac_addr[ETH_ALEN];
	/*TODO:COMBO: To be made per VIFF after mac80211 support */
	struct ieee80211_channel	*channel;
	int				channel_switch_in_progress;
	wait_queue_head_t		channel_switch_done;
	u8        channel_changed;   //add by yangfh 2015-5-15 16:57:38.
	u8				long_frame_max_tx_count;
	u8				short_frame_max_tx_count;
	/* TODO:COMBO: According to Hong aggregation will happen per VIFF.
	* Keeping in common structure for the time being. Will be moved to VIFF
	* after the mechanism is clear */
	u8				ba_tid_mask;
	int				ba_acc; /*TODO: Same as above */
	int				ba_cnt; /*TODO: Same as above */
	int				ba_cnt_rx; /*TODO: Same as above */
	int				ba_acc_rx; /*TODO: Same as above */
	int				ba_hist; /*TODO: Same as above */
	struct timer_list		ba_timer;/*TODO: Same as above */
	spinlock_t			ba_lock; /*TODO: Same as above */
	bool				ba_ena; /*TODO: Same as above */
	struct work_struct              ba_work; /*TODO: Same as above */
	struct xradio_pm_state		pm_state;

	bool				is_go_thru_go_neg;








	long			connet_time[XRWL_MAX_VIFS];
#ifdef CONFIG_XRADIO_SUSPEND_POWER_OFF
	atomic_t            suspend_state;
#endif
#ifdef HW_RESTART
	bool                hw_restart;
	struct work_struct  hw_restart_work;
#endif

	/* WSM */


	struct wsm_cbc			wsm_cbc;
	struct semaphore		tx_lock_sem;
	atomic_t				tx_lock;
	u32				pending_frame_id;
#ifdef CONFIG_XRADIO_TESTMODE
	/* Device Power Range */
	struct wsm_tx_power_range       txPowerRange[2];
	/* Advance Scan */
	struct advance_scan_elems	advanceScanElems;
	bool				enable_advance_scan;
	struct delayed_work		advance_scan_timeout;
#endif /* CONFIG_XRADIO_TESTMODE */

	/* WSM debug */

	/* Scan status */
	struct xradio_scan scan;

	/* TX/RX */
	unsigned long		rx_timestamp;

	/* WSM events */
	spinlock_t		event_queue_lock;
	struct list_head	event_queue;
	struct work_struct	event_handler;

	/* TX rate policy cache */
	struct tx_policy_cache tx_policy_cache;
	struct work_struct tx_policy_upload_work;
	atomic_t upload_count;

	/* cryptographic engine information */

	/* bit field of glowing LEDs */
	u16 softled_state;

	/* statistics */
	struct ieee80211_low_level_stats stats;

	struct xradio_ht_oper		ht_oper;
	int				tx_burst_idx;

	struct ieee80211_iface_limit		if_limits1[2];
	struct ieee80211_iface_limit		if_limits2[2];
	struct ieee80211_iface_limit		if_limits3[2];
	struct ieee80211_iface_combination	if_combs[3];


	struct delayed_work		rem_chan_timeout;
	atomic_t			remain_on_channel;
	int				roc_if_id;
	u64				roc_cookie;
	wait_queue_head_t		offchannel_wq;
	u16				offchannel_done;
	u16				prev_channel;
	int       if_id_selected;
	u32				key_map;
	struct wsm_add_key		keys[WSM_KEY_MAX_INDEX + 1];

#ifdef ROAM_OFFLOAD
	u8				auto_scanning;
	u8				frame_rcvd;
	u8				num_scanchannels;
	u8				num_2g_channels;
	u8				num_5g_channels;
	struct wsm_scan_ch		scan_channels[48];
	struct sk_buff 			*beacon;
	struct sk_buff 			*beacon_bkp;
	struct xradio_testframe 	testframe;
#endif /*ROAM_OFFLOAD*/
#ifdef CONFIG_XRADIO_TESTMODE
	struct xradio_testframe test_frame;
	struct xr_tsm_stats		tsm_stats;
	struct xradio_tsm_info		tsm_info;
	spinlock_t			tsm_lock;
	struct xradio_start_stop_tsm	start_stop_tsm;
#endif /* CONFIG_XRADIO_TESTMODE */
	u8          connected_sta_cnt;
	u16			vif0_throttle;
	u16			vif1_throttle;
};



struct xradio_sta_priv {
	int link_id;
	struct xradio_vif *priv;
};
enum xradio_data_filterid {
	IPV4ADDR_FILTER_ID = 0,
#ifdef IPV6_FILTERING
	IPV6ADDR_FILTER_ID,
#endif /*IPV6_FILTERING*/
};

#ifdef IPV6_FILTERING
/* IPV6 host addr info */
struct ipv6_addr_info {
	u8 filter_mode;
	u8 address_mode;
	u16 ipv6[8];
};
#endif /*IPV6_FILTERING*/

/* Datastructure for LLC-SNAP HDR */
#define P80211_OUI_LEN  3
struct ieee80211_snap_hdr {
        u8    dsap;   /* always 0xAA */
        u8    ssap;   /* always 0xAA */
        u8    ctrl;   /* always 0x03 */
        u8    oui[P80211_OUI_LEN];    /* organizational universal id */
} __packed;


#ifdef TES_P2P_0002_ROC_RESTART
extern s32  TES_P2P_0002_roc_dur;
extern s32  TES_P2P_0002_roc_sec;
extern s32  TES_P2P_0002_roc_usec;
extern u32  TES_P2P_0002_packet_id;
extern u32  TES_P2P_0002_state;

#define TES_P2P_0002_STATE_IDLE       0x00
#define TES_P2P_0002_STATE_SEND_RESP  0x01
#define TES_P2P_0002_STATE_GET_PKTID  0x02
#endif

/* debug.h must be here because refer to struct xradio_vif and 
   struct xradio_common.*/
#include "debug.h"

/*******************************************************
 interfaces for operations of vif.
********************************************************/
static inline
struct xr819 *xrwl_vifpriv_to_hwpriv(struct xradio_vif *priv)
{
	return priv->hw_priv;
}
static inline
struct xradio_vif *xrwl_get_vif_from_ieee80211(struct ieee80211_vif *vif)
{
	return  (struct xradio_vif *)vif->drv_priv;
}

static inline
struct xradio_vif *xrwl_hwpriv_to_vifpriv(struct xradio_common *hw_priv,
						int if_id)
{
	struct xradio_vif *vif;

	if (SYS_WARN((-1 == if_id) || (if_id > XRWL_MAX_VIFS)))
		return NULL;
	/* TODO:COMBO: During scanning frames can be received
	 * on interface ID 3 */
	spin_lock(&hw_priv->vif_list_lock);
	if (!hw_priv->vif_list[if_id]) {
		spin_unlock(&hw_priv->vif_list_lock);
		return NULL;
	}

	vif = xrwl_get_vif_from_ieee80211(hw_priv->vif_list[if_id]);
	SYS_WARN(!vif);
	if (vif)
		spin_lock(&vif->vif_lock);
	spin_unlock(&hw_priv->vif_list_lock);
	return vif;
}

static inline
struct xradio_vif *__xrwl_hwpriv_to_vifpriv(struct xradio_common *hw_priv,
					      int if_id)
{
	SYS_WARN((-1 == if_id) || (if_id > XRWL_MAX_VIFS));
	/* TODO:COMBO: During scanning frames can be received
	 * on interface ID 3 */
	if (!hw_priv->vif_list[if_id]) {
		return NULL;
	}

	return xrwl_get_vif_from_ieee80211(hw_priv->vif_list[if_id]);
}

static inline
struct xradio_vif *xrwl_get_activevif(struct xradio_common *hw_priv)
{
	return xrwl_hwpriv_to_vifpriv(hw_priv, ffs(hw_priv->if_id_slot)-1);
}

/*static inline bool is_hardware_xradio(struct xradio_common *hw_priv)
{
	return (hw_priv->hw_revision == XR819_HW_REV0);
}*/

/*static inline int xrwl_get_nr_hw_ifaces(struct xradio_common *hw_priv)
{
	switch(hw_priv->hw_revision) {
		case XR819_HW_REV0:
		default:
			return 1;
	}
}*/

#define xradio_for_each_vif(_hw_priv, _priv, _i)			\
	for(		\
		_i = 0; 							 \
		(_i < XRWL_MAX_VIFS)				  \
		&& ((_priv = _hw_priv->vif_list[_i] ?			 \
		xrwl_get_vif_from_ieee80211(_hw_priv->vif_list[_i]) : NULL),1);	 \
		_i++		 \
	)

/*******************************************************
 interfaces for operations of queue.
********************************************************/
static inline void xradio_tx_queues_lock(struct xradio_common *hw_priv)
{
	int i;
	for (i = 0; i < 4; ++i)
		xradio_queue_lock(&hw_priv->tx_queue[i]);
}

static inline void xradio_tx_queues_unlock(struct xradio_common *hw_priv)
{
	int i;
	for (i = 0; i < 4; ++i)
		xradio_queue_unlock(&hw_priv->tx_queue[i]);
}

#endif /* XRADIO_H */
