#ifndef __XR819_VIF_H
#define __XR819_VIF_H

#include <uapi/linux/if_ether.h>
#include <uapi/linux/in6.h>
#include <linux/skbuff.h>
#include <linux/ieee80211.h>

#define WSM_MAX_GRP_ADDRTABLE_ENTRIES		8
#define WSM_MAX_ARP_IP_ADDRTABLE_ENTRIES	1
#define WSM_MAX_NDP_IP_ADDRTABLE_ENTRIES	1

#define WSM_FW_LABEL 128
struct wsm_caps {
	u16 numInpChBufs;
	u16 sizeInpChBuf;
	u16 hardwareId;
	u16 hardwareSubId;
	u16 firmwareCap;
	u16 firmwareType;
	u16 firmwareApiVer;
	u16 firmwareBuildNumber;
	u16 firmwareVersion;
	char fw_label[WSM_FW_LABEL+2];
	int firmwareReady;
};

struct xradio_pm_state_vif {
	struct xradio_suspend_state *suspend_state;
};

/* Broadcast Addr Filter */
struct wsm_broadcast_addr_filter {
	u8 action_mode;
	u8 nummacaddr;
	u8 filter_mode;
	u8 address_mode;
	u8 MacAddr[6];
} __packed;

struct wsm_beacon_filter_table_entry {
	u8	ieId;
	u8	actionFlags;
	u8	oui[3];
	u8	matchData[3];
} __packed;

struct wsm_beacon_filter_table {
	__le32 numOfIEs;
	struct wsm_beacon_filter_table_entry entry[10];
} __packed;

struct wsm_beacon_filter_control {
	int enabled;
	int bcn_count;
};

struct wsm_association_mode {
	u8 flags;		/* WSM_ASSOCIATION_MODE_... */
	u8 preambleType;	/* WSM_JOIN_PREAMBLE_... */
	u8 greenfieldMode;	/* 1 for greenfield */
	u8 mpduStartSpacing;
	__le32 basicRateSet;
};

struct wsm_rx_filter {
	bool promiscuous;
	bool bssid;
	bool fcs;
	bool probeResponder;
	bool keepalive;
};

/* 3.25 */
struct wsm_set_pm {
	/* WSM_PSM_... */
	/* [in] */ u8 pmMode;

	/* in unit of 500us; 0 to use default */
	/* [in] */ u8 fastPsmIdlePeriod;

	/* in unit of 500us; 0 to use default */
	/* [in] */ u8 apPsmChangePeriod;

	/* in unit of 500us; 0 to disable auto-pspoll */
	/* [in] */ u8 minAutoPsPollPeriod;
};

/* 3.28 */
struct wsm_set_bss_params {
	/* The number of lost consecutive beacons after which */
	/* the WLAN device should indicate the BSS-Lost event */
	/* to the WLAN host driver. */
	u8 beaconLostCount;

	/* The AID received during the association process. */
	u16 aid;

	/* The operational rate set mask */
	u32 operationalRateSet;
};

/* 3.34 */
struct wsm_set_tx_queue_params {
	/* WSM_ACK_POLICY_... */
	u8 ackPolicy;

	/* Medium Time of TSPEC (in 32us units) allowed per */
	/* One Second Averaging Period for this queue. */
	u16 allowedMediumTime;

	/* dot11MaxTransmitMsduLifetime to be used for the */
	/* specified queue. */
	u32 maxTransmitLifetime;
};

struct wsm_tx_queue_params {
	/* NOTE: index is a linux queue id. */
	struct wsm_set_tx_queue_params params[4];
};

/* 3.36 */
struct wsm_edca_queue_params {
	/* CWmin (in slots) for the access class. */
	/* [in] */ u16 cwMin;

	/* CWmax (in slots) for the access class. */
	/* [in] */ u16 cwMax;

	/* AIFS (in slots) for the access class. */
	/* [in] */ u8 aifns;

	/* TX OP Limit (in microseconds) for the access class. */
	/* [in] */ u16 txOpLimit;

	/* dot11MaxReceiveLifetime to be used for the specified */
	/* the access class. Overrides the global */
	/* dot11MaxReceiveLifetime value */
	/* [in] */ u32 maxReceiveLifetime;

	/* UAPSD trigger support for the access class. */
	/* [in] */ bool uapsdEnable;
};

/* Multicat filtering - 4.5 */
struct wsm_multicast_filter {
	__le32 enable;
	__le32 numOfAddresses;
	u8 macAddress[WSM_MAX_GRP_ADDRTABLE_ENTRIES][ETH_ALEN];
} __packed;

/* ARP IPv4 filtering - 4.10 */
struct wsm_arp_ipv4_filter {
	__le32 enable;
	__be32 ipv4Address[WSM_MAX_ARP_IP_ADDRTABLE_ENTRIES];
} __packed;

#ifdef IPV6_FILTERING
/* NDP IPv6 filtering */
struct wsm_ndp_ipv6_filter {
	__le32 enable;
	struct in6_addr ipv6Address[WSM_MAX_NDP_IP_ADDRTABLE_ENTRIES];
} __packed;
/* IPV6 Addr Filter Info */
struct wsm_ip6_addr_info {
	u8 filter_mode;
	u8 address_mode;
	u8 Reserved[2];
	u8 ipv6[16];
};

/* IPV6 Addr Filter */
struct wsm_ipv6_filter {
	u8 numfilter;
	u8 action_mode;
	u8 Reserved[2];
	struct wsm_ip6_addr_info ipv6filter[0];
} __packed;
#endif /*IPV6_FILTERING*/

/* 4.26 SetUpasdInformation */
struct wsm_uapsd_info {
	__le16 uapsdFlags;
	__le16 minAutoTriggerInterval;
	__le16 maxAutoTriggerInterval;
	__le16 autoTriggerStep;
};

/* P2P Power Save Mode Info - 4.31 */
struct wsm_p2p_ps_modeinfo {
	u8	oppPsCTWindow;
	u8	count;
	u8	reserved;
	u8	dtimCount;
	__le32	duration;
	__le32	interval;
	__le32	startTime;
} __packed;

struct wsm_edca_params {
	/* NOTE: index is a linux queue id. */
	struct wsm_edca_queue_params params[4];
};

/* Virtual Interface State. One copy per VIF */
#define MAX_STA_IN_AP_MODE         (14)
#define XRADIO_MAX_TID              (8)

enum xradio_join_status {
	XRADIO_JOIN_STATUS_PASSIVE = 0,
	XRADIO_JOIN_STATUS_MONITOR,
	XRADIO_JOIN_STATUS_STA,
	XRADIO_JOIN_STATUS_AP,
};

enum xradio_link_status {
	XRADIO_LINK_OFF,
	XRADIO_LINK_RESERVE,
	XRADIO_LINK_SOFT,
	XRADIO_LINK_HARD,
#if defined(CONFIG_XRADIO_USE_EXTENSIONS)
	XRADIO_LINK_RESET,
	XRADIO_LINK_RESET_REMAP,
#endif
};

struct xradio_link_entry {
	unsigned long timestamp;
	enum xradio_link_status status;
#if defined(CONFIG_XRADIO_USE_EXTENSIONS)
	enum xradio_link_status prev_status;
#endif
	u8 mac[ETH_ALEN];
	u8 buffered[XRADIO_MAX_TID];
	struct sk_buff_head rx_queue;
};

struct xradio_vif {
	atomic_t enabled;
	spinlock_t vif_lock;
	int if_id;
	/*TODO: Split into Common and VIF parts */
	struct xradio_debug_priv *debug;
	/* BBP/MAC state */
	u8 bssid[ETH_ALEN];
	struct wsm_edca_params edca;
	struct wsm_tx_queue_params tx_queue_params;
	struct wsm_association_mode association_mode;
	struct wsm_set_bss_params bss_params;
	struct wsm_set_pm powersave_mode;
	struct wsm_set_pm firmware_ps_mode;
	int power_set_true;
	int user_power_set_true;
	u8 user_pm_mode;
	int cqm_rssi_thold;
	unsigned cqm_rssi_hyst;
	unsigned cqm_tx_failure_thold;
	unsigned cqm_tx_failure_count;
	bool cqm_use_rssi;
	int cqm_link_loss_count;
	int cqm_beacon_loss_count;
	int mode;
	bool enable_beacon;
	int beacon_int;
	size_t ssid_length;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
#ifdef HIDDEN_SSID
	bool hidden_ssid;
#endif
	bool listening;
	struct wsm_rx_filter rx_filter;
	struct wsm_beacon_filter_table bf_table;
	struct wsm_beacon_filter_control bf_control;
	struct wsm_multicast_filter multicast_filter;
	bool has_multicast_subscription;
	struct wsm_broadcast_addr_filter broadcast_filter;
	bool disable_beacon_filter;
	struct wsm_arp_ipv4_filter filter4;
#ifdef IPV6_FILTERING
	struct wsm_ndp_ipv6_filter filter6;
#endif /*IPV6_FILTERING*/
	struct work_struct update_filtering_work;
	struct work_struct set_beacon_wakeup_period_work;
	struct xradio_pm_state_vif pm_state_vif;
	/*TODO: Add support in mac80211 for psmode info per VIF */
	struct wsm_p2p_ps_modeinfo p2p_ps_modeinfo;
	struct wsm_uapsd_info uapsd_info;
	bool setbssparams_done;
	u32 listen_interval;
	u32 erp_info;
	bool powersave_enabled;

	/* WSM Join */
	enum xradio_join_status join_status;
	u8 join_bssid[ETH_ALEN];
	struct work_struct join_work;
	struct delayed_work join_timeout;
	struct work_struct unjoin_work;
	struct work_struct offchannel_work;
	int join_dtim_period;
	atomic_t delayed_unjoin;

	/* Security */
	s8 wep_default_key_id;
	struct work_struct wep_key_work;
	unsigned long rx_timestamp;
	u32 cipherType;

	/* AP powersave */
	u32 link_id_map;
	u32 max_sta_ap_mode;
	u32 link_id_after_dtim;
	u32 link_id_uapsd;
	u32 link_id_max;
	u32 wsm_key_max_idx;
	struct xradio_link_entry link_id_db[MAX_STA_IN_AP_MODE];
	struct work_struct link_id_work;
	struct delayed_work link_id_gc_work;
	u32 sta_asleep_mask;
	u32 pspoll_mask;
	bool aid0_bit_set;
	spinlock_t ps_state_lock;
	bool buffered_multicasts;
	bool tx_multicast;
	u8 last_tim[8];   //for softap dtim, add by yangfh
	struct work_struct set_tim_work;
	struct delayed_work set_cts_work;
	struct work_struct multicast_start_work;
	struct work_struct multicast_stop_work;
	struct timer_list mcast_timeout;

	/* CQM Implementation */
	struct delayed_work bss_loss_work;
	struct delayed_work connection_loss_work;
	struct work_struct tx_failure_work;
	int delayed_link_loss;
	spinlock_t bss_loss_lock;
	int bss_loss_status;
	int bss_loss_confirm_id;

	struct ieee80211_vif *vif;
	struct xr819 *hw_priv;
	struct ieee80211_hw *hw;

	/* ROC implementation */
	struct delayed_work pending_offchanneltx_work;
#if defined(CONFIG_XRADIO_USE_EXTENSIONS)
	/* Workaround for WFD testcase 6.1.10*/
	struct work_struct linkid_reset_work;
	u8 action_frame_sa[ETH_ALEN];
	u8 action_linkid;
#endif
	bool htcap;
#ifdef  AP_HT_CAP_UPDATE
	u16 ht_oper;
	struct work_struct ht_oper_update_work;
#endif

#ifdef AP_HT_COMPAT_FIX
	u16 ht_compat_cnt;
	u16 ht_compat_det;
#endif
};

#endif
