#ifndef __XR819_H
#define __XR819_H

#include <linux/mmc/sdio_func.h>

#include "xr819_vif.h"

#ifdef P2P_MULTIVIF
#define XRWL_MAX_VIFS        (3)
#else
#define XRWL_MAX_VIFS        (2)
#endif

#ifdef MCAST_FWDING
#define WSM_MAX_BUF		30
#endif

/* The maximum number of SSIDs that the device can scan for. */
#define WSM_SCAN_MAX_NUM_OF_SSIDS	(2)

struct wsm_buf {
	u8 *begin;
	u8 *data;
	u8 *end;
};

struct xr819_sdio {
	struct sdio_func *func;
	int irq;
};

struct xr819_firmware {
#ifdef USE_VFS_FIRMWARE
	const struct xr_file *sdd;
#else
	const struct firmware *sdd;
#endif
	u8 conf_listen_interval;
	bool is_BT_Present;
};

struct xr819_hardware {
	int hw_type;
	int hw_revision;
	int fw_revision;
};

struct xr819_bh {
	atomic_t interrupt;
	atomic_t rx;
	atomic_t tx;
	atomic_t term;
	atomic_t suspend;
	struct task_struct *thread;
	int error;
#ifdef BH_USE_SEMAPHORE
	struct semaphore sem;
	atomic_t wk;
#else
	wait_queue_head_t wq;
#endif
	wait_queue_head_t evt_wq;
	int hw_bufs_used;
	int hw_bufs_used_vif[XRWL_MAX_VIFS];

	struct sk_buff *skb_cache;
	struct sk_buff *skb_reserved;
	int skb_resv_len;
	u32 query_packetID;
	atomic_t query_cnt;
	struct work_struct query_work; /* for query packet */

	bool powersave_enabled;
	bool device_can_sleep;
	/* Keep xradio awake (WUP = 1) 1 second after each scan to avoid
	 * FW issue with sleeping/waking up. */
	atomic_t recent_scan;
};

struct wsm_cmd {
	spinlock_t lock;
	int done;
	u8 *ptr;
	size_t len;
	void *arg;
	int ret;
	u16 cmd;
};

struct xr819_wsm {
	struct wsm_caps caps;
	int rx_seq; /* byte */
	int tx_seq; /* byte */
	int enable_wsm_dumps;
	u32 dump_max_size;
#ifdef MCAST_FWDING
	struct wsm_buf wsm_release_buf[WSM_MAX_BUF];
	u8 buf_released;
#endif
	wait_queue_head_t wsm_startup_done;

	struct mutex wsm_cmd_mux;
	struct wsm_buf wsm_cmd_buf;
	struct wsm_cmd wsm_cmd;
	wait_queue_head_t wsm_cmd_wq;
	struct mutex wsm_oper_lock;
};

/* 3.9 */
struct wsm_ssid {
	u8 ssid[32];
	u32 length;
};

struct wsm_scan_ch {
	u16 number;
	u32 minChannelTime;
	u32 maxChannelTime;
	u32 txPowerLevel;
};

/* 3.13 */
struct wsm_scan_complete {
	/* WSM_STATUS_... */
	u32 status;

	/* WSM_PSM_... */
	u8 psm;

	/* Number of channels that the scan operation completed. */
	u8 numChannels;
#ifdef ROAM_OFFLOAD
u16 reserved;
#endif /*ROAM_OFFLOAD*/
};

struct xradio_scan {
struct semaphore lock;
struct work_struct work;
#ifdef ROAM_OFFLOAD
struct work_struct swork; /* scheduled scan work */
struct cfg80211_sched_scan_request *sched_req;
#endif /*ROAM_OFFLOAD*/
struct delayed_work timeout;
struct cfg80211_scan_request *req;
struct ieee80211_channel **begin;
struct ieee80211_channel **curr;
struct ieee80211_channel **end;
struct wsm_ssid ssids[WSM_SCAN_MAX_NUM_OF_SSIDS];
int output_power;
int n_ssids;
//add by liwei, for h64 ping WS550 BUG
struct semaphore status_lock;
int status;
atomic_t in_progress;
/* Direct probe requests workaround */
struct delayed_work probe_work;
int direct_probe;
u8 if_id;
};

struct xr819_txrx {
unsigned long rx_timestamp;
};

struct xr819 {

struct device *dev;

struct xr819_sdio sdio;
struct xr819_hardware hardware;
struct xr819_firmware firmware;
struct xr819_bh bh;
struct xr819_wsm wsm;
struct xr819_txrx txrx;

int buf_id_tx; /* byte */
int buf_id_rx; /* byte */

/* Scan status */
struct xradio_scan scan;

struct ieee80211_vif *vif_list[XRWL_MAX_VIFS];
spinlock_t vif_list_lock;

struct ieee80211_hw* netif;

};

#define XR819_HW_REV0       (8190)

static inline int xrwl_get_nr_hw_ifaces(struct xr819* hw_priv) {
switch (hw_priv->hardware.hw_revision) {
case XR819_HW_REV0:
default:
	return 1;
}
}

static inline bool is_hardware_xradio(struct xr819 *hw_priv) {
return (hw_priv->hardware.hw_revision == XR819_HW_REV0);
}

#endif
