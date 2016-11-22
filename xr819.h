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

struct xr819 {
	struct xr819_sdio sdio;
	struct xr819_hardware hardware;
	struct xr819_firmware firmware;
	struct xr819_bh bh;
	struct xr819_wsm wsm;

	int buf_id_tx; /* byte */
	int buf_id_rx; /* byte */

	struct ieee80211_vif *vif_list[XRWL_MAX_VIFS];

};

#endif
