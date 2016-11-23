/*
 * Data Transmission thread implementation for XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <net/mac80211.h>
#include <linux/kthread.h>

#include "bh.h"
#include "hwio.h"
#include "wsm.h"
#include "sdio.h"

/* TODO: Verify these numbers with WSM specification. */
#define DOWNLOAD_BLOCK_SIZE_WR	(0x1000 - 4)
/* an SPI message cannot be bigger than (2"12-1)*2 bytes
 * "*2" to cvt to bytes */
#define MAX_SZ_RD_WR_BUFFERS	(DOWNLOAD_BLOCK_SIZE_WR*2)
#define PIGGYBACK_CTRL_REG	(2)
#define EFFECTIVE_BUF_SIZE	(MAX_SZ_RD_WR_BUFFERS - PIGGYBACK_CTRL_REG)

/* Suspend state privates */
enum xradio_bh_pm_state {
	XRADIO_BH_RESUMED = 0,
	XRADIO_BH_SUSPEND,
	XRADIO_BH_SUSPENDED,
	XRADIO_BH_RESUME,
};
typedef int (*xradio_wsm_handler)(struct xr819 *hw_priv, u8 *data, size_t size);

#ifdef MCAST_FWDING
int wsm_release_buffer_to_fw(struct xradio_vif *priv, int count);
#endif
static int xradio_bh(void *arg);

int xradio_register_bh(struct xr819 *priv) {
	int err = 0;
	struct sched_param param = { .sched_priority = 1 };

	atomic_set(&priv->bh.rx, 0);
	atomic_set(&priv->bh.tx, 0);
	atomic_set(&priv->bh.term, 0);
	atomic_set(&priv->bh.suspend, XRADIO_BH_RESUMED);
	priv->buf_id_tx = 0;
	priv->buf_id_rx = 0;
#ifdef BH_USE_SEMAPHORE
	sema_init(&priv->bh.sem, 0);
	atomic_set(&priv->bh.wk, 0);
#else
	init_waitqueue_head(&priv->bh.wq);
#endif
	init_waitqueue_head(&priv->bh.evt_wq);

	priv->bh.thread = kthread_create(&xradio_bh, priv, XRADIO_BH_THREAD);
	if (IS_ERR(priv->bh.thread)) {
		err = PTR_ERR(priv->bh.thread);
		priv->bh.thread = NULL;
	} else {
		sched_setscheduler(priv->bh.thread, SCHED_FIFO, &param);
#ifdef HAS_PUT_TASK_STRUCT
		get_task_struct(priv->bh.thread);
#endif
		wake_up_process(priv->bh.thread);
	}
	return err;
}

void xradio_unregister_bh(struct xr819 *priv) {
	struct task_struct *thread = priv->bh.thread;
	priv->bh.thread = NULL;
	kthread_stop(thread);
#ifdef HAS_PUT_TASK_STRUCT
	put_task_struct(thread);
#endif
}

void xradio_irq_handler(struct xr819* priv) {
	atomic_inc(&priv->bh.interrupt);
	wake_up(&priv->bh.wq);
}

void xradio_bh_wakeup(struct xr819* priv) {
#ifdef BH_USE_SEMAPHORE
	atomic_add(1, &priv->bh.tx);
	if (atomic_add_return(1, &priv->bh.wk) == 1) {
		up(&priv->bh.sem);
	}
#else
	if (atomic_add_return(1, &priv->bh.tx) == 1) {
		wake_up(&priv->bh.wq);
	}
#endif
}

int xradio_bh_suspend(struct xr819* hw_priv) {

#ifdef MCAST_FWDING
	int i =0;
	struct xradio_vif *priv = NULL;
#endif

	if (hw_priv->bh.error) {
		return -EINVAL;
	}

#ifdef MCAST_FWDING
	xradio_for_each_vif(hw_priv, priv, i) {
		if (!priv)
		continue;
		if ( (priv->multicast_filter.enable)
				&& (priv->join_status == XRADIO_JOIN_STATUS_AP) ) {
			wsm_release_buffer_to_fw(priv,
					(hw_priv->wsm.caps.numInpChBufs - 1));
			break;
		}
	}
#endif

	atomic_set(&hw_priv->bh.suspend, XRADIO_BH_SUSPEND);
#ifdef BH_USE_SEMAPHORE
	up(&hw_priv->bh.sem);
#else
	wake_up(&hw_priv->bh.wq);
#endif
	return wait_event_timeout(hw_priv->bh.evt_wq,
			(hw_priv->bh.error
					|| XRADIO_BH_SUSPENDED == atomic_read(&hw_priv->bh.suspend)),
			1 * HZ) ? 0 : -ETIMEDOUT;
}

int xradio_bh_resume(struct xr819 *hw_priv) {

#ifdef MCAST_FWDING
	int ret;
	int i =0;
	struct xradio_vif *priv =NULL;
#endif

	if (hw_priv->bh.error) {
		return -EINVAL;
	}

	atomic_set(&hw_priv->bh.suspend, XRADIO_BH_RESUME);
#ifdef BH_USE_SEMAPHORE
	up(&hw_priv->bh.sem);
#else
	wake_up(&hw_priv->bh.wq);
#endif

#ifdef MCAST_FWDING
	ret = wait_event_timeout(hw_priv->bh.evt_wq, (hw_priv->bh.error ||
					XRADIO_BH_RESUMED == atomic_read(&hw_priv->bh.suspend))
			,1 * HZ)? 0 : -ETIMEDOUT;

	xradio_for_each_vif(hw_priv, priv, i) {
		if (!priv)
		continue;
		if ((priv->join_status == XRADIO_JOIN_STATUS_AP) &&
				(priv->multicast_filter.enable)) {
			u8 count = 0;
			//wsm_request_buffer_request(priv, &count);
			break;
		}
	}

	return ret;
#else
	return wait_event_timeout(hw_priv->bh.evt_wq,
			hw_priv->bh.error
					|| (XRADIO_BH_RESUMED == atomic_read(&hw_priv->bh.suspend)),
			1 * HZ) ? 0 : -ETIMEDOUT;
#endif

}

static inline void wsm_alloc_tx_buffer(struct xr819 *hw_priv) {
	++hw_priv->bh.hw_bufs_used;
}

int wsm_release_tx_buffer(struct xr819 *hw_priv, int count) {
	int ret = 0;
	int hw_bufs_used = hw_priv->bh.hw_bufs_used;

	hw_priv->bh.hw_bufs_used -= count;
	if (hw_priv->bh.hw_bufs_used < 0) {
		/* Tx data patch stops when all but one hw buffers are used.
		 So, re-start tx path in case we find hw_bufs_used equals
		 numInputChBufs - 1.
		 */
		dev_err(&hw_priv->sdio.func->dev, "hw_bufs_used=%d, count=%d.\n",
				hw_priv->bh.hw_bufs_used, count);
		ret = -1;
	} else if (hw_bufs_used >= (hw_priv->wsm.caps.numInpChBufs - 1))
		ret = 1;
	if (!hw_priv->bh.hw_bufs_used)
		wake_up(&hw_priv->bh.evt_wq);
	return ret;
}

int wsm_release_vif_tx_buffer(struct xr819 *hw_priv, int if_id, int count) {
	int ret = 0;

	hw_priv->bh.hw_bufs_used_vif[if_id] -= count;
	if (!hw_priv->bh.hw_bufs_used_vif[if_id])
		wake_up(&hw_priv->bh.evt_wq);

	if (hw_priv->bh.hw_bufs_used_vif[if_id] < 0)
		ret = -1;
	return ret;
}
#ifdef MCAST_FWDING
int wsm_release_buffer_to_fw(struct xradio_vif *priv, int count)
{
	int i;
	u8 flags;
	struct wsm_buf *buf;
	size_t buf_len;
	struct wsm_hdr *wsm;
	struct xr819 *hw_priv = priv->hw_priv;

	if (priv->join_status != XRADIO_JOIN_STATUS_AP) {
		return 0;
	}
	//bh_printk(XRADIO_DBG_NIY, "Rel buffer to FW %d, %d\n", count, hw_priv->bh.hw_bufs_used);

	for (i = 0; i < count; i++) {
		if ((hw_priv->bh.hw_bufs_used + 1) < hw_priv->wsm.caps.numInpChBufs) {
			flags = i ? 0: 0x1;

			wsm_alloc_tx_buffer(hw_priv);
			buf = &hw_priv->wsm.wsm_release_buf[i];
			buf_len = buf->data - buf->begin;

			/* Add sequence number */
			wsm = (struct wsm_hdr *)buf->begin;
			//SYS_BUG(buf_len < sizeof(*wsm));

			wsm->id &= __cpu_to_le32(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
			wsm->id |= cpu_to_le32(WSM_TX_SEQ(hw_priv->wsm.tx_seq));

			//bh_printk(XRADIO_DBG_NIY, "REL %d\n", hw_priv->wsm.tx_seq);
			if (xradio_data_write(hw_priv, buf->begin, buf_len)) {
				break;
			}
			hw_priv->wsm.buf_released = 1;
			hw_priv->wsm.tx_seq = (hw_priv->wsm.tx_seq + 1) & WSM_TX_SEQ_MAX;
		} else
		break;
	}

	if (i == count) {
		return 0;
	}

	/* Should not be here */
	dev_err(hw_priv->dev, "Error, Less HW buf %d,%d.\n",
			hw_priv->bh.hw_bufs_used, hw_priv->wsm.caps.numInpChBufs);
	return -1;
}
#endif

/* reserve a packet for the case dev_alloc_skb failed in bh.*/
int xradio_init_resv_skb(struct xr819 *hw_priv) {
	int len = (SDIO_BLOCK_SIZE << 2) + WSM_TX_EXTRA_HEADROOM + 8 + 12; /* TKIP IV + ICV and MIC */

	hw_priv->bh.skb_reserved = dev_alloc_skb(len);
	if (hw_priv->bh.skb_reserved) {
		hw_priv->bh.skb_resv_len = len;
	} else {
		dev_warn(hw_priv->dev, "xr_alloc_skb failed(%d)\n", len);
	}
	return 0;
}

void xradio_deinit_resv_skb(struct xr819 *hw_priv) {
	if (hw_priv->bh.skb_reserved) {
		dev_kfree_skb(hw_priv->bh.skb_reserved);
		hw_priv->bh.skb_reserved = NULL;
		hw_priv->bh.skb_resv_len = 0;
	}
}

int xradio_realloc_resv_skb(struct xr819 *hw_priv, struct sk_buff *skb) {
	if (!hw_priv->bh.skb_reserved && hw_priv->bh.skb_resv_len) {
		hw_priv->bh.skb_reserved = dev_alloc_skb(hw_priv->bh.skb_resv_len);
		if (!hw_priv->bh.skb_reserved) {
			hw_priv->bh.skb_reserved = skb;
			dev_warn(hw_priv->dev, "xr_alloc_skb failed(%d)\n",
					hw_priv->bh.skb_resv_len);
			return -1;
		}
	}
	return 0; /* realloc sbk success, deliver to upper.*/
}

static inline struct sk_buff *xradio_get_resv_skb(struct xr819 *hw_priv,
		size_t len) {
	struct sk_buff *skb = NULL;
	if (hw_priv->bh.skb_reserved && len <= hw_priv->bh.skb_resv_len) {
		skb = hw_priv->bh.skb_reserved;
		hw_priv->bh.skb_reserved = NULL;
	}
	return skb;
}

static inline int xradio_put_resv_skb(struct xr819 *hw_priv,
		struct sk_buff *skb) {
	if (!hw_priv->bh.skb_reserved && hw_priv->bh.skb_resv_len) {
		hw_priv->bh.skb_reserved = skb;
		return 0;
	}
	return 1; /* sbk not put to reserve*/
}

static struct sk_buff *xradio_get_skb(struct xr819 *hw_priv, size_t len) {
	struct sk_buff *skb = NULL;
	size_t alloc_len = (len > SDIO_BLOCK_SIZE) ? len : SDIO_BLOCK_SIZE;

	/* TKIP IV + TKIP ICV and MIC - Piggyback.*/
	alloc_len += WSM_TX_EXTRA_HEADROOM + 8 + 12 - 2;
	if (len > SDIO_BLOCK_SIZE || !hw_priv->bh.skb_cache) {
		skb = dev_alloc_skb(alloc_len);
		/* In AP mode RXed SKB can be looped back as a broadcast.
		 * Here we reserve enough space for headers. */
		if (skb) {
			skb_reserve(skb, WSM_TX_EXTRA_HEADROOM + 8 /* TKIP IV */
			- WSM_RX_EXTRA_HEADROOM);
		} else {
			skb = xradio_get_resv_skb(hw_priv, alloc_len);
			if (skb) {
				dev_dbg(hw_priv->dev, "get skb_reserved(%d)!\n", alloc_len);
				skb_reserve(skb, WSM_TX_EXTRA_HEADROOM + 8 /* TKIP IV */
				- WSM_RX_EXTRA_HEADROOM);
			} else {
				dev_err(hw_priv->dev, "xr_alloc_skb failed(%d)!\n", alloc_len);
			}
		}
	} else {
		skb = hw_priv->bh.skb_cache;
		hw_priv->bh.skb_cache = NULL;
	}
	return skb;
}

static void xradio_put_skb(struct xr819 *hw_priv, struct sk_buff *skb) {
	if (hw_priv->bh.skb_cache)
		dev_kfree_skb(skb);
	else
		hw_priv->bh.skb_cache = skb;
}

static int xradio_bh_read_ctrl_reg(struct xr819 *hw_priv, u16 *ctrl_reg) {
	int ret;
	ret = xradio_reg_read_16(hw_priv, HIF_CONTROL_REG_ID, ctrl_reg);
	if (ret) {
		ret = xradio_reg_read_16(hw_priv, HIF_CONTROL_REG_ID, ctrl_reg);
		if (ret) {
			dev_err(hw_priv->dev, "Failed to read control register.\n");
		}
	}
	return ret;
}

static int xradio_device_wakeup(struct xr819 *hw_priv) {
	u16 ctrl_reg;
	int ret, i = 0;

	/* To force the device to be always-on, the host sets WLAN_UP to 1 */
	ret = xradio_reg_write_16(hw_priv, HIF_CONTROL_REG_ID, HIF_CTRL_WUP_BIT);
	if (ret)
		return ret;

	ret = xradio_bh_read_ctrl_reg(hw_priv, &ctrl_reg);
	if (ret)
		return ret;

	/* If the device returns WLAN_RDY as 1, the device is active and will
	 * remain active. */
	while (!(ctrl_reg & HIF_CTRL_RDY_BIT) && i < 500) {
		ret = xradio_bh_read_ctrl_reg(hw_priv, &ctrl_reg);
		msleep(1);
		i++;
	}
	if (unlikely(i >= 500)) {
		dev_err(hw_priv->dev, "Device cannot wakeup.\n");
		return -1;
	} else if (unlikely(i >= 50))
		dev_warn(hw_priv->dev, "Device wakeup time=%dms.\n", i);
	dev_info(hw_priv->dev, "Device awake, t=%dms.\n", i);
	return 1;
}

/* Must be called from BH thread. */
void xradio_enable_powersave(struct xradio_vif *priv, bool enable) {
	priv->hw_priv->bh.powersave_enabled = enable;
	dev_dbg(priv->hw_priv->dev, "Powersave is %s.\n",
			enable ? "enabled" : "disabled");
}

static void xradio_bh_dump_rx(struct xr819* priv, u8* data, int len) {
	u16 msgid, ifid;
	u16 *p = (u16 *) data;
	msgid = (*(p + 1)) & 0xC3F;
	ifid = (*(p + 1)) >> 6;
	ifid &= 0xF;
	dev_dbg(priv->dev, "[DUMP] msgid 0x%.4X ifid %d len %d\n", msgid, ifid, *p);
	print_hex_dump_bytes("<-- ", DUMP_PREFIX_NONE, data, min(len, 32));
}

static int xradio_bh_rx(struct xr819* hw_priv) {
	size_t alloc_len;
	u8 *data;
	size_t read_len = 0;
	struct sk_buff *skb_rx = NULL;
	struct wsm_hdr *wsm;
	size_t wsm_len;
	int wsm_id;
	u8 wsm_seq;
	u16 ctrl_reg;

	// read control reg
	if (xradio_bh_read_ctrl_reg(hw_priv, &ctrl_reg)) {
		return -1;
	}

	// is something waiting?
	read_len = (ctrl_reg & HIF_CTRL_NEXT_LEN_MASK) << 1; //read_len=ctrl_reg*2.
	if (!read_len) {
		return 0;
	}

	if ((read_len < sizeof(struct wsm_hdr))
			|| (read_len > EFFECTIVE_BUF_SIZE)) {
		dev_err(hw_priv->dev, "ERR: Invalid read len: %d", read_len);
		return -1;
	}

	/* Add SIZE of PIGGYBACK reg (CONTROL Reg)
	 * to the NEXT Message length + 2 Bytes for SKB */
	read_len = read_len + 2;

	alloc_len = sdio_align_len(hw_priv, read_len);

	/* Check if not exceeding XRADIO capabilities */
	if (WARN_ON_ONCE(alloc_len > EFFECTIVE_BUF_SIZE)) {
		//bh_printk(XRADIO_DBG_MSG, "ERR: Read aligned len: %d\n",
		//		alloc_len);
	}

	/* Get skb buffer. */
	skb_rx = xradio_get_skb(hw_priv, alloc_len);
	if (!skb_rx) {
		dev_err(hw_priv->dev, "ERR: xradio_get_skb failed.\n");
		return -1;
	}

	skb_trim(skb_rx, 0);
	skb_put(skb_rx, read_len);
	data = skb_rx->data;

	if (!data) {
		dev_err(hw_priv->dev, "ERR: skb data is NULL.\n");
		return -1;
	}

	/* Read data from device. */
	if (xradio_data_read(hw_priv, data, alloc_len)) {
		dev_err(hw_priv->dev, "Failed to read from device, no data?");
		return 0;
	}

	/* Piggyback */
	ctrl_reg = __le16_to_cpu(((__le16 *) data)[(alloc_len >> 1) - 1]);

	/* check wsm length. */
	wsm = (struct wsm_hdr *) data;
	wsm_len = __le32_to_cpu(wsm->len);

	if (wsm_len > read_len) {
		dev_err(hw_priv->dev, "wsm frame is longer than read data");
		return -1;
	}

	/* dump rx data. */
	xradio_bh_dump_rx(hw_priv, data, wsm_len);

	/* extract wsm id and seq. */
	wsm_id = __le32_to_cpu(wsm->id) & 0xFFF;
	wsm_seq = (__le32_to_cpu(wsm->id) >> 13) & 7;
	skb_trim(skb_rx, wsm_len);

	/* process exceptions. */
	if (wsm_id == 0) {
		printk("wtf?\n");
	} else if (unlikely(wsm_id == 0x0800)) {
		dev_err(hw_priv->dev, "firmware exception!\n");
		//wsm_handle_exception(hw_priv, &data[sizeof(*wsm)],
		//		wsm_len - sizeof(*wsm));
		return 0;
	} /*else if (unlikely(!rx_resync)) {
	 if (wsm_seq != hw_priv->wsm.rx_seq) {
	 dev_err(hw_priv->dev, "wsm_seq=%d.\n", wsm_seq);
	 hw_priv->bh.error = __LINE__;
	 break;
	 }
	 }*/
	hw_priv->wsm.rx_seq = (wsm_seq + 1) & 7;
	//rx_resync = 0;
#if defined(DGB_XRADIO_HWT)
	rx_resync = 1;  //0 -> 1, HWT test, should not check this.
#endif

	/* Process tx frames confirm. */
	if (wsm_id & 0x0400) {
		int rc = wsm_release_tx_buffer(hw_priv, 1);
		if (rc < 0) {
			dev_err(hw_priv->dev,
					"have tx confirm when there are no tx frames to confirm\n");
			return -1;
		} //else if (rc > 0)
		  //tx = 1;
	}

	/* WSM processing frames. */
	if (wsm_handle_rx(hw_priv, wsm_id, wsm, &skb_rx)) {
		dev_err(&hw_priv->sdio.func->dev, "wsm_handle_rx failed.\n");
		return -1;
	}
	/* Reclaim the SKB buffer */
	if (skb_rx) {
		if (xradio_put_resv_skb(hw_priv, skb_rx))
			xradio_put_skb(hw_priv, skb_rx);
		skb_rx = NULL;
	}

	return 0;
}

static void xradio_bh_tx_dump(struct xr819* hw_priv, u8* data, int len) {
	u16 msgid, ifid;
	u16 *p = (u16 *) data;
	msgid = (*(p + 1)) & 0x3F;
	ifid = (*(p + 1)) >> 6;
	ifid &= 0xF;
	if (msgid == 0x0006) {
		dev_dbg(hw_priv->dev, "[DUMP] >>> msgid 0x%.4X ifid %d"
				"len %d MIB 0x%.4X\n", msgid, ifid, *p, *(p + 2));
	} else {
		dev_dbg(hw_priv->dev, "[DUMP] >>> msgid 0x%.4X ifid %d "
				"len %d\n", msgid, ifid, *p);
	}
	print_hex_dump_bytes("--> ", DUMP_PREFIX_NONE, data, min(len, 32));
}

static int xradio_bh_tx(struct xr819* hw_priv) {
	int tx, availablebuffers, vif_selected;
	struct wsm_hdr* wsm;
	int ret;
	u8 *data;
	size_t tx_len;

	availablebuffers = hw_priv->wsm.caps.numInpChBufs
			- hw_priv->bh.hw_bufs_used;

	if (availablebuffers) {
		dev_dbg(hw_priv->dev, "checking for outgoing data\n");

		/* Wake up the devices */
		//if (hw_priv->bh.device_can_sleep) {
		//	ret = xradio_device_wakeup(hw_priv);
		//	if (ret < 0) {
		//		return ret;
		//	} else if (ret) {
		//		hw_priv->bh.device_can_sleep = false;
		//	} else { /* Wait for "awake" interrupt */
		//		pending_tx = tx;
		//		return 0;
		//	}
		//}
		/* Increase Tx buffer*/
		wsm_alloc_tx_buffer(hw_priv);

		/* Get data to send and send it. */
		ret = wsm_get_tx(hw_priv, &data, &tx_len, &availablebuffers,
				&vif_selected);
		if (ret == 0) {
			// nothing to transmit
			wsm_release_tx_buffer(hw_priv, 1);
		} else if (ret < 0) {
			// error
			dev_err(hw_priv->dev, "wsm_get_tx=%d.\n", ret);
			return -1;
		} else {
			// lets do this
			wsm = (struct wsm_hdr *) data;
			//SYS_BUG(tx_len < sizeof(*wsm));
			//SYS_BUG(__le32_to_cpu(wsm->len) != tx_len);

			/* Continue to send next data if have any. */
			atomic_add(1, &hw_priv->bh.tx);

			/* Align tx length and check it. */

			if (tx_len <= 8)
				tx_len = 16;
			tx_len = sdio_align_len(hw_priv, tx_len);

			/* Check if not exceeding XRADIO capabilities */
			if (tx_len > EFFECTIVE_BUF_SIZE) {
				dev_warn(hw_priv->dev, "Write aligned len: %d\n", tx_len);
			}

			/* Make sequence number. */
			wsm->id &= __cpu_to_le32(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
			wsm->id |= cpu_to_le32(WSM_TX_SEQ(hw_priv->wsm.tx_seq));

			/* Send the data to devices. */
			if (xradio_data_write(hw_priv, data, tx_len)) {
				wsm_release_tx_buffer(hw_priv, 1);
				dev_err(hw_priv->dev, "xradio_data_write failed\n");
				return -1;
			}

			xradio_bh_tx_dump(hw_priv, data, tx_len);

			/* Process after data have sent. */
			if (vif_selected != -1) {
				hw_priv->bh.hw_bufs_used_vif[vif_selected]++;
			}

			wsm_txed(hw_priv, data);
			hw_priv->wsm.tx_seq = (hw_priv->wsm.tx_seq + 1) & WSM_TX_SEQ_MAX;

		}
	}
	return 0;
}

#if 0
static int xradio_bh_suspend() {
//bh_printk(XRADIO_DBG_NIY, "Host suspend request.\n");
	/* Check powersave setting again. */
	if (hw_priv->bh.powersave_enabled) {
		dev_dbg(hw_priv->dev,
				"Device idle(host suspend), can sleep.\n");
		//SYS_WARN(xradio_reg_write_16(hw_priv, HIF_CONTROL_REG_ID, 0));
		hw_priv->bh.device_can_sleep = true;
	}

	/* bh thread go to suspend. */
	atomic_set(&hw_priv->bh.suspend, XRADIO_BH_SUSPENDED);
	wake_up(&hw_priv->bh.evt_wq);

	status = wait_event_interruptible(hw_priv->bh.wq,
			XRADIO_BH_RESUME == atomic_read(&hw_priv->bh.suspend));

	if (status < 0) {
		dev_err(hw_priv->dev, "ERR: Failed to wait for resume: %ld.\n",
				status);
		hw_priv->bh.error = __LINE__;
		break;
	}
	dev_info(hw_priv->dev, "Host resume.\n");
	atomic_set(&hw_priv->bh.suspend, XRADIO_BH_RESUMED);
	wake_up(&hw_priv->bh.evt_wq);
	atomic_add(1, &hw_priv->bh.rx);
	continue;
}
#endif

static int xradio_bh(void *arg) {
	struct xr819 *hw_priv = arg;

	int busy = 0;

	int interrupt;

	int tx = 0, term, suspend;
	int rx_resync = 1;

	int rx_burst = 0;
	long status, wakeuptimeout;
	u32 dummy;

	for (;;) {

		/* Check if devices can sleep, and set time to wait for interrupt. */
		//if (!hw_priv->bh.hw_bufs_used && !pending_tx
		//		&& hw_priv->bh.powersave_enabled
		//		&& !hw_priv->bh.device_can_sleep
		//		&& !atomic_read(&hw_priv->bh.recent_scan)
		//		&& atomic_read(&hw_priv->bh.rx) == 0
		//		&& atomic_read(&hw_priv->bh.tx) == 0) {
		//	dev_dbg(hw_priv->dev, "Device idle, can sleep.\n");
		//	//SYS_WARN(xradio_reg_write_16(hw_priv, HIF_CONTROL_REG_ID, 0));
		//	hw_priv->bh.device_can_sleep = true;
		//	wakeuptimeout = HZ / 8;    //125ms
		//} else if (hw_priv->bh.hw_bufs_used) {
		//	/* don't wait too long if some frames to confirm
		//	 * and miss interrupt.*/
		//	wakeuptimeout = HZ / 20;   //50ms.
		//} else {
		wakeuptimeout = HZ / 8;    //125ms
		//}

		/* Dummy Read for SDIO retry mechanism*/
		//if (atomic_read(&hw_priv->bh.rx) == 0
		//		&& atomic_read(&hw_priv->bh.tx) == 0) {
		//	xradio_reg_read(hw_priv, HIF_CONFIG_REG_ID, &dummy, sizeof(dummy));
		//}
		/* If a packet has already been txed to the device then read the
		 * control register for a probable interrupt miss before going
		 * further to wait for interrupt; if the read length is non-zero
		 * then it means there is some data to be received */
		//if (hw_priv->bh.hw_bufs_used) {
		//	xradio_bh_read_ctrl_reg(hw_priv, &ctrl_reg);
		//	if (ctrl_reg & HIF_CTRL_NEXT_LEN_MASK) {
		//		rx = 1;
		//		goto data_proc;
		//	}
		//}
		/* Wait for Events in HZ/8 */

		status = wait_event_interruptible_timeout(hw_priv->bh.wq, ( {
					interrupt = atomic_xchg(&hw_priv->bh.interrupt, 0);
					tx = atomic_xchg(&hw_priv->bh.tx, 0);
					term = kthread_should_stop();
					//suspend = pending_tx ? 0 : atomic_read(&hw_priv->bh.suspend);
				(interrupt || tx || term);}), wakeuptimeout);

		/* got told to terminate */
		if (term) {
			dev_dbg(hw_priv->dev, "xradio_bh exit!\n");
			break;
		}

		/* got interrupted */
		if (status < 0) {
			dev_err(hw_priv->dev, "got interrupted, exiting");
			break;
		}

		/* 2--Wait for interrupt time out */
		//if (!status) {
		//	/* Check if miss interrupt. */
		//	xradio_bh_read_ctrl_reg(hw_priv, &ctrl_reg);
		//	if (ctrl_reg & HIF_CTRL_NEXT_LEN_MASK) {
		//		dev_info(&hw_priv->sdio.func->dev, "miss interrupt!\n");
		//		rx = 1;
		//		goto data_proc;
		//	}
		//
		//	/* There are some frames to be confirmed. */
		//	if (hw_priv->bh.hw_bufs_used) {
		//		long timeout = 0;
		//		bool pending = 0;
		//bh_printk(XRADIO_DBG_NIY, "Need confirm:%d!\n",
		//		hw_priv->bh.hw_bufs_used);
		/* Check if frame transmission is timed out. */
		//pending = xradio_query_txpkt_timeout(hw_priv, XRWL_ALL_IFS,
		//                                    hw_priv->pending_frame_id, &timeout);
		/* There are some frames confirm time out. */
		//		if (pending && timeout < 0) {
		//			dev_err(hw_priv->dev, "query_txpkt_timeout:%ld!\n",
		//					timeout);
		//			break;
		//		}
		//		rx = 1; /* Go to check rx again. */
		//	} else if (!pending_tx) {
		//		if (hw_priv->bh.powersave_enabled
		//				&& !hw_priv->bh.device_can_sleep
		//				&& !atomic_read(&hw_priv->bh.recent_scan)) {
		//			/* Device is idle, we can go to sleep. */
		//			dev_info(hw_priv->dev,
		//					"Device idle(timeout), can sleep.\n");
		//			//SYS_WARN(
		//			//		xradio_reg_write_16(hw_priv, HIF_CONTROL_REG_ID,
		//			//				0));
		//			hw_priv->bh.device_can_sleep = true;
		//		}
		//		continue;
		//	}
		//
		//	/* 3--Host suspend request. */
		//} else if (suspend) {
		//	//xradio_bh_suspend();
		//}
		/* query stuck frames in firmware. */
		//if (atomic_xchg(&hw_priv->bh.query_cnt, 0)) {
		//	if (schedule_work(&hw_priv->bh.query_work) <= 0)
		//		atomic_add(1, &hw_priv->bh.query_cnt);
		//}
		if (xradio_bh_rx(hw_priv))
			break;

		if (xradio_bh_tx(hw_priv))
			break;

		/* Check if there are frames to be read. */
		//xradio_bh_read_ctrl_reg(hw_priv, &ctrl_reg);
		//if (ctrl_reg & HIF_CTRL_NEXT_LEN_MASK) {
		//	goto data_proc;
		//}
	} /* for (;;)*/

	dev_err(hw_priv->dev, "bh thread terminating\n");

	return 0;
}
