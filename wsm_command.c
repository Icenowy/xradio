#include "wsm_command.h"
#include "wsm_putget.h"
#include "wsm_buf.h"
#include "wsm.h"
#include "bh.h"

#define WSM_CMD_TIMEOUT		(2 * HZ) /* With respect to interrupt loss */
#define WSM_CMD_JOIN_TIMEOUT	(7 * HZ) /* Join timeout is 5 sec. in FW   */
#define WSM_CMD_START_TIMEOUT	(7 * HZ)
#define WSM_CMD_RESET_TIMEOUT	(3 * HZ) /* 2 sec. timeout was observed.   */
#define WSM_CMD_DEFAULT_TIMEOUT	(3 * HZ)

static inline void wsm_cmd_lock(struct xr819 *hw_priv) {
	mutex_lock(&hw_priv->wsm.wsm_cmd_mux);
}

static inline void wsm_cmd_unlock(struct xr819 *hw_priv) {
	mutex_unlock(&hw_priv->wsm.wsm_cmd_mux);
}

int wsm_cmd_send(struct xr819 *hw_priv, struct wsm_buf *buf, void *arg, u16 cmd,
		long tmo, int if_id) {
	size_t buf_len = buf->data - buf->begin;
	int ret;

	if (cmd == 0x0006 || cmd == 0x0005) /* Write/Read MIB */
		dev_dbg(hw_priv->dev, ">>> 0x%.4X [MIB: 0x%.4X] (%d)\n", cmd,
				__le16_to_cpu(((__le16 *) buf->begin)[2]), buf_len);
	else
		dev_dbg(hw_priv->dev, ">>> 0x%.4X (%d)\n", cmd, buf_len);

#ifdef HW_RESTART && 0
//	if (hw_priv->hw_restart) {
//		wsm_printk(XRADIO_DBG_NIY, "hw reset!>>> 0x%.4X (%d)\n", cmd, buf_len);
//		wsm_buf_reset(buf);
//		return 0; /*return success, don't process cmd in power off.*/
//	}
#endif

	//if (unlikely(hw_priv->bh.error)) {
	//	wsm_buf_reset(buf);
	//	wsm_printk(XRADIO_DBG_ERROR, "bh error!>>> 0x%.4X (%d)\n", cmd,
	//			buf_len);
	//	return -ETIMEDOUT;
	//}

	/* Fill HI message header */
	/* BH will add sequence number */

	/* TODO:COMBO: Add if_id from  to the WSM header */
	/* if_id == -1 indicates that command is HW specific,
	 * eg. wsm_configuration which is called during driver initialzation
	 *  (mac80211 .start callback called when first ifce is created. )*/

	/* send hw specific commands on if 0 */
	if (if_id == -1)
		if_id = 0;

	((__le16 *) buf->begin)[0] = __cpu_to_le16(buf_len);
	((__le16 *) buf->begin)[1] = __cpu_to_le16(
			cmd | ((is_hardware_xradio(hw_priv)) ? (if_id << 6) : 0));

	spin_lock(&hw_priv->wsm.wsm_cmd.lock);
	//SYS_BUG(hw_priv->wsm.wsm_cmd.ptr);
	hw_priv->wsm.wsm_cmd.done = 0;
	hw_priv->wsm.wsm_cmd.ptr = buf->begin;
	hw_priv->wsm.wsm_cmd.len = buf_len;
	hw_priv->wsm.wsm_cmd.arg = arg;
	hw_priv->wsm.wsm_cmd.cmd = cmd;
	spin_unlock(&hw_priv->wsm.wsm_cmd.lock);
	xradio_bh_wakeup(hw_priv);

	if (unlikely(hw_priv->bh.error)) {
		/* Do not wait for timeout if BH is dead. Exit immediately. */
		ret = 0;
	} else {
		unsigned long wsm_cmd_max_tmo;

		/* Give start cmd a little more time */
		if (unlikely(tmo == WSM_CMD_START_TIMEOUT))
			wsm_cmd_max_tmo = WSM_CMD_START_TIMEOUT;
		else
			wsm_cmd_max_tmo = WSM_CMD_DEFAULT_TIMEOUT;

		/*Set max timeout.*/
		wsm_cmd_max_tmo = jiffies + wsm_cmd_max_tmo;

		/* Firmware prioritizes data traffic over control confirm.
		 * Loop below checks if data was RXed and increases timeout
		 * accordingly. */
		do {
			/* It's safe to use unprotected access to wsm_cmd.done here */
			ret = wait_event_timeout(hw_priv->wsm.wsm_cmd_wq,
					hw_priv->wsm.wsm_cmd.done, tmo);

			/* check time since last rxed and max timeout.*/
		} while (!ret
				&& time_before_eq(jiffies, hw_priv->txrx.rx_timestamp + tmo)
				&& time_before(jiffies, wsm_cmd_max_tmo));

	}

	if (unlikely(ret == 0)) {
		u16 raceCheck;

		spin_lock(&hw_priv->wsm.wsm_cmd.lock);
		raceCheck = hw_priv->wsm.wsm_cmd.cmd;
		hw_priv->wsm.wsm_cmd.arg = NULL;
		hw_priv->wsm.wsm_cmd.ptr = NULL;
		spin_unlock(&hw_priv->wsm.wsm_cmd.lock);

		dev_err(hw_priv->dev,
				"***CMD timeout!>>> 0x%.4X (%d), buf_use=%d, bh_state=%d\n",
				cmd, buf_len, hw_priv->bh.hw_bufs_used, hw_priv->bh.error);
		/* Race condition check to make sure _confirm is not called
		 * after exit of _send */
		if (raceCheck == 0xFFFF) {
			/* If wsm_handle_rx got stuck in _confirm we will hang
			 * system there. It's better than silently currupt
			 * stack or heap, isn't it? */
			//SYS_BUG(
			//		wait_event_timeout(hw_priv->wsm.wsm_cmd_wq,
			//				hw_priv->wsm.wsm_cmd.done,
			//				WSM_CMD_LAST_CHANCE_TIMEOUT) <= 0);
		}

		/* Kill BH thread to report the error to the top layer. */
		hw_priv->bh.error = 1;
		wake_up(&hw_priv->bh.wq);
		ret = -ETIMEDOUT;
	} else {
		spin_lock(&hw_priv->wsm.wsm_cmd.lock);
		//SYS_BUG(!hw_priv->wsm.wsm_cmd.done);
		ret = hw_priv->wsm.wsm_cmd.ret;
		spin_unlock(&hw_priv->wsm.wsm_cmd.lock);
	}
	wsm_buf_reset(buf);
	return ret;
}

int wsm_write_mib(struct xr819 *hw_priv, u16 mibId, void *_buf, size_t buf_size,
		int if_id) {
	int ret = 0;
	struct wsm_buf *buf = &hw_priv->wsm.wsm_cmd_buf;
	struct wsm_mib mib_buf =
			{ .mibId = mibId, .buf = _buf, .buf_size = buf_size, };

	wsm_cmd_lock(hw_priv);
	WSM_PUT16(buf, mibId);
	WSM_PUT16(buf, buf_size);
	WSM_PUT(buf, _buf, buf_size);

	ret = wsm_cmd_send(hw_priv, buf, &mib_buf, 0x0006, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

	nomem: wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

int wsm_switch_channel(struct xr819 *hw_priv,
		const struct wsm_switch_channel *arg, int if_id) {
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm.wsm_cmd_buf;

	//wsm_lock_tx(hw_priv);
	wsm_cmd_lock(hw_priv);

	WSM_PUT8(buf, arg->channelMode);
	WSM_PUT8(buf, arg->channelSwitchCount);
	WSM_PUT16(buf, arg->newChannelNumber);

	//hw_priv->channel_switch_in_progress = 1;

	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0016, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	if (ret) {
		//wsm_unlock_tx(hw_priv);
		//hw_priv->channel_switch_in_progress = 0;
	}
	return ret;

	nomem: wsm_cmd_unlock(hw_priv);
	//wsm_unlock_tx(hw_priv);
	return -ENOMEM;
}

int wsm_configuration(struct xr819 *hw_priv, struct wsm_configuration *arg,
		int if_id) {
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm.wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);

	WSM_PUT32(buf, arg->dot11MaxTransmitMsduLifeTime);
	WSM_PUT32(buf, arg->dot11MaxReceiveLifeTime);
	WSM_PUT32(buf, arg->dot11RtsThreshold);

	/* DPD block. */
	WSM_PUT16(buf, arg->dpdData_size + 12);
	WSM_PUT16(buf, 1); /* DPD version */
	WSM_PUT(buf, arg->dot11StationId, ETH_ALEN);
	WSM_PUT16(buf, 5); /* DPD flags */
	WSM_PUT(buf, arg->dpdData, arg->dpdData_size);

	ret = wsm_cmd_send(hw_priv, buf, arg, 0x0009, WSM_CMD_TIMEOUT, if_id);

	wsm_cmd_unlock(hw_priv);
	return ret;

	nomem: wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}
