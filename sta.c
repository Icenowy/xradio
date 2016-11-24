/*
 * STA APIs for XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <net/cfg80211.h>

#include "xr819.h"
#include "wsm.h"
#include "wsm_command.h"

int xradio_start(struct ieee80211_hw *dev) {
	struct xr819 *hw_priv = dev->priv;
	wiphy_debug(dev->wiphy, "start\n");

	// this seems to turn on the chip
	int ret = 0, if_id;
	if (hw_priv->firmware.sdd) {
		struct wsm_configuration cfg = { .dot11StationId =
				&hw_priv->netif.mac_addr[0], .dpdData =
				hw_priv->firmware.sdd->data, .dpdData_size =
				hw_priv->firmware.sdd->size, };
		for (if_id = 0; if_id < xrwl_get_nr_hw_ifaces(hw_priv); if_id++) {
			/* Set low-power mode. */
			ret |= wsm_configuration(hw_priv, &cfg, if_id);
		}
		/* wsm_configuration only once, so release it */
		release_firmware(hw_priv->firmware.sdd);
		hw_priv->firmware.sdd = NULL;
	}

	return 0;
}
void xradio_stop(struct ieee80211_hw *dev) {
	wiphy_debug(dev->wiphy, "stop\n");
}

static void xradio_channel_switch(struct ieee80211_hw *dev,
		struct ieee80211_channel *ch) {
	struct xr819* hw_priv = dev->priv;
	struct wsm_switch_channel switch_arg = { .channelMode = 1,
			.channelSwitchCount = 0, .newChannelNumber = ch->hw_value };
	wiphy_debug(dev->wiphy, " changing freq %d (wsm ch: %d).\n",
			ch->center_freq, ch->hw_value);
	wsm_switch_channel(hw_priv, &switch_arg, 0);
}

int xradio_config(struct ieee80211_hw *dev, u32 changed) {
	int ret = 0;
	struct ieee80211_conf *conf = &dev->conf;
	wiphy_debug(dev->wiphy, "config\n");
	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		xradio_channel_switch(dev, conf->chandef.chan);
	}
	return ret;
}

int xradio_conf_tx(struct ieee80211_hw *dev, struct ieee80211_vif *vif,
		u16 queue, const struct ieee80211_tx_queue_params *params) {
	wiphy_debug(dev->wiphy, "conf tx\n");

	int ret;
	struct xr819* hw_priv = dev->priv;
	struct wsm_edca_params edca;
	struct wsm_tx_queue_params tx_queue_params;

	WSM_TX_QUEUE_SET(&tx_queue_params, queue, 0, 0, 0);
	ret = wsm_set_tx_queue_params(hw_priv, &tx_queue_params.params[queue],
			queue, 0);
	if (ret) {
		wiphy_debug(dev->wiphy, "wsm_set_tx_queue_params failed!\n");
		ret = -EINVAL;
		goto out;
	}

	WSM_EDCA_SET(&edca, queue, params->aifs, params->cw_min, params->cw_max,
			params->txop, 0xc8, params->uapsd);
	ret = wsm_set_edca_params(hw_priv, &edca, 0);
	if (ret) {
		wiphy_debug(dev->wiphy, "wsm_set_edca_params failed!\n");
		ret = -EINVAL;
		goto out;
	}

	//if (priv->mode == NL80211_IFTYPE_STATION) {
		//ret = xradio_set_uapsd_param(priv, &priv->edca);
		//if (!ret && priv->setbssparams_done
		//		&& (priv->join_status == XRADIO_JOIN_STATUS_STA)
		//		&& (old_uapsdFlags != priv->uapsd_info.uapsdFlags))
		//xradio_set_pm(priv, &priv->powersave_mode);
	//}

	out: return 0;
}

int xradio_add_interface(struct ieee80211_hw *dev, struct ieee80211_vif *vif) {
	wiphy_debug(dev->wiphy, "add interface\n");
	struct xr819 *hw_priv = dev->priv;

	int ret;
	int ifid = 0;

	struct wsm_rcpi_rssi_threshold threshold = { .rssiRcpiMode =
	WSM_RCPI_RSSI_THRESHOLD_ENABLE | WSM_RCPI_RSSI_DONT_USE_UPPER
			| WSM_RCPI_RSSI_DONT_USE_LOWER, .rollingAverageCount = 16, };

	ret = wsm_set_rcpi_rssi_threshold(hw_priv, &threshold, ifid);

	return ret;
}

void xradio_remove_interface(struct ieee80211_hw *dev,
		struct ieee80211_vif *vif) {
	wiphy_debug(dev->wiphy, "remove interface\n");
}

void xradio_configure_filter(struct ieee80211_hw *dev,
		unsigned int changed_flags, unsigned int *total_flags, u64 multicast) {
	wiphy_debug(dev->wiphy, "configure filter\n");
}
