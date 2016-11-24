/*
 * sta interfaces for XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __XRADIO_STA_H
#define __XRADIO_STA_H

#include <net/mac80211.h>

int xradio_start(struct ieee80211_hw *dev);
void xradio_stop(struct ieee80211_hw *dev);
int xradio_config(struct ieee80211_hw *dev, u32 changed);
int xradio_conf_tx(struct ieee80211_hw *dev, struct ieee80211_vif *vif,
		u16 queue, const struct ieee80211_tx_queue_params *params);
int xradio_add_interface(struct ieee80211_hw *dev, struct ieee80211_vif *vif);
void xradio_remove_interface(struct ieee80211_hw *dev,
		struct ieee80211_vif *vif);
void xradio_configure_filter(struct ieee80211_hw *dev,
		unsigned int changed_flags, unsigned int *total_flags, u64 multicast);

#endif
