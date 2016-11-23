/*
 * STA and AP APIs for XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __XRADIO_AP_H
#define __XRADIO_AP_H

#include <net/mac80211.h>

int xradio_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		struct ieee80211_sta *sta);
int xradio_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		struct ieee80211_sta *sta);

#endif
