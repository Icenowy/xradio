/*
 * txrx interfaces for XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __XRADIO_TXRX_H
#define __XRADIO_TXRX_H

#include <net/mac80211.h>

void xradio_tx(struct ieee80211_hw *dev, struct ieee80211_tx_control *control,
		struct sk_buff *skb);

#endif
