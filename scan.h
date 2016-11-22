/*
 * Scan interfaces for XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef SCAN_H_INCLUDED
#define SCAN_H_INCLUDED

#include <linux/semaphore.h>
#include "wsm.h"

/* external */ struct sk_buff;
/* external */ struct cfg80211_scan_request;
/* external */ struct ieee80211_channel;
/* external */ struct ieee80211_hw;
/* external */ struct work_struct;

#define SCAN_MAX_DELAY      (3*HZ)   //3s, add by yangfh for connect

int xradio_hw_scan(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
                   struct ieee80211_scan_request *req);
#ifdef ROAM_OFFLOAD
int xradio_hw_sched_scan_start(struct ieee80211_hw *hw,
                               struct ieee80211_vif *vif,
                               struct cfg80211_sched_scan_request *req,
                               struct ieee80211_sched_scan_ies *ies);
void xradio_hw_sched_scan_stop(struct xradio_common *priv);
void xradio_sched_scan_work(struct work_struct *work);
#endif /*ROAM_OFFLOAD*/
void xradio_scan_work(struct work_struct *work);
void xradio_scan_timeout(struct work_struct *work);
void xradio_scan_complete_cb(struct xradio_common *priv,
                             struct wsm_scan_complete *arg);

/* ******************************************************************** */
/* Raw probe requests TX workaround					*/
void xradio_probe_work(struct work_struct *work);
#ifdef CONFIG_XRADIO_TESTMODE
/* Advance Scan Timer							*/
void xradio_advance_scan_timeout(struct work_struct *work);
#endif

#endif
