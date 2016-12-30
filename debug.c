/*
 * Debug code for XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*Linux version 3.4.0 compilation*/
//#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0))
#include<linux/module.h>
//#endif
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/rtc.h>
#include <linux/time.h>

#include "xradio.h"
#include "hwio.h"
#include "debug.h"

/*added by yangfh, for host debuglevel*/
u8 dbg_common  = 0xff;
u8 dbg_sbus    = 0xff;
u8 dbg_bh      = 0xff;
u8 dbg_txrx    = 0xff;
u8 dbg_wsm     = 0xff;
u8 dbg_sta     = 0xff;
u8 dbg_scan    = 0xff;
u8 dbg_ap      = 0xff;
u8 dbg_pm      = 0xff;
u8 dbg_itp     = 0xff;
u8 dbg_logfile = XRADIO_DBG_ERROR;


#define FRAME_TYPE(xx) ieee80211_is_ ## xx(fctl)
#define FT_MSG_PUT(f, ...) do{ \
	if(flags&f) frame_msg += sprintf(frame_msg, __VA_ARGS__); \
	}while(0)
	
#define PT_MSG_PUT(f, ...) do{ \
	if(flags&f) proto_msg += sprintf(proto_msg, __VA_ARGS__); \
	}while(0)

#define FRAME_PARSE(f, name) do{ \
		if(FRAME_TYPE(name)) { FT_MSG_PUT(f, "%s", #name); goto outprint;} \
  } while(0)

#define IS_FRAME_PRINT (frame_msg != (char *)&framebuf[0])
#define IS_PROTO_PRINT (proto_msg != (char *)&protobuf[0])


char framebuf[512] = {0};
char protobuf[512] = {0};

char * p2p_frame_type[] = {
	"GO Negotiation Request",
	"GO Negotiation Response",
	"GO Negotiation Confirmation",
	"P2P Invitation Request",
	"P2P Invitation Response",
	"Device Discoverability Request",
	"Device Discoverability Response",
	"Provision Discovery Request",
	"Provision Discovery Response",
	"Reserved"
};

#if defined(DGB_XRADIO_HWT)
/***************************for HWT, yangfh********************************/
struct sk_buff *hwt_skb = NULL;
int    sent_num   = 0;
int get_hwt_hif_tx(struct xradio_common *hw_priv, u8 **data, 
                   size_t *tx_len, int *burst, int *vif_selected)
{

	HWT_PARAMETERS *hwt_tx_hdr = NULL;
	if (!hwt_tx_en || !hwt_tx_len || !hwt_tx_num ||
		  sent_num >= hwt_tx_num) {
		if (hwt_skb) {
			dev_kfree_skb(hwt_skb);
			hwt_skb = NULL;
		}
		return 0;
	}

	if (!hwt_skb) {
		hwt_skb = xr_alloc_skb(1504);
		if (!hwt_skb) {
			xradio_dbg(XRADIO_DBG_ERROR, "%s:skb is NULL!\n", __func__);
			return 0;
		}
		if ((u32)hwt_skb->data & 3) {
			u8 align = 4-((u32)hwt_skb->data & 3);
			skb_reserve(hwt_skb, align);
		}
		skb_put(hwt_skb, 1500);
	}
	
	//fill the header info
	if (hwt_tx_len < sizeof(HWT_PARAMETERS))
		hwt_tx_len = sizeof(HWT_PARAMETERS);
	if (hwt_tx_len > 1500)
		hwt_tx_len = 1500;
	hwt_tx_hdr = hwt_skb->data;
	hwt_tx_hdr->MsgID  = 0x0024;
	hwt_tx_hdr->Msglen = hwt_tx_len;
	hwt_tx_hdr->TestID = 0x0001;
	hwt_tx_hdr->Data   = 0x1234;
	
	//send the packet
	*data   = hwt_skb->data;
	*tx_len = hwt_tx_hdr->Msglen;
	*vif_selected = 0;
	*burst = 2; //burst > 1 for continuous tx.
	sent_num++;
	
	//first packet.
	if (sent_num == 1) {
		do_gettimeofday(&hwt_start_time);
	}

	//set confirm
	hwt_tx_hdr->Params = 0;
	if (sent_num >= hwt_tx_num) {
		hwt_tx_hdr->Params = 0x101;  //last packet
		hwt_tx_en  = 0;  //disable hwt_tx_en
		xradio_dbg(XRADIO_DBG_ALWY, "%s:sent last packet!\n", __func__);
	} else if (hwt_tx_cfm) {
		hwt_tx_hdr->Params = !(sent_num%hwt_tx_cfm);
	}

	return 1;
}
#endif
