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
u8 dbg_common  = XRADIO_DBG_ALWY|XRADIO_DBG_ERROR|XRADIO_DBG_WARN;
u8 dbg_sbus    = XRADIO_DBG_ALWY|XRADIO_DBG_ERROR|XRADIO_DBG_WARN;
u8 dbg_bh      = XRADIO_DBG_ALWY|XRADIO_DBG_ERROR|XRADIO_DBG_WARN;
u8 dbg_txrx    = XRADIO_DBG_ALWY|XRADIO_DBG_ERROR;
u8 dbg_wsm     = XRADIO_DBG_ALWY|XRADIO_DBG_ERROR|XRADIO_DBG_WARN;
u8 dbg_sta     = XRADIO_DBG_ALWY|XRADIO_DBG_ERROR|XRADIO_DBG_WARN;
u8 dbg_scan    = XRADIO_DBG_ALWY|XRADIO_DBG_ERROR|XRADIO_DBG_WARN;
u8 dbg_ap      = XRADIO_DBG_ALWY|XRADIO_DBG_ERROR|XRADIO_DBG_WARN;
u8 dbg_pm      = XRADIO_DBG_ALWY|XRADIO_DBG_ERROR|XRADIO_DBG_WARN;
u8 dbg_itp     = XRADIO_DBG_ALWY|XRADIO_DBG_ERROR|XRADIO_DBG_WARN;
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
#if (defined(CONFIG_XRADIO_DEBUG))
void xradio_parse_frame(u8* mac_data, u8 iv_len, u16 flags, u8 if_id)
{
	char * frame_msg = &framebuf[0];
	char * proto_msg = &protobuf[0];
	struct ieee80211_hdr *frame = (struct ieee80211_hdr *)mac_data;
	u16  fctl = frame->frame_control;

	memset(frame_msg, 0, sizeof(frame_msg));
	memset(proto_msg, 0, sizeof(proto_msg));

	if(ieee80211_is_data(fctl)) {
		u8  machdrlen = ieee80211_hdrlen(fctl);
		u8* llc_data  = mac_data+machdrlen+iv_len;
		
		if(ieee80211_is_qos_nullfunc(fctl)){
			FT_MSG_PUT(PF_DATA, "QoS-NULL");
			goto outprint;
		}
		if(ieee80211_is_data_qos(fctl))
			FT_MSG_PUT(PF_DATA, "QoS");
		if(ieee80211_is_nullfunc(fctl)) {
			FT_MSG_PUT(PF_DATA, "NULL(ps=%d)", !!(fctl&IEEE80211_FCTL_PM));
			goto outprint;
		}
		FT_MSG_PUT(PF_DATA, "data(TDFD=%d%d,R=%d,P=%d)",
		          !!(fctl&IEEE80211_FCTL_TODS), !!(fctl&IEEE80211_FCTL_FROMDS), 
		          !!(fctl&IEEE80211_FCTL_RETRY),!!(fctl&IEEE80211_FCTL_PROTECTED));
		
		if (is_SNAP(llc_data)) {
			if(is_ip(llc_data)) {
				u8 * ip_hdr    = llc_data+LLC_LEN;
				u8 * ipaddr_s  = ip_hdr+IP_S_ADD_OFF;
				u8 * ipaddr_d  = ip_hdr+IP_D_ADD_OFF;
				u8 * proto_hdr = ip_hdr+((ip_hdr[0]&0xf)<<2);  //ihl:words

				if(is_tcp(llc_data)) {
					PT_MSG_PUT(PF_TCP, "TCP%s%s, src=%d, dest=%d, seq=0x%08x, ack=0x%08x",
					          (proto_hdr[13]&0x01)?"(S)":"", (proto_hdr[13]&0x02)?"(F)":"",
					          (proto_hdr[0]<<8)|proto_hdr[1], (proto_hdr[2]<<8)|proto_hdr[3], 
					          (proto_hdr[4]<<24)|(proto_hdr[5]<<16)|(proto_hdr[6]<<8)|proto_hdr[7],
					          (proto_hdr[8]<<24)|(proto_hdr[9]<<16)|(proto_hdr[10]<<8)|proto_hdr[11]);
	
				} else if(is_udp(llc_data)) {
					if(is_dhcp(llc_data)) {
						u8  Options_len = BOOTP_OPS_LEN;
						u32 dhcp_magic  = cpu_to_be32(DHCP_MAGIC);
						u8 * dhcphdr = proto_hdr+UDP_LEN+UDP_BOOTP_LEN;
						while(Options_len) {
							if(*(u32*)dhcphdr == dhcp_magic)
								break;
							dhcphdr++;
							Options_len--;
						}
						PT_MSG_PUT(PF_DHCP, "DHCP, Opt=%d, MsgType=%d", *(dhcphdr+4), *(dhcphdr+6));
					} else {
						PT_MSG_PUT(PF_UDP, "UDP, source=%d, dest=%d", 
						          (proto_hdr[0]<<8)|proto_hdr[1], (proto_hdr[2]<<8)|proto_hdr[3]);
					}
				} else if(is_icmp(llc_data)) {
					PT_MSG_PUT(PF_ICMP,"ICMP%s%s, Seq=%d", (8 == proto_hdr[0])?"(ping)":"",
					           (0 == proto_hdr[0])?"(reply)":"", (proto_hdr[6]<<8)|proto_hdr[7]);
				} else if(is_igmp(llc_data)) {
					PT_MSG_PUT(PF_UNKNWN, "IGMP, type=0x%x", proto_hdr[0]);
				} else {
					PT_MSG_PUT(PF_UNKNWN, "unknown IP type=%d", *(ip_hdr+IP_PROTO_OFF));
				}
				if (IS_PROTO_PRINT) {
					PT_MSG_PUT(PF_IPADDR, "-%d.%d.%d.%d(s)", \
					           ipaddr_s[0], ipaddr_s[1], ipaddr_s[2], ipaddr_s[3]);
					PT_MSG_PUT(PF_IPADDR, "-%d.%d.%d.%d(d)", \
					           ipaddr_d[0], ipaddr_d[1], ipaddr_d[2], ipaddr_d[3]);
				}

			} else if (is_8021x(llc_data)) {
				PT_MSG_PUT(PF_8021X, "8021X");
			} else { //other protol, no detail.
				switch(cpu_to_be16(*(u16*)(llc_data+LLC_TYPE_OFF))) {
				case ETH_P_IPV6:  //0x08dd
					PT_MSG_PUT(PF_UNKNWN, "IPv6");
					break;
				case ETH_P_ARP:   //0x0806
					PT_MSG_PUT(PF_UNKNWN, "ARP");
					break;
				case ETH_P_RARP:   //0x8035
					PT_MSG_PUT(PF_UNKNWN, "RARP");
					break;
				case ETH_P_DNA_RC:  //0x6002
					PT_MSG_PUT(PF_UNKNWN, "DNA Remote Console");
					break;
				case ETH_P_DNA_RT:  //0x6003
					PT_MSG_PUT(PF_UNKNWN, "DNA Routing");
					break;
				case ETH_P_8021Q:  //0x8100
					PT_MSG_PUT(PF_UNKNWN, "802.1Q VLAN");
					break;
				case ETH_P_LINK_CTL:  //0x886c
					PT_MSG_PUT(PF_UNKNWN, "wlan link local tunnel(HPNA)");
					break;
				case ETH_P_PPP_DISC:  //0x8863
					PT_MSG_PUT(PF_UNKNWN, "PPPoE discovery");
					break;
				case ETH_P_PPP_SES:  //0x8864
					PT_MSG_PUT(PF_UNKNWN, "PPPoE session");
					break;
				case ETH_P_MPLS_UC:  //0x8847
					PT_MSG_PUT(PF_UNKNWN, "MPLS Unicast");
					break;
				case ETH_P_MPLS_MC:  //0x8848
					PT_MSG_PUT(PF_UNKNWN, "MPLS Multicast");
					break;
				default:
					PT_MSG_PUT(PF_UNKNWN, "unknown Ethernet type=0x%04x", cpu_to_be16(*(u16*)(llc_data+LLC_TYPE_OFF)));
					break;
				}
			} 
		} else if (is_STP(llc_data)){
			//spanning tree proto.
			PT_MSG_PUT(PF_UNKNWN, "spanning tree");
		} else {
			PT_MSG_PUT(PF_UNKNWN, "unknown LLC type=0x%08x,0x%08x", *(u32*)(llc_data), *((u32*)(llc_data)+1));
		}
		
	} else if(ieee80211_is_mgmt(fctl) && (PF_MGMT&flags)) {
		
		FRAME_PARSE(PF_MGMT, auth      );
		FRAME_PARSE(PF_MGMT, deauth    );
		FRAME_PARSE(PF_MGMT, assoc_req );
		FRAME_PARSE(PF_MGMT, assoc_resp);
		FRAME_PARSE(PF_MGMT, disassoc  );
		FRAME_PARSE(PF_MGMT, atim      );

		//for more information about action frames.
		if (FRAME_TYPE(action)) {
			struct ieee80211_mgmt *mgmt = (struct ieee80211_mgmt *)frame;
			FT_MSG_PUT(PF_MGMT, "%s", "action");

			if (mgmt->u.action.category == WLAN_CATEGORY_PUBLIC) {
				u8  *action     = (u8*)&mgmt->u.action.category;
				u32 oui	        = *(u32 *)&action[2];
				u8  oui_subtype = action[6] > 8? 9 : action[6];
				if (action[1] == 0x09 && oui == 0x099A6F50)
					FT_MSG_PUT(PF_MGMT, "(%s)", p2p_frame_type[oui_subtype]);
			} else if (mgmt->u.action.category == WLAN_CATEGORY_BACK &&
				mgmt->u.action.u.addba_req.action_code == WLAN_ACTION_ADDBA_REQ) {
				FT_MSG_PUT(PF_MGMT, "(ADDBA_REQ-%d)", mgmt->u.action.u.addba_req.start_seq_num);
			} else if (mgmt->u.action.category == WLAN_CATEGORY_BACK &&
				mgmt->u.action.u.addba_req.action_code == WLAN_ACTION_ADDBA_RESP) {
				FT_MSG_PUT(PF_MGMT, "(ADDBA_RESP-%d)", mgmt->u.action.u.addba_resp.status);
			} else {
				FT_MSG_PUT(PF_MGMT, "(%d)", mgmt->u.action.category);
			}
			goto outprint;
		}

		//too much scan results, don't print if no need.
		FRAME_PARSE(PF_SCAN, probe_req   );
		FRAME_PARSE(PF_SCAN, probe_resp  );
		FRAME_PARSE(PF_SCAN, beacon      );
		//must be last.
		FT_MSG_PUT(PF_UNKNWN, "unknown mgmt");
		
	} else if (ieee80211_is_ctl(fctl) && (PF_CTRL&flags)){
		
		flags &= (~PF_MAC_SN);  //no seq ctrl in ctrl frames.
		FRAME_PARSE(PF_CTRL, back    );
		FRAME_PARSE(PF_CTRL, back_req);
		FRAME_PARSE(PF_CTRL, ack     );
		FRAME_PARSE(PF_CTRL, rts     );
		FRAME_PARSE(PF_CTRL, cts     );
		FRAME_PARSE(PF_CTRL, pspoll  );
		//must be last.
		FT_MSG_PUT(PF_UNKNWN, "unknown ctrl");
	} else {
		FT_MSG_PUT(PF_UNKNWN, "unknown mac frame, fctl=0x%04x\n", fctl);
	}

outprint:

	FT_MSG_PUT(PF_MAC_SN, "-SN=%d(%d)", (frame->seq_ctrl>>4), (frame->seq_ctrl&0xf));

	//output all msg.
	if(IS_FRAME_PRINT || IS_PROTO_PRINT) {
		u8 *related  = NULL;
		u8 *own      = NULL;
		char *r_type = NULL;
		char *o_type = NULL;
		u8  machdrlen = ieee80211_hdrlen(fctl);
		u8 *sa  = ieee80211_get_SA(frame);
		u8 *da  = ieee80211_get_DA(frame);

		if (flags&PF_RX) {
			related = frame->addr2;
			own     = frame->addr1;
			r_type  = "TA";
			o_type  = "RA";
		} else {
			related = frame->addr1;
			own     = frame->addr2;
			r_type  = "RA";
			o_type  = "TA";
		}

		if (machdrlen >= 16) { //if ACK or BA, don't print.
			FT_MSG_PUT(PF_MACADDR, "-%02x:%02x:%02x:%02x:%02x:%02x(%s)",
			           related[0], related[1], related[2],
			           related[3], related[4], related[5],
			           r_type);
			FT_MSG_PUT(PF_OWNMAC, "-%02x:%02x:%02x:%02x:%02x:%02x(%s)",
			           own[0],own[1],own[2],own[3],own[4],own[5],
			           o_type);
			FT_MSG_PUT(PF_SA_DA, "-%02x:%02x:%02x:%02x:%02x:%02x(DA)",
			           da[0], da[1], da[2], da[3], da[4], da[5]);
			FT_MSG_PUT(PF_SA_DA, "-%02x:%02x:%02x:%02x:%02x:%02x(SA)",
			           sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
		}

		xradio_dbg(XRADIO_DBG_ALWY, "if%d-%s%s--%s\n", if_id,
		           (PF_RX&flags)?"RX-":"TX-",framebuf, protobuf);
	}
}
#endif
#undef FT_MSG_PUT
#undef PT_MSG_PUT
#undef FRAME_PARSE
#undef FRAME_TYPE

#ifdef DGB_LOG_FILE
u8  log_buffer[DGB_LOG_BUF_LEN];
u16 log_pos = 0;
struct file *fp_log   = NULL;
atomic_t     file_ref = {0};
#define T_LABEL_LEN  32
char last_time_label[T_LABEL_LEN] = {0};

int xradio_logfile(char *buffer, int buf_len, u8 b_time)
{
	int ret=-1;
	int size = buf_len;
	mm_segment_t old_fs = get_fs();

	if (!buffer)
		return ret;

	if (buf_len < 0)
		size = strlen(buffer);
	if (!size)
		return ret;

	if (atomic_add_return(1, &file_ref) == 1) {
		fp_log = filp_open(DGB_LOG_PATH0,  O_CREAT|O_WRONLY, 0666);
		if (IS_ERR(fp_log)) {
			printk(KERN_ERR "[XRADIO] ERR, can't open %s(%d).\n", 
			       DGB_LOG_PATH0, (int)fp_log);
			goto exit;
		}
	}
	//printk(KERN_ERR "[XRADIO] file_ref=%d\n", atomic_read(&file_ref));

	if(fp_log->f_op->write == NULL) {
		printk(KERN_ERR "[XRADIO] ERR, %s:File is not allow to write!\n", 
		       __FUNCTION__);
		goto exit;
	} else {
		set_fs(KERNEL_DS);
		if (fp_log->f_op->llseek != NULL) {
			vfs_llseek(fp_log, 0, SEEK_END);
		} else {
			fp_log->f_pos = 0;
		}
		if (b_time) {
			struct timeval time_now = {0};
			struct rtc_time tm;
			int    hour = 0;
			char time_label[T_LABEL_LEN] = {0};
			do_gettimeofday(&time_now);
			rtc_time_to_tm(time_now.tv_sec, &tm);
			hour = tm.tm_hour-sys_tz.tz_minuteswest/60;
			sprintf(time_label,"\n%d-%02d-%02d_%02d-%02d-%02d\n",
			        tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday+hour/24,
			        hour%24, tm.tm_min-sys_tz.tz_minuteswest%60, tm.tm_sec);
			if (memcmp(last_time_label, time_label, T_LABEL_LEN)) {
				memcpy(last_time_label, time_label, T_LABEL_LEN);
				ret = vfs_write(fp_log, time_label, strlen(time_label), &fp_log->f_pos);
			}
		}
		ret = vfs_write(fp_log, buffer, size, &fp_log->f_pos);
		set_fs(old_fs);
	}

exit:
	if (atomic_read(&file_ref) == 1) {
		if (!IS_ERR(fp_log)) {
			filp_close(fp_log, NULL);
			fp_log = (struct file *)-ENOENT;
		}
	}
	atomic_sub(1, &file_ref);
	return ret;
}
#endif

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
