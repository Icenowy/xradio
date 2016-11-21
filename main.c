/*
 * Main code of XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>

#include "xr_version.h"
#include "xradio.h"
#include "sbus.h"

MODULE_AUTHOR("XRadioTech");
MODULE_DESCRIPTION("XRadioTech WLAN driver core");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xradio_core");

char *drv_version   = XRADIO_VERSION;

/*************************************** functions ***************************************/
void xradio_version_show(void)
{
/* Show XRADIO version and compile time */
	xradio_dbg(XRADIO_DBG_ALWY, "Driver Label:%s\n", 
	           DRV_VERSION);

/************* Linux Kernel config *************/
#ifdef CONFIG_XRADIO_NON_POWER_OF_TWO_BLOCKSIZES
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_XRADIO_NON_POWER_OF_TWO_BLOCKSIZES]\n");
#endif

#ifdef CONFIG_XRADIO_USE_GPIO_IRQ
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_XRADIO_USE_GPIO_IRQ]\n");
#endif

#ifdef CONFIG_XRADIO_5GHZ_SUPPORT
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_XRADIO_5GHZ_SUPPORT]\n");
#endif

#ifdef CONFIG_XRADIO_WAPI_SUPPORT
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_XRADIO_WAPI_SUPPORT]\n");
#endif

#ifdef CONFIG_XRADIO_USE_EXTENSIONS
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_XRADIO_USE_EXTENSIONS]\n");
#endif

#ifdef CONFIG_XRADIO_USE_EXTENSIONS
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_XRADIO_USE_EXTENSIONS]\n");
#endif

#ifdef CONFIG_PM
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_PM]\n");
#endif

#ifdef CONFIG_XRADIO_SDIO
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_XRADIO_SDIO]\n");
#endif

#ifdef CONFIG_XRADIO_DUMP_ON_ERROR
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_XRADIO_DUMP_ON_ERROR]\n");
#endif

#ifdef CONFIG_XRADIO_DEBUGFS
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_XRADIO_DEBUGFS]\n");
#endif

#ifdef CONFIG_XRADIO_ITP
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_XRADIO_ITP]\n");
#endif

#ifdef CONFIG_XRADIO_TESTMODE
	xradio_dbg(XRADIO_DBG_NIY, "[CONFIG_XRADIO_TESTMODE]\n");
#endif

/************ XRADIO Make File config ************/
#ifdef P2P_MULTIVIF
	xradio_dbg(XRADIO_DBG_NIY, "[P2P_MULTIVIF]\n");
#endif

#ifdef MCAST_FWDING
	xradio_dbg(XRADIO_DBG_NIY, "[MCAST_FWDING]\n");
#endif

#ifdef XRADIO_SUSPEND_RESUME_FILTER_ENABLE
	xradio_dbg(XRADIO_DBG_NIY, "[XRADIO_SUSPEND_RESUME_FILTER_ENABLE]\n");
#endif

#ifdef AP_AGGREGATE_FW_FIX
	xradio_dbg(XRADIO_DBG_NIY, "[AP_AGGREGATE_FW_FIX]\n");
#endif

#ifdef AP_HT_CAP_UPDATE
	xradio_dbg(XRADIO_DBG_NIY, "[AP_HT_CAP_UPDATE]\n");
#endif

#ifdef PROBE_RESP_EXTRA_IE
	xradio_dbg(XRADIO_DBG_NIY, "[PROBE_RESP_EXTRA_IE]\n");
#endif

#ifdef IPV6_FILTERING
	xradio_dbg(XRADIO_DBG_NIY, "[IPV6_FILTERING]\n");
#endif

#ifdef ROAM_OFFLOAD
	xradio_dbg(XRADIO_DBG_NIY, "[ROAM_OFFLOAD]\n");
#endif

#ifdef TES_P2P_0002_ROC_RESTART
	xradio_dbg(XRADIO_DBG_NIY, "[TES_P2P_0002_ROC_RESTART]\n");
#endif

#ifdef TES_P2P_000B_EXTEND_INACTIVITY_CNT
	xradio_dbg(XRADIO_DBG_NIY, "[TES_P2P_000B_EXTEND_INACTIVITY_CNT]\n");
#endif

#ifdef TES_P2P_000B_DISABLE_EAPOL_FILTER
	xradio_dbg(XRADIO_DBG_NIY, "[TES_P2P_000B_DISABLE_EAPOL_FILTER]\n");
#endif

#ifdef HAS_PUT_TASK_STRUCT
	xradio_dbg(XRADIO_DBG_NIY, "[HAS_PUT_TASK_STRUCT]\n");
#endif

/************* XRADIO.h config *************/
#ifdef HIDDEN_SSID
	xradio_dbg(XRADIO_DBG_NIY, "[HIDDEN_SSID]\n");
#endif

#ifdef ROC_DEBUG
	xradio_dbg(XRADIO_DBG_NIY, "[ROC_DEBUG]\n");
#endif

#ifdef XRADIO_RRM
	xradio_dbg(XRADIO_DBG_NIY, "[XRADIO_RRM]\n");
#endif
}

int xradio_core_init(void)
{
	int ret = sbus_sdio_init();
	return ret;
}
EXPORT_SYMBOL_GPL(xradio_core_init);

void xradio_core_deinit(void)
{
	sbus_sdio_deinit();
}
EXPORT_SYMBOL_GPL(xradio_core_deinit);

static int __init xradio_core_entry(void)
{
	int ret;
	ret = xradio_host_dbg_init();
	ret = xradio_core_init();
	return ret;
}

static void __exit xradio_core_exit(void)
{
	xradio_core_deinit();
	xradio_host_dbg_deinit();
}

module_init(xradio_core_entry);
module_exit(xradio_core_exit);

