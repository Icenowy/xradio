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
#include "sdio.h"

MODULE_AUTHOR("XRadioTech");
MODULE_DESCRIPTION("XRadioTech WLAN driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xradio_core");

static int __init xradio_core_entry(void)
{
	int ret = sbus_sdio_init();
	return ret;
}

static void __exit xradio_core_exit(void)
{
	sbus_sdio_deinit();
}

module_init( xradio_core_entry);
module_exit( xradio_core_exit);
