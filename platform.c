/*
 * Platform interfaces for XRadio drivers
 * 
 * Implemented by platform vendor(such as AllwinnerTech).
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
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/ioport.h>

#include <linux/regulator/consumer.h>

#include "xradio.h"
#include "platform.h"
#include "sbus.h"

int xradio_wlan_power(int on)
{
	return 0;
}

int xradio_sdio_detect(int enable)
{
	return 0;
}

int xradio_plat_init(void)
{
	return 0;
}

void xradio_plat_deinit(void)
{
}
