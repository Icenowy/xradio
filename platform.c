/*
 * Platform interfaces for XRadio drivers
 */

#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "xradio.h"
#include "platform.h"
#include "sbus.h"

#define GPIO 359
#define MMCHOST "1c10000.mmc"

struct mmc_host* mmchost;
struct regulator* wifireg;

int xradio_wlan_power(int on) {

	if (on) {
		printk("turning on wifi power\n");
		gpio_set_value(GPIO, 1);
		regulator_enable(wifireg);
		msleep(50);
		gpio_set_value(GPIO, 0);
		msleep(2);
		gpio_set_value(GPIO, 1);
		msleep(50);
	} else {
		printk("turning off wifi power\n");
		regulator_disable(wifireg);
	}

	mmc_detect_change(mmchost, 0);

	out: return 0;
}

int xradio_sdio_detect(int enable) {
	return 0;
}

int xradio_plat_init(void) {
	int ret = 0;
	struct device* mmchost_dev;

	mmchost_dev = bus_find_device_by_name(&platform_bus_type, NULL,
	MMCHOST);
	if (mmchost_dev == NULL) {
		printk("failed to get mmc host device\n");
		goto out;
	}

	mmchost = platform_get_drvdata(to_platform_device(mmchost_dev));
	if (mmchost == NULL) {
		printk("failed to get mmc host\n");
		goto out;
	}

	wifireg = regulator_get(NULL, "wifi");

	ret = gpio_request(GPIO, "wifi_rst");
	if (ret) {
		printk("failed to get reset gpio\n");
		goto out;
	}

	gpio_direction_output(GPIO, 1);

	out: return ret;
}

void xradio_plat_deinit(void) {
	gpio_set_value(GPIO, 0);
	gpio_free(GPIO);
	regulator_disable(wifireg);
	regulator_put(wifireg);
}
