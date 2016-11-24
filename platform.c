/*
 * Platform interfaces for XRadio drivers
 */

int xradio_wlan_power(int on) {

	return 0;
}

int xradio_sdio_detect(int enable) {
	return 0;
}

int xradio_plat_init(void) {
	int ret = 0;
	return ret;
}

void xradio_plat_deinit(void) {
}
