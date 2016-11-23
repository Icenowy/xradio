#ifndef __XRADIO_WSM_COMMAND_H
#define __XRADIO_WSM_COMMAND_H

#include "xr819.h"
#include "wsm_mibs.h"

int wsm_write_mib(struct xr819* priv, u16 mibId, void *buf, size_t buf_size,
		int if_id);

static inline int wsm_set_host_sleep(struct xr819* priv, u8 host_sleep,
		int if_id) {
	return wsm_write_mib(priv, WSM_MIB_ID_SET_HOST_SLEEP, &host_sleep,
			sizeof(host_sleep), if_id);
}

static inline int wsm_set_output_power(struct xr819* priv, int power_level,
		int if_id) {
	__le32 val = __cpu_to_le32(power_level);
	return wsm_write_mib(priv, WSM_MIB_ID_DOT11_CURRENT_TX_POWER_LEVEL, &val,
			sizeof(val), if_id);
}

static inline int wsm_set_beacon_wakeup_period(struct xr819* priv,
		unsigned dtim_interval, unsigned listen_interval, int if_id) {
	struct {
		u8 numBeaconPeriods;
		u8 reserved;
		__le16 listenInterval;
	} val = { dtim_interval, 0, __cpu_to_le16(listen_interval) };
	if (dtim_interval > 0xFF || listen_interval > 0xFFFF)
		return -EINVAL;
	else
		return wsm_write_mib(priv, WSM_MIB_ID_BEACON_WAKEUP_PERIOD, &val,
				sizeof(val), if_id);
}

struct wsm_rcpi_rssi_threshold {
	u8 rssiRcpiMode; /* WSM_RCPI_RSSI_... */
	u8 lowerThreshold;
	u8 upperThreshold;
	u8 rollingAverageCount;
};

static inline int wsm_set_rcpi_rssi_threshold(struct xr819* priv,
		struct wsm_rcpi_rssi_threshold *arg, int if_id) {
	return wsm_write_mib(priv, WSM_MIB_ID_RCPI_RSSI_THRESHOLD, arg,
			sizeof(*arg), if_id);
}

struct wsm_switch_channel {
	/* 1 - means the STA shall not transmit any further */
	/* frames until the channel switch has completed */
	/* [in] */
	u8 channelMode;

	/* Number of TBTTs until channel switch occurs. */
	/* 0 - indicates switch shall occur at any time */
	/* 1 - occurs immediately before the next TBTT */
	/* [in] */
	u8 channelSwitchCount;

	/* The new channel number to switch to. */
	/* Note this is defined as per section 2.7. */
	/* [in] */
	u16 newChannelNumber;
};

int wsm_switch_channel(struct xr819* priv, const struct wsm_switch_channel *arg,
		int if_id);

struct wsm_tx_power_range {
	int min_power_level;
	int max_power_level;
	u32 stepping;
};

/* 3.1 */
struct wsm_configuration {
	/* [in] */
	u32 dot11MaxTransmitMsduLifeTime;
	/* [in] */
	u32 dot11MaxReceiveLifeTime;
	/* [in] */
	u32 dot11RtsThreshold;
	/* [in, out] */
	u8 *dot11StationId;
	/* [in] */
	const void *dpdData;
	/* [in] */
	size_t dpdData_size;
	/* [out] */
	u8 dot11FrequencyBandsSupported;
	/* [out] */
	u32 supportedRateMask;
	/* [out] */
	struct wsm_tx_power_range txPowerRange[2];
};

int wsm_configuration(struct xr819* priv, struct wsm_configuration *arg,
		int if_id);

#endif
