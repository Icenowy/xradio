/* TODO: use rates and channels from the device */
#define RATETAB_ENT(_rate, _rateid, _flags)		\
	{						\
		.bitrate  = (_rate),    \
		.hw_value = (_rateid),  \
		.flags    = (_flags),   \
	}

static struct ieee80211_rate xradio_rates[] = {
	RATETAB_ENT(10,  0,   0),
	RATETAB_ENT(20,  1,   0),
	RATETAB_ENT(55,  2,   0),
	RATETAB_ENT(110, 3,   0),
	RATETAB_ENT(60,  6,  0),
	RATETAB_ENT(90,  7,  0),
	RATETAB_ENT(120, 8,  0),
	RATETAB_ENT(180, 9,  0),
	RATETAB_ENT(240, 10, 0),
	RATETAB_ENT(360, 11, 0),
	RATETAB_ENT(480, 12, 0),
	RATETAB_ENT(540, 13, 0),
};

static struct ieee80211_rate xradio_mcs_rates[] = {
	RATETAB_ENT(65,  14, IEEE80211_TX_RC_MCS),
	RATETAB_ENT(130, 15, IEEE80211_TX_RC_MCS),
	RATETAB_ENT(195, 16, IEEE80211_TX_RC_MCS),
	RATETAB_ENT(260, 17, IEEE80211_TX_RC_MCS),
	RATETAB_ENT(390, 18, IEEE80211_TX_RC_MCS),
	RATETAB_ENT(520, 19, IEEE80211_TX_RC_MCS),
	RATETAB_ENT(585, 20, IEEE80211_TX_RC_MCS),
	RATETAB_ENT(650, 21, IEEE80211_TX_RC_MCS),
};

#define xradio_g_rates      (xradio_rates + 0)
#define xradio_a_rates      (xradio_rates + 4)
#define xradio_n_rates      (xradio_mcs_rates)

#define xradio_g_rates_size (ARRAY_SIZE(xradio_rates))
#define xradio_a_rates_size (ARRAY_SIZE(xradio_rates) - 4)
#define xradio_n_rates_size (ARRAY_SIZE(xradio_mcs_rates))

#define CHAN2G(_channel, _freq, _flags) {   \
	.band             = NL80211_BAND_2GHZ,  \
	.center_freq      = (_freq),              \
	.hw_value         = (_channel),           \
	.flags            = (_flags),             \
	.max_antenna_gain = 0,                    \
	.max_power        = 30,                   \
}

#define CHAN5G(_channel, _flags) {   \
	.band             = NL80211_BAND_5GHZ,     \
	.center_freq      = 5000 + (5 * (_channel)), \
	.hw_value         = (_channel),              \
	.flags            = (_flags),                \
	.max_antenna_gain = 0,                       \
	.max_power        = 30,                      \
}

static struct ieee80211_channel xradio_2ghz_chantable[] = {
	CHAN2G(1, 2412, 0),
	CHAN2G(2, 2417, 0),
	CHAN2G(3, 2422, 0),
	CHAN2G(4, 2427, 0),
	CHAN2G(5, 2432, 0),
	CHAN2G(6, 2437, 0),
	CHAN2G(7, 2442, 0),
	CHAN2G(8, 2447, 0),
	CHAN2G(9, 2452, 0),
	CHAN2G(10, 2457, 0),
	CHAN2G(11, 2462, 0),
	CHAN2G(12, 2467, 0),
	CHAN2G(13, 2472, 0),
	CHAN2G(14, 2484, 0),
};

#ifdef CONFIG_XRADIO_5GHZ_SUPPORT
static struct ieee80211_channel xradio_5ghz_chantable[] = {
	CHAN5G(34, 0),		CHAN5G(36, 0),
	CHAN5G(38, 0),		CHAN5G(40, 0),
	CHAN5G(42, 0),		CHAN5G(44, 0),
	CHAN5G(46, 0),		CHAN5G(48, 0),
	CHAN5G(52, 0),		CHAN5G(56, 0),
	CHAN5G(60, 0),		CHAN5G(64, 0),
	CHAN5G(100, 0),		CHAN5G(104, 0),
	CHAN5G(108, 0),		CHAN5G(112, 0),
	CHAN5G(116, 0),		CHAN5G(120, 0),
	CHAN5G(124, 0),		CHAN5G(128, 0),
	CHAN5G(132, 0),		CHAN5G(136, 0),
	CHAN5G(140, 0),		CHAN5G(149, 0),
	CHAN5G(153, 0),		CHAN5G(157, 0),
	CHAN5G(161, 0),		CHAN5G(165, 0),
	CHAN5G(184, 0),		CHAN5G(188, 0),
	CHAN5G(192, 0),		CHAN5G(196, 0),
	CHAN5G(200, 0),		CHAN5G(204, 0),
	CHAN5G(208, 0),		CHAN5G(212, 0),
	CHAN5G(216, 0),
};
#endif /* CONFIG_XRADIO_5GHZ_SUPPORT */

static struct ieee80211_supported_band xradio_band_2ghz = {
	.channels = xradio_2ghz_chantable,
	.n_channels = ARRAY_SIZE(xradio_2ghz_chantable),
	.bitrates = xradio_g_rates,
	.n_bitrates = xradio_g_rates_size,
	.ht_cap = {
		.cap = IEEE80211_HT_CAP_GRN_FLD |
		       (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT),
		.ht_supported  = 1,
		.ampdu_factor  = IEEE80211_HT_MAX_AMPDU_32K,
		.ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE,
		.mcs = {
			.rx_mask[0] = 0xFF,
			.rx_highest = __cpu_to_le16(0x41),
			.tx_params  = IEEE80211_HT_MCS_TX_DEFINED,
		},
	},
};

#ifdef CONFIG_XRADIO_5GHZ_SUPPORT
static struct ieee80211_supported_band xradio_band_5ghz = {
	.channels   = xradio_5ghz_chantable,
	.n_channels = ARRAY_SIZE(xradio_5ghz_chantable),
	.bitrates   = xradio_a_rates,
	.n_bitrates = xradio_a_rates_size,
	.ht_cap = {
		.cap = IEEE80211_HT_CAP_GRN_FLD |
		       (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT),
		.ht_supported  = 1,
		.ampdu_factor  = IEEE80211_HT_MAX_AMPDU_8K,
		.ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE,
		.mcs = {
			.rx_mask[0] = 0xFF,
			.rx_highest = __cpu_to_le16(0x41),
			.tx_params  = IEEE80211_HT_MCS_TX_DEFINED,
		},
	},
};
#endif /* CONFIG_XRADIO_5GHZ_SUPPORT */

static const unsigned long xradio_ttl[] = {
	1 * HZ,	/* VO */
	2 * HZ,	/* VI */
	5 * HZ, /* BE */
	10 * HZ	/* BK */
};
