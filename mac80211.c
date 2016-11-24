#include <linux/of_net.h>
#include <linux/etherdevice.h>
#include <net/mac80211.h>
#include <net/cfg80211.h>

#include "mac80211.h"
#include "txrx.h"
#include "sta.h"
#include "ap.h"
#include "queue.h"

#include "bandsandrates.h"

static const struct ieee80211_ops xradio_ops = {
// ops from txrx.c
		.tx = xradio_tx, //
// ops from sta.c
		.start = xradio_start, //
		.stop = xradio_stop, //
		.config = xradio_config, //
		.conf_tx = xradio_conf_tx, //
		.add_interface = xradio_add_interface, //
		.remove_interface = xradio_remove_interface, //
		.configure_filter = xradio_configure_filter,
// ops from ap.c
		.sta_add = xradio_sta_add, //
		.sta_remove = xradio_sta_remove, //
		};

int netif_init(struct device* dev, struct xr819** priv) {
	unsigned char randomaddr[ETH_ALEN];
	const unsigned char *addr;
	struct ieee80211_hw *hw;
	struct ieee80211_supported_band *sband;

	if (dev->of_node)
		addr = of_get_mac_address(dev->of_node);
	if (!addr) {
		dev_warn(dev, "no mac address provided, using random\n");
		eth_random_addr(randomaddr);
		addr = randomaddr;
	}

	hw = ieee80211_alloc_hw(sizeof(**priv), &xradio_ops);

	if (!hw) {
		dev_err(dev, "failed to allocate netif\n");
		return -ENOMEM;
	}
	memset(hw->priv, 0, sizeof(**priv));

	memcpy(hw->wiphy->perm_addr, addr, ETH_ALEN);

	ieee80211_hw_set(hw, SIGNAL_DBM);
	//ieee80211_hw_set(hw, SUPPORTS_PS);
	//ieee80211_hw_set(hw, SUPPORTS_DYNAMIC_PS);
	//ieee80211_hw_set(hw, REPORTS_TX_ACK_STATUS);
	//ieee80211_hw_set(hw, CONNECTION_MONITOR);

	hw->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);

	//| BIT(NL80211_IFTYPE_ADHOC) | BIT(NL80211_IFTYPE_AP)
	//| BIT(NL80211_IFTYPE_MESH_POINT) | BIT(NL80211_IFTYPE_P2P_CLIENT)
	//| BIT(NL80211_IFTYPE_P2P_GO);

	// interface combinations
	//hw->wiphy->iface_combinations = &hw_priv->if_combs[0];
	//hw->wiphy->n_iface_combinations = 1;

	// channel bands
	hw->wiphy->bands[NL80211_BAND_2GHZ] = &xradio_band_2ghz;

	// hardware queues
	hw->queues = AC_QUEUE_NUM;

	// scanning parameters
	hw->wiphy->max_scan_ssids = WSM_SCAN_MAX_NUM_OF_SSIDS;
	hw->wiphy->max_scan_ie_len = IEEE80211_MAX_DATA_LEN;

	// set mac address
	//SET_IEEE80211_PERM_ADDR(hw, hw_priv->addresses[0].addr);

	*priv = hw->priv;
	(*priv)->mac80211 = hw;
	return 0;
}

int netif_register(struct xr819* priv) {
	int err = 0;
	SET_IEEE80211_DEV(priv->mac80211, priv->dev);
	err = ieee80211_register_hw(priv->mac80211);
	if (err) {
		dev_err(priv->dev, "Cannot register network interface: %d.\n", err);
		return err;
	}
	dev_dbg(priv->dev, "network interface registered as '%s'\n",
			wiphy_name(priv->mac80211->wiphy));
	return 0;
}
