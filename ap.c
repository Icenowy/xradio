#include "ap.h"

int xradio_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		struct ieee80211_sta *sta) {
	wiphy_debug(hw->wiphy, "station add\n");
	return 0;
}

int xradio_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		struct ieee80211_sta *sta) {
	wiphy_debug(hw->wiphy, "station remove\n");
	return 0;
}
