CONFIG_XRADIO := m
CONFIG_XRADIO_SDIO := y
CONFIG_XRADIO_USE_EXTENSIONS := y
CONFIG_XRADIO_5GHZ_SUPPORT := y

xradio_wlan-y := \
	fwio.o \
	txrx.o \
	main.o \
	queue.o \
	hwio.o \
	bh.o \
	wsm.o \
	sta.o \
	ap.o \
	scan.o \
	debug.o \
	module.o

xradio_wlan-$(CONFIG_PM)		+= pm.o
xradio_wlan-$(CONFIG_XRADIO_SDIO)	+= sdio.o
xradio_wlan-$(CONFIG_XRADIO_ITP)	+= itp.o

ccflags-y += -DP2P_MULTIVIF
ccflags-y += -DMCAST_FWDING
ccflags-y += -DXRADIO_SUSPEND_RESUME_FILTER_ENABLE
ccflags-y += -DAP_AGGREGATE_FW_FIX
ccflags-y += -DAP_HT_CAP_UPDATE
ccflags-y += -DHW_ERROR_WIFI_RESET
ccflags-y += -DAP_HT_COMPAT_FIX
#ccflags-y += -DCONFIG_XRADIO_DEBUG
ccflags-y += -DCONFIG_XRADIO_DUMP_ON_ERROR
ccflags-y += -DCONFIG_XRADIO_DEBUGFS
ccflags-y += -DCONFIG_XRADIO_NON_POWER_OF_TWO_BLOCKSIZES

ccflags-y += -DCONFIG_XRADIO_SUSPEND_POWER_OFF
# Use vfs for firmware load when request_firmware 
# can't work on other platform.
# ccflags-y += -DUSE_VFS_FIRMWARE

# Extra IE for probe response from upper layer is needed in P2P GO
# For offloading probe response to FW, the extra IE must be included
# in the probe response template
ccflags-y += -DPROBE_RESP_EXTRA_IE
ccflags-y += -DIPV6_FILTERING

# Modified by wzw
ccflags-y += -DTES_P2P_0002_ROC_RESTART
ccflags-y += -DTES_P2P_000B_EXTEND_INACTIVITY_CNT
ccflags-y += -DTES_P2P_000B_DISABLE_EAPOL_FILTER
ccflags-y += -DXRADIO_USE_LONG_DTIM_PERIOD
ccflags-y += -DXRADIO_USE_LONG_KEEP_ALIVE_PERIOD

# Use semaphore to sync bh txrx.
#ccflags-y += -DBH_USE_SEMAPHORE

#ccflags-y += -DDEBUG

ldflags-y += --strip-debug

obj-$(CONFIG_XRADIO) += xradio_wlan.o

