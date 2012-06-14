# bcmdhd
DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                     \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DWLBTAMP -DBCMFILEIMAGE  \
	-DDHDTHREAD -DDHD_GPL -DDHD_SCHED -DDHD_DEBUG -DSDTEST -DBDC -DTOE    \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DDONGLEOVERLAYS -DBCMDBG               \
	-DCUSTOMER_HW2    \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
	-DNEW_COMPAT_WIRELESS -DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT          \
	-DKEEP_ALIVE -DPKT_FILTER_SUPPORT     \
	-DEMBEDDED_PLATFORM           \
	-DSET_RANDOM_MAC_SOFTAP -DWL_CFG80211_STA_EVENT                       \
	-Idrivers/net/wireless/bcmdhd -Idrivers/net/wireless/bcmdhd/include

ifeq ($(CONFIG_BCMDHD_WIFI_CONTROL_FUNC),y)
DHDCFLAGS += -DCONFIG_WIFI_CONTROL_FUNC
else
DHDCFLAGS += -DCUSTOM_OOB_GPIO_NUM=2
endif

ifeq ($(CONFIG_BCMDHD_HW_OOB),y)
DHDCFLAGS += -DHW_OOB -DOOB_INTR_ONLY
else
DHDCFLAGS += -DSDIO_ISR_THREAD
endif

ifeq ($(CONFIG_BCMDHD_CSCAN_ENABLE),y)
DHDCFLAGS += -DCSCAN -DPNO_SUPPORT
endif

ifeq ($(CONFIG_BCMDHD_INSMOD_NO_FW_LOAD),y)
DHDCFLAGS += -DENABLE_INSMOD_NO_FW_LOAD
endif

ifeq ($(CONFIG_BCMDHD_CUSTOM_REGULATORY_DOMAIN),y)
DHDCFLAGS += -DENABLE_CUSTOM_REGULATORY_DOMAIN
endif

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o bcmwifi.o dhd_sdio.o bcmevent.o dhd_bta.o hndpmu.o  \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o linux_osl.o            \
	bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o dhd_cfg80211.o

obj-$(CONFIG_BCMDHD) += bcmdhd.o
bcmdhd-objs += $(DHDOFILES)

ifeq ($(CONFIG_BCMDHD_WEXT),y)
bcmdhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP -DWL_WIRELESS_EXT
endif

ifneq ($(CONFIG_CFG80211),)
bcmdhd-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o
DHDCFLAGS += -DWL_CFG80211
endif
ifneq ($(CONFIG_DHD_USE_SCHED_SCAN),)
DHDCFLAGS += -DWL_SCHED_SCAN
endif
ifneq ($(CONFIG_DHD_ENABLE_P2P),)
DHDCFLAGS += -DWL_ENABLE_P2P_IF
endif
EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCMDHD),m)
EXTRA_LDFLAGS += --strip-debug
endif
