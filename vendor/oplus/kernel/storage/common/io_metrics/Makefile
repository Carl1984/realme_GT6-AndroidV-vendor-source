LINUXINCLUDE += -I$(srctree)/

ifeq ($(CONFIG_OPLUS_FEATURE_STORAGE_IO_METRICS_QCOM),y)
ccflags-y += -DSOC_PLATFORM_IS_QCOM
else ifeq ($(CONFIG_OPLUS_FEATURE_STORAGE_IO_METRICS_MTK),y)
ccflags-y += -DSOC_PLATFORM_IS_MTK
endif

ifeq ($(CONFIG_OPLUS_FEATURE_STORAGE_IO_METRICS_DEBUG),y)
ccflags-y += -DCONFIG_OPLUS_FEATURE_STORAGE_IO_METRICS_DEBUG=1
endif

ifeq ($(OPLUS_OUT_OF_TREE_KO),y)
ccflags-y += -DCONFIG_OPLUS_FEATURE_STORAGE_IO_METRICS=1
endif

ifeq ($(CONFIG_OPLUS_FEATURE_STORAGE_IO_METRICS),y)
CONFIG_OPLUS_FEATURE_STORAGE_F2FS=y
endif

obj-$(CONFIG_OPLUS_FEATURE_STORAGE_IO_METRICS) += oplus_bsp_storage_io_metrics.o
oplus_bsp_storage_io_metrics-y += procfs.o
oplus_bsp_storage_io_metrics-y += io_metrics_entry.o
oplus_bsp_storage_io_metrics-y += block_metrics.o
oplus_bsp_storage_io_metrics-$(CONFIG_OPLUS_FEATURE_STORAGE_F2FS) += f2fs_metrics.o
oplus_bsp_storage_io_metrics-y += ufs_metrics.o
oplus_bsp_storage_io_metrics-y += abnormal_io.o
