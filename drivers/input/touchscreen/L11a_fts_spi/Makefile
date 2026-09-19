# SPDX-License-Identifier: GPL-2.0
#
# FocalTech / ST FTS v5.2.1 SPI touchscreen controller
# (Xiaomi Redmi K50 / rubens, l11a panel, "st_fts_L11a.ftb" firmware).
#
obj-$(CONFIG_TOUCHSCREEN_ST_FTS_V521_SPI_L11A) += fts_touch_spi.o

fts_touch_spi-y := fts.o \
		   fts_proc.o \
		   fts_lib/ftsCompensation.o \
		   fts_lib/ftsCore.o \
		   fts_lib/ftsError.o \
		   fts_lib/ftsFrame.o \
		   fts_lib/ftsIO.o \
		   fts_lib/ftsTest.o \
		   fts_lib/ftsTool.o \
		   fts_lib/ftsFlash.o \
		   fts_lib/ftsGesture.o
