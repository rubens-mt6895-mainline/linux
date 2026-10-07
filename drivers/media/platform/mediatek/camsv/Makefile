# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_VIDEO_MTK_CAMSV) += cam_cap.o

# The pixel pipeline in this driver is the frame-rate ceiling of the two fast
# sensor modes, so it is compiled the same way the shipped module is.
CFLAGS_cam_cap.o := -O3
