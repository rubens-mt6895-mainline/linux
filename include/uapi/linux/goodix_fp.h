/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Userspace API for the Goodix gf_spi fingerprint sensor SPI driver.
 *
 * The sensor is a Goodix capacitive fingerprint chip (gf3xxx family) on the
 * MTK SPI3_A pads.  All matching logic lives in the vendor trustlet which
 * runs as an ordinary aarch64 ELF in userspace; this driver is only a raw
 * SPI transport bridge plus power/reset/IRQ control.
 */
#ifndef _UAPI_LINUX_GOODIX_FP_H
#define _UAPI_LINUX_GOODIX_FP_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define GOODIX_FP_IOC_MAGIC     0xfd

/* Maximum single raw SPI transfer the ioctl accepts (arbitrary, matches
 * the trustlet's largest transaction). */
#define GOODIX_FP_RAW_MAX       262144

/* Power modes for GOODIX_FP_IOC_POWER */
#define GOODIX_FP_POWER_OFF     0
#define GOODIX_FP_POWER_ON      1
#define GOODIX_FP_POWER_CYCLE   2

/**
 * struct goodix_fp_raw_xfer - raw SPI transfer: transmit then receive
 * @tx_len: bytes to transmit (may be 0)
 * @rx_len: bytes to receive after the transmit (may be 0)
 * @tx:     user pointer to tx_len bytes
 * @rx:     user pointer to rx_len bytes
 *
 * The same primitive the vendor TEE platform layer (ut_pf_spi_send_and_receive
 * in the goodix-fp trustlet) would use against the sensor.
 */
struct goodix_fp_raw_xfer {
	__u32 tx_len;
	__u32 rx_len;
	__u64 tx;
	__u64 rx;
};

#define GOODIX_FP_IOC_POWER         _IOW(GOODIX_FP_IOC_MAGIC, 0x01, __u32)
#define GOODIX_FP_IOC_RESET         _IO(GOODIX_FP_IOC_MAGIC, 0x02)
#define GOODIX_FP_IOC_WAIT_FINGER    _IOWR(GOODIX_FP_IOC_MAGIC, 0x03, __u32)
#define GOODIX_FP_IOC_FINGER_STATE   _IOR(GOODIX_FP_IOC_MAGIC, 0x04, __u32)
#define GOODIX_FP_IOC_RAW_XFER       _IOWR(GOODIX_FP_IOC_MAGIC, 0x0a, struct goodix_fp_raw_xfer)

#define GOODIX_FP_IOC_MAXNR          0x0a

#endif /* _UAPI_LINUX_GOODIX_FP_H */