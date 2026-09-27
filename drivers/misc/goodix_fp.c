// SPDX-License-Identifier: GPL-2.0
/*
 * Goodix gf_spi fingerprint sensor SPI driver (rubens, MT6895).
 *
 * The sensor is a Goodix capacitive fingerprint chip on the MTK SPI3_A pads
 * (CLK=GPIO217, CS=GPIO218, MI=GPIO219, MO=GPIO220), powered from the 3.3 V
 * mt6368_vfp rail with a hardware reset GPIO and a finger-detect IRQ GPIO.
 *
 * All fingerprint logic (enroll / identify / template DB) lives in the vendor
 * "goodix-fp" trustlet (thh/ta/8888c03f....ta), which the userspace helper
 * (ta_host) runs as an ordinary aarch64 ELF with the TEE imports stubbed.
 * This driver is deliberately dumb: it only provides the SPI transport the
 * trustlet needs through ut_pf_spi_send_and_receive plus power/reset and
 * finger-IRQ waiting.
 *
 * Userspace interface: /dev/goodix_fp (misc device, see <linux/goodix_fp.h>):
 *   ioctl()   POWER / RESET / WAIT_FINGER / FINGER_STATE / RAW_XFER
 * plus sysfs attributes "power" and "finger" on the SPI device.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <linux/goodix_fp.h>

#define GOODIX_FP_DRV_NAME      "goodix_fp"

struct goodix_fp {
	struct device *dev;
	struct spi_device *spi;
	struct mutex lock;

	struct regulator *vdd;
	struct gpio_desc *reset;
	struct gpio_desc *irq;
	int irq_num;

	struct miscdevice misc;

	/* finger-detect state + wait queue */
	atomic_t finger;
	wait_queue_head_t finger_wq;
};

/* ------------------------------------------------------------------------- */
/* power / reset                                                             */
/* ------------------------------------------------------------------------- */

static void goodix_fp_reset_pulse(struct goodix_fp *f)
{
	/*
	 * Reset is asserted LOW on this part and the sensor runs with the line
	 * left HIGH, the same convention the fpc1540 driver uses on these very
	 * pads (assert low, release high, then idle high).
	 */
	gpiod_set_value_cansleep(f->reset, 1);
	udelay(100);
	gpiod_set_value_cansleep(f->reset, 0);
	udelay(1000);
	gpiod_set_value_cansleep(f->reset, 1);
	udelay(1250);
}

static int goodix_fp_power_on(struct goodix_fp *f)
{
	int ret;

	if (!f->vdd)
		return 0;

	/*
	 * xaga/xagapro ship an FPC or a Goodix sensor on the same SPI3_A
	 * footprint; the FPC one is fed from mt6368_vibr at 1.8 V, the Goodix
	 * one from mt6368_vfp at 3.3 V.
	 */
	ret = regulator_set_voltage(f->vdd, 3300000, 3300000);
	if (ret)
		return ret;

	ret = regulator_enable(f->vdd);
	if (ret)
		return ret;
	msleep(20);

	if (f->reset)
		goodix_fp_reset_pulse(f);

	msleep(10);
	return 0;
}

static int goodix_fp_power_off(struct goodix_fp *f)
{
	/* assert reset (drive the line low) before dropping the rail */
	if (f->reset)
		gpiod_set_value_cansleep(f->reset, 0);
	if (f->vdd)
		regulator_disable(f->vdd);
	msleep(20);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* finger IRQ                                                                */
/* ------------------------------------------------------------------------- */

static irqreturn_t goodix_fp_irq_thread(int irq, void *data)
{
	struct goodix_fp *f = data;

	atomic_set(&f->finger, 1);
	wake_up_interruptible(&f->finger_wq);
	return IRQ_HANDLED;
}

static int goodix_fp_request_irq(struct goodix_fp *f)
{
	unsigned long flags = IRQF_ONESHOT;
	int irq;

	if (f->irq) {
		/* dedicated irq-gpios property */
		irq = gpiod_to_irq(f->irq);
		if (irq < 0)
			return irq;
		flags |= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	} else if (f->spi->irq > 0) {
		/* the EINT is described with the standard "interrupts" property,
		 * which already carries the trigger type */
		irq = f->spi->irq;
	} else {
		return 0;
	}
	f->irq_num = irq;

	return devm_request_threaded_irq(f->dev, irq, NULL, goodix_fp_irq_thread,
					 flags, dev_name(f->dev), f);
}

/* ------------------------------------------------------------------------- */
/* raw SPI transfer                                                          */
/* ------------------------------------------------------------------------- */

static int goodix_fp_raw_transfer(struct goodix_fp *f, const u8 *tx, u32 tx_len,
				  u8 *rx, u32 rx_len)
{
	struct spi_transfer t[2];
	int n = 0, ret;

	/*
	 * The trustlet's platform primitive (ut_pf_spi_send_and_receive) is full
	 * duplex: it clocks the same number of bytes out of TX and into RX in one
	 * transfer.  That matters, because the sensor's answer stays byte aligned
	 * with the command (rx[0] is the byte clocked while the command byte went
	 * out).  Driving TX and RX as two separate phases instead makes the reply
	 * arrive late by however many bytes we pushed out first, which slides it
	 * around the capture window.
	 */
	if (tx_len && tx_len == rx_len) {
		struct spi_transfer full = {
			.tx_buf = tx,
			.rx_buf = rx,
			.len = tx_len,
		};

		mutex_lock(&f->lock);
		ret = spi_sync_transfer(f->spi, &full, 1);
		mutex_unlock(&f->lock);
		return ret;
	}

	memset(t, 0, sizeof(t));
	if (tx_len) {
		t[n].tx_buf = tx;
		t[n].len = tx_len;
		/* Small delay keeps CS asserted between tx and rx like the
		 * vendor TEE platform layer does. */
		t[n].delay.value = 2;
		t[n].delay.unit = SPI_DELAY_UNIT_USECS;
		n++;
	}
	if (rx_len) {
		t[n].rx_buf = rx;
		t[n].len = rx_len;
		n++;
	}
	if (!n)
		return 0;

	mutex_lock(&f->lock);
	ret = spi_sync_transfer(f->spi, t, n);
	mutex_unlock(&f->lock);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* ioctl                                                                     */
/* ------------------------------------------------------------------------- */

static long goodix_fp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct goodix_fp *f = file->private_data;
	void __user *uarg = (void __user *)arg;
	int ret = 0;

	switch (cmd) {
	case GOODIX_FP_IOC_POWER: {
		u32 mode;

		if (get_user(mode, (u32 __user *)arg))
			return -EFAULT;
		mutex_lock(&f->lock);
		switch (mode) {
		case GOODIX_FP_POWER_OFF:
			ret = goodix_fp_power_off(f);
			break;
		case GOODIX_FP_POWER_ON:
			ret = goodix_fp_power_on(f);
			break;
		case GOODIX_FP_POWER_CYCLE:
			goodix_fp_power_off(f);
			ret = goodix_fp_power_on(f);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		mutex_unlock(&f->lock);
		return ret;
	}

	case GOODIX_FP_IOC_RESET:
		mutex_lock(&f->lock);
		if (f->reset)
			goodix_fp_reset_pulse(f);
		mutex_unlock(&f->lock);
		return 0;

	case GOODIX_FP_IOC_WAIT_FINGER: {
		u32 timeout;

		if (get_user(timeout, (u32 __user *)arg))
			return -EFAULT;
		atomic_set(&f->finger, 0);
		if (!wait_event_interruptible_timeout(f->finger_wq,
					atomic_read(&f->finger) != 0,
					msecs_to_jiffies(timeout)))
			return -ETIMEDOUT;
		return put_user((u32)atomic_read(&f->finger), (u32 __user *)arg);
	}

	case GOODIX_FP_IOC_FINGER_STATE:
		return put_user((u32)atomic_read(&f->finger), (u32 __user *)arg);

	case GOODIX_FP_IOC_RAW_XFER: {
		struct goodix_fp_raw_xfer x;
		u8 *buf;
		u32 total;

		if (copy_from_user(&x, uarg, sizeof(x)))
			return -EFAULT;
		if (x.tx_len > GOODIX_FP_RAW_MAX || x.rx_len > GOODIX_FP_RAW_MAX)
			return -EINVAL;
		total = x.tx_len + x.rx_len;
		if (!total)
			return 0;
		buf = kzalloc(total, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
		if (x.tx_len &&
		    copy_from_user(buf, (void __user *)(uintptr_t)x.tx, x.tx_len)) {
			kfree(buf);
			return -EFAULT;
		}
		ret = goodix_fp_raw_transfer(f, buf, x.tx_len,
					    buf + x.tx_len, x.rx_len);
		if (!ret && x.rx_len &&
		    copy_to_user((void __user *)(uintptr_t)x.rx,
				 buf + x.tx_len, x.rx_len))
			ret = -EFAULT;
		kfree(buf);
		return ret;
	}
	}

	return -ENOTTY;
}

static int goodix_fp_open(struct inode *inode, struct file *file)
{
	struct goodix_fp *f = container_of(file->private_data,
					   struct goodix_fp, misc);

	file->private_data = f;
	return 0;
}

static const struct file_operations goodix_fp_fops = {
	.owner		= THIS_MODULE,
	.open		= goodix_fp_open,
	.unlocked_ioctl = goodix_fp_ioctl,
	.llseek		= noop_llseek,
};

/* ------------------------------------------------------------------------- */
/* sysfs                                                                     */
/* ------------------------------------------------------------------------- */

static ssize_t finger_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct goodix_fp *f = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", atomic_read(&f->finger));
}
static DEVICE_ATTR_RO(finger);

static ssize_t powered_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct goodix_fp *f = dev_get_drvdata(dev);

	if (!f->vdd)
		return sysfs_emit(buf, "0\n");
	return sysfs_emit(buf, "%d\n", regulator_is_enabled(f->vdd) > 0);
}

static ssize_t powered_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct goodix_fp *f = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	mutex_lock(&f->lock);
	ret = on ? goodix_fp_power_on(f) : goodix_fp_power_off(f);
	mutex_unlock(&f->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(powered);

/*
 * Bring-up helpers.  The stock DT only says GPIO216 is "reset", not which
 * electrical level releases the sensor, and mt6368_vfp can be driven anywhere
 * between 1.2 V and 3.5 V.  Both knobs are exposed so the level and the rail
 * voltage can be swept from userspace while the chip ID register is polled,
 * rather than guessed at with a reflash per attempt.
 */
static ssize_t reset_level_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct goodix_fp *f = dev_get_drvdata(dev);

	if (!f->reset)
		return sysfs_emit(buf, "-1\n");
	return sysfs_emit(buf, "%d\n", gpiod_get_value_cansleep(f->reset));
}

static ssize_t reset_level_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct goodix_fp *f = dev_get_drvdata(dev);
	int v, ret;

	ret = kstrtoint(buf, 10, &v);
	if (ret)
		return ret;
	if (f->reset)
		gpiod_set_value_cansleep(f->reset, v ? 1 : 0);
	return count;
}
static DEVICE_ATTR_RW(reset_level);

static ssize_t vdd_microvolts_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct goodix_fp *f = dev_get_drvdata(dev);
	int ret;

	if (!f->vdd)
		return sysfs_emit(buf, "-1\n");
	ret = regulator_get_voltage(f->vdd);
	return sysfs_emit(buf, "%d\n", ret < 0 ? 0 : ret);
}

static ssize_t vdd_microvolts_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct goodix_fp *f = dev_get_drvdata(dev);
	int v, ret;

	ret = kstrtoint(buf, 10, &v);
	if (ret)
		return ret;
	if (!f->vdd)
		return -ENODEV;
	ret = regulator_set_voltage(f->vdd, v, v);
	if (ret)
		return ret;
	return count;
}
static DEVICE_ATTR_RW(vdd_microvolts);

/*
 * The stock DT fixes neither the SPI clock phase nor the bus speed for this
 * part, and both are easy to get wrong: a sensor sampled on the wrong edge
 * still clocks data back, it just comes out as aliased noise.  Both are
 * therefore settable at runtime while a register is polled.
 */
static ssize_t spi_mode_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct goodix_fp *f = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", (unsigned int)(f->spi->mode & SPI_MODE_X_MASK));
}

static ssize_t spi_mode_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct goodix_fp *f = dev_get_drvdata(dev);
	u32 v;
	int ret;

	if (kstrtou32(buf, 10, &v) || v > SPI_MODE_X_MASK)
		return -EINVAL;

	mutex_lock(&f->lock);
	f->spi->mode = v;
	ret = spi_setup(f->spi);
	mutex_unlock(&f->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(spi_mode);

static ssize_t speed_hz_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct goodix_fp *f = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", (unsigned int)f->spi->max_speed_hz);
}

static ssize_t speed_hz_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct goodix_fp *f = dev_get_drvdata(dev);
	u32 v;
	int ret;

	if (kstrtou32(buf, 10, &v) || !v)
		return -EINVAL;

	mutex_lock(&f->lock);
	f->spi->max_speed_hz = v;
	ret = spi_setup(f->spi);
	mutex_unlock(&f->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(speed_hz);

static struct attribute *goodix_fp_attrs[] = {
	&dev_attr_finger.attr,
	&dev_attr_powered.attr,
	&dev_attr_reset_level.attr,
	&dev_attr_vdd_microvolts.attr,
	&dev_attr_spi_mode.attr,
	&dev_attr_speed_hz.attr,
	NULL,
};

static const struct attribute_group goodix_fp_group = {
	.attrs = goodix_fp_attrs,
};

/* ------------------------------------------------------------------------- */
/* probe / remove                                                            */
/* ------------------------------------------------------------------------- */

static int goodix_fp_probe(struct spi_device *spi)
{
	struct goodix_fp *f;
	int ret;

	f = devm_kzalloc(&spi->dev, sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;

	f->dev = &spi->dev;
	f->spi = spi;
	spi_set_drvdata(spi, f);
	dev_set_drvdata(f->dev, f);

	mutex_init(&f->lock);
	init_waitqueue_head(&f->finger_wq);
	atomic_set(&f->finger, 0);

	/* Default, mirroring the working rubens FPC setup. */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	if (!spi->max_speed_hz)
		spi->max_speed_hz = 1000000;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(f->dev, ret, "spi_setup failed\n");

	f->vdd = devm_regulator_get_optional(f->dev, "vdd");
	if (IS_ERR(f->vdd)) {
		if (PTR_ERR(f->vdd) != -ENODEV)
			return dev_err_probe(f->dev, PTR_ERR(f->vdd), "no vdd\n");
		f->vdd = NULL;
	}

	f->reset = devm_gpiod_get_optional(f->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(f->reset))
		return dev_err_probe(f->dev, PTR_ERR(f->reset), "reset gpio\n");

	f->irq = devm_gpiod_get_optional(f->dev, "irq", GPIOD_IN);
	if (IS_ERR(f->irq))
		return dev_err_probe(f->dev, PTR_ERR(f->irq), "irq gpio\n");

	ret = goodix_fp_power_on(f);
	if (ret)
		return dev_err_probe(f->dev, ret, "power on failed\n");

	ret = devm_device_add_group(f->dev, &goodix_fp_group);
	if (ret)
		return dev_err_probe(f->dev, ret, "sysfs group failed\n");

	f->misc.name = GOODIX_FP_DRV_NAME;
	f->misc.minor = MISC_DYNAMIC_MINOR;
	f->misc.fops = &goodix_fp_fops;
	f->misc.parent = f->dev;
	ret = misc_register(&f->misc);
	if (ret)
		return dev_err_probe(f->dev, ret, "misc_register failed\n");

	ret = goodix_fp_request_irq(f);
	if (ret)
		dev_warn(f->dev, "finger IRQ unavailable (%d)\n", ret);

	dev_info(f->dev, "Goodix gf_spi fingerprint sensor ready\n");
	return 0;
}

static void goodix_fp_remove(struct spi_device *spi)
{
	struct goodix_fp *f = spi_get_drvdata(spi);

	goodix_fp_power_off(f);
	misc_deregister(&f->misc);
}

static const struct spi_device_id goodix_fp_id[] = {
	{ "gf-spi", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, goodix_fp_id);

static const struct of_device_id goodix_fp_of_match[] = {
	{ .compatible = "goodix,gf-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, goodix_fp_of_match);

static struct spi_driver goodix_fp_driver = {
	.driver = {
		.name = GOODIX_FP_DRV_NAME,
		.of_match_table = goodix_fp_of_match,
	},
	.id_table = goodix_fp_id,
	.probe = goodix_fp_probe,
	.remove = goodix_fp_remove,
};
module_spi_driver(goodix_fp_driver);

MODULE_DESCRIPTION("Goodix gf_spi fingerprint sensor SPI driver");
MODULE_AUTHOR("rubens bring-up");
MODULE_LICENSE("GPL");