/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

/**
 *  @addtogroup  DRIVERS
 *  @brief       Hardware drivers.
 *
 *  @{
 *      @file    inv_gyro.c
 *      @brief   A sysfs device driver for Invensense devices
 *      @details This driver currently works for the ITG3500, MPU6050, MPU9150
 *               MPU3050
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include "inv_mpu_iio.h"
#include "../../sysfs.h"
#define CHECK_DMP	do \
	{ \
		if ((st->chip_config.is_asleep) || \
		(0 == st->chip_config.firmware_loaded)) \
			return -EPERM; \
		result = kstrtoul(buf, 10, (long unsigned int *)&data); \
		if (result) \
			return result; \
	} while (0);
static void inv_setup_reg(struct inv_reg_map_s *reg)
{
	reg->who_am_i		= 0x75;
	reg->sample_rate_div	= 0x19;
	reg->lpf		= 0x1A;
	reg->product_id		= 0x0C;
	reg->bank_sel		= 0x6D;
	reg->user_ctrl		= 0x6A;
	reg->fifo_en		= 0x23;
	reg->gyro_config	= 0x1B;
	reg->accl_config	= 0x1C;
	reg->fifo_count_h	= 0x72;
	reg->fifo_r_w		= 0x74;
	reg->raw_gyro		= 0x43;
	reg->raw_accl		= 0x3B;
	reg->temperature	= 0x41;
	reg->int_enable		= 0x38;
	reg->int_status		= 0x3A;
	reg->pwr_mgmt_1		= 0x6B;
	reg->pwr_mgmt_2		= 0x6C;
	reg->mem_start_addr	= 0x6E;
	reg->mem_r_w		= 0x6F;
	reg->prgm_strt_addrh	= 0x70;
};
static const struct inv_hw_s hw_info[INV_NUM_PARTS] = {
	{119, "ITG3500"},
	{ 63, "MPU3050"},
	{117, "MPU6050"},
	{118, "MPU9150"}
};
/**
 *  inv_i2c_read() - Read one or more bytes from the device registers.
 *  @st:	Device driver instance.
 *  @reg:	First device register to be read from.
 *  @length:	Number of bytes to read.
 *  @data:	Data read from device.
 *  NOTE: The slave register will not increment when reading from the FIFO.
 */
int inv_i2c_read_base(struct inv_gyro_state_s *st, unsigned short i2c_addr,
	unsigned char reg, unsigned short length, unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (!data)
		return -EINVAL;

	msgs[0].addr = i2c_addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;

	msgs[1].addr = i2c_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = length;

	res = i2c_transfer(st->sl_handle, msgs, 2);
	if (res < 2) {
		if (res >= 0)
			res = -EIO;
		return res;
	} else
		return 0;
}

/**
 *  inv_i2c_single_write() - Write a byte to a device register.
 *  @st:	Device driver instance.
 *  @reg:	Device register to be written to.
 *  @data:	Byte to write to device.
 */
int inv_i2c_single_write_base(struct inv_gyro_state_s *st,
	unsigned short i2c_addr, unsigned char reg, unsigned char data)
{
	unsigned char tmp[2];
	struct i2c_msg msg;
	int res;

	tmp[0] = reg;
	tmp[1] = data;

	msg.addr = i2c_addr;
	msg.flags = 0;	/* write */
	msg.buf = tmp;
	msg.len = 2;

	/*printk(KERN_ERR "WS%02X%02X%02X\n", i2c_addr, reg, data);*/
	res = i2c_transfer(st->sl_handle, &msg, 1);
	if (res < 1) {
		if (res == 0)
			res = -EIO;
		return res;
	} else
		return 0;
}
static int set_power_itg(struct inv_gyro_state_s *st,
	unsigned char power_on)
{
	struct inv_reg_map_s *reg;
	unsigned char data;
	int result;

	reg = &st->reg;
	if (power_on)
		data = 0;
	else
		data = BIT_SLEEP;
	if (st->chip_config.lpa_mode)
		data |= BIT_CYCLE;
	if (st->chip_config.gyro_enable) {
		result = inv_i2c_single_write(st,
			reg->pwr_mgmt_1, data | INV_CLK_PLL);
		if (result)
			return result;
		st->chip_config.clk_src = INV_CLK_PLL;
	} else {
		result = inv_i2c_single_write(st,
			reg->pwr_mgmt_1, data | INV_CLK_INTERNAL);
		if (result)
			return result;
		st->chip_config.clk_src = INV_CLK_INTERNAL;
	}

	if (power_on) {
		msleep(POWER_UP_TIME);
		data = 0;
		if (0 == st->chip_config.accl_enable)
			data |= BIT_PWR_ACCL_STBY;
		if (0 == st->chip_config.gyro_enable)
			data |= BIT_PWR_GYRO_STBY;
		data |= (st->chip_config.lpa_freq << LPA_FREQ_SHIFT);

		result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
		if (result)
			return result;
		msleep(POWER_UP_TIME);
		st->chip_config.is_asleep = 0;
	} else
		st->chip_config.is_asleep = 1;
	return 0;
}
/**
 *  inv_set_power_state() - Turn device on/off.
 *  @st:	Device driver instance.
 *  @power_on:	1 to turn on, 0 to suspend.
 */
int inv_set_power_state(struct inv_gyro_state_s *st,
	unsigned char power_on)
{
	if (INV_MPU3050 == st->chip_type)
		return set_power_mpu3050(st, power_on);
	else
		return set_power_itg(st, power_on);
	return 0;
}

/**
 *  inv_init_config() - Initialize hardware, disable FIFO.
 *  @indio_dev:	Device driver instance.
 *  Initial configuration:
 *  FSR: +/- 2000DPS
 *  DLPF: 42Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 */
static int inv_init_config(struct iio_dev *indio_dev)
{
	struct inv_reg_map_s *reg;
	int result;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	if (st->chip_config.is_asleep)
		return -EPERM;
	reg = &st->reg;
	result = set_inv_enable(indio_dev, 0);
	if (result)
		return result;

	result = inv_i2c_single_write(st, reg->gyro_config,
		INV_FSR_2000DPS << GYRO_CONFIG_FSR_SHIFT);
	if (result)
		return result;
	st->chip_config.fsr = INV_FSR_2000DPS;

	result = inv_i2c_single_write(st, reg->lpf, INV_FILTER_42HZ);
	if (result)
		return result;
	st->chip_config.lpf = INV_FILTER_42HZ;

	result = inv_i2c_single_write(st, reg->sample_rate_div,
					ONE_K_HZ/INIT_FIFO_RATE - 1);
	if (result)
		return result;
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG,
				st->plat_data.int_config & (~BIT_BYPASS_EN));
	if (result)
		return result;
	st->chip_config.fifo_rate = INIT_FIFO_RATE;
	st->irq_dur_us            = INIT_DUR_TIME;
	st->chip_config.prog_start_addr = DMP_START_ADDR;
	st->chip_config.gyro_enable = 1;
	st->chip_config.gyro_fifo_enable = 1;
	if (INV_ITG3500 != st->chip_type) {
		st->chip_config.accl_enable = 1;
		st->chip_config.accl_fifo_enable = 1;
		st->chip_config.accl_fs = INV_FS_02G;
		result = inv_i2c_single_write(st, reg->accl_config,
			(INV_FS_02G << ACCL_CONFIG_FSR_SHIFT));
		if (result)
			return result;
		st->tap.time = INIT_TAP_TIME;
		st->tap.thresh = INIT_TAP_THRESHOLD;
		st->tap.min_count = INIT_TAP_MIN_COUNT;
	}
	return 0;
}
/**
 *  inv_compass_scale_show() - show compass scale.
 */
static int inv_compass_scale_show(struct inv_gyro_state_s *st, int *scale)
{
	if (COMPASS_ID_AK8975 == st->plat_data.sec_slave_id)
		*scale = DATA_AKM8975_SCALE;
	else if (COMPASS_ID_AK8972 == st->plat_data.sec_slave_id)
		*scale = DATA_AKM8972_SCALE;
	else if (COMPASS_ID_AK8963 == st->plat_data.sec_slave_id)
		if (st->compass_scale)
			*scale = DATA_AKM8963_SCALE1;
		else
			*scale = DATA_AKM8963_SCALE0;
	else
		return -EINVAL;
	*scale *= (1L << 15);
	return IIO_VAL_INT;
}

/**
 *  mpu_read_raw() - read raw method.
 */
static int mpu_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask) {
	struct inv_gyro_state_s  *st = iio_priv(indio_dev);
	int result;
	if (st->chip_config.is_asleep)
		return -EINVAL;
	switch (mask) {
	case 0:
		if (chan->type == IIO_ANGL_VEL) {
			*val = st->raw_gyro[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}
		if (chan->type == IIO_ACCEL) {
			*val = st->raw_accel[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}
		if (chan->type == IIO_MAGN) {
			*val = st->raw_compass[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}
		return -EINVAL;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_ANGL_VEL) {
			*val = (1 << st->chip_config.fsr)*GYRO_DPS_SCALE;
			return IIO_VAL_INT;
		}
		if (chan->type == IIO_ACCEL) {
			*val = (2 << st->chip_config.accl_fs);
			return IIO_VAL_INT;
		}
		if (chan->type == IIO_MAGN)
			return inv_compass_scale_show(st, val);
		return -EINVAL;
	case IIO_CHAN_INFO_CALIBBIAS:
		if (st->chip_config.self_test_run_once == 0) {
			result = inv_do_test(st, 0,  st->gyro_bias,
				st->accel_bias);
			if (result)
				return result;
			st->chip_config.self_test_run_once = 1;
		}

		if (chan->type == IIO_ANGL_VEL) {
			*val = st->gyro_bias[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}
		if (chan->type == IIO_ACCEL) {
			*val = st->accel_bias[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

/**
 *  inv_write_fsr() - Configure the gyro's scale range.
 */
static int inv_write_fsr(struct inv_gyro_state_s *st, int fsr)
{
	struct inv_reg_map_s *reg;
	int result;
	reg = &st->reg;
	if ((fsr < 0) || (fsr > MAX_GYRO_FS_PARAM))
		return -EINVAL;
	if (fsr == st->chip_config.fsr)
		return 0;

	if (INV_MPU3050 == st->chip_type) {
		result = inv_i2c_single_write(st, reg->lpf,
			(fsr << GYRO_CONFIG_FSR_SHIFT) | st->chip_config.lpf);
	} else {
		result = inv_i2c_single_write(st, reg->gyro_config,
			fsr << GYRO_CONFIG_FSR_SHIFT);
	}
	if (result)
		return result;
	st->chip_config.fsr = fsr;
	return 0;
}

/**
 *  inv_write_accel_fs() - Configure the accelerometer's scale range.
 */
static int inv_write_accel_fs(struct inv_gyro_state_s *st, int fs)
{
	int result;
	struct inv_reg_map_s *reg;
	reg = &st->reg;

	if (fs < 0 || fs > MAX_ACCL_FS_PARAM)
		return -EINVAL;
	if (fs == st->chip_config.accl_fs)
		return 0;
	if (INV_MPU3050 == st->chip_type) {
		result = st->mpu_slave->set_fs(st, fs);
		if (result)
			return result;
	} else {
		result = inv_i2c_single_write(st, reg->accl_config,
				(fs << ACCL_CONFIG_FSR_SHIFT));
		if (result)
			return result;
	}
	/* reset fifo because the data could be mixed with old bad data */
	st->chip_config.accl_fs = fs;
	return 0;
}
/**
 *  inv_write_compass_scale() - Configure the compass's scale range.
 */
static int inv_write_compass_scale(struct inv_gyro_state_s  *st, int data)
{
	char d, en;
	int result;
	if (COMPASS_ID_AK8963 != st->plat_data.sec_slave_id)
		return 0;
	if (data)
		en = 1;
	else
		en = 0;
	if (st->compass_scale == en)
		return 0;
	d = (1 | (st->compass_scale << AKM8963_SCALE_SHIFT));
	result = inv_i2c_single_write(st, REG_I2C_SLV1_DO, d);
	if (result)
		return result;
	st->compass_scale = en;
	return 0;

}

/**
 *  mpu_write_raw() - write raw method.
 */
static int mpu_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask) {
	struct inv_gyro_state_s  *st = iio_priv(indio_dev);
	int result;
	if (st->chip_config.is_asleep)
		return -EPERM;
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		result = -EINVAL;
		if (chan->type == IIO_ANGL_VEL)
			result = inv_write_fsr(st, val);
		if (chan->type == IIO_ACCEL)
			result = inv_write_accel_fs(st, val);
		if (chan->type == IIO_MAGN)
			result = inv_write_compass_scale(st, val);
		return result;
	default:
		return -EINVAL;
	}
	return 0;
}

/**
 *  inv_set_lpf() - set low pass filer based on fifo rate.
 */
static int inv_set_lpf(struct inv_gyro_state_s *st, int rate)
{
	const short hz[] = {188, 98, 42, 20, 10, 5};
	const int   d[] = {INV_FILTER_188HZ, INV_FILTER_98HZ,
			INV_FILTER_42HZ, INV_FILTER_20HZ,
			INV_FILTER_10HZ, INV_FILTER_5HZ};
	int i, h, data, result;
	struct inv_reg_map_s *reg;
	reg = &st->reg;
	h = (rate >> 1);
	i = 0;
	while ((h < hz[i]) && (i < ARRAY_SIZE(d)))
		i++;
	if (i == ARRAY_SIZE(d))
		i -= 1;
	data = d[i];
	if (INV_MPU3050 == st->chip_type) {
		if (st->mpu_slave != NULL) {
			result = st->mpu_slave->set_lpf(st, rate);
			if (result)
				return result;
		}
		result = inv_i2c_single_write(st, reg->lpf, data |
			(st->chip_config.fsr << GYRO_CONFIG_FSR_SHIFT));
		if (result)
			return result;
	} else
		result = inv_i2c_single_write(st, reg->lpf, data);
	if (result)
		return result;
	st->chip_config.lpf = data;
	return 0;
}

/**
 *  inv_fifo_rate_store() - Set fifo rate.
 */
static ssize_t inv_fifo_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long fifo_rate;
	unsigned char data;
	int result;
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	reg = &st->reg;

	if (st->chip_config.is_asleep)
		return -EPERM;
	if (kstrtoul(buf, 10, &fifo_rate))
		return -EINVAL;
	if ((fifo_rate < MIN_FIFO_RATE) || (fifo_rate > MAX_FIFO_RATE))
		return -EINVAL;
	if (fifo_rate == st->chip_config.fifo_rate)
		return count;
	if (st->chip_config.has_compass) {
		data = COMPASS_RATE_SCALE*fifo_rate/ONE_K_HZ;
		if (data > 0)
			data -= 1;
		st->compass_divider = data;
		st->compass_counter = 0;
		/* I2C_MST_DLY is set according to sample rate,
		   AKM cannot be read or set at sample rate higher than 100Hz*/
		result = inv_i2c_single_write(st, REG_I2C_SLV4_CTRL, data);
		if (result)
			return result;
	}
	data = ONE_K_HZ / fifo_rate - 1;
	result = inv_i2c_single_write(st, reg->sample_rate_div, data);
	if (result)
		return result;
	st->chip_config.fifo_rate = fifo_rate;
	result = inv_set_lpf(st, fifo_rate);
	if (result)
		return result;
	st->irq_dur_us = (data + 1) * ONE_K_HZ;
	st->last_isr_time = iio_get_time_ns();
	return count;
}
/**
 *  inv_fifo_rate_show() - Get the current sampling rate.
 */
static ssize_t inv_fifo_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.fifo_rate);
}

/**
 *  inv_power_state_store() - Turn device on/off.
 */
static ssize_t inv_power_state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int result;
	unsigned long power_state;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	if (kstrtoul(buf, 10, &power_state))
		return -EINVAL;
	if (!power_state == st->chip_config.is_asleep)
		return count;
	result = inv_set_power_state(st, power_state);
	return count;
}

/**
 *  inv_power_state_show() - Check if the device is on or in sleep mode.
 */
static ssize_t inv_power_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->chip_config.is_asleep)
		return sprintf(buf, "0\n");
	else
		return sprintf(buf, "1\n");
}

/**
 * inv_firmware_loaded_store() -  calling this function will change
 *                        firmware load
 */
static ssize_t inv_firmware_loaded_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned long data, result;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;
	if (data != 0)
		return -EINVAL;
	st->chip_config.firmware_loaded = 0;
	st->chip_config.dmp_on = 0;
	st->chip_config.quaternion_on = 0;
	return count;
}
/**
 * inv_firmware_loaded_show() -  calling this function will show current
 *                        firmware load status
 */
static ssize_t inv_firmware_loaded_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.firmware_loaded);
}

/**
 *  inv_lpa_mode_store() - store current low power settings
 */
static ssize_t inv_lpa_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned long result, lpa_mode;
	unsigned char d;
	struct inv_reg_map_s *reg;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &lpa_mode);
	if (result)
		return result;

	reg = &st->reg;
	result = inv_i2c_read(st, reg->pwr_mgmt_1, 1, &d);
	if (result)
		return result;
	d &= ~BIT_CYCLE;
	if (lpa_mode)
		d |= BIT_CYCLE;
	result = inv_i2c_single_write(st, reg->pwr_mgmt_1, d);
	if (result)
		return result;
	st->chip_config.lpa_mode = lpa_mode;
	return count;
}
/**
 *  inv_lpa_mode_show() - show current low power settings
 */
static ssize_t inv_lpa_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.lpa_mode);
}

/**
 *  inv_lpa_freq_store() - store current low power frequency setting.
 */
static ssize_t inv_lpa_freq_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned long result, lpa_freq;
	unsigned char d;
	struct inv_reg_map_s *reg;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &lpa_freq);
	if (result)
		return result;
	if (lpa_freq > MAX_LPA_FREQ_PARAM)
		return -EINVAL;
	reg = &st->reg;
	result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &d);
	if (result)
		return result;
	d &= ~BIT_LPA_FREQ;
	d |= (unsigned char)(lpa_freq << LPA_FREQ_SHIFT);
	result = inv_i2c_single_write(st, reg->pwr_mgmt_2, d);
	if (result)
		return result;
	st->chip_config.lpa_freq = lpa_freq;
	return count;
}
/**
 *  inv_lpa_freq_show() - show current low power frequency setting
 */
static ssize_t inv_lpa_freq_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	switch (st->chip_config.lpa_freq) {
	case 0:
		return sprintf(buf, "1.25\n");
	case 1:
		return sprintf(buf, "5\n");
	case 2:
		return sprintf(buf, "20\n");
	case 3:
		return sprintf(buf, "40\n");
	default:
		return sprintf(buf, "0\n");
	}
}
/**
 * inv_dmp_on_store() -  calling this function will store current dmp on
 */
static ssize_t inv_dmp_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	st->chip_config.dmp_on = !!data;
	return count;
}

/**
 * inv_dmp_on_show() -  calling this function will show current dmp_on
 */
static ssize_t inv_dmp_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.dmp_on);
}
/**
 * inv_dmp_int_on_store() -  calling this function will store current dmp int on
 */
static ssize_t inv_dmp_int_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	st->chip_config.dmp_int_on = !!data;
	return count;
}

/**
 * inv_dmp_int_on_show() -  calling this function will show current dmp_int_on
 */
static ssize_t inv_dmp_int_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.dmp_int_on);
}

/**
 * inv_dmp_output_rate_store() -  calling this function store dmp_output_rate
 */
static ssize_t inv_dmp_output_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	unsigned int result, data;
	st = iio_priv(indio_dev);

	CHECK_DMP
	if (0 == data)
		return -EINVAL;
	result = inv_set_fifo_rate(st, data);
	if (result)
		return result;
	st->chip_config.dmp_output_rate = data;
	return count;
}

/**
 * inv_dmp_output_rate_show() -  calling this shows dmp_output_rate
 */
static ssize_t inv_dmp_output_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.dmp_output_rate);
}

/**
 * inv_orientation_on_store() -  calling this function will store
 *                                 current orientation on
 */
static ssize_t inv_orientation_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data, en;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	en = !!data;
	result = inv_enable_orientation_dmp(st, en);
	if (result)
		return result;
	st->chip_config.orientation_on = en;
	return count;
}
/**
 * inv_orientation_on_show() -  calling this function will show
 *				current orientation_on
 */
static ssize_t inv_orientation_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.orientation_on);
}

/**
 * inv_display_orient_on_store() -  calling this function will store
 *                                 current display_orient on
 */
static ssize_t inv_display_orient_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data, en;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	en = !!data;
	result = inv_set_display_orient_interrupt_dmp(st, en);
	if (result)
		return result;
	st->chip_config.display_orient_on = en;
	return count;
}
/**
 * inv_display_orient_on_show() -  calling this function will show
 *				current display_orient_on
 */
static ssize_t inv_display_orient_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.display_orient_on);
}

/**
 * inv_quaternion_on_store() -  calling this function will store
 *                                 current quaternion on
 */
static ssize_t inv_quaternion_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data, en;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct iio_buffer *ring = indio_dev->buffer;
	st = iio_priv(indio_dev);

	CHECK_DMP
	en = !!data;
	result = inv_send_quaternion(st, en);
	if (result)
		return result;
	st->chip_config.quaternion_on = en;
	if (0 == en) {
		clear_bit(INV_MPU_SCAN_QUAT_R, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_QUAT_X, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_QUAT_Y, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_QUAT_Z, ring->scan_mask);
	}

	return count;
}
/**
 * inv_quaternion_on_show() -  calling this function will show
 *				current orientation_on
 */
static ssize_t inv_quaternion_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.quaternion_on);
}

/**
 * inv_tap_on_store() -  calling this function will store current tap on
 */
static ssize_t inv_tap_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	st->chip_config.tap_on = !!data;
	result = inv_enable_tap_dmp(st, st->chip_config.tap_on);
	return count;
}

/**
 * inv_tap_on_show() -  calling this function will show current tap_on
 */
static ssize_t inv_tap_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.tap_on);
}
/**
 * inv_tap_time_store() -  calling this function will store current tap time
 */
static ssize_t inv_tap_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	result = inv_set_tap_time_dmp(st, data);
	if (result)
		return result;
	st->tap.time = data;
	return count;
}
/**
 * inv_tap_time_show() -  calling this function will show current tap time
 */
static ssize_t inv_tap_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->tap.time);
}

/**
 * inv_tap_min_count_store() -  calling this function will store tap count
 */
static ssize_t inv_tap_min_count_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	result = inv_set_min_taps_dmp(st, data);
	if (result)
		return result;
	st->tap.min_count = data;
	return count;
}
/**
 * inv_tap_min_count_show() -  calling this function show minimum count
 */
static ssize_t inv_tap_min_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->tap.min_count);
}

/**
 * inv_tap_threshold_store() -  calling this function will store tap threshold
 */
static ssize_t inv_tap_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	result = inv_set_tap_threshold_dmp(st, INV_TAP_AXIS_X, data);
	if (result)
		return result;
	result = inv_set_tap_threshold_dmp(st, INV_TAP_AXIS_Y, data);
	if (result)
		return result;
	result = inv_set_tap_threshold_dmp(st, INV_TAP_AXIS_Z, data);
	if (result)
		return result;

	st->tap.thresh = data;
	return count;
}
/**
 * inv_tap_thresh_show() -  calling this function show current tap threshold
 */
static ssize_t inv_tap_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->tap.thresh);
}
/**
 *  inv_clk_src_show() - Show the device's clock source.
 */
static ssize_t inv_clk_src_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	switch (st->chip_config.clk_src) {
	case INV_CLK_INTERNAL:
		return sprintf(buf, "INTERNAL\n");
	case INV_CLK_PLL:
		return sprintf(buf, "Gyro PLL\n");
	default:
		return -EPERM;
	}
}
/**
 *  inv_reg_dump_show() - Register dump for testing.
 *  TODO: Only for testing.
 */
static ssize_t inv_reg_dump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ii;
	char data;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	ssize_t bytes_printed = 0;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	for (ii = 0; ii < st->hw->num_reg; ii++) {
		inv_i2c_read(st, ii, 1, &data);
		bytes_printed += sprintf(buf+bytes_printed, "%#2x: %#2x\n",
			ii, data);
	}
	return bytes_printed;
}

/**
 * inv_self_test_show() - self test result. 0 for fail; 1 for success.
 *                        calling this function will trigger self test
 *                        and return test result.
 */
static ssize_t inv_self_test_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int result;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (INV_MPU3050 == st->chip_type)
		result = 0;
	else
		result = inv_hw_self_test(st);
	return sprintf(buf, "%d\n", result);
}
/**
 * inv_key_show() -  calling this function will show the key
 *
 */
static ssize_t inv_key_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	unsigned char *key;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	key = st->plat_data.key;
	return sprintf(buf,
	"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		key[0],  key[1],  key[2],  key[3],
		key[4],  key[5],  key[6],  key[7],
		key[8],  key[9],  key[10], key[11],
		key[12], key[13], key[14], key[15]);
}
/**
 * inv_gyro_matrix_show() - show orientation matrix
 */
static ssize_t inv_gyro_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	signed char *m;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	m = st->plat_data.orientation;
	return sprintf(buf,
	"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}
/**
 * inv_accl_matrix_show() - show orientation matrix
 */
static ssize_t inv_accl_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	signed char *m;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->plat_data.sec_slave_type == SECONDARY_SLAVE_TYPE_ACCEL)
		m = st->plat_data.secondary_orientation;
	else
		m = st->plat_data.orientation;
	return sprintf(buf,
	"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}
/**
 * inv_compass_matrix_show() - show orientation matrix
 */
static ssize_t inv_compass_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	signed char *m;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->plat_data.sec_slave_type == SECONDARY_SLAVE_TYPE_COMPASS)
		m = st->plat_data.secondary_orientation;
	else
		return -1;
	return sprintf(buf,
	"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}

/**
 * inv_flick_lower_store() -  calling this function will store current
 *                        flick lower bound
 */
static ssize_t inv_flick_lower_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtol(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;

	result = mem_w_key(KEY_FLICK_LOWER, 4, p);
	if (result)
		return result;
	st->flick.lower = data;
	return count;
}

/**
 * inv_flick_lower_show() -  calling this function will show current
 *                        flick lower bound
 */
static ssize_t inv_flick_lower_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->flick.lower);
}
/**
 * inv_flick_upper_store() -  calling this function will store current
 *                        flick upper bound
 */
static ssize_t inv_flick_upper_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_UPPER, 4, p);
	if (result)
		return result;
	st->flick.upper = data;
	return count;
}

/**
 * inv_flick_upper_show() -  calling this function will show current
 *                        flick upper bound
 */
static ssize_t inv_flick_upper_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->flick.upper);
}
/**
 * inv_flick_counter_store() -  calling this function will store current
 *                        flick counter value
 */
static ssize_t inv_flick_counter_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_COUNTER, 4, p);
	if (result)
		return result;
	st->flick.counter = data;

	return count;
}

/**
 * inv_flick_counter_show() -  calling this function will show current
 *                        flick counter value
 */
static ssize_t inv_flick_counter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->flick.counter);
}

/**
 * inv_flick_int_on_store() -  calling this function will store current
 *                        flick interrupt on value
 */
static ssize_t inv_flick_int_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long result, data;
	unsigned char d[4];
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;
	if (data)
		/* Use interrupt to signal when gesture was observed */
		d[0] = DIND40+4;
	else
		d[0] = DINAA0+8;
	result = mem_w_key(KEY_CGNOTICE_INTR, 1, d);
	if (result)
		return result;
	st->chip_config.flick_int_on = data;
	return count;
}

/**
 * inv_flick_int_on_show() -  calling this function will show current
 *                        flick interrupt on value
 */
static ssize_t inv_flick_int_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.flick_int_on);
}
/**
 * inv_flick_axis_store() -  calling this function will store current
 *                        flick axis value
 */
static ssize_t inv_flick_axis_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long result, data;
	unsigned char d[4];
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;

	if (data == 0)
		d[0] = DINBC2;
	else if (data == 2)
		d[2] = DINBC6;
	else
		d[0] = DINBC4;
	result = mem_w_key(KEY_CFG_FLICK_IN, 1, d);
	if (result)
		return result;
	st->flick.axis = data;

	return count;
}

/**
 * inv_flick_axis_show() -  calling this function will show current
 *                        flick axis value
 */
static ssize_t inv_flick_axis_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->flick.axis);
}
/**
 * inv_flick_msg_on_store() -  calling this function will store current
 *                        flick message on value
 */
static ssize_t inv_flick_msg_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	if (data)
		data = DATA_MSG_ON;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_MSG, 4, p);
	if (result)
		return result;
	st->flick.msg_on = data;

	return count;
}

/**
 * inv_flick_msg_on_show() -  calling this function will show current
 *                        flick message on value
 */
static ssize_t inv_flick_msg_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->flick.msg_on);
}

/**
 * inv_pedometer_steps_store() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_steps_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_D_PEDSTD_STEPCTR, 4, p);
	if (result)
		return result;

	return count;
}

/**
 * inv_pedometer_steps_show() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_steps_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int result, data;
	unsigned char d[4];
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = mpu_memory_read(st->sl_handle, st->i2c_addr,
		inv_dmp_get_address(KEY_D_PEDSTD_STEPCTR), 4, d);
	if (result)
		return result;
	data = be32_to_cpup((int *)d);
	return sprintf(buf, "%d\n", data);
}
/**
 * inv_pedometer_time_store() -  calling this function will store current
 *                        pedometer time into MPU memory
 */
static ssize_t inv_pedometer_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_D_PEDSTD_TIMECTR, 4, p);
	if (result)
		return result;
	return count;
}
/**
 * inv_pedometer_time_show() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int result, data;
	unsigned char d[4];
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = mpu_memory_read(st->sl_handle, st->i2c_addr,
		inv_dmp_get_address(KEY_D_PEDSTD_TIMECTR), 4, d);
	if (result)
		return result;
	data = be32_to_cpup((int *)d);
	return sprintf(buf, "%d\n", data*20);
}

/**
 * inv_dmp_flick_show() -  calling this function will show flick event.
 *                         This event must use poll.
 */
static ssize_t inv_dmp_flick_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}
/**
 * inv_dmp_orient_show() -  calling this function will show orientation
 *                         This event must use poll.
 */
static ssize_t inv_dmp_orient_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->orient_data);
}

/**
 * inv_dmp_display_orient_show() -  calling this function will
 *			show orientation This event must use poll.
 */
static ssize_t inv_dmp_display_orient_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->display_orient_data);
}

/**
 * inv_dmp_tap_show() -  calling this function will show tap
 *                         This event must use poll.
 */
static ssize_t inv_dmp_tap_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->tap_data);
}
/**
 *  inv_temperature_show() - Read temperature data directly from registers.
 */
static ssize_t inv_temperature_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct inv_reg_map_s *reg;
	int result;
	short temp;
	long scale_t;
	unsigned char data[2];
	reg = &st->reg;

	if (st->chip_config.is_asleep)
		return -EPERM;
	result = inv_i2c_read(st, reg->temperature, 2, data);
	if (result) {
		printk(KERN_ERR "Could not read temperature register.\n");
		return result;
	}
	temp = (signed short)(be16_to_cpup((short *)&data[0]));

	if (INV_MPU3050 == st->chip_type)
		scale_t = MPU3050_TEMP_OFFSET +
			inv_q30_mult((long)temp << MPU_TEMP_SHIFT,
				MPU3050_TEMP_SCALE);
	else
		scale_t = MPU6050_TEMP_OFFSET +
			inv_q30_mult((long)temp << MPU_TEMP_SHIFT,
				MPU6050_TEMP_SCALE);
	return sprintf(buf, "%ld %lld\n", scale_t, iio_get_time_ns());
}
static int inv_switch_gyro_engine(struct inv_gyro_state_s *st, int en)
{
	struct inv_reg_map_s *reg;
	unsigned char data, p;
	int result;
	reg = &st->reg;
	if (INV_MPU3050 == st->chip_type) {
		if (en) {
			data = INV_CLK_PLL;
			p = (BITS_3050_POWER1 | data);
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
			p = (BITS_3050_POWER2 | data);
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
			p = data;
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
		} else {
			p = BITS_3050_GYRO_STANDBY;
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
		}
	} else {
		result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &data);
		if (result)
			return result;
		if (en)
			data &= (~BIT_PWR_GYRO_STBY);
		else
			data |= BIT_PWR_GYRO_STBY;
		result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
		if (result)
			return result;
		msleep(SENSOR_UP_TIME);
	}
	if (en)
		st->chip_config.clk_src = INV_CLK_PLL;
	else
		st->chip_config.clk_src = INV_CLK_INTERNAL;

	return 0;
}
static int inv_switch_accl_engine(struct inv_gyro_state_s *st, int en)
{
	struct inv_reg_map_s *reg;
	unsigned char data;
	int result;
	reg = &st->reg;
	if (INV_MPU3050 == st->chip_type) {
		if (NULL == st->mpu_slave)
			return -EPERM;
		if (en)
			result = st->mpu_slave->resume(st);
		else
			result = st->mpu_slave->suspend(st);
		if (result)
			return result;
	} else {
		result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &data);
		if (result)
			return result;
		if (en)
			data &= (~BIT_PWR_ACCL_STBY);
		else
			data |= BIT_PWR_ACCL_STBY;
		result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
		if (result)
			return result;
		msleep(SENSOR_UP_TIME);
	}
	return 0;
}

/**
 *  inv_gyro_enable_store() - Enable/disable gyro.
 */
static ssize_t inv_gyro_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data,  en;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	int result;

	if (st->chip_config.is_asleep)
		return -EPERM;
	if (st->chip_config.enable)
		return -EPERM;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return -EINVAL;
	en = !!data;
	if (en == st->chip_config.gyro_enable)
		return count;
	result = inv_switch_gyro_engine(st, en);
	if (result)
		return result;

	if (0 == en) {
		st->chip_config.gyro_fifo_enable = 0;
		clear_bit(INV_MPU_SCAN_GYRO_X, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_GYRO_Y, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_GYRO_Z, ring->scan_mask);
	}
	st->chip_config.gyro_enable = en;
	return count;
}
/**
 *  inv_gyro_enable_show() - Check if the FIFO and ring buffer are enabled.
 */
static ssize_t inv_gyro_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.gyro_enable);
}

/**
 *  inv_accl_enable_store() - Enable/disable accl.
 */
static ssize_t inv_accl_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long en, data;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	int result;

	if (st->chip_config.is_asleep)
		return -EPERM;
	if (st->chip_config.enable)
		return -EPERM;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return -EINVAL;
	if (data)
		en = 1;
	else
		en = 0;
	if (en == st->chip_config.accl_enable)
		return count;
	result = inv_switch_accl_engine(st, en);
	if (result)
		return result;
	st->chip_config.accl_enable = en;
	if (0 == en) {
		st->chip_config.accl_fifo_enable = 0;
		clear_bit(INV_MPU_SCAN_ACCL_X, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_ACCL_Y, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_ACCL_Z, ring->scan_mask);
	}
	return count;
}
/**
 *  inv_accl_enable_show() - Check if the FIFO and ring buffer are enabled.
 */
static ssize_t inv_accl_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.accl_enable);
}

/**
 * inv_compass_en_store() -  calling this function will store compass
 *                         enable
 */
static ssize_t inv_compass_en_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, result, en;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	if (st->chip_config.is_asleep)
		return -EPERM;
	if (st->chip_config.enable)
		return -EPERM;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;
	if (data)
		en = 1;
	else
		en = 0;
	if (en == st->chip_config.compass_enable)
		return count;
	st->chip_config.compass_enable = en;
	if (0 == en) {
		st->chip_config.compass_fifo_enable = 0;
		clear_bit(INV_MPU_SCAN_MAGN_X, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_MAGN_Y, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_MAGN_Z, ring->scan_mask);
	}

	return count;
}
/**
 * inv_compass_en_show() -  calling this function will show compass
 *                         enable status
 */
static ssize_t inv_compass_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.compass_enable);
}

static const struct iio_chan_spec gyro_channels[] = {
	/*there is only one gyro, with modifier X, Y, Z
	So it is not indexed. no modifier name, only simple, x, y,z
	the scale should be shared while bias is not so each
	axis has different bias*/
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	},
	IIO_CHAN_SOFT_TIMESTAMP(INV_MPU_SCAN_TIMESTAMP)
};

static const struct iio_chan_spec gyro_accel_channels[] = {
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_R,
		.scan_index = INV_MPU_SCAN_QUAT_R,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.scan_index = INV_MPU_SCAN_QUAT_X,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.scan_index = INV_MPU_SCAN_QUAT_Y,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.scan_index = INV_MPU_SCAN_QUAT_Z,
		.scan_type = IIO_ST('s', 32, 32, 0)
	},
	IIO_CHAN_SOFT_TIMESTAMP(INV_MPU_SCAN_TIMESTAMP)
};
static const struct iio_chan_spec gyro_accel_compass_channels[] = {
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_MAGN_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_MAGN_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_MAGN_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	},
	{
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_R,
		.scan_index = INV_MPU_SCAN_QUAT_R,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.scan_index = INV_MPU_SCAN_QUAT_X,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.scan_index = INV_MPU_SCAN_QUAT_Y,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.scan_index = INV_MPU_SCAN_QUAT_Z,
		.scan_type = IIO_ST('s', 32, 32, 0)
	},
	IIO_CHAN_SOFT_TIMESTAMP(INV_MPU_SCAN_TIMESTAMP)
};

static struct inv_chip_chan_info chip_channel_info[] = {
	{
		.channels = gyro_channels,
		.num_channels = ARRAY_SIZE(gyro_channels),
	},
	{
		.channels = gyro_accel_channels,
		.num_channels = ARRAY_SIZE(gyro_accel_channels),
	},
	{
		.channels = gyro_accel_compass_channels,
		.num_channels = ARRAY_SIZE(gyro_accel_compass_channels),
	}
};

/*constant IIO attribute */
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("10 50 100 200 500");
static IIO_DEV_ATTR_SAMP_FREQ(S_IRUGO | S_IWUSR, inv_fifo_rate_show,
	inv_fifo_rate_store);
static DEVICE_ATTR(temperature, S_IRUGO, inv_temperature_show, NULL);
static DEVICE_ATTR(clock_source, S_IRUGO, inv_clk_src_show, NULL);
static DEVICE_ATTR(power_state, S_IRUGO | S_IWUSR, inv_power_state_show,
	inv_power_state_store);
static DEVICE_ATTR(firmware_loaded, S_IRUGO | S_IWUSR,
	inv_firmware_loaded_show, inv_firmware_loaded_store);
static DEVICE_ATTR(lpa_mode, S_IRUGO | S_IWUSR, inv_lpa_mode_show,
	inv_lpa_mode_store);
static DEVICE_ATTR(lpa_freq, S_IRUGO | S_IWUSR, inv_lpa_freq_show,
	inv_lpa_freq_store);
static DEVICE_ATTR(reg_dump, S_IRUGO, inv_reg_dump_show, NULL);
static DEVICE_ATTR(self_test, S_IRUGO, inv_self_test_show, NULL);
static DEVICE_ATTR(key, S_IRUGO, inv_key_show, NULL);
static DEVICE_ATTR(gyro_matrix, S_IRUGO, inv_gyro_matrix_show, NULL);
static DEVICE_ATTR(accl_matrix, S_IRUGO, inv_accl_matrix_show, NULL);
static DEVICE_ATTR(compass_matrix, S_IRUGO, inv_compass_matrix_show, NULL);
static DEVICE_ATTR(flick_lower, S_IRUGO | S_IWUSR, inv_flick_lower_show,
	inv_flick_lower_store);
static DEVICE_ATTR(flick_upper, S_IRUGO | S_IWUSR, inv_flick_upper_show,
	inv_flick_upper_store);
static DEVICE_ATTR(flick_counter, S_IRUGO | S_IWUSR, inv_flick_counter_show,
	inv_flick_counter_store);
static DEVICE_ATTR(flick_message_on, S_IRUGO | S_IWUSR, inv_flick_msg_on_show,
	inv_flick_msg_on_store);
static DEVICE_ATTR(flick_int_on, S_IRUGO | S_IWUSR, inv_flick_int_on_show,
	inv_flick_int_on_store);
static DEVICE_ATTR(flick_axis, S_IRUGO | S_IWUSR, inv_flick_axis_show,
	inv_flick_axis_store);
static DEVICE_ATTR(dmp_on, S_IRUGO | S_IWUSR, inv_dmp_on_show,
	inv_dmp_on_store);
static DEVICE_ATTR(dmp_int_on, S_IRUGO | S_IWUSR, inv_dmp_int_on_show,
	inv_dmp_int_on_store);
static DEVICE_ATTR(dmp_output_rate, S_IRUGO | S_IWUSR,
	inv_dmp_output_rate_show, inv_dmp_output_rate_store);
static DEVICE_ATTR(orientation_on, S_IRUGO | S_IWUSR,
	inv_orientation_on_show, inv_orientation_on_store);
static DEVICE_ATTR(quaternion_on, S_IRUGO | S_IWUSR,
	inv_quaternion_on_show, inv_quaternion_on_store);
static DEVICE_ATTR(display_orientation_on, S_IRUGO | S_IWUSR,
	inv_display_orient_on_show, inv_display_orient_on_store);
static DEVICE_ATTR(tap_on, S_IRUGO | S_IWUSR, inv_tap_on_show,
	inv_tap_on_store);
static DEVICE_ATTR(tap_time, S_IRUGO | S_IWUSR, inv_tap_time_show,
	inv_tap_time_store);
static DEVICE_ATTR(tap_min_count, S_IRUGO | S_IWUSR, inv_tap_min_count_show,
	inv_tap_min_count_store);
static DEVICE_ATTR(tap_threshold, S_IRUGO | S_IWUSR, inv_tap_threshold_show,
	inv_tap_threshold_store);
static DEVICE_ATTR(pedometer_time, S_IRUGO | S_IWUSR, inv_pedometer_time_show,
	inv_pedometer_time_store);
static DEVICE_ATTR(pedometer_steps, S_IRUGO | S_IWUSR,
		inv_pedometer_steps_show, inv_pedometer_steps_store);
static DEVICE_ATTR(event_flick, S_IRUGO, inv_dmp_flick_show, NULL);
static DEVICE_ATTR(event_orientation, S_IRUGO, inv_dmp_orient_show, NULL);
static DEVICE_ATTR(event_tap, S_IRUGO, inv_dmp_tap_show, NULL);
static DEVICE_ATTR(event_display_orientation, S_IRUGO,
			inv_dmp_display_orient_show, NULL);
static DEVICE_ATTR(gyro_enable, S_IRUGO | S_IWUSR, inv_gyro_enable_show,
	inv_gyro_enable_store);
static DEVICE_ATTR(accl_enable, S_IRUGO | S_IWUSR, inv_accl_enable_show,
	inv_accl_enable_store);
static DEVICE_ATTR(compass_enable, S_IRUGO | S_IWUSR, inv_compass_en_show,
	inv_compass_en_store);

static const struct attribute *inv_gyro_attributes[] = {
	&dev_attr_gyro_enable.attr,
	&dev_attr_temperature.attr,
	&dev_attr_clock_source.attr,
	&dev_attr_power_state.attr,
	&dev_attr_reg_dump.attr,
	&dev_attr_self_test.attr,
	&dev_attr_key.attr,
	&dev_attr_gyro_matrix.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
};

static const struct attribute *inv_mpu6050_attributes[] = {
	&dev_attr_accl_enable.attr,
	&dev_attr_accl_matrix.attr,
	&dev_attr_firmware_loaded.attr,
	&dev_attr_lpa_mode.attr,
	&dev_attr_lpa_freq.attr,
	&dev_attr_flick_lower.attr,
	&dev_attr_flick_upper.attr,
	&dev_attr_flick_counter.attr,
	&dev_attr_flick_message_on.attr,
	&dev_attr_flick_int_on.attr,
	&dev_attr_flick_axis.attr,
	&dev_attr_dmp_on.attr,
	&dev_attr_dmp_int_on.attr,
	&dev_attr_dmp_output_rate.attr,
	&dev_attr_orientation_on.attr,
	&dev_attr_quaternion_on.attr,
	&dev_attr_display_orientation_on.attr,
	&dev_attr_tap_on.attr,
	&dev_attr_tap_time.attr,
	&dev_attr_tap_min_count.attr,
	&dev_attr_tap_threshold.attr,
	&dev_attr_pedometer_time.attr,
	&dev_attr_pedometer_steps.attr,
	&dev_attr_event_flick.attr,
	&dev_attr_event_orientation.attr,
	&dev_attr_event_display_orientation.attr,
	&dev_attr_event_tap.attr,
};

static const struct attribute *inv_compass_attributes[] = {
	&dev_attr_compass_matrix.attr,
	&dev_attr_compass_enable.attr,
};

static const struct attribute *inv_mpu3050_attributes[] = {
	&dev_attr_accl_matrix.attr,
	&dev_attr_accl_enable.attr,
};

static struct attribute *inv_attributes[ARRAY_SIZE(inv_gyro_attributes) +
				ARRAY_SIZE(inv_mpu6050_attributes) +
				ARRAY_SIZE(inv_compass_attributes) + 1];
static const struct attribute_group inv_attribute_group = {
	.name = "mpu",
	.attrs = inv_attributes
};

static const struct iio_info mpu_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &mpu_read_raw,
	.write_raw = &mpu_write_raw,
	.attrs = &inv_attribute_group,
};

/**
 *  inv_setup_compass() - Configure compass.
 */
static int inv_setup_compass(struct inv_gyro_state_s *st)
{
	int result;
	unsigned char data[4];

	result = inv_i2c_read(st, REG_YGOFFS_TC, 1, data);
	if (result)
		return result;
	data[0] &= ~BIT_I2C_MST_VDDIO;
	if (st->plat_data.level_shifter)
		data[0] |= BIT_I2C_MST_VDDIO;
	/*set up VDDIO register */
	result = inv_i2c_single_write(st, REG_YGOFFS_TC, data[0]);
	if (result)
		return result;
	/* set to bypass mode */
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG, BIT_BYPASS_EN);
	if (result)
		return result;
	/*read secondary i2c ID register */
	result = inv_secondary_read(REG_AKM_ID, 1, data);
	if (result)
		return result;
	if (data[0] != DATA_AKM_ID)
		return -ENXIO;
	/*set AKM to Fuse ROM access mode */
	result = inv_secondary_write(REG_AKM_MODE, DATA_AKM_MODE_PW_FR);
	if (result)
		return result;
	result = inv_secondary_read(REG_AKM_SENSITIVITY, THREE_AXIS,
					st->chip_info.compass_sens);
	if (result)
		return result;
	/*revert to power down mode */
	result = inv_secondary_write(REG_AKM_MODE, DATA_AKM_MODE_PW_DN);
	if (result)
		return result;
	pr_err("senx=%d, seny=%d,senz=%d\n",
		st->chip_info.compass_sens[0],
		st->chip_info.compass_sens[1],
		st->chip_info.compass_sens[2]);
	/*restore to non-bypass mode */
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG, 0);
	if (result)
		return result;

	/*setup master mode and master clock and ES bit*/
	result = inv_i2c_single_write(st, REG_I2C_MST_CTRL, BIT_WAIT_FOR_ES);
	if (result)
		return result;
	/* slave 0 is used to read data from compass */
	/*read mode */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_ADDR, BIT_I2C_READ|
		st->plat_data.secondary_i2c_addr);
	if (result)
		return result;
	/* AKM status register address is 2 */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_REG, REG_AKM_STATUS);
	if (result)
		return result;
	/* slave 0 is enabled at the beginning, read 8 bytes from here */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_CTRL, BIT_SLV_EN |
				NUM_BYTES_COMPASS_SLAVE);
	if (result)
		return result;
	/*slave 1 is used for AKM mode change only*/
	result = inv_i2c_single_write(st, REG_I2C_SLV1_ADDR,
		st->plat_data.secondary_i2c_addr);
	if (result)
		return result;
	/* AKM mode register address is 0x0A */
	result = inv_i2c_single_write(st, REG_I2C_SLV1_REG, REG_AKM_MODE);
	if (result)
		return result;
	/* slave 1 is enabled, byte length is 1 */
	result = inv_i2c_single_write(st, REG_I2C_SLV1_CTRL, BIT_SLV_EN | 1);
	if (result)
		return result;
	/* output data for slave 1 is fixed, single measure mode*/
	st->compass_scale = 1;
	data[0] = 1;
	if (COMPASS_ID_AK8975 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8975_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8975_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8975_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8975_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8975_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8975_ST_Z_LW;
	} else if (COMPASS_ID_AK8972 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8972_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8972_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8972_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8972_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8972_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8972_ST_Z_LW;
	} else if (COMPASS_ID_AK8963 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8963_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8963_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8963_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8963_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8963_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8963_ST_Z_LW;
		data[0] |= (st->compass_scale << AKM8963_SCALE_SHIFT);
	}
	result = inv_i2c_single_write(st, REG_I2C_SLV1_DO, data[0]);
	if (result)
		return result;
	/* slave 0 and 1 timer action is enabled every sample*/
	result = inv_i2c_single_write(st, REG_I2C_MST_DELAY_CTRL,
				BIT_SLV0_DLY_EN | BIT_SLV1_DLY_EN);
	return result;
}

/**
 *  inv_check_chip_type() - check and setup chip type.
 */
static int inv_check_chip_type(struct inv_gyro_state_s *st,
		const struct i2c_device_id *id)
{
	struct inv_reg_map_s *reg;
	int result, chan_index;
	int t_ind;
	if (!strcmp(id->name, "itg3500"))
		st->chip_type = INV_ITG3500;
	else if (!strcmp(id->name, "mpu3050"))
		st->chip_type = INV_MPU3050;
	else if (!strcmp(id->name, "mpu6050"))
		st->chip_type = INV_MPU6050;
	else if (!strcmp(id->name, "mpu9150"))
		st->chip_type = INV_MPU9150;
	else
		return -EPERM;
	st->hw  = (struct inv_hw_s *)(hw_info  + st->chip_type);
	st->mpu_slave = NULL;
	chan_index = CHAN_INDEX_GYRO;
	if (INV_MPU9150 == st->chip_type) {
		st->plat_data.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS;
		st->plat_data.sec_slave_id = COMPASS_ID_AK8975;
		st->chip_config.has_compass = 1;
		chan_index = CHAN_INDEX_GYRO_ACCL_MAGN;
	}
	if (SECONDARY_SLAVE_TYPE_ACCEL == st->plat_data.sec_slave_type) {
		if (ACCEL_ID_BMA250 == st->plat_data.sec_slave_id)
			inv_register_bma250_slave(st);
		chan_index = CHAN_INDEX_GYRO_ACCL;
	}
	if (SECONDARY_SLAVE_TYPE_COMPASS == st->plat_data.sec_slave_type)
		st->chip_config.has_compass = 1;
	else
		st->chip_config.has_compass = 0;
	if (INV_MPU6050 == st->chip_type) {
		if (st->chip_config.has_compass)
			chan_index = CHAN_INDEX_GYRO_ACCL_MAGN;
		else
			chan_index = CHAN_INDEX_GYRO_ACCL;
	}
	st->chan_info = &chip_channel_info[chan_index];
	reg = &st->reg;
	if (INV_MPU3050 == st->chip_type)
		inv_setup_reg_mpu3050(reg);
	else
		inv_setup_reg(reg);
	st->chip_config.gyro_enable = 1;
	result = inv_set_power_state(st, 1);
	if (result)
		return result;

	if (INV_ITG3500 != st->chip_type && INV_MPU3050 != st->chip_type) {
		result = inv_get_silicon_rev_mpu6050(st);
		if (result) {
			inv_i2c_single_write(st, reg->pwr_mgmt_1,
					BIT_SLEEP | INV_CLK_PLL);
			return result;
		}
	}
	if (st->chip_config.has_compass) {
		result = inv_setup_compass(st);
		if (result) {
			inv_i2c_single_write(st, reg->pwr_mgmt_1,
					BIT_SLEEP | INV_CLK_PLL);
			return result;
		}
	}

	t_ind = 0;
	memcpy(&inv_attributes[t_ind], inv_gyro_attributes,
			sizeof(inv_gyro_attributes));
	t_ind = ARRAY_SIZE(inv_gyro_attributes);

	if (INV_MPU3050 == st->chip_type && st->mpu_slave != NULL) {
		memcpy(&inv_attributes[t_ind], inv_mpu3050_attributes,
				sizeof(inv_mpu3050_attributes));
		t_ind += ARRAY_SIZE(inv_mpu3050_attributes);
		inv_attributes[t_ind] = NULL;
		return 0;
	}

	if (chan_index > CHAN_INDEX_GYRO) {
		memcpy(&inv_attributes[t_ind], inv_mpu6050_attributes,
				sizeof(inv_mpu6050_attributes));
		t_ind += ARRAY_SIZE(inv_mpu6050_attributes);
	}

	if (chan_index > CHAN_INDEX_GYRO_ACCL) {
		memcpy(&inv_attributes[t_ind], inv_compass_attributes,
				sizeof(inv_compass_attributes));
		t_ind += ARRAY_SIZE(inv_compass_attributes);
	}
	inv_attributes[t_ind] = NULL;
	return 0;
}

/**
 *  inv_create_dmp_sysfs() - create binary sysfs dmp entry.
 */
static const struct bin_attribute dmp_firmware = {
	.attr = {
		.name = "dmp_firmware",
		.mode = S_IRUGO | S_IWUSR
	},
	.size = 4096,
	.read = inv_dmp_firmware_read,
	.write = inv_dmp_firmware_write,
};

static int inv_create_dmp_sysfs(struct iio_dev *ind)
{
	int result;
	result = sysfs_create_bin_file(&ind->dev.kobj, &dmp_firmware);
	return result;
}

/**
 *  inv_mpu_probe() - probe function.
 */
static int inv_mpu_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev;
	int result, reg_done;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}
	indio_dev = iio_allocate_device(sizeof(*st));
	if (indio_dev == NULL) {
		result =  -ENOMEM;
		goto out_no_free;
	}
	reg_done = 0;
	st = iio_priv(indio_dev);
	st->i2c = client;
	st->sl_handle = client->adapter;
	st->i2c_addr = client->addr;
	st->plat_data =
		*(struct mpu_platform_data *)dev_get_platdata(&client->dev);
	/* power is turned on inside check chip type*/
	result = inv_check_chip_type(st, id);
	if (result)
		goto out_free;
	if (INV_MPU3050 == st->chip_type)
		result = inv_init_config_mpu3050(indio_dev);
	else
		result = inv_init_config(indio_dev);
	if (result) {
		dev_err(&client->adapter->dev,
			"Could not initialize device.\n");
		goto out_free;
	}
	result = inv_set_power_state(st, 1);
	if (result) {
		dev_err(&client->adapter->dev,
			"%s could not be turned off.\n", st->hw->name);
		goto out_free;
	}

	/* Make state variables available to all _show and _store functions. */
	i2c_set_clientdata(client, indio_dev);
	indio_dev->dev.parent = &client->dev;
	indio_dev->name = id->name;
	indio_dev->channels = st->chan_info->channels;
	indio_dev->num_channels = st->chan_info->num_channels;
	indio_dev->info = &mpu_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;

	result = inv_mpu_configure_ring(indio_dev);
	if (result)
		goto out_free;
	result = iio_buffer_register(indio_dev, st->chan_info->channels,
					st->chan_info->num_channels);
	if (result)
		goto out_unreg_ring;
	st->irq = client->irq;
	result = inv_mpu_probe_trigger(indio_dev);
	if (result)
		goto out_remove_ring;

	result = iio_device_register(indio_dev);
	if (result)
		goto out_remove_trigger;
	if (INV_MPU6050 == st->chip_type || INV_MPU9150 == st->chip_type) {
		result = inv_create_dmp_sysfs(indio_dev);
		if (result)
			goto out_unreg_iio;
	}

	INIT_KFIFO(st->timestamps);
	spin_lock_init(&st->time_stamp_lock);
	pr_info("%s: Probe name %s\n", __func__, id->name);
	dev_info(&client->adapter->dev, "%s is ready to go!\n", st->hw->name);
	return 0;
out_unreg_iio:
	iio_device_unregister(indio_dev);
out_remove_trigger:
	if (indio_dev->modes & INDIO_BUFFER_TRIGGERED)
		inv_mpu_remove_trigger(indio_dev);
out_remove_ring:
	iio_buffer_unregister(indio_dev);
out_unreg_ring:
	inv_mpu_unconfigure_ring(indio_dev);
out_free:
	iio_free_device(indio_dev);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return -EIO;
}

/**
 *  inv_mpu_remove() - remove function.
 */
static int inv_mpu_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	kfifo_free(&st->timestamps);
	iio_device_unregister(indio_dev);
	inv_mpu_remove_trigger(indio_dev);
	iio_buffer_unregister(indio_dev);
	inv_mpu_unconfigure_ring(indio_dev);
	iio_free_device(indio_dev);

	dev_info(&client->adapter->dev, "inv-mpu-iio module removed.\n");
	return 0;
}
static const unsigned short normal_i2c[] = { I2C_CLIENT_END };
/* device id table is used to identify what device can be
 * supported by this driver
 */
static const struct i2c_device_id inv_mpu_id[] = {
	{"itg3500", INV_ITG3500},
	{"mpu3050", INV_MPU3050},
	{"mpu6050", INV_MPU6050},
	{"mpu9150", INV_MPU9150},
	{}
};

MODULE_DEVICE_TABLE(i2c, inv_mpu_id);

static struct i2c_driver inv_mpu_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		=	inv_mpu_probe,
	.remove		=	inv_mpu_remove,
	.id_table	=	inv_mpu_id,
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	"inv-mpu-iio",
	},
	.address_list = normal_i2c,
};

static int __init inv_mpu_init(void)
{
	int result = i2c_add_driver(&inv_mpu_driver);
	if (result) {
		pr_err("%s failed\n", __func__);
		return result;
	}
	return 0;
}

static void __exit inv_mpu_exit(void)
{
	i2c_del_driver(&inv_mpu_driver);
}

module_init(inv_mpu_init);
module_exit(inv_mpu_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("inv-mpu-iio");
/**
 *  @}
 */

