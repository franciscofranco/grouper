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
 *      @file    inv_gyro_misc.c
 *      @brief   A sysfs device driver for Invensense gyroscopes.
 *      @details This file is part of inv_gyro driver code
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
#include "inv_mpu_iio.h"
#include "../../iio.h"
#include "../../kfifo_buf.h"
#include "../../trigger_consumer.h"
#include "../../sysfs.h"

/**
 *  reset_fifo_mpu3050() - Reset FIFO related registers
 *  @st:	Device driver instance.
 */
static int reset_fifo_mpu3050(struct iio_dev *indio_dev)
{
	struct inv_reg_map_s *reg;
	int result;
	unsigned char val, user_ctrl;
	struct inv_gyro_state_s  *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;

	reg = &st->reg;
	if (iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_GYRO_X) ||
		iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_GYRO_Y) ||
		iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_GYRO_Z))
		st->chip_config.gyro_fifo_enable = 1;
	else
		st->chip_config.gyro_fifo_enable = 0;

	if (iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_ACCL_X) ||
		iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_ACCL_Y) ||
		iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_ACCL_Z))
		st->chip_config.accl_fifo_enable = 1;
	else
		st->chip_config.accl_fifo_enable = 0;

	/* disable interrupt */
	result = inv_i2c_single_write(st, reg->int_enable, 0);
	if (result)
		return result;
	/* disable the sensor output to FIFO */
	result = inv_i2c_single_write(st, reg->fifo_en, 0);
	if (result)
		goto reset_fifo_fail;
	result = inv_i2c_read(st, reg->user_ctrl, 1, &user_ctrl);
	if (result)
		goto reset_fifo_fail;
	/* disable fifo reading */
	user_ctrl &= ~BIT_FIFO_EN;
	st->chip_config.has_footer = 0;
	/* reset fifo */
	val = (BIT_3050_FIFO_RST | user_ctrl);
	result = inv_i2c_single_write(st, reg->user_ctrl, val);
	if (result)
		goto reset_fifo_fail;
	st->last_isr_time = iio_get_time_ns();
	if (st->chip_config.dmp_on) {
		/* enable interrupt when DMP is done */
		result = inv_i2c_single_write(st, reg->int_enable,
					BIT_DMP_INT_EN);
		if (result)
			return result;

		result = inv_i2c_single_write(st, reg->user_ctrl,
			BIT_FIFO_EN|user_ctrl);
		if (result)
			return result;
	} else {
		/* enable interrupt */
		if (st->chip_config.accl_fifo_enable ||
			st->chip_config.gyro_fifo_enable){
			result = inv_i2c_single_write(st, reg->int_enable,
							BIT_DATA_RDY_EN);
			if (result)
				return result;
		}
		/* enable FIFO reading and I2C master interface*/
		result = inv_i2c_single_write(st, reg->user_ctrl,
			BIT_FIFO_EN | user_ctrl);
		if (result)
			return result;
		/* enable sensor output to FIFO and FIFO footer*/
		val = 1;
		if (st->chip_config.accl_fifo_enable)
			val |= BITS_3050_ACCL_OUT;
		if (st->chip_config.gyro_fifo_enable)
			val |= BITS_GYRO_OUT;
		result = inv_i2c_single_write(st, reg->fifo_en, val);
		if (result)
			return result;
	}

	return 0;
reset_fifo_fail:
	if (st->chip_config.dmp_on)
		val = BIT_DMP_INT_EN;
	else
		val = BIT_DATA_RDY_EN;
	inv_i2c_single_write(st, reg->int_enable, val);
	pr_err("%s failed\n", __func__);
	return result;
}
/**
 *  reset_fifo_itg() - Reset FIFO related registers.
 *  @st:	Device driver instance.
 */
static int reset_fifo_itg(struct iio_dev *indio_dev)
{
	struct inv_reg_map_s *reg;
	int result;
	unsigned char val;
	struct inv_gyro_state_s  *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;

	reg = &st->reg;
	if (iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_GYRO_X) ||
		iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_GYRO_Y) ||
		iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_GYRO_Z))
		st->chip_config.gyro_fifo_enable = 1;
	else
		st->chip_config.gyro_fifo_enable = 0;

	if (iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_ACCL_X) ||
		iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_ACCL_Y) ||
		iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_ACCL_Z))
		st->chip_config.accl_fifo_enable = 1;
	else
		st->chip_config.accl_fifo_enable = 0;

	if (iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_MAGN_X) ||
		iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_MAGN_Y) ||
		iio_scan_mask_query(indio_dev, ring, INV_MPU_SCAN_MAGN_Z))
		st->chip_config.compass_fifo_enable = 1;
	else
		st->chip_config.compass_fifo_enable = 0;

	/* disable interrupt */
	result = inv_i2c_single_write(st, reg->int_enable, 0);
	if (result) {
		pr_err("%s failed\n", __func__);
		return result;
	}
	/* disable the sensor output to FIFO */
	result = inv_i2c_single_write(st, reg->fifo_en, 0);
	if (result)
		goto reset_fifo_fail;
	/* disable fifo reading */
	result = inv_i2c_single_write(st, reg->user_ctrl, 0);
	if (result)
		goto reset_fifo_fail;

	if (st->chip_config.dmp_on) {
		val = (BIT_FIFO_RST | BIT_DMP_RST);
		result = inv_i2c_single_write(st, reg->user_ctrl, val);
		if (result)
			goto reset_fifo_fail;
		st->last_isr_time = iio_get_time_ns();
		if (st->chip_config.dmp_int_on) {
			result = inv_i2c_single_write(st, reg->int_enable,
							BIT_DMP_INT_EN);
			if (result)
				return result;
		}
		val = (BIT_DMP_EN | BIT_FIFO_EN);
		if (st->chip_config.compass_enable)
			val |= BIT_I2C_MST_EN;
		result = inv_i2c_single_write(st, reg->user_ctrl, val);
		if (result)
			goto reset_fifo_fail;
	} else {
		/* reset FIFO and possibly reset I2C*/
		val = BIT_FIFO_RST;
		result = inv_i2c_single_write(st, reg->user_ctrl, val);
		if (result)
			goto reset_fifo_fail;
		st->last_isr_time = iio_get_time_ns();
		/* enable interrupt */
		if (st->chip_config.accl_fifo_enable ||
			st->chip_config.gyro_fifo_enable ||
			st->chip_config.compass_enable){
			result = inv_i2c_single_write(st, reg->int_enable,
						BIT_DATA_RDY_EN);
			if (result)
				return result;
		}
		/* enable FIFO reading and I2C master interface*/
		val = BIT_FIFO_EN;
		if (st->chip_config.compass_enable)
			val |= BIT_I2C_MST_EN;
		result = inv_i2c_single_write(st, reg->user_ctrl, val);
		if (result)
			goto reset_fifo_fail;
		/* enable sensor output to FIFO */
		val = 0;
		if (st->chip_config.gyro_fifo_enable)
			val |= BITS_GYRO_OUT;
		if (st->chip_config.accl_fifo_enable)
			val |= BIT_ACCEL_OUT;
		result = inv_i2c_single_write(st, reg->fifo_en, val);
		if (result)
			goto reset_fifo_fail;
	}
	return 0;
reset_fifo_fail:
	if (st->chip_config.dmp_on)
		val = BIT_DMP_INT_EN;
	else
		val = BIT_DATA_RDY_EN;
	inv_i2c_single_write(st, reg->int_enable, val);
	pr_err("%s failed\n", __func__);
	return result;
}
/**
 *  inv_reset_fifo() - Reset FIFO related registers.
 *  @st:	Device driver instance.
 */
static int inv_reset_fifo(struct iio_dev *indio_dev)
{
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (INV_MPU3050 == st->chip_type)
		return reset_fifo_mpu3050(indio_dev);
	else
		return reset_fifo_itg(indio_dev);
}
/**
 *  set_inv_enable() - Reset FIFO related registers.
 *			This also powers on the chip if needed.
 *  @st:	Device driver instance.
 *  @fifo_enable: enable/disable
 */
int set_inv_enable(struct iio_dev *indio_dev,
			unsigned long enable) {
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct inv_reg_map_s *reg;
	int result;

	if (st->chip_config.is_asleep)
		return -EINVAL;
	reg = &st->reg;
	if (enable) {
		result = inv_reset_fifo(indio_dev);
		if (result)
			return result;
		st->chip_config.enable = 1;
	} else {
		result = inv_i2c_single_write(st, reg->fifo_en, 0);
		if (result)
			return result;
		result = inv_i2c_single_write(st, reg->int_enable, 0);
		if (result)
			return result;
		/* disable fifo reading */
		if (INV_MPU3050 != st->chip_type) {
			result = inv_i2c_single_write(st, reg->user_ctrl, 0);
			if (result)
				return result;
		}
		st->chip_config.enable = 0;
	}
	return 0;
}

/**
 *  inv_clear_kfifo() - clear time stamp fifo
 *  @st:	Device driver instance.
 */
void inv_clear_kfifo(struct inv_gyro_state_s *st)
{
	unsigned long flags;
	spin_lock_irqsave(&st->time_stamp_lock, flags);
	kfifo_reset(&st->timestamps);
	spin_unlock_irqrestore(&st->time_stamp_lock, flags);
}

/**
 *  inv_irq_handler() - Cache a timestamp at each data ready interrupt.
 */
static irqreturn_t inv_irq_handler(int irq, void *dev_id)
{
	struct inv_gyro_state_s *st;
	long long timestamp;
	int result, catch_up;
	unsigned int time_since_last_irq;

	st = (struct inv_gyro_state_s *)dev_id;
	timestamp = iio_get_time_ns();
	time_since_last_irq = ((unsigned int)(timestamp
		- st->last_isr_time))/ONE_K_HZ;
	spin_lock(&st->time_stamp_lock);
	catch_up = 0;
	while ((time_since_last_irq > st->irq_dur_us*2)
		&& (catch_up < MAX_CATCH_UP)
		&& (0 == st->chip_config.lpa_mode)) {

		st->last_isr_time += st->irq_dur_us*ONE_K_HZ;
		result = kfifo_in(&st->timestamps,
			&st->last_isr_time, 1);
		time_since_last_irq = ((unsigned int)(timestamp
			- st->last_isr_time))/ONE_K_HZ;
		catch_up++;
	}
	result = kfifo_in(&st->timestamps, &timestamp, 1);
	st->last_isr_time = timestamp;
	spin_unlock(&st->time_stamp_lock);

	return IRQ_WAKE_THREAD;
}
static int put_scan_to_buf(struct iio_dev *indio_dev, unsigned char *d,
				short *s, int scan_index, int d_ind) {
	struct iio_buffer *ring = indio_dev->buffer;
	int st;
	int i;
	for (i = 0; i < 3; i++) {
		st = iio_scan_mask_query(indio_dev, ring, scan_index + i);
		if (st) {
			memcpy(&d[d_ind], &s[i], sizeof(s[i]));
			d_ind += sizeof(s[i]);
		}
	}
	return d_ind;
}
static int put_scan_to_buf_q(struct iio_dev *indio_dev, unsigned char *d,
				int *s, int scan_index, int d_ind) {
	struct iio_buffer *ring = indio_dev->buffer;
	int st;
	int i;
	for (i = 0; i < 4; i++) {
		st = iio_scan_mask_query(indio_dev, ring, scan_index + i);
		if (st) {
			memcpy(&d[d_ind], &s[i], sizeof(s[i]));
			d_ind += sizeof(s[i]);
		}
	}
	return d_ind;
}

static void inv_report_data_3050(struct iio_dev *indio_dev, s64 t,
			int has_footer, unsigned char *data)
{
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	int ind, i, d_ind;
	struct inv_chip_config_s *conf;
	short g[3], a[3];
	s64 buf[8];
	unsigned char *tmp;
	int bytes_per_datum, scan_count;
	conf = &st->chip_config;

	scan_count = bitmap_weight(indio_dev->active_scan_mask,
				       indio_dev->masklength);
	bytes_per_datum = scan_count * 2;

	ind = 0;
	if (has_footer)
		ind += 2;
	tmp = (unsigned char *)buf;
	d_ind = 0;
	if (conf->gyro_fifo_enable) {
		g[0] =	be16_to_cpup((__be16 *)(&data[ind]));
		g[1] =	be16_to_cpup((__be16 *)(&data[ind+2]));
		g[2] =	be16_to_cpup((__be16 *)(&data[ind+4]));
		ind += 6;
		d_ind = put_scan_to_buf(indio_dev, tmp, g,
			INV_MPU_SCAN_GYRO_X, d_ind);
	}
	if (conf->accl_fifo_enable) {
		st->mpu_slave->combine_data(&data[ind], a);
		ind += 6;
		d_ind = put_scan_to_buf(indio_dev, tmp, a,
			INV_MPU_SCAN_ACCL_X, d_ind);
	}

	i = (bytes_per_datum + 7)/8;
	if (ring->scan_timestamp)
		buf[i] = t;
	ring->access->store_to(indio_dev->buffer, (u8 *) buf, t);
}
/**
 *  inv_read_fifo_mpu3050() - Transfer data from FIFO to ring buffer for mpu3050.
 */
irqreturn_t inv_read_fifo_mpu3050(int irq, void *dev_id)
{

	struct inv_gyro_state_s *st = (struct inv_gyro_state_s *)dev_id;
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	int bytes_per_datum;
	unsigned char data[64];
	int result;
	short fifo_count, byte_read;
	unsigned int copied;
	s64 timestamp;
	struct inv_reg_map_s *reg;
	reg = &st->reg;
	/* It is impossible that chip is asleep or enable is
	zero when interrupt is on
	*  because interrupt is now connected with enable */
	if (st->chip_config.dmp_on)
		bytes_per_datum = BYTES_FOR_DMP;
	else
		bytes_per_datum = (st->chip_config.accl_fifo_enable +
			st->chip_config.gyro_fifo_enable)*BYTES_PER_SENSOR;
	if (st->chip_config.has_footer)
		byte_read = bytes_per_datum + 2;
	else
		byte_read = bytes_per_datum;

	fifo_count = 0;
	if (byte_read != 0) {
		result = inv_i2c_read(st, reg->fifo_count_h,
				FIFO_COUNT_BYTE, data);
		if (result)
			goto end_session;
		fifo_count = (data[0] << 8) + data[1];
		if (fifo_count < byte_read)
			goto end_session;
		if (fifo_count%2)
			goto flush_fifo;
		if (fifo_count > FIFO_THRESHOLD)
			goto flush_fifo;
		/* Timestamp mismatch. */
		if (kfifo_len(&st->timestamps) <
			fifo_count / byte_read)
			goto flush_fifo;
		if (kfifo_len(&st->timestamps) >
			fifo_count / byte_read + TIME_STAMP_TOR) {
			if (st->chip_config.dmp_on) {
				result = kfifo_to_user(&st->timestamps,
				&timestamp, sizeof(timestamp), &copied);
				if (result)
					goto flush_fifo;
			} else
				goto flush_fifo;
		}
	}
	while ((bytes_per_datum != 0) && (fifo_count >= byte_read)) {
		result = inv_i2c_read(st, reg->fifo_r_w, byte_read, data);
		if (result)
			goto flush_fifo;

		result = kfifo_to_user(&st->timestamps,
			&timestamp, sizeof(timestamp), &copied);
		if (result)
			goto flush_fifo;
		inv_report_data_3050(indio_dev, timestamp,
				st->chip_config.has_footer, data);
		fifo_count -= byte_read;
		if (st->chip_config.has_footer == 0) {
			st->chip_config.has_footer = 1;
			byte_read = bytes_per_datum + MPU3050_FOOTER_SIZE;
		}
	}
end_session:
	return IRQ_HANDLED;
flush_fifo:
	/* Flush HW and SW FIFOs. */
	inv_reset_fifo(indio_dev);
	inv_clear_kfifo(st);
	return IRQ_HANDLED;
}
static int inv_report_gyro_accl_compass(struct iio_dev *indio_dev,
					unsigned char *data, s64 t)
{
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	short g[3], a[3], c[3];
	int q[4];
	int result, ind, d_ind;
	s64 buf[8];
	unsigned int word;
	unsigned char d[8];
	unsigned char *tmp;
	int source;
	struct inv_chip_config_s *conf;
#define INT_SRC_TAP    0x01
#define INT_SRC_ORIENT 0x02
#define INT_SRC_DISPLAY_ORIENT  0x08
#define INT_SRC_SHAKE           0x10

	conf = &st->chip_config;
	ind = 0;
	if (conf->quaternion_on & conf->dmp_on) {
		q[0] =	be32_to_cpup((__be32 *)(&data[ind]));
		q[1] =	be32_to_cpup((__be32 *)(&data[ind+4]));
		q[2] =	be32_to_cpup((__be32 *)(&data[ind+8]));
		q[3] =	be32_to_cpup((__be32 *)(&data[ind+12]));
		ind += 16;
	}
	if (conf->accl_fifo_enable | conf->dmp_on) {
		a[0] =	be16_to_cpup((__be16 *)(&data[ind]));
		a[1] =	be16_to_cpup((__be16 *)(&data[ind+2]));
		a[2] =	be16_to_cpup((__be16 *)(&data[ind+4]));

		a[0] *= st->chip_info.multi;
		a[1] *= st->chip_info.multi;
		a[2] *= st->chip_info.multi;
		st->raw_accel[0] = a[0];
		st->raw_accel[1] = a[1];
		st->raw_accel[2] = a[2];
		ind += 6;
	}
	if (conf->gyro_fifo_enable | conf->dmp_on) {
		g[0] =	be16_to_cpup((__be16 *)(&data[ind]));
		g[1] =	be16_to_cpup((__be16 *)(&data[ind+2]));
		g[2] =	be16_to_cpup((__be16 *)(&data[ind+4]));

		st->raw_gyro[0] = g[0];
		st->raw_gyro[1] = g[1];
		st->raw_gyro[2] = g[2];
		ind += 6;
	}
	if (conf->dmp_on) {
		word = (unsigned int)(be32_to_cpup((unsigned int *)&data[ind]));
		source = (word/65536)%256;
		if (source) {
			st->tap_data = 0x3f & (word%256);
			st->orient_data = (word/256)%256;
			st->display_orient_data = ((0xc0 & (word%256))>>6);
		}

		/* report tap information */
		if (source & INT_SRC_TAP)
			sysfs_notify(&indio_dev->dev.kobj, NULL, "event_tap");
		/* report orientation information */
		if (source & INT_SRC_ORIENT)
			sysfs_notify(&indio_dev->dev.kobj, NULL,
			"event_orientation");
		/* report orientation information */
		if (source & INT_SRC_DISPLAY_ORIENT)
			sysfs_notify(&indio_dev->dev.kobj, NULL,
			"event_display_orientation");
	}
	/*divider and counter is used to decrease the speed of read in
		high frequency sample rate*/
	if (conf->compass_fifo_enable) {
		c[0] = c[1] = c[2] = 0;
		if (st->compass_divider == st->compass_counter) {
			/*read from external sensor data register */
			result = inv_i2c_read(st, REG_EXT_SENS_DATA_00, 8, d);
			/* d[7] is status 2 register */
			/*for AKM8975, bit 2 and 3 should be all be zero*/
			/* for AMK8963, bit 3 should be zero*/
			if ((DATA_AKM_DRDY == d[0])
			&& (0 == (d[7] & DATA_AKM_STAT_MASK))
			&& (!result)) {
				unsigned char *sens;
				sens = st->chip_info.compass_sens;
				c[0] = (short)((d[2] << 8) | d[1]);
				c[1] = (short)((d[4] << 8) | d[3]);
				c[2] = (short)((d[6] << 8) | d[5]);
				c[0] = ((c[0] * (sens[0] + 128)) >> 8);
				c[1] = ((c[1] * (sens[1] + 128)) >> 8);
				c[2] = ((c[2] * (sens[2] + 128)) >> 8);
				st->raw_compass[0] = c[0];
				st->raw_compass[1] = c[1];
				st->raw_compass[2] = c[2];
			}
			st->compass_counter = 0;
		} else if (st->compass_divider != 0)
			st->compass_counter++;
	}

	tmp = (unsigned char *)buf;
	d_ind = 0;
	if (conf->quaternion_on & conf->dmp_on)
		d_ind = put_scan_to_buf_q(indio_dev, tmp, q,
				INV_MPU_SCAN_QUAT_R, d_ind);
	if (conf->gyro_fifo_enable)
		d_ind = put_scan_to_buf(indio_dev, tmp, g,
				INV_MPU_SCAN_GYRO_X, d_ind);
	if (conf->accl_fifo_enable)
		d_ind = put_scan_to_buf(indio_dev, tmp, a,
				INV_MPU_SCAN_ACCL_X, d_ind);
	if (conf->compass_fifo_enable)
		d_ind = put_scan_to_buf(indio_dev, tmp, c,
				INV_MPU_SCAN_MAGN_X, d_ind);
	if (ring->scan_timestamp)
		buf[(d_ind + 7)/8] = t;
	ring->access->store_to(indio_dev->buffer, (u8 *) buf, t);

	return 0;
}

/**
 *  inv_read_fifo() - Transfer data from FIFO to ring buffer.
 */
irqreturn_t inv_read_fifo(int irq, void *dev_id)
{

	struct inv_gyro_state_s *st = (struct inv_gyro_state_s *)dev_id;
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	size_t bytes_per_datum;
	int result;
	unsigned char data[BYTES_FOR_DMP + QUATERNION_BYTES];
	unsigned short fifo_count;
	unsigned int copied;
	s64 timestamp;
	struct inv_reg_map_s *reg;
	s64 buf[8];
	unsigned char *tmp;
	reg = &st->reg;
	if (!(st->chip_config.accl_fifo_enable |
		st->chip_config.gyro_fifo_enable |
		st->chip_config.dmp_on |
		st->chip_config.compass_fifo_enable))
		goto end_session;
	if (st->chip_config.dmp_on && st->chip_config.flick_int_on) {
		/*dmp interrupt status */
		inv_i2c_read(st, REG_DMP_INT_STATUS, 1, data);
		if (data[0] & 8)
			sysfs_notify(&indio_dev->dev.kobj, NULL, "event_flick");
	}
	if (st->chip_config.lpa_mode) {
		result = inv_i2c_read(st, reg->raw_accl, 6, data);
		if (result)
			goto end_session;
		inv_report_gyro_accl_compass(indio_dev, data,
						iio_get_time_ns());
		goto end_session;
	}

	if (st->chip_config.dmp_on)
		if (st->chip_config.quaternion_on)
			bytes_per_datum = BYTES_FOR_DMP + QUATERNION_BYTES;
		else
			bytes_per_datum = BYTES_FOR_DMP;
	else
		bytes_per_datum = (st->chip_config.accl_fifo_enable +
		st->chip_config.gyro_fifo_enable)*BYTES_PER_SENSOR;
	fifo_count = 0;
	if (bytes_per_datum != 0) {
		result = inv_i2c_read(st, reg->fifo_count_h,
				FIFO_COUNT_BYTE, data);
		if (result)
			goto end_session;
		fifo_count = (data[0] << 8) + data[1];
		if (fifo_count < bytes_per_datum)
			goto end_session;
		if (fifo_count%2)
			goto flush_fifo;
		if (fifo_count >  FIFO_THRESHOLD)
			goto flush_fifo;
		/* Timestamp mismatch. */
		if (kfifo_len(&st->timestamps) <
			fifo_count / bytes_per_datum)
			goto flush_fifo;
		if (kfifo_len(&st->timestamps) >
			fifo_count / bytes_per_datum + TIME_STAMP_TOR) {
			if (st->chip_config.dmp_on) {
				result = kfifo_to_user(&st->timestamps,
				&timestamp, sizeof(timestamp), &copied);
				if (result)
					goto flush_fifo;
			} else
				goto flush_fifo;
		}
	}
	if (bytes_per_datum == 0) {
		result = kfifo_to_user(&st->timestamps,
			&timestamp, sizeof(timestamp), &copied);
		if (result)
			goto flush_fifo;
	}
	tmp = (char *)buf;
	while ((bytes_per_datum != 0) && (fifo_count >= bytes_per_datum)) {
		result = inv_i2c_read(st, reg->fifo_r_w, bytes_per_datum,
			data);
		if (result)
			goto flush_fifo;

		result = kfifo_to_user(&st->timestamps,
			&timestamp, sizeof(timestamp), &copied);
		if (result)
			goto flush_fifo;
		inv_report_gyro_accl_compass(indio_dev, data, timestamp);
		fifo_count -= bytes_per_datum;
	}
	if (bytes_per_datum == 0)
		inv_report_gyro_accl_compass(indio_dev, data, timestamp);
end_session:
	return IRQ_HANDLED;
flush_fifo:
	/* Flush HW and SW FIFOs. */
	inv_reset_fifo(indio_dev);
	inv_clear_kfifo(st);
	return IRQ_HANDLED;
}

void inv_mpu_unconfigure_ring(struct iio_dev *indio_dev)
{
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	free_irq(st->i2c->irq, st);
	iio_kfifo_free(indio_dev->buffer);
};

int inv_postenable(struct iio_dev *indio_dev)
{
	return set_inv_enable(indio_dev, 1);

}
int inv_predisable(struct iio_dev *indio_dev)
{
	return set_inv_enable(indio_dev, 0);
}

static const struct iio_buffer_setup_ops inv_mpu_ring_setup_ops = {
	.preenable = &iio_sw_buffer_preenable,
	.postenable = &inv_postenable,
	.predisable = &inv_predisable,
};

int inv_mpu_configure_ring(struct iio_dev *indio_dev)
{
	int ret = 0;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring;

	ring = iio_kfifo_allocate(indio_dev);
	if (!ring) {
		ret = -ENOMEM;
		return ret;
	}
	indio_dev->buffer = ring;
	/* setup ring buffer */
	ring->scan_timestamp = true;
	indio_dev->setup_ops = &inv_mpu_ring_setup_ops;
	/*scan count double count timestamp. should subtract 1. but
	number of channels still includes timestamp*/
	if (INV_MPU3050 == st->chip_type)
		ret = request_threaded_irq(st->i2c->irq, inv_irq_handler,
			inv_read_fifo_mpu3050,
			IRQF_TRIGGER_RISING | IRQF_SHARED, "inv_irq", st);
	else
		ret = request_threaded_irq(st->i2c->irq, inv_irq_handler,
			inv_read_fifo,
			IRQF_TRIGGER_RISING | IRQF_SHARED, "inv_irq", st);
	if (ret)
		goto error_iio_sw_rb_free;

	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	return 0;
error_iio_sw_rb_free:
	iio_kfifo_free(indio_dev->buffer);
	return ret;
}
/**
 *  @}
 */

