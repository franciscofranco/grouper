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
 *      @file    inv_mpu3050.c
 *      @brief   A sysfs device driver for Invensense devices
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
#include <linux/spinlock.h>

#include "inv_mpu_iio.h"
#define MPU3050_NACK_TIME (2*1000)
#define MPU3050_ONE_MPU_TIME (20)
#define MPU3050_BOGUS_ADDR (0x7F)

int set_3050_bypass(struct inv_gyro_state_s *st, int enable)
{
	struct inv_reg_map_s *reg;
	int result;
	unsigned char b;

	reg = &st->reg;
	result = inv_i2c_read(st, reg->user_ctrl, 1, &b);
	if (result)
		return result;
	if (((b & BIT_3050_AUX_IF_EN) == 0) && enable)
		return 0;
	if ((b & BIT_3050_AUX_IF_EN) && (enable == 0))
		return 0;
	b &= ~BIT_3050_AUX_IF_EN;
	if (!enable) {
		b |= BIT_3050_AUX_IF_EN;
		result = inv_i2c_single_write(st, reg->user_ctrl, b);
		return result;
	} else {
		/* Coming out of I2C is tricky due to several erratta.  Do not
		* modify this algorithm
		*/
		/*
		* 1) wait for the right time and send the command to change
		* the aux i2c slave address to an invalid address that will
		* get nack'ed
		*
		* 0x00 is broadcast.  0x7F is unlikely to be used by any aux.
		*/
		result = inv_i2c_single_write(st, REG_3050_SLAVE_ADDR,
						MPU3050_BOGUS_ADDR);
		if (result)
			return result;
		/*
		* 2) wait enough time for a nack to occur, then go into
		*    bypass mode:
		*/
		usleep_range(MPU3050_NACK_TIME, MPU3050_NACK_TIME);
		result = inv_i2c_single_write(st, reg->user_ctrl, b);
		if (result)
			return result;
		/*
		* 3) wait for up to one MPU cycle then restore the slave
		*    address
		*/
		msleep(MPU3050_ONE_MPU_TIME);

		result = inv_i2c_single_write(st, REG_3050_SLAVE_ADDR,
			st->plat_data.secondary_i2c_addr);
		if (result)
			return result;
		result = inv_i2c_single_write(st, reg->user_ctrl, b);
		if (result)
			return result;
		usleep_range(MPU3050_NACK_TIME, MPU3050_NACK_TIME);
	}
	return 0;
}

void inv_setup_reg_mpu3050(struct inv_reg_map_s *reg)
{
	reg->fifo_en         = 0x12;
	reg->sample_rate_div = 0x15;
	reg->lpf             = 0x16;
	reg->fifo_count_h    = 0x3a;
	reg->fifo_r_w        = 0x3c;
	reg->user_ctrl       = 0x3d;
	reg->pwr_mgmt_1      = 0x3e;
	reg->raw_gyro        = 0x1d;
	reg->raw_accl        = 0x23;
	reg->temperature     = 0x1b;
	reg->int_enable      = 0x17;
	reg->int_status      = 0x1a;
}

/**
 *  inv_init_config_mpu3050() - Initialize hardware, disable FIFO.
 *  @st:	Device driver instance.
 *  Initial configuration:
 *  FSR: +/- 2000DPS
 *  DLPF: 42Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 */
int inv_init_config_mpu3050(struct iio_dev *indio_dev)
{
	struct inv_reg_map_s *reg;
	int result;
	unsigned char data;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->chip_config.is_asleep)
		return -EPERM;
	/*reading AUX VDDIO register */
	result = inv_i2c_read(st, REG_3050_AUX_VDDIO, 1, &data);
	if (result)
		return result;
	data &= ~BIT_3050_VDDIO;
	if (st->plat_data.level_shifter)
		data |= BIT_3050_VDDIO;
	result = inv_i2c_single_write(st, REG_3050_AUX_VDDIO, data);
	if (result)
		return result;

	reg = &st->reg;
	result = set_inv_enable(indio_dev, 0);
	if (result)
		return result;
	/*2000dps full scale range*/
	result = inv_i2c_single_write(st, reg->lpf,
				(INV_FSR_2000DPS << GYRO_CONFIG_FSR_SHIFT)
				| INV_FILTER_42HZ);
	if (result)
		return result;
	st->chip_config.fsr = INV_FSR_2000DPS;
	st->chip_config.lpf = INV_FILTER_42HZ;
	result = inv_i2c_single_write(st, reg->sample_rate_div,
					ONE_K_HZ/INIT_FIFO_RATE - 1);
	if (result)
		return result;
	st->chip_config.fifo_rate = INIT_FIFO_RATE;
	st->irq_dur_us            = INIT_DUR_TIME;
	st->chip_config.prog_start_addr = DMP_START_ADDR;
	st->chip_config.gyro_enable = 1;
	st->chip_config.gyro_fifo_enable = 1;
	if (SECONDARY_SLAVE_TYPE_ACCEL == st->plat_data.sec_slave_type) {
		result = st->mpu_slave->setup(st);
		if (result)
			return result;
		result = st->mpu_slave->set_fs(st, INV_FS_02G);
		if (result)
			return result;
		result = st->mpu_slave->set_lpf(st, INIT_FIFO_RATE);
		if (result)
			return result;
		st->chip_config.accl_enable = 1;
		st->chip_config.accl_fifo_enable = 1;
	}
	return 0;
}
/**
 *  set_power_mpu3050() - set power of mpu3050.
 *  @st:	Device driver instance.
 *  @power_on:  on/off
 */
int set_power_mpu3050(struct inv_gyro_state_s *st,
	unsigned char power_on)
{
	struct inv_reg_map_s *reg;
	unsigned char data, p;
	int result;
	reg = &st->reg;
	if (power_on)
		data = 0;
	else {
		if (st->mpu_slave) {
			result = st->mpu_slave->suspend(st);
			if (result)
				return result;
		}
		data = BIT_SLEEP;
	}
	if (st->chip_config.gyro_enable) {
		p = (BITS_3050_POWER1 | INV_CLK_PLL);
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1, data | p);
		if (result)
			return result;

		p = (BITS_3050_POWER2 | INV_CLK_PLL);
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1, data | p);
		if (result)
			return result;

		p = INV_CLK_PLL;
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1, data | p);
		if (result)
			return result;

		st->chip_config.clk_src = INV_CLK_PLL;
	} else {
		data |= (BITS_3050_GYRO_STANDBY | INV_CLK_INTERNAL);
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1, data);
		if (result)
			return result;
		st->chip_config.clk_src = INV_CLK_INTERNAL;
	}
	if (power_on) {
		msleep(POWER_UP_TIME);
		if (st->mpu_slave) {
			result = st->mpu_slave->resume(st);
			if (result)
				return result;
		}
		st->chip_config.is_asleep = 0;
	} else
		st->chip_config.is_asleep = 1;
	return 0;
}
/**
 *  @}
 */

