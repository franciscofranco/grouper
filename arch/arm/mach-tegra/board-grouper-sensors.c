
/*
 * arch/arm/mach-tegra/board-grouper-sensors.c
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/cm3217.h>
#include <linux/mpu.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <media/ov2710.h>
#include "board.h"
#include "board-grouper.h"
#include "cpu-tegra.h"
#include <linux/nct1008.h>
#include <mach/thermal.h>
#include <linux/slab.h>
static struct regulator *grouper_1v8_cam3;
static struct regulator *grouper_vdd_cam3;

static const struct i2c_board_info cardhu_i2c1_board_info_al3010[] = {
	{
		I2C_BOARD_INFO("al3010",0x1C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ2),
	},
};

static struct cm3217_platform_data grouper_cm3217_pdata = {
	.levels = {10, 160, 225, 320, 640, 1280, 2600, 5800, 8000, 10240},
	.golden_adc = 0,
	.power = 0,
};

static struct i2c_board_info grouper_i2c0_cm3217_board_info[] = {
	{
		I2C_BOARD_INFO("cm3217", 0x10),
		.platform_data = &grouper_cm3217_pdata,
	},
};

static int grouper_camera_init(void)
{
	int ret;

	tegra_gpio_enable(CAM2_POWER_DWN_GPIO);
	ret = gpio_request(CAM2_POWER_DWN_GPIO, "cam2_power_en");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM2_POWER_DWN_GPIO");
	}

	gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
	mdelay(10);

	tegra_gpio_enable(CAM2_RST_GPIO);
	ret = gpio_request(CAM2_RST_GPIO, "cam2_reset");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM2_RST_GPIO");
	}

	gpio_direction_output(CAM2_RST_GPIO, 0);
	mdelay(5);

	return 0;
}

static int grouper_ov2710_power_on(void)
{
	gpio_direction_output(CAM2_POWER_DWN_GPIO, 0);
	mdelay(10);

	if (grouper_vdd_cam3 == NULL) {
		grouper_vdd_cam3 = regulator_get(NULL, "vdd_cam3");
		if (WARN_ON(IS_ERR(grouper_vdd_cam3))) {
			pr_err("%s: couldn't get regulator vdd_cam3: %d\n",
				__func__, PTR_ERR(grouper_vdd_cam3));
			goto reg_get_vdd_cam3_fail;
		}
	}
	regulator_enable(grouper_vdd_cam3);

	if (grouper_1v8_cam3 == NULL) {
		grouper_1v8_cam3 = regulator_get(NULL, "vdd_1v8_cam3");
		if (WARN_ON(IS_ERR(grouper_1v8_cam3))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam3: %d\n",
				__func__, PTR_ERR(grouper_1v8_cam3));
			goto reg_get_vdd_1v8_cam3_fail;
		}
	}
	regulator_enable(grouper_1v8_cam3);
	mdelay(5);

	gpio_direction_output(CAM2_RST_GPIO, 1);
	mdelay(10);

	return 0;

reg_get_vdd_1v8_cam3_fail:
	grouper_1v8_cam3 = NULL;
	regulator_put(grouper_vdd_cam3);

reg_get_vdd_cam3_fail:
	grouper_vdd_cam3 = NULL;

	return -ENODEV;
}

static int grouper_ov2710_power_off(void)
{
	gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);

	gpio_direction_output(CAM2_RST_GPIO, 0);

	if (grouper_1v8_cam3)
		regulator_disable(grouper_1v8_cam3);
	if (grouper_vdd_cam3)
		regulator_disable(grouper_vdd_cam3);

	return 0;
}

struct ov2710_platform_data grouper_ov2710_data = {
	.power_on = grouper_ov2710_power_on,
	.power_off = grouper_ov2710_power_off,
};

static struct i2c_board_info grouper_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("ov2710", 0x36),
		.platform_data = &grouper_ov2710_data,
	},
};

/* MPU board file definition */

#if (MPU_GYRO_TYPE == MPU_TYPE_MPU3050)
#define MPU_GYRO_NAME		"mpu3050"
#endif
#if (MPU_GYRO_TYPE == MPU_TYPE_MPU6050)
#define MPU_GYRO_NAME		"mpu6050"
#endif

static struct mpu_platform_data mpu_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	.orientation	= MPU_GYRO_ORIENTATION,
};

#if (MPU_GYRO_TYPE == MPU_TYPE_MPU3050)
static struct ext_slave_platform_data mpu_accel_data = {
	.address	= MPU_ACCEL_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_ACCEL_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_SECONDARY,
	.orientation	= MPU_ACCEL_ORIENTATION,
};
#endif

static struct ext_slave_platform_data mpu_compass_data = {
	.address	= MPU_COMPASS_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_COMPASS_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_PRIMARY,
	.orientation	= MPU_COMPASS_ORIENTATION,
};

static struct i2c_board_info __initdata inv_mpu_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(MPU_GYRO_IRQ_GPIO),
		.platform_data = &mpu_gyro_data,
	},
#if (MPU_GYRO_TYPE == MPU_TYPE_MPU3050)
	{
		I2C_BOARD_INFO(MPU_ACCEL_NAME, MPU_ACCEL_ADDR),
#if MPU_ACCEL_IRQ_GPIO
		.irq = TEGRA_GPIO_TO_IRQ(MPU_ACCEL_IRQ_GPIO),
#endif
		.platform_data = &mpu_accel_data,
	},
#endif
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
#if MPU_COMPASS_IRQ_GPIO
		.irq = TEGRA_GPIO_TO_IRQ(MPU_COMPASS_IRQ_GPIO),
#endif
		.platform_data = &mpu_compass_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;

	pr_info("*** MPU START *** mpuirq_init...\n");

#if (MPU_GYRO_TYPE == MPU_TYPE_MPU3050)
#if MPU_ACCEL_IRQ_GPIO
	/* ACCEL-IRQ assignment */
	tegra_gpio_enable(MPU_ACCEL_IRQ_GPIO);
	ret = gpio_request(MPU_ACCEL_IRQ_GPIO, MPU_ACCEL_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_ACCEL_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_ACCEL_IRQ_GPIO);
		return;
	}
#endif
#endif

	/* MPU-IRQ assignment */
	tegra_gpio_enable(MPU_GYRO_IRQ_GPIO);
	ret = gpio_request(MPU_GYRO_IRQ_GPIO, MPU_GYRO_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_GYRO_IRQ_GPIO);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c0_board_info,
		ARRAY_SIZE(inv_mpu_i2c0_board_info));
}

#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data, shutdown_temp);
}

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *thermal_device;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	thermal_device->name = "nct1008";
	thermal_device->data = data;
	thermal_device->offset = TDIODE_OFFSET;
	thermal_device->get_temp = nct_get_temp;
	thermal_device->get_temp_low = nct_get_temp_low;
	thermal_device->set_limits = nct_set_limits;
	thermal_device->set_alert = nct_set_alert;
	thermal_device->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_set_device(thermal_device);
}
#endif

static struct nct1008_platform_data grouper_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
	.probe_callback = nct1008_probe_callback,
#endif
};

static struct i2c_board_info cardhu_i2c4_bq27541_board_info[] = {
	{
		I2C_BOARD_INFO("bq27541-battery", 0x55),
	}
};

static struct i2c_board_info grouper_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &grouper_nct1008_pdata,
		.irq = -1,
	}
};

static int grouper_nct1008_init(void)
{
	int nct1008_port = -1;
	int ret = 0;

	nct1008_port = TEGRA_GPIO_PCC2;
	if (nct1008_port >= 0) {
		/* FIXME: enable irq when throttling is supported */
		grouper_i2c4_nct1008_board_info[0].irq = TEGRA_GPIO_TO_IRQ(nct1008_port);

		ret = gpio_request(nct1008_port, "temp_alert");
		if (ret < 0)
			return ret;

		ret = gpio_direction_input(nct1008_port);
		if (ret < 0)
			gpio_free(nct1008_port);
		else
			tegra_gpio_enable(nct1008_port);
	}

	return ret;
}

int __init grouper_sensors_init(void)
{
	grouper_camera_init();

	i2c_register_board_info(2, grouper_i2c2_board_info,
		ARRAY_SIZE(grouper_i2c2_board_info));

	i2c_register_board_info(0, grouper_i2c0_cm3217_board_info,
		ARRAY_SIZE(grouper_i2c0_cm3217_board_info));

	i2c_register_board_info(4, grouper_i2c4_nct1008_board_info,
		ARRAY_SIZE(grouper_i2c4_nct1008_board_info));

	mpuirq_init();

	i2c_register_board_info(2, cardhu_i2c1_board_info_al3010,
		ARRAY_SIZE(cardhu_i2c1_board_info_al3010));

	return 0;
}
