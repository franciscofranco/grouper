

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
#include <linux/mpu.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
#ifdef CONFIG_VIDEO_OV2710
#include <media/ov2710.h>
#endif
#include <media/yuv_sensor.h>
#include "board.h"
#include "board-grouper.h"
#include "cpu-tegra.h"
#include <linux/nct1008.h>
#include <mach/thermal.h>
#include <linux/slab.h>
#include <mach/board-grouper-misc.h>

#define CAM1_LDO_EN_GPIO		TEGRA_GPIO_PR6
#define FRONT_YUV_SENSOR_RST_GPIO	TEGRA_GPIO_PO0

static struct regulator *grouper_1v8_ldo5;
static struct regulator *grouper_1v8_cam3;
static struct regulator *grouper_vdd_cam3;

static unsigned int pmic_id;

static const struct i2c_board_info cardhu_i2c1_board_info_al3010[] = {
	{
		I2C_BOARD_INFO("al3010",0x1C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ2),
	},
};

static int grouper_camera_init(void)
{
	pmic_id = grouper_query_pmic_id();
	printk("%s: pmic_id= 0x%X", __FUNCTION__, pmic_id);
#if 0
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
#endif
	return 0;
}

#ifdef CONFIG_VIDEO_OV2710
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
#endif

static int yuv_front_sensor_power_on(void)
{
	int ret;
	printk("yuv_front_sensor_power_on+\n");

	/* AVDD_CAM1, 2.85V, controlled by CAM1_LDO_EN */
	pr_info("gpio %d read as %d\n",CAM1_LDO_EN_GPIO, gpio_get_value(CAM1_LDO_EN_GPIO));
	tegra_gpio_enable(CAM1_LDO_EN_GPIO);
	ret = gpio_request(CAM1_LDO_EN_GPIO, "cam1_ldo_en");
	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %s, ret= %d\n",
			__func__, "CAM1_LDO_EN_GPIO", ret);
	pr_info("gpio %d: %d", CAM1_LDO_EN_GPIO, gpio_get_value(CAM1_LDO_EN_GPIO));
	gpio_set_value(CAM1_LDO_EN_GPIO, 1);
	gpio_direction_output(CAM1_LDO_EN_GPIO, 1);
	pr_info("--> %d\n", gpio_get_value(CAM1_LDO_EN_GPIO));

	msleep(5);

	if (!grouper_1v8_ldo5) {
		if(pmic_id == GROUPER_PMIC_MAXIM) {
			grouper_1v8_ldo5 = regulator_get(NULL, "vdd_sensor_1v8");
		} else if (pmic_id == GROUPER_PMIC_TI) {
			grouper_1v8_ldo5 = regulator_get(NULL, "avdd_vdac");
		}
		if (IS_ERR_OR_NULL(grouper_1v8_ldo5)) {
			grouper_1v8_ldo5 = NULL;
			pr_err("Can't get grouper_1v8_ldo5.\n");
			goto fail_to_get_reg;
		}
		regulator_set_voltage(grouper_1v8_ldo5, 1800000, 1800000);
		regulator_enable(grouper_1v8_ldo5);
	}

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CAM_MCLK, TEGRA_TRI_NORMAL);

	/* yuv_sensor_rst_lo*/
	tegra_gpio_enable(FRONT_YUV_SENSOR_RST_GPIO);
	ret = gpio_request(FRONT_YUV_SENSOR_RST_GPIO, "yuv_sensor_rst_lo");

	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %s, ret= %d\n",
			__func__, "FRONT_YUV_SENSOR_RST_GPIO", ret);
	pr_info("gpio %d: %d", FRONT_YUV_SENSOR_RST_GPIO, gpio_get_value(FRONT_YUV_SENSOR_RST_GPIO));
	gpio_set_value(FRONT_YUV_SENSOR_RST_GPIO, 1);
	gpio_direction_output(FRONT_YUV_SENSOR_RST_GPIO, 1);
	pr_info("--> %d\n", gpio_get_value(FRONT_YUV_SENSOR_RST_GPIO));

	printk("yuv_front_sensor_power_on-\n");
	return 0;

fail_to_get_reg:
	if (grouper_1v8_ldo5) {
		regulator_put(grouper_1v8_ldo5);
		grouper_1v8_ldo5 = NULL;
	}

	gpio_set_value(CAM1_LDO_EN_GPIO, 0);
	gpio_direction_output(CAM1_LDO_EN_GPIO, 0);
	gpio_free(CAM1_LDO_EN_GPIO);

	printk("yuv_front_sensor_power_on- : -ENODEV\n");
	return -ENODEV;
}

static int yuv_front_sensor_power_off(void)
{
	printk("%s+\n", __FUNCTION__);

	if((pmic_id == GROUPER_PMIC_MAXIM) || (pmic_id == GROUPER_PMIC_TI)) {
		gpio_set_value(FRONT_YUV_SENSOR_RST_GPIO, 0);
		gpio_direction_output(FRONT_YUV_SENSOR_RST_GPIO, 0);
		gpio_free(FRONT_YUV_SENSOR_RST_GPIO);

		tegra_pinmux_set_tristate(TEGRA_PINGROUP_CAM_MCLK, TEGRA_TRI_TRISTATE);

		if (grouper_1v8_ldo5) {
			regulator_disable(grouper_1v8_ldo5);
			regulator_put(grouper_1v8_ldo5);
			grouper_1v8_ldo5 = NULL;
		}

		msleep(5);

		gpio_set_value(CAM1_LDO_EN_GPIO, 0);
		gpio_direction_output(CAM1_LDO_EN_GPIO, 0);
		gpio_free(CAM1_LDO_EN_GPIO);

		printk("%s-\n", __FUNCTION__);
		return 0;
	} else {
		printk("%s- Unknow pmic_id: 0x%X\n", __FUNCTION__, pmic_id);
		return -ENODEV;
	}
}

struct yuv_sensor_platform_data yuv_front_sensor_data = {
	.power_on = yuv_front_sensor_power_on,
	.power_off = yuv_front_sensor_power_off,
};

static struct i2c_board_info front_sensor_i2c2_board_info[] = {  //ddebug
	{
		I2C_BOARD_INFO("mi1040", 0x48),
		.platform_data = &yuv_front_sensor_data,
	},
};
/* MPU board file definition	*/
#if (MPU_GYRO_TYPE == MPU_TYPE_MPU3050)
#define MPU_GYRO_NAME		"mpu3050"
static struct mpu_platform_data mpu_gyro_data = {
	.int_config  = 0x10,
	.level_shifter = 0,
	.orientation = MPU_GYRO_ORIENTATION,	/* Located in board_[platformname].h	*/
	.sec_slave_type = SECONDARY_SLAVE_TYPE_ACCEL,
	.sec_slave_id   = ACCEL_ID_BMA250,
	.secondary_i2c_addr = MPU_ACCEL_ADDR,
	.secondary_orientation = MPU_ACCEL_ORIENTATION,	/* Located in board_[platformname].h	*/
	.key = {221, 22, 205, 7,   217, 186, 151, 55,
		206, 254, 35, 144, 225, 102,  47, 50},
};
#endif
#if (MPU_GYRO_TYPE == MPU_TYPE_MPU6050)
#define MPU_GYRO_NAME		"mpu6050"
static struct mpu_platform_data mpu_gyro_data = {
        .int_config  = 0x10,
        .level_shifter = 0,
        .orientation = MPU_GYRO_ORIENTATION,	/* Located in board_[platformname].h	*/
        .sec_slave_type = SECONDARY_SLAVE_TYPE_NONE,
        .key = {221, 22, 205, 7,   217, 186, 151, 55,
                206, 254, 35, 144, 225, 102,  47, 50},
};
#endif
static struct mpu_platform_data mpu_compass_data = {
	.orientation = MPU_COMPASS_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct i2c_board_info __initdata inv_mpu_i2c2_board_info[] = {
        {
                I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
                .irq = TEGRA_GPIO_TO_IRQ(MPU_GYRO_IRQ_GPIO),
                .platform_data = &mpu_gyro_data,
        },
        {
                I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
                .platform_data = &mpu_compass_data,
        },
};

static void mpuirq_init(void)
{
	int ret = 0;

	pr_info("*** MPU START *** mpuirq_init...\n");

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

	i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c2_board_info,
		ARRAY_SIZE(inv_mpu_i2c2_board_info));
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

	nct1008_port = TEGRA_GPIO_PS3;
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
	int err;
	grouper_camera_init();

#ifdef CONFIG_VIDEO_OV2710
	i2c_register_board_info(2, grouper_i2c2_board_info,
		ARRAY_SIZE(grouper_i2c2_board_info));

#endif
/* Front Camera mi1040 + */
    pr_info("mi1040 i2c_register_board_info");
	i2c_register_board_info(2, front_sensor_i2c2_board_info,
		ARRAY_SIZE(front_sensor_i2c2_board_info));

	err = grouper_nct1008_init();
	if (err)
		printk("[Error] Thermal: Configure GPIO_PCC2 as an irq fail!");
	i2c_register_board_info(4, grouper_i2c4_nct1008_board_info,
		ARRAY_SIZE(grouper_i2c4_nct1008_board_info));

	mpuirq_init();

	i2c_register_board_info(2, cardhu_i2c1_board_info_al3010,
		ARRAY_SIZE(cardhu_i2c1_board_info_al3010));

	return 0;
}
