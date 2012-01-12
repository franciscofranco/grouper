/*
 * arch/arm/mach-tegra/board-kai-power.c
 *
 * Copyright (C) 2012 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/max77663-core.h>
#include <linux/regulator/max77663-regulator.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/regulator/gpio-switch-regulator.h>
#include <linux/power/gpio-charger.h>

#include <asm/mach-types.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/edp.h>

#include "gpio-names.h"
#include "board.h"
#include "board-kai.h"
#include "pm.h"
#include "wakeups-t3.h"
#include "tegra3_tsensor.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

static struct regulator_consumer_supply max77663_sd0_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply max77663_sd1_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply max77663_sd2_supply[] = {
	REGULATOR_SUPPLY("vdd_gen1v8", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vddio_lcd_pmu", NULL),
	REGULATOR_SUPPLY("pwrdet_lcd", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
	REGULATOR_SUPPLY("vddio_vi", NULL),
	REGULATOR_SUPPLY("pwrdet_vi", NULL),
	REGULATOR_SUPPLY("vcore_nand", NULL),
	REGULATOR_SUPPLY("pwrdet_nand", NULL),
};

static struct regulator_consumer_supply max77663_sd3_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr3l_1v35", NULL),
};

static struct regulator_consumer_supply max77663_ldo0_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply max77663_ldo1_supply[] = {
};

static struct regulator_consumer_supply max77663_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
};

static struct regulator_consumer_supply max77663_ldo3_supply[] = {
	REGULATOR_SUPPLY("vmmc", NULL),
};

static struct regulator_consumer_supply max77663_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply max77663_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_sensor_2v8", NULL),
};

static struct regulator_consumer_supply max77663_ldo6_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
};

static struct regulator_consumer_supply max77663_ldo7_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
};

static struct regulator_consumer_supply max77663_ldo8_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d2", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
};

static struct max77663_regulator_fps_cfg max77663_fps_cfgs[] = {
	{
		.src = FPS_SRC_0,
		.en_src = FPS_EN_SRC_EN0,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
	{
		.src = FPS_SRC_1,
		.en_src = FPS_EN_SRC_EN1,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
	{
		.src = FPS_SRC_2,
		.en_src = FPS_EN_SRC_EN0,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
};

#define MAX77663_PDATA_INIT(_id, _min_uV, _max_uV, _supply_reg,		\
			    _always_on, _boot_on, _apply_uV,		\
			    _init_apply, _init_enable, _init_uV,	\
			    _fps_src, _fps_pu_period, _fps_pd_period, _flags) \
	static struct max77663_regulator_platform_data max77663_regulator_pdata_##_id = \
	{								\
		.init_data = {						\
			.constraints = {				\
				.min_uV = _min_uV,			\
				.max_uV = _max_uV,			\
				.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
						     REGULATOR_MODE_STANDBY), \
				.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
						   REGULATOR_CHANGE_STATUS |  \
						   REGULATOR_CHANGE_VOLTAGE), \
				.always_on = _always_on,		\
				.boot_on = _boot_on,			\
				.apply_uV = _apply_uV,			\
			},						\
			.num_consumer_supplies =			\
				ARRAY_SIZE(max77663_##_id##_supply),	\
			.consumer_supplies = max77663_##_id##_supply,	\
			.supply_regulator = _supply_reg,		\
		},							\
		.init_apply = _init_apply,				\
		.init_enable = _init_enable,				\
		.init_uV = _init_uV,					\
		.fps_src = _fps_src,					\
		.fps_pu_period = _fps_pu_period,			\
		.fps_pd_period = _fps_pd_period,			\
		.fps_cfgs = max77663_fps_cfgs,				\
		.flags = _flags,					\
	}

MAX77663_PDATA_INIT(sd0,  600000, 3387500, NULL, 1, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, EN2_CTRL_SD0 | SD_FSRADE_DISABLE);

MAX77663_PDATA_INIT(sd1,  800000, 1587500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, SD_FSRADE_DISABLE);

MAX77663_PDATA_INIT(sd2,  600000, 3387500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(sd3,  600000, 3387500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo0, 800000, 2350000, max77663_rails(sd3), 1, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(ldo1, 800000, 2350000, max77663_rails(sd3), 0, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo2, 800000, 3950000, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(ldo3, 800000, 3950000, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo4, 800000, 1587500, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo5, 800000, 2800000, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo6, 800000, 3950000, NULL, 0, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo7, 800000, 3950000, max77663_rails(sd3), 0, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo8, 800000, 3950000, max77663_rails(sd3), 0, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, 0);

#define MAX77663_REG(_id, _data)					\
	{								\
		.name = "max77663-regulator",				\
		.id = MAX77663_REGULATOR_ID_##_id,			\
		.platform_data = &max77663_regulator_pdata_##_data,		\
	}

#define MAX77663_RTC()							\
	{								\
		.name = "max77663-rtc",					\
		.id = 0,						\
	}

static struct mfd_cell max77663_subdevs[] = {
	MAX77663_REG(SD0, sd0),
	MAX77663_REG(SD1, sd1),
	MAX77663_REG(SD2, sd2),
	MAX77663_REG(SD3, sd3),
	MAX77663_REG(LDO0, ldo0),
	MAX77663_REG(LDO1, ldo1),
	MAX77663_REG(LDO2, ldo2),
	MAX77663_REG(LDO3, ldo3),
	MAX77663_REG(LDO4, ldo4),
	MAX77663_REG(LDO5, ldo5),
	MAX77663_REG(LDO6, ldo6),
	MAX77663_REG(LDO7, ldo7),
	MAX77663_REG(LDO8, ldo8),
	MAX77663_RTC(),
};

static struct max77663_gpio_config max77663_gpio_cfgs[] = {
	{
		.gpio = MAX77663_GPIO0,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO1,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO2,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO3,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO4,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.alternate = GPIO_ALT_ENABLE,
	},
	{
		.gpio = MAX77663_GPIO5,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO6,
		.dir = GPIO_DIR_IN,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO7,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
};

static struct max77663_platform_data max7763_pdata = {
	.irq_base	= MAX77663_IRQ_BASE,
	.gpio_base	= MAX77663_GPIO_BASE,

	.num_gpio_cfgs	= ARRAY_SIZE(max77663_gpio_cfgs),
	.gpio_cfgs	= max77663_gpio_cfgs,

	.num_subdevs	= ARRAY_SIZE(max77663_subdevs),
	.sub_devices	= max77663_subdevs,
};

static struct i2c_board_info __initdata max77663_regulators[] = {
	{
		/* The I2C address was determined by OTP factory setting */
		I2C_BOARD_INFO("max77663", 0x3c),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &max7763_pdata,
	},
};

static int __init kai_max77663_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	i2c_register_board_info(4, max77663_regulators,
				ARRAY_SIZE(max77663_regulators));

	return 0;
}

static struct regulator_consumer_supply gpio_switch_en_3v3_sys_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3", NULL),
	REGULATOR_SUPPLY("vdd_3v3_devices", NULL),
	REGULATOR_SUPPLY("debug_cons", NULL),
	REGULATOR_SUPPLY("pwrdet_pex_ctl", NULL),
};
static int gpio_switch_en_3v3_sys_voltages[] = { 3300};

static struct regulator_consumer_supply gpio_switch_en_avdd_hdmi_usb_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
};
static int gpio_switch_en_avdd_hdmi_usb_voltages[] = { 3300};

static struct regulator_consumer_supply gpio_switch_en_1v8_cam_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_cam1", NULL),
	REGULATOR_SUPPLY("vdd_1v8_cam2", NULL),
	REGULATOR_SUPPLY("vdd_1v8_cam3", NULL),
};
static int gpio_switch_en_1v8_cam_voltages[] = { 1800};

static struct regulator_consumer_supply gpio_switch_en_vddio_vid_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_con", NULL),
};
static int gpio_switch_en_vddio_vid_voltages[] = { 5000};

static struct regulator_consumer_supply gpio_switch_en_3v3_modem_supply[] = {
	REGULATOR_SUPPLY("vdd_mini_card", NULL),
};
static int gpio_switch_en_3v3_modem_voltages[] = { 3300};

static struct regulator_consumer_supply gpio_switch_en_vdd_pnl_supply[] = {
	REGULATOR_SUPPLY("vdd_lvds", NULL),
	REGULATOR_SUPPLY("vdd_lcd_panel", NULL),
	REGULATOR_SUPPLY("vdd_touch", NULL),
	REGULATOR_SUPPLY("vddio_ts", NULL),
};
static int gpio_switch_en_vdd_pnl_voltages[] = { 3300};

static struct regulator_consumer_supply gpio_switch_en_cam3_ldo_supply[] = {
	REGULATOR_SUPPLY("vdd_cam3", NULL),
};
static int gpio_switch_en_cam3_ldo_voltages[] = { 3300};

static struct regulator_consumer_supply gpio_switch_en_vdd_com_supply[] = {
	REGULATOR_SUPPLY("vdd_com_bd", NULL),
};
static int gpio_switch_en_vdd_com_voltages[] = { 3300};

static struct regulator_consumer_supply gpio_switch_en_vdd_sdmmc1_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.0"),
};
static int gpio_switch_en_vdd_sdmmc1_voltages[] = { 3300};

static struct regulator_consumer_supply gpio_switch_en_3v3_fuse_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};
static int gpio_switch_en_3v3_fuse_voltages[] = { 3300};

static struct regulator_consumer_supply gpio_switch_cdc_en_supply[] = {
	REGULATOR_SUPPLY("cdc_en", NULL),
};
static int gpio_switch_cdc_en_voltages[] = { 1200};

/* Macro for defining gpio switch regulator sub device data */
#define GREG_INIT(_id, _var, _name, _input_supply, _always_on, _boot_on, \
	_gpio_nr, _active_low, _init_state, _pg, _enable, _disable)	 \
	static struct gpio_switch_regulator_subdev_data gpio_pdata_##_var =  \
	{								\
		.regulator_name	= "gpio-switch-"#_name,			\
		.input_supply	= _input_supply,			\
		.id		= _id,					\
		.gpio_nr	= _gpio_nr,				\
		.pin_group	= _pg,					\
		.active_low	= _active_low,				\
		.init_state	= _init_state,				\
		.voltages	= gpio_switch_##_name##_voltages,	\
		.n_voltages	= ARRAY_SIZE(gpio_switch_##_name##_voltages), \
		.num_consumer_supplies =				\
				ARRAY_SIZE(gpio_switch_##_name##_supply), \
		.consumer_supplies = gpio_switch_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					     REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					   REGULATOR_CHANGE_STATUS |	\
					   REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,			\
			.boot_on = _boot_on,				\
		},							\
		.enable_rail = _enable,					\
		.disable_rail = _disable,				\
	}

GREG_INIT(1, en_3v3_sys,	en_3v3_sys,		NULL,
	1,	0,	MAX77663_GPIO_BASE + MAX77663_GPIO3,	false,	1,	0,	0,	0);
GREG_INIT(2, en_avdd_hdmi_usb,	en_avdd_hdmi_usb,	"vdd_3v3_devices",
	1,	0,	MAX77663_GPIO_BASE + MAX77663_GPIO2,	false,	1,	0,	0,	0);
GREG_INIT(3, en_1v8_cam,	en_1v8_cam,		"vdd_gen1v8",
	0,	0,	TEGRA_GPIO_PS0,				false,	0,	0,	0,	0);
GREG_INIT(4, en_vddio_vid_oc,	en_vddio_vid,		NULL,
	0,	0,	TEGRA_GPIO_PB2,				false,	0,	0,	0,	0);
GREG_INIT(5, en_3v3_modem,	en_3v3_modem,		NULL,
	0,	0,	TEGRA_GPIO_PP0,				false,	0,	0,	0,	0);
GREG_INIT(6, en_vdd_pnl,	en_vdd_pnl,		"vdd_3v3_devices",
	0,	0,	TEGRA_GPIO_PW1,				false,	0,	0,	0,	0);
GREG_INIT(7, en_cam3_ldo,	en_cam3_ldo,		"vdd_3v3_devices",
	0,	0,	TEGRA_GPIO_PR7,				false,	0,	0,	0,	0);
GREG_INIT(8, en_vdd_com,	en_vdd_com,		"vdd_3v3_devices",
	0,	0,	TEGRA_GPIO_PD0,				false,	0,	0,	0,	0);
GREG_INIT(9,  en_vdd_sdmmc1,	en_vdd_sdmmc1,		"vdd_3v3_devices",
	0,	0,	TEGRA_GPIO_PC6,				false,	0,	0,	0,	0);
GREG_INIT(10, en_3v3_fuse,	en_3v3_fuse,		"vdd_3v3_devices",
	0,	0,	TEGRA_GPIO_PC1,				false,	0,	0,	0,	0);
GREG_INIT(11, cdc_en,		cdc_en,			"vddio_audio",
	0,	1,	TEGRA_GPIO_PX2,				false,	0,	0,	0,	0);


#define ADD_GPIO_REG(_name) &gpio_pdata_##_name

#define E1565_GPIO_REG \
	ADD_GPIO_REG(en_3v3_sys),		\
	ADD_GPIO_REG(en_avdd_hdmi_usb),		\
	ADD_GPIO_REG(en_1v8_cam),		\
	ADD_GPIO_REG(en_vddio_vid_oc),		\
	ADD_GPIO_REG(en_3v3_modem),		\
	ADD_GPIO_REG(en_vdd_pnl),		\
	ADD_GPIO_REG(en_cam3_ldo),		\
	ADD_GPIO_REG(en_vdd_com),		\
	ADD_GPIO_REG(en_vdd_sdmmc1),		\
	ADD_GPIO_REG(en_3v3_fuse),		\
	ADD_GPIO_REG(cdc_en),			\


static struct gpio_switch_regulator_subdev_data *gswitch_subdevs[] = {
	E1565_GPIO_REG
};

static struct gpio_switch_regulator_platform_data gswitch_pdata = {
	.subdevs = gswitch_subdevs,
	.num_subdevs = ARRAY_SIZE(gswitch_subdevs),
};

static struct platform_device gswitch_regulator_pdata = {
	.name = "gpio-switch-regulator",
	.id   = -1,
	.dev  = {
	     .platform_data = &gswitch_pdata,
	},
};

static int __init kai_max77663_gpio_switch_regulator_init(void)
{
	int i;

	for (i = 0; i < gswitch_pdata.num_subdevs; ++i) {
		struct gpio_switch_regulator_subdev_data *gswitch_data =
						gswitch_pdata.subdevs[i];
		if (gswitch_data->gpio_nr <= TEGRA_NR_GPIOS)
			tegra_gpio_enable(gswitch_data->gpio_nr);
	}

	return platform_device_register(&gswitch_regulator_pdata);
}

int __init kai_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
	int ret;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	ret = kai_max77663_regulator_init();
	if (ret < 0)
		return ret;

	return kai_max77663_gpio_switch_regulator_init();
}

static void kai_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void kai_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data kai_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.cpu_lp2_min_residency = 2000,
	.board_suspend = kai_board_suspend,
	.board_resume = kai_board_resume,
};

int __init kai_suspend_init(void)
{
	tegra_init_suspend(&kai_suspend_data);
	return 0;
}

static void kai_power_off(void)
{
	int ret;
	pr_err("kai: Powering off the device\n");
	ret = max77663_power_off();
	if (ret)
		pr_err("kai: failed to power off\n");

	while (1)
		;
}

int __init kai_power_off_init(void)
{
	pm_power_off = kai_power_off;

	return 0;
}

static struct tegra_tsensor_pmu_data  tpdata = {
	.poweroff_reg_addr = 0x3F,
	.poweroff_reg_data = 0x80,
	.reset_tegra = 1,
	.controller_type = 0,
	.i2c_controller_id = 4,
	.pinmux = 0,
	.pmu_16bit_ops = 0,
	.pmu_i2c_addr = 0x2D,
};

void __init kai_tsensor_init(void)
{
	tegra3_tsensor_init(&tpdata);
}

#ifdef CONFIG_TEGRA_EDP_LIMITS

int __init kai_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA)
		regulator_mA = 6000; /* regular T30/s */
	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_cpu_edp_limits(regulator_mA);
	return 0;
}
#endif
