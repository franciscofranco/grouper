/*
 * arch/arm/mach-tegra/board-grouper-panel.c
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
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/pwm_backlight.h>
#include <asm/atomic.h>
#include <linux/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "board.h"
#include "board-grouper.h"
#include "devices.h"
#include "gpio-names.h"

/* grouper default display board pins */
#define grouper_lvds_avdd_en		TEGRA_GPIO_PH6
#define grouper_lvds_rst			TEGRA_GPIO_PG7
#define grouper_lvds_shutdown		TEGRA_GPIO_PN6
#define grouper_lvds_rs			TEGRA_GPIO_PV6
#define grouper_lvds_lr			TEGRA_GPIO_PG1

/* grouper A00 display board pins */
#define grouper_lvds_rs_a00		TEGRA_GPIO_PH1

/* common pins( backlight ) for all display boards */
//#define grouper_bl_enb			TEGRA_GPIO_PH3
#define grouper_bl_pwm			TEGRA_GPIO_PH0
#define grouper_hdmi_hpd			TEGRA_GPIO_PN7

#ifdef CONFIG_TEGRA_DC
static struct regulator *grouper_hdmi_reg;
static struct regulator *grouper_hdmi_pll;
static struct regulator *grouper_hdmi_vddio;
#endif

static atomic_t sd_brightness = ATOMIC_INIT(255);

static struct regulator *grouper_lvds_reg;
static struct regulator *grouper_lvds_vdd_panel;

static tegra_dc_bl_output grouper_bl_output_measured = {
	0, 13, 13, 13, 13, 13, 13, 13,
	13, 13, 13, 13, 13, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 49, 50, 51, 52, 53, 54,
	55, 56, 57, 58, 59, 60, 61, 62,
	63, 64, 65, 66, 67, 68, 69, 70,
	70, 72, 73, 74, 75, 76, 77, 78,
	79, 80, 81, 82, 83, 84, 85, 86,
	87, 88, 89, 90, 91, 92, 93, 94,
	95, 96, 97, 98, 99, 100, 101, 102,
	103, 104, 105, 106, 107, 108, 110, 111,
	112, 113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 123, 124, 124, 125, 126,
	127, 128, 129, 130, 131, 132, 133, 133,
	134, 135, 136, 137, 138, 139, 140, 141,
	142, 143, 144, 145, 146, 147, 148, 148,
	149, 150, 151, 152, 153, 154, 155, 156,
	157, 158, 159, 160, 161, 162, 163, 164,
	165, 166, 167, 168, 169, 170, 171, 172,
	173, 174, 175, 176, 177, 179, 180, 181,
	182, 184, 185, 186, 187, 188, 189, 190,
	191, 192, 193, 194, 195, 196, 197, 198,
	199, 200, 201, 202, 203, 204, 205, 206,
	207, 208, 209, 211, 212, 213, 214, 215,
	216, 217, 218, 219, 220, 221, 222, 223,
	224, 225, 226, 227, 228, 229, 230, 231,
	232, 233, 234, 235, 236, 237, 238, 239,
	240, 241, 242, 243, 244, 245, 246, 247,
	248, 249, 250, 251, 252, 253, 254, 255
};

static p_tegra_dc_bl_output bl_output;

static int grouper_backlight_init(struct device *dev)
{
	int ret = 0;

	bl_output = grouper_bl_output_measured;

	if (WARN_ON(ARRAY_SIZE(grouper_bl_output_measured) != 256))
		pr_err("bl_output array does not have 256 elements\n");

	tegra_gpio_disable(grouper_bl_pwm);

      /*
	ret = gpio_request(grouper_bl_enb, "backlight_enb");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(grouper_bl_enb, 1);
	if (ret < 0)
		gpio_free(grouper_bl_enb);
	else
		tegra_gpio_enable(grouper_bl_enb);
	*/

	return ret;
};

static void grouper_backlight_exit(struct device *dev)
{
	/* int ret; */
	/*ret = gpio_request(grouper_bl_enb, "backlight_enb");*/
	/*
	gpio_set_value(grouper_bl_enb, 0);
	gpio_free(grouper_bl_enb);
	tegra_gpio_disable(grouper_bl_enb);
	*/
	return;
}

static int grouper_backlight_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* Set the backlight GPIO pin mode to 'backlight_enable' */
	//gpio_set_value(grouper_bl_enb, !!brightness);

	/* SD brightness is a percentage, 8-bit value. */
	brightness = (brightness * cur_sd_brightness) / 255;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = bl_output[brightness];

	return brightness;
}

static int grouper_disp1_check_fb(struct device *dev, struct fb_info *info);

static struct platform_pwm_backlight_data grouper_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 40,
	.pwm_period_ns	= 50000,
	.init		= grouper_backlight_init,
	.exit		= grouper_backlight_exit,
	.notify		= grouper_backlight_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= grouper_disp1_check_fb,
};

static struct platform_device grouper_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &grouper_backlight_data,
	},
};

static int grouper_panel_postpoweron(void)
{
	if (grouper_lvds_reg == NULL) {
		grouper_lvds_reg = regulator_get(NULL, "vdd_lvds");
		if (WARN_ON(IS_ERR(grouper_lvds_reg)))
			pr_err("%s: couldn't get regulator vdd_lvds: %ld\n",
			       __func__, PTR_ERR(grouper_lvds_reg));
		else
			regulator_enable(grouper_lvds_reg);
	}

	mdelay(20);

//	gpio_set_value(grouper_lvds_avdd_en, 1);
//	mdelay(5);

	//gpio_set_value(grouper_lvds_stdby, 1);
//	gpio_set_value(grouper_lvds_rst, 1);
	gpio_set_value(grouper_lvds_shutdown, 1);
//	gpio_set_value(grouper_lvds_lr, 1);

	mdelay(200);

	return 0;
}

static int grouper_panel_enable(void)
{

	if (grouper_lvds_vdd_panel == NULL) {
		grouper_lvds_vdd_panel = regulator_get(NULL, "vdd_lcd_panel");
		if (WARN_ON(IS_ERR(grouper_lvds_vdd_panel)))
			pr_err("%s: couldn't get regulator vdd_lcd_panel: %ld\n",
			       __func__, PTR_ERR(grouper_lvds_vdd_panel));
		else
			regulator_enable(grouper_lvds_vdd_panel);
	}
	msleep(20);

	return 0;
}

static int grouper_panel_disable(void)
{
//	gpio_set_value(grouper_lvds_lr, 0);
//	mdelay(200);
//	gpio_set_value(grouper_lvds_shutdown, 0);
//	gpio_set_value(grouper_lvds_rst, 0);
	//gpio_set_value(grouper_lvds_stdby, 0);

//	gpio_set_value(grouper_lvds_avdd_en, 0);

	mdelay(5);
	if (grouper_lvds_reg) {
		regulator_disable(grouper_lvds_reg);
		regulator_put(grouper_lvds_reg);
		grouper_lvds_reg = NULL;
	}

	if (grouper_lvds_vdd_panel) {
		regulator_disable(grouper_lvds_vdd_panel);
		regulator_put(grouper_lvds_vdd_panel);
		grouper_lvds_vdd_panel = NULL;
	}

	return 0;
}

#ifdef CONFIG_TEGRA_DC
static int grouper_hdmi_vddio_enable(void)
{
	int ret;
	if (!grouper_hdmi_vddio) {
		grouper_hdmi_vddio = regulator_get(NULL, "vdd_hdmi_con");
		if (IS_ERR_OR_NULL(grouper_hdmi_vddio)) {
			ret = PTR_ERR(grouper_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator vdd_hdmi_con\n");
			grouper_hdmi_vddio = NULL;
			return ret;
		}
	}
	ret = regulator_enable(grouper_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator vdd_hdmi_con\n");
		regulator_put(grouper_hdmi_vddio);
		grouper_hdmi_vddio = NULL;
		return ret;
	}
	return ret;
}

static int grouper_hdmi_vddio_disable(void)
{
	if (grouper_hdmi_vddio) {
		regulator_disable(grouper_hdmi_vddio);
		regulator_put(grouper_hdmi_vddio);
		grouper_hdmi_vddio = NULL;
	}
	return 0;
}

static int grouper_hdmi_enable(void)
{
	int ret;
	if (!grouper_hdmi_reg) {
		grouper_hdmi_reg = regulator_get(NULL, "avdd_hdmi");
		if (IS_ERR_OR_NULL(grouper_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			grouper_hdmi_reg = NULL;
			return PTR_ERR(grouper_hdmi_reg);
		}
	}
	ret = regulator_enable(grouper_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!grouper_hdmi_pll) {
		grouper_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(grouper_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			grouper_hdmi_pll = NULL;
			regulator_put(grouper_hdmi_reg);
			grouper_hdmi_reg = NULL;
			return PTR_ERR(grouper_hdmi_pll);
		}
	}
	ret = regulator_enable(grouper_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int grouper_hdmi_disable(void)
{
	regulator_disable(grouper_hdmi_reg);
	regulator_put(grouper_hdmi_reg);
	grouper_hdmi_reg = NULL;

	regulator_disable(grouper_hdmi_pll);
	regulator_put(grouper_hdmi_pll);
	grouper_hdmi_pll = NULL;
	return 0;
}

static struct resource grouper_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by grouper_panel_init() */
		.end	= 0,	/* Filled in by grouper_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource grouper_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
		.start	= 0,
		.end	= 0,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif

static struct tegra_dc_mode grouper_panel_modes[] = {
	{
		/* 1280x800@60Hz */
		.pclk = 68000000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 24,
		.v_sync_width = 1,
		.h_back_porch = 32,
		.v_back_porch = 2,
		.h_active = 800,
		.v_active = 1280,
		.h_front_porch = 24,
		.v_front_porch = 5,
	},
};

static struct tegra_dc_sd_settings grouper_sd_settings = {
	.enable = 1, /* enabled by default. */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 1,
	.phase_in_adjustments = true,
	.panel_min_brightness = 13,
	.use_vid_luma = false,
	/* Default video coefficients */
	.coeff = {5, 9, 2},
	.fc = {0, 0},
	/* Immediate backlight changes */
	.blp = {1024, 255},
	/* Gammas: R: 2.2 G: 2.2 B: 2.2 */
	/* Default BL TF */
	.bltf = {
			{
				{57, 65, 74, 83},
				{93, 103, 114, 126},
				{138, 151, 165, 179},
				{194, 209, 225, 242},
			},
			{
				{58, 66, 75, 84},
				{94, 105, 116, 127},
				{140, 153, 166, 181},
				{196, 211, 227, 244},
			},
			{
				{60, 68, 77, 87},
				{97, 107, 119, 130},
				{143, 156, 170, 184},
				{199, 215, 231, 248},
			},
			{
				{64, 73, 82, 91},
				{102, 113, 124, 137},
				{149, 163, 177, 192},
				{207, 223, 240, 255},
			},
		},
	/* Default LUT */
	.lut = {
			{
				{250, 250, 250},
				{194, 194, 194},
				{149, 149, 149},
				{113, 113, 113},
				{82, 82, 82},
				{56, 56, 56},
				{34, 34, 34},
				{15, 15, 15},
				{0, 0, 0},
			},
			{
				{246, 246, 246},
				{191, 191, 191},
				{147, 147, 147},
				{111, 111, 111},
				{80, 80, 80},
				{55, 55, 55},
				{33, 33, 33},
				{14, 14, 14},
				{0, 0, 0},
			},
			{
				{239, 239, 239},
				{185, 185, 185},
				{142, 142, 142},
				{107, 107, 107},
				{77, 77, 77},
				{52, 52, 52},
				{30, 30, 30},
				{12, 12, 12},
				{0, 0, 0},
			},
			{
				{224, 224, 224},
				{173, 173, 173},
				{133, 133, 133},
				{99, 99, 99},
				{70, 70, 70},
				{46, 46, 46},
				{25, 25, 25},
				{7, 7, 7},
				{0, 0, 0},
			},
		},
	.sd_brightness = &sd_brightness,
	.bl_device = &grouper_backlight_device,
};

#ifdef CONFIG_TEGRA_DC
static struct tegra_fb_data grouper_fb_data = {
	.win		= 0,
	.xres		= 800,
	.yres		= 1280,
	.bits_per_pixel	= 32,
//	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data grouper_hdmi_fb_data = {
	.win		= 0,
	.xres		= 800,
	.yres		= 1280,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out grouper_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 3,
	.hotplug_gpio	= grouper_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= grouper_hdmi_enable,
	.disable	= grouper_hdmi_disable,

	.postsuspend	= grouper_hdmi_vddio_disable,
	.hotplug_init	= grouper_hdmi_vddio_enable,
};

static struct tegra_dc_platform_data grouper_disp2_pdata = {
	.flags		= 0,
	.default_out	= &grouper_disp2_out,
	.fb		= &grouper_hdmi_fb_data,
	.emc_clk_rate	= 300000000,
};
#endif

static struct tegra_dc_out grouper_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.sd_settings	= &grouper_sd_settings,
	.parent_clk	= "pll_p",
	.parent_clk_backup = "pll_d2_out0",

	.type		= TEGRA_DC_OUT_RGB,
	.depth		= 18,
	.dither		= TEGRA_DC_ORDERED_DITHER,

	.modes		= grouper_panel_modes,
	.n_modes	= ARRAY_SIZE(grouper_panel_modes),

	.enable		= grouper_panel_enable,
	.disable	= grouper_panel_disable,
	.postpoweron	= grouper_panel_postpoweron,

	.height		= 162,
	.width		= 104,
};

#ifdef CONFIG_TEGRA_DC
static struct tegra_dc_platform_data grouper_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &grouper_disp1_out,
	.emc_clk_rate	= 300000000,
	.fb		= &grouper_fb_data,
};

static struct nvhost_device grouper_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= grouper_disp1_resources,
	.num_resources	= ARRAY_SIZE(grouper_disp1_resources),
	.dev = {
		.platform_data = &grouper_disp1_pdata,
	},
};

static int grouper_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &grouper_disp1_device.dev;
}

static struct nvhost_device grouper_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= grouper_disp2_resources,
	.num_resources	= ARRAY_SIZE(grouper_disp2_resources),
	.dev = {
		.platform_data = &grouper_disp2_pdata,
	},
};
#else
static int grouper_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return 0;
}
#endif

#if defined(CONFIG_TEGRA_NVMAP)
static struct nvmap_platform_carveout grouper_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by grouper_panel_init() */
		.size		= 0,	/* Filled in by grouper_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data grouper_nvmap_data = {
	.carveouts	= grouper_carveouts,
	.nr_carveouts	= ARRAY_SIZE(grouper_carveouts),
};

static struct platform_device grouper_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &grouper_nvmap_data,
	},
};
#endif

static struct platform_device *grouper_gfx_devices[] __initdata = {
#if defined(CONFIG_TEGRA_NVMAP)
	&grouper_nvmap_device,
#endif
	&tegra_pwfm0_device,
	&grouper_backlight_device,
};


#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend grouper_panel_early_suspender;

static void grouper_panel_early_suspend(struct early_suspend *h)
{
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);

#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_save_default_governor();
	cpufreq_set_conservative_governor();
	cpufreq_set_conservative_governor_param("up_threshold",
			SET_CONSERVATIVE_GOVERNOR_UP_THRESHOLD);

	cpufreq_set_conservative_governor_param("down_threshold",
			SET_CONSERVATIVE_GOVERNOR_DOWN_THRESHOLD);

	cpufreq_set_conservative_governor_param("freq_step",
		SET_CONSERVATIVE_GOVERNOR_FREQ_STEP);
#endif

#ifdef CONFIG_PM_DEBUG
	pr_info("%sed\n", __func__);
#endif

}

static void grouper_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_governor();
#endif
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);

#ifdef CONFIG_PM_DEBUG
	pr_info("%sd\n", __func__);
#endif
}
#endif

int __init grouper_panel_init(void)
{
	int err;
	struct resource __maybe_unused *res;
	struct board_info board_info;

	tegra_get_board_info(&board_info);

#if defined(CONFIG_TEGRA_NVMAP)
	grouper_carveouts[1].base = tegra_carveout_start;
	grouper_carveouts[1].size = tegra_carveout_size;
#endif
/*
	gpio_request(grouper_lvds_avdd_en, "lvds_avdd_en");
	gpio_direction_output(grouper_lvds_avdd_en, 1);
	tegra_gpio_enable(grouper_lvds_avdd_en);

	//gpio_request(grouper_lvds_stdby, "lvds_stdby");
	//gpio_direction_output(grouper_lvds_stdby, 1);
	//tegra_gpio_enable(grouper_lvds_stdby);

	gpio_request(grouper_lvds_rst, "lvds_rst");
	gpio_direction_output(grouper_lvds_rst, 1);
	tegra_gpio_enable(grouper_lvds_rst);

	if (board_info.fab == BOARD_FAB_A00) {
		gpio_request(grouper_lvds_rs_a00, "lvds_rs");
		gpio_direction_output(grouper_lvds_rs_a00, 0);
		tegra_gpio_enable(grouper_lvds_rs_a00);
	} else {
		gpio_request(grouper_lvds_rs, "lvds_rs");
		gpio_direction_output(grouper_lvds_rs, 0);
		tegra_gpio_enable(grouper_lvds_rs);
	}

	gpio_request(grouper_lvds_lr, "lvds_lr");
	gpio_direction_output(grouper_lvds_lr, 1);
	tegra_gpio_enable(grouper_lvds_lr);
*/
/*
	gpio_request(grouper_lvds_shutdown, "lvds_shutdown");
	gpio_direction_output(grouper_lvds_shutdown, 1);
	tegra_gpio_enable(grouper_lvds_shutdown);
*/
	tegra_gpio_enable(grouper_hdmi_hpd);
	gpio_request(grouper_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(grouper_hdmi_hpd);

#ifdef CONFIG_HAS_EARLYSUSPEND
	grouper_panel_early_suspender.suspend = grouper_panel_early_suspend;
	grouper_panel_early_suspender.resume = grouper_panel_late_resume;
	grouper_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&grouper_panel_early_suspender);
#endif

#ifdef CONFIG_TEGRA_GRHOST
	err = nvhost_device_register(&tegra_grhost_device);
	if (err)
		return err;
#endif

	err = platform_add_devices(grouper_gfx_devices,
				ARRAY_SIZE(grouper_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&grouper_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
//	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
//				min(tegra_fb_size, tegra_bootloader_fb_size));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
		err = nvhost_device_register(&grouper_disp1_device);

	res = nvhost_get_resource_byname(&grouper_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
	if (!err)
		err = nvhost_device_register(&grouper_disp2_device);
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err)
		err = nvhost_device_register(&nvavp_device);
#endif
	return err;
}
