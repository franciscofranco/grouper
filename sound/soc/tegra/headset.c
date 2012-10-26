/*
 *  Headset device detection driver.
 *
 * Copyright (C) 2011 ASUSTek Corporation.
 *
 * Authors:
 *  Jason Cheng <jason4_cheng@asus.com>
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
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/timer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <asm/string.h>
#include <sound/soc.h>
#include "../gpio-names.h"
#include "../codecs/rt5640.h"
#include <mach/board-grouper-misc.h>
#include <mach/pinmux.h>
#include "../board.h"
#include "../board-grouper.h"
MODULE_DESCRIPTION("Headset detection driver");
MODULE_LICENSE("GPL");

#define DEFAULT_PINMUX(_pingroup, _mux, _pupd, _tri, _io)       \
        {                                                       \
                .pingroup       = TEGRA_PINGROUP_##_pingroup,   \
                .func           = TEGRA_MUX_##_mux,             \
                .pupd           = TEGRA_PUPD_##_pupd,           \
                .tristate       = TEGRA_TRI_##_tri,             \
                .io             = TEGRA_PIN_##_io,              \
                .lock           = TEGRA_PIN_LOCK_DEFAULT,       \
                .od             = TEGRA_PIN_OD_DEFAULT,         \
                .ioreset        = TEGRA_PIN_IO_RESET_DEFAULT,   \
        }

/*----------------------------------------------------------------------------
** FUNCTION DECLARATION
**----------------------------------------------------------------------------*/
static int   __init     	headset_init(void);
static void __exit    headset_exit(void);
static irqreturn_t   	detect_irq_handler(int irq, void *dev_id);
static void 		detection_work(struct work_struct *work);
static int               	jack_config_gpio(void);
static irqreturn_t   	lineout_irq_handler(int irq, void *dev_id);
static void 		lineout_work_queue(struct work_struct *work);
static void		dock_work_queue(struct work_struct *work);
static int               	lineout_config_gpio(u32 project_info);
static void 		detection_work(struct work_struct *work);
static int               	btn_config_gpio(void);
static int                      switch_config_gpio(void);
int 			hs_micbias_power(int on);
static irqreturn_t	dockin_irq_handler(int irq, void *dev_id);
static void		set_dock_switches(void);
/*----------------------------------------------------------------------------
** GLOBAL VARIABLES
**----------------------------------------------------------------------------*/
#define JACK_GPIO		(TEGRA_GPIO_PW2)
#define LINEOUT_GPIO_NAKASI	(TEGRA_GPIO_PW3)
#define LINEOUT_GPIO_BACH	(TEGRA_GPIO_PX6)
#define HOOK_GPIO		(TEGRA_GPIO_PX2)
#define UART_HEADPHONE_SWITCH (TEGRA_GPIO_PS2)
#define ON	(1)
#define OFF	(0)

enum{
	NO_DEVICE = 0,
	HEADSET_WITH_MIC = 1,
	HEADSET_WITHOUT_MIC = 2,
};

struct headset_data {
	struct switch_dev sdev;
	struct input_dev *input;
	unsigned int irq;
	struct hrtimer timer;
	ktime_t debouncing_time;
};

static struct headset_data *hs_data;
bool headset_alive = false;
EXPORT_SYMBOL(headset_alive);
bool lineout_alive;
EXPORT_SYMBOL(lineout_alive);

static struct workqueue_struct *g_detection_work_queue;
static DECLARE_WORK(g_detection_work, detection_work);

extern struct snd_soc_codec *rt5640_audio_codec;
struct work_struct headset_work;
struct work_struct lineout_work;
struct work_struct dock_work;
static bool UART_enable = false;
static unsigned int revision;
static u32 lineout_gpio;
static int gpio_dock_in = 0;

static struct switch_dev dock_switch = {
	.name = "dock",
};

static struct switch_dev audio_switch = {
	.name = "usb_audio",
};

static ssize_t headset_name_show(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs_data->sdev)){
	case NO_DEVICE:{
		return sprintf(buf, "%s\n", "No Device");
		}
	case HEADSET_WITH_MIC:{
		return sprintf(buf, "%s\n", "HEADSET");
		}
	case HEADSET_WITHOUT_MIC:{
		return sprintf(buf, "%s\n", "HEADPHONE");
		}
	}
	return -EINVAL;
}

static ssize_t headset_state_show(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs_data->sdev)){
	case NO_DEVICE:
		return sprintf(buf, "%d\n", 0);
	case HEADSET_WITH_MIC:
		return sprintf(buf, "%d\n", 1);
	case HEADSET_WITHOUT_MIC:
		return sprintf(buf, "%d\n", 2);
	}
	return -EINVAL;
}

static void tristate_uart(void)
{
        enum tegra_pingroup pingroup = TEGRA_PINGROUP_ULPI_DATA0;
        enum tegra_pullupdown pupd = TEGRA_PUPD_PULL_DOWN;
        enum tegra_pin_io io = TEGRA_PIN_INPUT;
	enum tegra_tristate tristate = TEGRA_TRI_TRISTATE;

        tegra_pinmux_set_pullupdown(pingroup, pupd);
        tegra_pinmux_set_tristate(pingroup, tristate);
}


static void pulldown_uart(void)
{
	enum tegra_pingroup pingroupTx = TEGRA_PINGROUP_ULPI_DATA0;
	enum tegra_pingroup pingroupRx = TEGRA_PINGROUP_ULPI_DATA1;
        enum tegra_pullupdown pupd = TEGRA_PUPD_PULL_DOWN;
        enum tegra_pin_io io = TEGRA_PIN_INPUT;

	tegra_pinmux_set_pullupdown(pingroupTx, pupd);
	tegra_pinmux_set_io(pingroupTx, io);
//        tegra_pinmux_set_pullupdown(pingroupRx, pupd);
  //      tegra_pinmux_set_io(pingroupRx, io);
}


static void normal_uart(void)
{
	struct tegra_pingroup_config debug_uart [] = {
        DEFAULT_PINMUX(ULPI_DATA0,      UARTA,          NORMAL,     NORMAL,     OUTPUT),
        };
        tegra_pinmux_config_table(debug_uart, ARRAY_SIZE(debug_uart));

   //     tegra_pinmux_set_pullupdown(pingroupRx, pupd);
    //    tegra_pinmux_set_io(pingroupRx, ioIn);

}


static void enable_uart(void)
{
	struct tegra_pingroup_config debug_uart [] = {
        DEFAULT_PINMUX(ULPI_DATA0,      ULPI,          NORMAL,    NORMAL,   OUTPUT),
        };
        tegra_pinmux_config_table(debug_uart, ARRAY_SIZE(debug_uart));
}


static void disable_uart(void)
{
	struct tegra_pingroup_config debug_uart [] = {
	DEFAULT_PINMUX(ULPI_DATA0,      ULPI,          PULL_UP,    TRISTATE,   OUTPUT),
	};
	tegra_pinmux_config_table(debug_uart, ARRAY_SIZE(debug_uart));
}
static void insert_headset(void)
{
        struct snd_soc_dapm_context *dapm;

        dapm = &rt5640_audio_codec->dapm;

	if(gpio_get_value(lineout_gpio) == 0 && UART_enable){
                printk("%s: debug board\n", __func__);
                switch_set_state(&hs_data->sdev, NO_DEVICE);
                hs_micbias_power(OFF);
                headset_alive = false;
		gpio_direction_output(UART_HEADPHONE_SWITCH, 0);
		normal_uart();
	}else if(gpio_get_value(HOOK_GPIO)){ 
		printk("%s: headphone\n", __func__);
		switch_set_state(&hs_data->sdev, HEADSET_WITHOUT_MIC);
		hs_micbias_power(OFF);
		pulldown_uart();
		gpio_direction_output(UART_HEADPHONE_SWITCH, 1);
		headset_alive = false;
	}else{
		printk("%s: headset\n", __func__);
		switch_set_state(&hs_data->sdev, HEADSET_WITHOUT_MIC);
		hs_micbias_power(ON);
		pulldown_uart();
		gpio_direction_output(UART_HEADPHONE_SWITCH, 1);
		headset_alive = true;
	}
	hs_data->debouncing_time = ktime_set(0, 100000000);  /* 100 ms */
}
static void remove_headset(void)
{
	switch_set_state(&hs_data->sdev, NO_DEVICE);
	hs_data->debouncing_time = ktime_set(0, 100000000);  /* 100 ms */
	headset_alive = false;
	tristate_uart();
	gpio_direction_output(UART_HEADPHONE_SWITCH, 0);
}

static void detection_work(struct work_struct *work)
{
	unsigned long irq_flags;
	int cable_in1;
	int mic_in = 0;
	hs_micbias_power(ON);
	/* Disable headset interrupt while detecting.*/
	local_irq_save(irq_flags);
	disable_irq(hs_data->irq);
	local_irq_restore(irq_flags);

	/* Delay 1000ms for pin stable. */
	msleep(1000);

	/* Restore IRQs */
	local_irq_save(irq_flags);
	enable_irq(hs_data->irq);
	local_irq_restore(irq_flags);

	if (gpio_get_value(JACK_GPIO) != 0) {
		/* Headset not plugged in */

//		if (switch_get_state(&hs_data->sdev) == HEADSET_WITH_MIC || 
//			switch_get_state(&hs_data->sdev) == HEADSET_WITHOUT_MIC)
			remove_headset();
		goto closed_micbias;
	}

	cable_in1 = gpio_get_value(JACK_GPIO);
	mic_in  = gpio_get_value(HOOK_GPIO);
	if (cable_in1 == 0) {
	    printk("HOOK_GPIO value: %d\n", mic_in);
		if(switch_get_state(&hs_data->sdev) == NO_DEVICE)
			insert_headset();
		else if ( mic_in == 1)
			goto closed_micbias;
	} else{
		printk("HEADSET: Jack-in GPIO is low, but not a headset \n");
		goto closed_micbias;
	}
	return;

closed_micbias:
	hs_micbias_power(OFF);
	return;
}

static enum hrtimer_restart detect_event_timer_func(struct hrtimer *data)
{
	queue_work(g_detection_work_queue, &g_detection_work);
	return HRTIMER_NORESTART;
}

/**********************************************************
**  Function: Jack detection-in gpio configuration function
**  Parameter: none
**  Return value: if sucess, then returns 0
**
************************************************************/
static int jack_config_gpio()
{
	int ret;

	printk("HEADSET: Config Jack-in detection gpio\n");
	hs_micbias_power(ON);
	tegra_gpio_enable(JACK_GPIO);
	ret = gpio_request(JACK_GPIO, "h2w_detect");
	ret = gpio_direction_input(JACK_GPIO);

	hs_data->irq = gpio_to_irq(JACK_GPIO);
	ret = request_irq(hs_data->irq, detect_irq_handler,
			  IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "h2w_detect", NULL);

	ret = irq_set_irq_wake(hs_data->irq, 1);
	msleep(1);
	if (gpio_get_value(JACK_GPIO) == 0){
		insert_headset();
	}else {
		hs_micbias_power(OFF);
		headset_alive = false;
		switch_set_state(&hs_data->sdev, NO_DEVICE);
		remove_headset();
	}

	return 0;
}

/**********************************************************
**  Function: Headset Hook Key Detection interrupt handler
**  Parameter: irq
**  Return value: IRQ_HANDLED
**  High: Hook button pressed
************************************************************/
static int btn_config_gpio()
{
	int ret;

	printk("HEADSET: Config Headset Button detection gpio\n");

	tegra_gpio_enable(HOOK_GPIO);
	ret = gpio_request(HOOK_GPIO, "btn_INT");
	ret = gpio_direction_input(HOOK_GPIO);

	return 0;
}

static void lineout_work_queue(struct work_struct *work)
{
	msleep(300);

	if (gpio_get_value(lineout_gpio) == 0){
		printk("LINEOUT: LineOut inserted\n");
		lineout_alive = true;
	}else if(gpio_get_value(lineout_gpio)){
		printk("LINEOUT: LineOut removed\n");
		lineout_alive = false;
	}

}

static void set_dock_switches(void)
{
	bool docked = !gpio_get_value(gpio_dock_in);

	/* LE desk dock == 3, undocked == 0. */
	switch_set_state(&dock_switch, docked ? 3 : 0);

	/*
	 * Analog audio == 1, no audio == 0.
	 * Note that because we cannot detect the presence of a 3.5mm jack
	 * in the dock's audio socket, when docked, audio is always on.
	 */
	switch_set_state(&audio_switch, docked ? 1 : 0);
}

static void dock_work_queue(struct work_struct *work)
{
	set_dock_switches();
}

/**********************************************************
**  Function: LineOut Detection configuration function
**  Parameter: none
**  Return value: IRQ_HANDLED
**
************************************************************/
static int lineout_config_gpio(u32 project_info)
{
	int ret;

	printk("HEADSET: Config LineOut detection gpio\n");
	if(project_info == GROUPER_PROJECT_BACH)
		lineout_gpio = LINEOUT_GPIO_BACH;
	else if(project_info == GROUPER_PROJECT_NAKASI)
		lineout_gpio = LINEOUT_GPIO_NAKASI;
	tegra_gpio_enable(lineout_gpio);
	ret = gpio_request(lineout_gpio, "lineout_int");
	ret = gpio_direction_input(lineout_gpio);
#if 0
	ret = request_irq(gpio_to_irq(LINEOUT_GPIO), &lineout_irq_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "lineout_int", 0);
#endif
	if (gpio_get_value(lineout_gpio) == 0)
		lineout_alive = true;
	else
		lineout_alive = false;

	return 0;
}

static int switch_config_gpio()
{
        int ret;

        printk("HEADSET: Config uart<->headphone gpio\n");

        tegra_gpio_enable(UART_HEADPHONE_SWITCH);
        ret = gpio_request(UART_HEADPHONE_SWITCH, "uart_headphone_switch");

        return 0;
}

static int dockin_config_gpio()
{
	int ret = 0;
	int irq_num = 0;

	irq_num = gpio_to_irq(gpio_dock_in);
	ret = request_irq(irq_num, dockin_irq_handler,
		IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING | IRQF_SHARED, "dock_detect", hs_data);
	if(ret < 0)
		printk("%s: request irq fail errno = %d\n", __func__, ret);

        return ret;
}

static irqreturn_t dockin_irq_handler(int irq, void *dev_id)
{
	schedule_work(&dock_work);

        return IRQ_HANDLED;
}

/**********************************************************
**  Function: LineOut detection interrupt handler
**  Parameter: dedicated irq
**  Return value: if sucess, then returns IRQ_HANDLED
**
************************************************************/
static irqreturn_t lineout_irq_handler(int irq, void *dev_id)
{
	schedule_work(&lineout_work);
	return IRQ_HANDLED;
}

/**********************************************************
**  Function: Headset jack-in detection interrupt handler
**  Parameter: dedicated irq
**  Return value: if sucess, then returns IRQ_HANDLED
**
************************************************************/
static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	do {
		value1 = gpio_get_value(JACK_GPIO);
		irq_set_irq_type(hs_data->irq, value1 ?
				IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
		value2 = gpio_get_value(JACK_GPIO);
	}while (value1 != value2 && retry_limit-- > 0);

	if ((switch_get_state(&hs_data->sdev) == NO_DEVICE) ^ value2){
		hrtimer_start(&hs_data->timer, hs_data->debouncing_time, HRTIMER_MODE_REL);
	}

	return IRQ_HANDLED;
}

static int codec_micbias_power(int on)
{
	if(on){
		//for ALC5642
		if(rt5640_audio_codec == NULL){
			printk("%s: No rt5640_audio_codec - set micbias on fail\n", __func__);
			return 0;
		}
#if 0
		snd_soc_update_bits(rt5640_audio_codec, RT5640_PWR_ANLG1, RT5640_PWR_LDO2, RT5640_PWR_LDO2); /* Enable LDO2 */
		snd_soc_update_bits(rt5640_audio_codec, RT5640_PWR_ANLG2, RT5640_PWR_MB1, RT5640_PWR_MB1); /*Enable MicBias1 */
		//for ALC5642
#endif
	}else{
		//for ALC5642
		if(rt5640_audio_codec == NULL){
			printk("%s: No rt5640_audio_codec - set micbias off fail\n", __func__);
			return 0;
		}
		snd_soc_update_bits(rt5640_audio_codec, RT5640_PWR_ANLG2, RT5640_PWR_MB1, 0); /* Disable MicBias1 */
		snd_soc_update_bits(rt5640_audio_codec, RT5640_PWR_ANLG1, RT5640_PWR_LDO2, 0); /* Disable LDO2 */
		//for ALC5642
	}
	return 0;
}


int hs_micbias_power(int on)
{
	static int nLastVregStatus = -1;

	if(on && nLastVregStatus!=ON){
		printk("HEADSET: Turn on micbias power\n");
		nLastVregStatus = ON;
		codec_micbias_power(ON);
	}
	else if(!on && nLastVregStatus!=OFF){
		printk("HEADSET: Turn off micbias power\n");
		nLastVregStatus = OFF;
		codec_micbias_power(OFF);
	}
	return 0;
}
EXPORT_SYMBOL(hs_micbias_power);

/**********************************************************
**  Function: Headset driver init function
**  Parameter: none
**  Return value: none
**
************************************************************/
static int __init headset_init(void)
{
	u32 project_info = grouper_get_project_id();
	u32 pmic_id = grouper_query_pmic_id();

	printk(KERN_INFO "%s+ #####\n", __func__);
	int ret;

	printk("HEADSET: Headset detection init\n");

	if (project_info == GROUPER_PROJECT_BACH)
		gpio_dock_in = TEGRA_GPIO_PO5;
	else
		gpio_dock_in = TEGRA_GPIO_PU4;

	if(project_info == GROUPER_PROJECT_BACH ||
		(project_info == GROUPER_PROJECT_NAKASI && pmic_id ==GROUPER_PMIC_TI))
		UART_enable = true;

	revision = grouper_query_pcba_revision();

	hs_data = kzalloc(sizeof(struct headset_data), GFP_KERNEL);
	if (!hs_data)
		return -ENOMEM;

	hs_data->debouncing_time = ktime_set(0, 100000000);  /* 100 ms */
	hs_data->sdev.name = "h2w";
	hs_data->sdev.print_name = headset_name_show;
	hs_data->sdev.print_state = headset_state_show;

	ret = switch_dev_register(&hs_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	WARN_ON(switch_dev_register(&dock_switch));
	WARN_ON(switch_dev_register(&audio_switch));
	/* Make sure dock switches are correct at boot */
	set_dock_switches();

	g_detection_work_queue = create_workqueue("detection");

	hrtimer_init(&hs_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hs_data->timer.function = detect_event_timer_func;

	printk("HEADSET: Headset detection mode\n");
	lineout_config_gpio(project_info);
	btn_config_gpio();/*Config hook detection GPIO*/
	switch_config_gpio(); /*Config uart and headphone switch*/
	jack_config_gpio();/*Config jack detection GPIO*/
	INIT_WORK(&lineout_work, lineout_work_queue);
	INIT_WORK(&dock_work, dock_work_queue);
	dockin_config_gpio();

	printk(KERN_INFO "%s- #####\n", __func__);
	return 0;

err_switch_dev_register:
	printk(KERN_ERR "Headset: Failed to register driver\n");

	return ret;
}

/**********************************************************
**  Function: Headset driver exit function
**  Parameter: none
**  Return value: none
**
************************************************************/
static void __exit headset_exit(void)
{
	printk("HEADSET: Headset exit\n");
	if (switch_get_state(&hs_data->sdev))
		remove_headset();
	gpio_free(JACK_GPIO);
	gpio_free(HOOK_GPIO);
	gpio_free(lineout_gpio);

	free_irq(hs_data->irq, 0);
	destroy_workqueue(g_detection_work_queue);
	switch_dev_unregister(&hs_data->sdev);
	switch_dev_unregister(&dock_switch);
	switch_dev_unregister(&audio_switch);
}

module_init(headset_init);
module_exit(headset_exit);
