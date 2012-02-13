#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/switch.h>

#include "baseband-xmm-power.h"
#include "ril.h"
#include "ril_sim.h"

#define NAME_SIM_PLUG "ril_sim_plug"

//**** external symbols

extern void baseband_xmm_ap_resume_work(void);


//**** constants

#define SIM_PLUG_STATE_ABSENT  1
#define SIM_PLUG_STATE_PLUGGED 0


//**** local variable declaration

static struct workqueue_struct *workqueue;
static struct delayed_work hotplug_work_task;
static struct switch_dev sim_sdev;
static int sim_plug_state;


//**** callbacks for switch device

static ssize_t print_sim_plug_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", NAME_SIM_PLUG);
}

static ssize_t print_sim_plug_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", sim_plug_state);
}


//**** IRQ event handler

static int get_sim_plug_state_from_pin(void)
{
	int ping_state = gpio_get_value(SIM_CARD_DET);
	if (0 == ping_state) {
		return SIM_PLUG_STATE_ABSENT;
	} else {
		return SIM_PLUG_STATE_PLUGGED;
	}
}

static void hotplug_work_handle(struct work_struct *work)
{
	// refresh sysfs with current GPIO state
	sim_plug_state = get_sim_plug_state_from_pin();
	RIL_INFO("sim state = %d\n", sim_plug_state);
	switch_set_state(&sim_sdev, sim_plug_state);

	// wake up modem to refresh card state
	baseband_xmm_ap_resume_work();
}

irqreturn_t sim_interrupt_handle(int irq, void *dev_id)
{
	const unsigned long WORK_DELAY = HZ / 10;
	queue_delayed_work(workqueue, &hotplug_work_task, WORK_DELAY);
	return IRQ_HANDLED;
}


//**** initialize and finalize

int sim_hot_plug_init(struct workqueue_struct *queue)
{
	int rc = 0;
	int sim_irq = gpio_to_irq(SIM_CARD_DET);
	sim_plug_state = get_sim_plug_state_from_pin();

	RIL_INFO(
	  "GPIO = %d , irq = %d, state = %d\n",
	  SIM_CARD_DET,
	  sim_irq,
	  sim_plug_state);

	// init work queue and delayed work
	workqueue = queue;
	INIT_DELAYED_WORK(&hotplug_work_task, hotplug_work_handle);

	// register switch class
	sim_sdev.name = NAME_SIM_PLUG;
	sim_sdev.print_name = print_sim_plug_name;
	sim_sdev.print_state = print_sim_plug_state;
	rc = switch_dev_register(&sim_sdev);
	// Because switch_dev_register will initiate sdev.state to 0,
	// sdev.state will be initiated after switch_dev_register.
	sim_sdev.state = sim_plug_state;

	if (rc < 0) {
		RIL_ERR("Could not register switch device, rc = %d\n", rc);
		return -1;
	}

	RIL_INFO("request switch class successfully\n");
	return 0;
}

void sim_hot_plug_exit(void)
{
	switch_dev_unregister(&sim_sdev);
}

