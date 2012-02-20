#ifndef RIL_SIM_H
#define RIL_SIM_H

int sim_hot_plug_init(struct device *target_device, struct workqueue_struct *queue);
void sim_hot_plug_exit(void);
irqreturn_t sim_interrupt_handle(int irq, void *dev_id);

void ril_sim_notify_modem_reset(bool is_reseting);

#endif
