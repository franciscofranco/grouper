#ifndef RIL_MODEM_CRASH_H
#define RIL_MODEM_CRASH_H

int ril_modem_crash_init(struct device *target_device, struct workqueue_struct *queue);
void ril_modem_crash_exit(void);

#endif

