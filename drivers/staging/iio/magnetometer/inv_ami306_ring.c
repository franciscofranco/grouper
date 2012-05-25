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
#include "inv_ami306_iio.h"
#include "../iio.h"
#include "../kfifo_buf.h"
#include "../trigger_consumer.h"
#include "../sysfs.h"

/**
 *  inv_irq_handler() - Cache a timestamp at each data ready interrupt.
 */
static irqreturn_t inv_ami_irq_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct inv_ami306_state_s *st = iio_priv(indio_dev);
	st->timestamp = iio_get_time_ns();

	return IRQ_WAKE_THREAD;
}
static int put_scan_to_buf(struct iio_dev *indio_dev, unsigned char *d,
				short *s, int scan_index) {
	struct iio_buffer *ring = indio_dev->buffer;
	int st;
	int i, d_ind;
	d_ind = 0;
	for (i = 0; i < 3; i++) {
		st = iio_scan_mask_query(indio_dev, ring, scan_index + i);
		if (st) {
			memcpy(&d[d_ind], &s[i], sizeof(s[i]));
			d_ind += sizeof(s[i]);
		}
	}
	return d_ind;
}

/**
 *  inv_read_fifo() - Transfer data from FIFO to ring buffer.
 */
static irqreturn_t inv_read_fifo(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct inv_ami306_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	int result, status, d_ind;
	char b;
	char *tmp;
	s64 tmp_buf[2];

	result = i2c_smbus_read_i2c_block_data(st->i2c, REG_AMI_STA1, 1, &b);
	if (result < 0)
		goto end_session;
	if (b & AMI_STA1_DRDY_BIT) {
		status = ami306_read_raw_data(st, st->compass_data);
		if (status) {
			pr_err("error reading raw\n");
			goto end_session;
		}
		tmp = (unsigned char *)tmp_buf;
		d_ind = put_scan_to_buf(indio_dev, tmp, st->compass_data,
						INV_AMI306_SCAN_MAGN_X);
		if (ring->scan_timestamp)
			tmp_buf[(d_ind + 7)/8] = st->timestamp;
		ring->access->store_to(indio_dev->buffer, tmp, st->timestamp);
	} else if (b & AMI_STA1_DOR_BIT)
		pr_err("not ready\n");
end_session:
	b = AMI_CTRL3_FORCE_BIT;
	result = i2c_smbus_write_i2c_block_data(st->i2c, REG_AMI_CTRL3, 1, &b);

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

void inv_ami306_unconfigure_ring(struct iio_dev *indio_dev)
{
	iio_dealloc_pollfunc(indio_dev->pollfunc);
	iio_kfifo_free(indio_dev->buffer);
};

static const struct iio_buffer_setup_ops inv_ami306_ring_setup_ops = {
	.preenable = &iio_sw_buffer_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};

int inv_ami306_configure_ring(struct iio_dev *indio_dev)
{
	int ret = 0;
	struct iio_buffer *ring;

	ring = iio_kfifo_allocate(indio_dev);
	if (!ring) {
		ret = -ENOMEM;
		return ret;
	}
	indio_dev->buffer = ring;
	/* setup ring buffer */
	ring->scan_timestamp = true;
	indio_dev->setup_ops = &inv_ami306_ring_setup_ops;
	/*scan count double count timestamp. should subtract 1. but
	number of channels still includes timestamp*/
	indio_dev->pollfunc = iio_alloc_pollfunc(&inv_ami_irq_handler,
					&inv_read_fifo,
					IRQF_ONESHOT,
					indio_dev,
					"%s_consumer%d",
					indio_dev->name,
					indio_dev->id);
	if (indio_dev->pollfunc == NULL) {
		ret = -ENOMEM;
		goto error_iio_sw_rb_free;
	}

	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	return 0;
error_iio_sw_rb_free:
	iio_kfifo_free(indio_dev->buffer);
	return ret;
}
/**
 *  @}
 */

