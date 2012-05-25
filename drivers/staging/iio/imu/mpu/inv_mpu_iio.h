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
 *  @addtogroup DRIVERS
 *  @brief      Hardware drivers.
 *
 *  @{
 *      @file  inv_gyro.h
 *      @brief Struct definitions for the Invensense gyro driver.
 */

#ifndef _INV_GYRO_H_
#define _INV_GYRO_H_

#include <linux/i2c.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/spinlock.h>
#include <linux/mpu.h>
#include "../../iio.h"
#include "../../buffer.h"
#include "dmpKey.h"
/**
 *  struct inv_reg_map_s - Notable slave registers.
 *  @who_am_i:		Upper 6 bits of the device's slave address.
 *  @sample_rate_div:	Divider applied to gyro output rate.
 *  @lpf:		Configures internal LPF.
 *  @product_id:	Product revision.
 *  @bank_sel:		Selects between memory banks.
 *  @user_ctrl:		Enables/resets the FIFO.
 *  @fifo_en:		Determines which data will appear in FIFO.
 *  @gyro_config:	gyro config register.
 *  @accl_config:	accel config register
 *  @fifo_count_h:	Upper byte of FIFO count.
 *  @fifo_r_w:		FIFO register.
 *  @raw_gyro		Address of first gyro register.
 *  @raw_accl		Address of first accel register.
 *  @temperature	temperature register
 *  @int_enable:	Interrupt enable register.
 *  @int_status:	Interrupt flags.
 *  @pwr_mgmt_1:	Controls chip's power state and clock source.
 *  @pwr_mgmt_2:	Controls power state of individual sensors.
 *  @mem_start_addr:	Address of first memory read.
 *  @mem_r_w:		Access to memory.
 *  @prgm_strt_addrh	firmware program start address register
 */
struct inv_reg_map_s {
	unsigned char who_am_i;
	unsigned char sample_rate_div;
	unsigned char lpf;
	unsigned char product_id;
	unsigned char bank_sel;
	unsigned char user_ctrl;
	unsigned char fifo_en;
	unsigned char gyro_config;
	unsigned char accl_config;
	unsigned char fifo_count_h;
	unsigned char fifo_r_w;
	unsigned char raw_gyro;
	unsigned char raw_accl;
	unsigned char temperature;
	unsigned char int_enable;
	unsigned char int_status;
	unsigned char pwr_mgmt_1;
	unsigned char pwr_mgmt_2;
	unsigned char mem_start_addr;
	unsigned char mem_r_w;
	unsigned char prgm_strt_addrh;
};

enum inv_devices {
	INV_ITG3500 = 0,
	INV_MPU3050 = 1,
	INV_MPU6050 = 2,
	INV_MPU9150 = 3,
	INV_NUM_PARTS
};

/**
 *  struct test_setup_t - set up parameters for self test.
 *  @gyro_sens: sensitity for gyro.
 *  @sample_rate: sample rate, i.e, fifo rate.
 *  @lpf:	low pass filter.
 *  @fsr:	full scale range.
 *  @accl_fs:	accel full scale range.
 *  @accl_sens:	accel sensitivity
 */
struct test_setup_t {
	int gyro_sens;
	int sample_rate;
	int lpf;
	int fsr;
	int accl_fs;
	unsigned int accl_sens[3];
};

/**
 *  struct inv_hw_s - Other important hardware information.
 *  @num_reg:	Number of registers on device.
 *  @name:      name of the chip
 */
struct inv_hw_s {
	unsigned char num_reg;
	unsigned char *name;
};

/**
 *  struct inv_chip_config_s - Cached chip configuration data.
 *  @fsr:		Full scale range.
 *  @lpf:		Digital low pass filter frequency.
 *  @clk_src:		Clock source.
 *  @accl_fs:		accel full scale range.
 *  @lpa_freq:		low power frequency
 *  @self_test_run_once flag for self test run ever.
 *  @has_footer:	MPU3050 specific work around.
 *  @has_compass:	has compass or not.
 *  @enable:		master enable to enable output
 *  @accl_enable:	enable accel functionality
 *  @accl_fifo_enable:	enable accel data output
 *  @gyro_enable:	enable gyro functionality
 *  @gyro_fifo_enable:	enable gyro data output
 *  @compass_enable:	enable compass
 *  @compass_fifo_enable: enable compass data output
 *  @is_asleep:		1 if chip is powered down.
 *  @dmp_on:		dmp is on/off.
 *  @dmp_int_on:        dmp interrupt on/off.
 *  @orientation_on:	dmp is on/off.
 *  @firmware_loaded:	flag indicate firmware loaded or not.
 *  @lpa_mod:		low power mode.
 *  @tap_on:		tap on/off.
 *  @flick_int_on:	flick interrupt on/off.
 *  @quaternion_on:	send quaternion data on/off.
 *  @display_orient_on:	display orientation on/off.
 *  @prog_start_addr:	firmware program start address.
 *  @dmp_output_rate:   dmp output rate.
 *  @fifo_rate:		FIFO update rate.
 */
struct inv_chip_config_s {
	unsigned int fsr:2;
	unsigned int lpf:3;
	unsigned int clk_src:1;
	unsigned int accl_fs:2;
	unsigned int lpa_freq:2;
	unsigned int self_test_run_once:1;
	unsigned int has_footer:1;
	unsigned int has_compass:1;
	unsigned int enable:1;
	unsigned int accl_enable:1;
	unsigned int accl_fifo_enable:1;
	unsigned int gyro_enable:1;
	unsigned int gyro_fifo_enable:1;
	unsigned int compass_enable:1;
	unsigned int compass_fifo_enable:1;
	unsigned int is_asleep:1;
	unsigned int dmp_on:1;
	unsigned int dmp_int_on:1;
	unsigned int orientation_on:1;
	unsigned int firmware_loaded:1;
	unsigned int lpa_mode:1;
	unsigned int tap_on:1;
	unsigned int flick_int_on:1;
	unsigned int quaternion_on:1;
	unsigned int display_orient_on:1;
	unsigned short  prog_start_addr;
	unsigned short fifo_rate;
	unsigned char dmp_output_rate;
};

/**
 *  struct inv_chip_info_s - Chip related information.
 *  @product_id:	Product id.
 *  @product_revision:	Product revision.
 *  @silicon_revision:	Silicon revision.
 *  @software_revision:	software revision.
 *  @multi:		accel specific multiplier.
 *  @compass_sens:	compass sensitivity.
 *  @gyro_sens_trim:	Gyro sensitivity trim factor.
 *  @accl_sens_trim:    accel sensitivity trim factor.
 */
struct inv_chip_info_s {
	unsigned char product_id;
	unsigned char product_revision;
	unsigned char silicon_revision;
	unsigned char software_revision;
	unsigned char multi;
	unsigned char compass_sens[3];
	unsigned long gyro_sens_trim;
	unsigned long accl_sens_trim;
};
/**
 *  struct inv_chip_chan_info - Chip channel information.
 *  @channels:	channel specification.
 *  @num_channels:	number of channels.
 */
struct inv_chip_chan_info {
	const struct iio_chan_spec *channels;
	int num_channels;
};

/**
 *  struct inv_flick_s structure to store flick data.
 *  @lower:	lower bound of flick.
 *  @upper:     upper bound of flick.
 *  @counter:	counter of flick.
 *  @msg_on;    message to carry flick
 *  @axis:      axis of flick
 */
struct inv_flick_s {
	int lower;
	int upper;
	int counter;
	char msg_on;
	char axis;
};
/**
 *  struct inv_tap_s structure to store tap data.
 *  @min_count:  minimum taps counted.
 *  @thresh:    tap threshold.
 *  @time:	tap time.
 */
struct inv_tap_s {
	char min_count;
	short thresh;
	short time;
};
struct inv_mpu_slave;
/**
 *  struct inv_gyro_state_s - Driver state variables.
 *  @chip_config:	Cached attribute information.
 *  @chip_info:		Chip information from read-only registers.
 *  @trig;              iio trigger.
 *  @flick:		flick data structure
 *  @tap:               tap data structure
 *  @reg:		Map of important registers.
 *  @hw:		Other hardware-specific information.
 *  @chip_type:		chip type.
 *  @time_stamp_lock:	spin lock to time stamp.
 *  @i2c:		i2c client handle.
 *  @plat_data:		platform data.
 *  @mpu_slave:		mpu slave handle.
 *  @chan_info:         channel information
 *  @timestamps:        kfifo queue to store time stamp.
 *  @compass_st_upper:  compass self test upper limit.
 *  @compass_st_lower:  compass self test lower limit.
 *  @irq:               irq number store.
 *  @accel_bias:        accel bias store.
 *  @gyro_bias:         gyro bias store.
 *  @raw_gyro:          raw gyro data.
 *  @raw_accel:         raw accel data.
 *  @raw_compass:       raw compass.
 *  @compass_scale:	compass scale.
 *  @i2c_addr:		i2c address.
 *  @compass_divider:	slow down compass rate.
 *  @compass_counter:	slow down compass rate.
 *  @sample_divider:    sample divider for dmp.
 *  @fifo_divider:      fifo divider for dmp.
 *  @orient_data:       orientation data.
 *  @display_orient_data:     display orient data.
 *  @tap_data:          tap data.
 *  @sl_handle:         Handle to I2C port.
 *  @irq_dur_us:	duration between each irq.
 *  @last_isr_time:	last isr time.
 */
struct inv_gyro_state_s {
#define TIMESTAMP_FIFO_SIZE 16
	struct inv_chip_config_s chip_config;
	struct inv_chip_info_s chip_info;
	struct iio_trigger  *trig;
	struct inv_flick_s flick;
	struct inv_tap_s   tap;
	struct inv_reg_map_s reg;
	struct inv_hw_s *hw;
	enum   inv_devices chip_type;
	spinlock_t time_stamp_lock;
	struct i2c_client *i2c;
	struct mpu_platform_data plat_data;
	struct inv_mpu_slave *mpu_slave;
	struct inv_chip_chan_info *chan_info;
	DECLARE_KFIFO(timestamps, long long, TIMESTAMP_FIFO_SIZE);
	short compass_st_upper[3];
	short compass_st_lower[3];
	short irq;
	int accel_bias[3];
	int gyro_bias[3];
	short raw_gyro[3];
	short raw_accel[3];
	short raw_compass[3];
	unsigned char compass_scale;
	unsigned char i2c_addr;
	unsigned char compass_divider;
	unsigned char compass_counter;
	unsigned char sample_divider;
	unsigned char fifo_divider;
	unsigned char orient_data;
	unsigned char display_orient_data;
	unsigned char tap_data;
	void *sl_handle;
	unsigned int irq_dur_us;
	long long last_isr_time;
};
/* produces an unique identifier for each device based on the
   combination of product version and product revision */
struct prod_rev_map_t {
	unsigned short mpl_product_key;
	unsigned char silicon_rev;
	unsigned short gyro_trim;
	unsigned short accel_trim;
};
/**
 *  struct inv_mpu_slave - MPU slave structure.
 *  @suspend:		suspend operation.
 *  @resume:		resume operation.
 *  @setup:		setup chip. initialization.
 *  @combine_data:	combine raw data into meaningful data.
 *  @get_mode:		get current chip mode.
 *  @set_lpf            set low pass filter.
 *  @set_fs             set full scale
 */
struct inv_mpu_slave {
	int (*suspend)(struct inv_gyro_state_s *);
	int (*resume)(struct inv_gyro_state_s *);
	int (*setup)(struct inv_gyro_state_s *);
	int (*combine_data)(unsigned char *in, short *out);
	int (*get_mode)(struct inv_gyro_state_s *);
	int (*set_lpf)(struct inv_gyro_state_s *, int rate);
	int (*set_fs)(struct inv_gyro_state_s *, int fs);
};
/* AKM definitions */
#define REG_AKM_ID              (0x00)
#define REG_AKM_STATUS          (0x02)
#define REG_AKM_MEASURE_DATA    (0x03)
#define REG_AKM_MODE            (0x0A)
#define REG_AKM_ST_CTRL         (0x0C)
#define REG_AKM_SENSITIVITY     (0x10)
#define REG_AKM8963_CNTL1       (0x0A)

#define DATA_AKM_ID             (0x48)
#define DATA_AKM_MODE_PW_DN     (0x00)
#define DATA_AKM_MODE_PW_SM     (0x01)
#define DATA_AKM_MODE_PW_ST     (0x08)
#define DATA_AKM_MODE_PW_FR     (0x0F)
#define DATA_AKM_SELF_TEST      (0x40)
#define DATA_AKM_DRDY           (0x01)
#define DATA_AKM8963_BIT        (0x10)
#define DATA_AKM_STAT_MASK      (0x0C)

#define DATA_AKM8975_SCALE      (9830)
#define DATA_AKM8972_SCALE      (19661)
#define DATA_AKM8963_SCALE0     (19661)
#define DATA_AKM8963_SCALE1     (4915)
#define AKM8963_SCALE_SHIFT     (4)
#define NUM_BYTES_COMPASS_SLAVE (8)

#define DATA_AKM8975_ST_X_UP    (100)
#define DATA_AKM8975_ST_X_LW    (-100)
#define DATA_AKM8975_ST_Y_UP    (100)
#define DATA_AKM8975_ST_Y_LW    (-100)
#define DATA_AKM8975_ST_Z_UP    (-300)
#define DATA_AKM8975_ST_Z_LW    (-1000)

#define DATA_AKM8972_ST_X_UP    (50)
#define DATA_AKM8972_ST_X_LW    (-50)
#define DATA_AKM8972_ST_Y_UP    (50)
#define DATA_AKM8972_ST_Y_LW    (-50)
#define DATA_AKM8972_ST_Z_UP    (-100)
#define DATA_AKM8972_ST_Z_LW    (-500)

#define DATA_AKM8963_ST_X_UP    (200)
#define DATA_AKM8963_ST_X_LW    (-200)
#define DATA_AKM8963_ST_Y_UP    (200)
#define DATA_AKM8963_ST_Y_LW    (-200)
#define DATA_AKM8963_ST_Z_UP    (-800)
#define DATA_AKM8963_ST_Z_LW    (-3200)


/* register definition*/
#define REG_3050_AUX_VDDIO      (0x13)
#define REG_3050_SLAVE_ADDR     (0x14)
#define REG_3050_AUX_BST_ADDR   (0x18)
#define REG_3050_AUX_XOUT_H     (0x23)

#define REG_3500_OTP            (0x00)

#define REG_YGOFFS_TC           (0x01)
#define REG_XA_OFFS_L_TC        (0x07)
#define REG_ST_GCT_X            (0x0D)
#define REG_I2C_MST_CTRL        (0x24)
#define REG_I2C_SLV0_ADDR       (0x25)
#define REG_I2C_SLV0_REG        (0x26)
#define REG_I2C_SLV0_CTRL       (0x27)
#define REG_I2C_SLV1_ADDR       (0x28)
#define REG_I2C_SLV1_REG        (0x29)
#define REG_I2C_SLV1_CTRL       (0x2A)

#define REG_I2C_SLV4_CTRL       (0x34)
#define REG_INT_PIN_CFG         (0x37)
#define REG_DMP_INT_STATUS      (0x39)
#define REG_EXT_SENS_DATA_00    (0x49)
#define REG_I2C_SLV1_DO         (0x64)
#define REG_I2C_MST_DELAY_CTRL  (0x67)
#define REG_BANK_SEL            (0x6D)
#define REG_MEM_START           (0x6E)
#define REG_MEM_RW              (0x6F)

/* bit definitions */
#define BIT_3050_VDDIO          (0x04)
#define BIT_3050_AUX_IF_EN      (0x20)
#define BIT_3050_FIFO_RST       (0x02)

#define BIT_BYPASS_EN           (0x2)
#define BIT_WAIT_FOR_ES         (0x40)
#define BIT_I2C_READ            (0x80)
#define BIT_SLV_EN              (0x80)
#define BIT_I2C_MST_VDDIO       (0x80)

#define BIT_DMP_EN              (0x80)
#define BIT_FIFO_EN		(0x40)
#define BIT_I2C_MST_EN          (0x20)
#define BIT_DMP_RST             (0x08)
#define BIT_FIFO_RST		(0x04)

#define BIT_SLV0_DLY_EN         (0x01)
#define BIT_SLV1_DLY_EN         (0x02)

#define BIT_FIFO_OVERFLOW	(0x10)
#define BIT_DATA_RDY_EN		(0x01)
#define BIT_DMP_INT_EN          (0x02)

#define BIT_PWR_ACCL_STBY       (0x38)
#define BIT_PWR_GYRO_STBY       (0x07)

#define BIT_GYRO_XOUT		(0x40)
#define BIT_GYRO_YOUT		(0x20)
#define BIT_GYRO_ZOUT		(0x10)
#define BIT_ACCEL_OUT		(0x08)
#define BITS_GYRO_OUT		(0x70)
#define BITS_SELF_TEST_EN       (0xE0)
#define BITS_3050_ACCL_OUT	(0x0E)
#define BITS_3050_POWER1        (0x30)
#define BITS_3050_POWER2        (0x10)
#define BITS_3050_GYRO_STANDBY  (0x38)
#define BITS_FSR		(0x18)
#define BITS_LPF		(0x07)
#define BITS_CLK		(0x07)
#define BIT_3500_FIFO_OVERFLOW	(0x10)
#define BIT_SLEEP		(0x40)
#define BIT_CYCLE               (0x20)
#define BIT_LPA_FREQ            (0xC0)

#define DMP_START_ADDR          (0x400)
#define BYTES_FOR_DMP           (16)
#define QUATERNION_BYTES        (16)
#define BYTES_PER_SENSOR        (6)
#define MPU3050_FOOTER_SIZE     (2)
#define FIFO_COUNT_BYTE         (2)
#define FIFO_THRESHOLD          (500)
#define POWER_UP_TIME           (100)
#define SENSOR_UP_TIME          (30)
#define MPU_MEM_BANK_SIZE       (256)
#define MPU3050_TEMP_OFFSET	(5383314L)
#define MPU3050_TEMP_SCALE      (3834792L)
#define MPU6050_TEMP_OFFSET	(2462307L)
#define MPU6050_TEMP_SCALE      (2977653L)
#define MPU_TEMP_SHIFT          (16)
#define LPA_FREQ_SHIFT          (6)
#define COMPASS_RATE_SCALE      (10)
#define MAX_GYRO_FS_PARAM       (3)
#define MAX_ACCL_FS_PARAM       (3)
#define MAX_LPA_FREQ_PARAM      (3)
#define THREE_AXIS              (3)
#define GYRO_CONFIG_FSR_SHIFT   (3)
#define ACCL_CONFIG_FSR_SHIFT   (3)
#define GYRO_DPS_SCALE          (250)
#define MEM_ADDR_PROD_REV       (0x6)
#define SOFT_PROD_VER_BYTES     (5)
#define CHAN_INDEX_GYRO         (0)
#define CHAN_INDEX_GYRO_ACCL    (1)
#define CHAN_INDEX_GYRO_ACCL_MAGN (2)

/* init parameters */
#define INIT_FIFO_RATE          (50)
#define INIT_DUR_TIME           ((1000/INIT_FIFO_RATE)*1000)
#define INIT_TAP_THRESHOLD      (100)
#define INIT_TAP_TIME           (100)
#define INIT_TAP_MIN_COUNT      (2)
#define MPL_PROD_KEY(ver, rev) (ver * 100 + rev)
#define NUM_OF_PROD_REVS (ARRAY_SIZE(prod_rev_map))
/*---- MPU6050 Silicon Revisions ----*/
#define MPU_SILICON_REV_A2              1       /* MPU6050A2 Device */
#define MPU_SILICON_REV_B1              2       /* MPU6050B1 Device */

#define BIT_PRFTCH_EN                           0x40
#define BIT_CFG_USER_BANK                       0x20
#define BITS_MEM_SEL                            0x1f
/* time stamp tolerance */
#define TIME_STAMP_TOR           (5)
#define MAX_CATCH_UP             (5)
#define DEFAULT_ACCL_TRIM        (16384)
#define MAX_FIFO_RATE            (1000)
#define MIN_FIFO_RATE            (4)
#define ONE_K_HZ                 (1000)

/* flick related defines */
#define DATA_INT            (2097)
#define DATA_MSG_ON         (262144)

/*tap related defines */
#define INV_TAP                               0x08
#define INV_NUM_TAP_AXES (3)

#define INV_TAP_AXIS_X_POS                    0x20
#define INV_TAP_AXIS_X_NEG                    0x10
#define INV_TAP_AXIS_Y_POS                    0x08
#define INV_TAP_AXIS_Y_NEG                    0x04
#define INV_TAP_AXIS_Z_POS                    0x02
#define INV_TAP_AXIS_Z_NEG                    0x01
#define INV_TAP_ALL_DIRECTIONS                0x3f

#define INV_TAP_AXIS_X                        0x1
#define INV_TAP_AXIS_Y                        0x2
#define INV_TAP_AXIS_Z                        0x4

#define INV_TAP_AXIS_ALL                      \
		(INV_TAP_AXIS_X            |   \
		INV_TAP_AXIS_Y             |   \
		INV_TAP_AXIS_Z)

#define INT_SRC_TAP    0x01
#define INT_SRC_ORIENT 0x02

/*orientation related */
#define INV_X_UP                          0x01
#define INV_X_DOWN                        0x02
#define INV_Y_UP                          0x04
#define INV_Y_DOWN                        0x08
#define INV_Z_UP                          0x10
#define INV_Z_DOWN                        0x20
#define INV_ORIENTATION_ALL               0x3F

#define INV_ORIENTATION_FLIP              0x40
#define INV_X_AXIS_INDEX                 (0x00)
#define INV_Y_AXIS_INDEX                 (0x01)
#define INV_Z_AXIS_INDEX                 (0x02)

#define INV_ELEMENT_1                    (0x0001)
#define INV_ELEMENT_2                    (0x0002)
#define INV_ELEMENT_3                    (0x0004)
#define INV_ELEMENT_4                    (0x0008)
#define INV_ELEMENT_5                    (0x0010)
#define INV_ELEMENT_6                    (0x0020)
#define INV_ELEMENT_7                    (0x0040)
#define INV_ELEMENT_8                    (0x0080)
#define INV_ALL                          (0xFFFF)
#define INV_ELEMENT_MASK                 (0x00FF)
#define INV_GYRO_ACC_MASK                (0x007E)
/* scan element definition */
enum inv_mpu_scan {
	INV_MPU_SCAN_QUAT_R = 0,
	INV_MPU_SCAN_QUAT_X,
	INV_MPU_SCAN_QUAT_Y,
	INV_MPU_SCAN_QUAT_Z,
	INV_MPU_SCAN_GYRO_X,
	INV_MPU_SCAN_GYRO_Y,
	INV_MPU_SCAN_GYRO_Z,
	INV_MPU_SCAN_ACCL_X,
	INV_MPU_SCAN_ACCL_Y,
	INV_MPU_SCAN_ACCL_Z,
	INV_MPU_SCAN_MAGN_X,
	INV_MPU_SCAN_MAGN_Y,
	INV_MPU_SCAN_MAGN_Z,
	INV_MPU_SCAN_TIMESTAMP,
};

enum inv_filter_e {
	INV_FILTER_256HZ_NOLPF2 = 0,
	INV_FILTER_188HZ,
	INV_FILTER_98HZ,
	INV_FILTER_42HZ,
	INV_FILTER_20HZ,
	INV_FILTER_10HZ,
	INV_FILTER_5HZ,
	INV_FILTER_2100HZ_NOLPF,
	NUM_FILTER
};
/*==== MPU6050B1 MEMORY ====*/
enum MPU_MEMORY_BANKS {
	MEM_RAM_BANK_0 = 0,
	MEM_RAM_BANK_1,
	MEM_RAM_BANK_2,
	MEM_RAM_BANK_3,
	MEM_RAM_BANK_4,
	MEM_RAM_BANK_5,
	MEM_RAM_BANK_6,
	MEM_RAM_BANK_7,
	MEM_RAM_BANK_8,
	MEM_RAM_BANK_9,
	MEM_RAM_BANK_10,
	MEM_RAM_BANK_11,
	MPU_MEM_NUM_RAM_BANKS,
	MPU_MEM_OTP_BANK_0 = 16
};

enum inv_accl_fs_e {
	INV_FS_02G = 0,
	INV_FS_04G,
	INV_FS_08G,
	INV_FS_16G,
	NUM_ACCL_FSR
};

enum inv_fsr_e {
	INV_FSR_250DPS = 0,
	INV_FSR_500DPS,
	INV_FSR_1000DPS,
	INV_FSR_2000DPS,
	NUM_FSR
};

enum inv_clock_sel_e {
	INV_CLK_INTERNAL = 0,
	INV_CLK_PLL,
	NUM_CLK
};

void inv_wake_up(void);
int inv_set_power_state(struct inv_gyro_state_s *st, unsigned char power_on);
ssize_t inv_dmp_firmware_write(struct file *fp, struct kobject *kobj,
	struct bin_attribute *attr, char *buf, loff_t pos, size_t size);
ssize_t inv_dmp_firmware_read(struct file *filp,
				struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count);

int inv_mpu_configure_ring(struct iio_dev *indio_dev);
int inv_mpu_probe_trigger(struct iio_dev *indio_dev);
void inv_mpu_unconfigure_ring(struct iio_dev *indio_dev);
void inv_mpu_remove_trigger(struct iio_dev *indio_dev);
int inv_init_config_mpu3050(struct iio_dev *indio_dev);
int inv_get_silicon_rev_mpu6050(struct inv_gyro_state_s *st);
int set_3050_bypass(struct inv_gyro_state_s *st, int enable);
int inv_register_bma250_slave(struct inv_gyro_state_s *st);
void inv_setup_reg_mpu3050(struct inv_reg_map_s *reg);
int set_power_mpu3050(struct inv_gyro_state_s *st, unsigned char power_on);
int set_inv_enable(struct iio_dev *indio_dev, unsigned long enable);
int inv_send_quaternion(struct inv_gyro_state_s *st, int on);
int inv_set_display_orient_interrupt_dmp(struct inv_gyro_state_s *st, int on);
int inv_enable_orientation_dmp(struct inv_gyro_state_s *st, int on);
int inv_set_fifo_rate(struct inv_gyro_state_s *st, unsigned long fifo_rate);
unsigned short inv_dmp_get_address(unsigned short key);
long inv_q30_mult(long a, long b);
int inv_set_tap_threshold_dmp(struct inv_gyro_state_s *st,
				unsigned int axis, unsigned short threshold);
int inv_set_min_taps_dmp(struct inv_gyro_state_s *st, unsigned int min_taps);
int  inv_set_tap_time_dmp(struct inv_gyro_state_s *st, unsigned int time);
int inv_enable_tap_dmp(struct inv_gyro_state_s *st, unsigned char on);
int inv_i2c_read_base(struct inv_gyro_state_s *st, unsigned short i2c_addr,
	unsigned char reg, unsigned short length, unsigned char *data);
int inv_i2c_single_write_base(struct inv_gyro_state_s *st,
	unsigned short i2c_addr, unsigned char reg, unsigned char data);
int inv_do_test(struct inv_gyro_state_s *st, int self_test_flag,
		int *gyro_result, int *accl_result);
int mpu_memory_write(struct i2c_adapter *i2c_adap,
			    unsigned char mpu_addr,
			    unsigned short mem_addr,
			    unsigned int len, unsigned char const *data);
int mpu_memory_read(struct i2c_adapter *i2c_adap,
			   unsigned char mpu_addr,
			   unsigned short mem_addr,
			   unsigned int len, unsigned char *data);
int inv_hw_self_test(struct inv_gyro_state_s *st);

#define mem_w(a, b, c) mpu_memory_write(st->sl_handle,\
			st->i2c_addr, a, b, c)
#define mem_w_key(key, b, c) mpu_memory_write(st->sl_handle,\
			st->i2c_addr, inv_dmp_get_address(key), b, c)
#define inv_i2c_read(st, reg, len, data) \
	inv_i2c_read_base(st, st->i2c_addr, reg, len, data)
#define inv_i2c_single_write(st, reg, data) \
	inv_i2c_single_write_base(st, st->i2c_addr, reg, data)
#define inv_secondary_read(reg, len, data) \
	inv_i2c_read_base(st, st->plat_data.secondary_i2c_addr, reg, len, data)
#define inv_secondary_write(reg, data) \
	inv_i2c_single_write_base(st, st->plat_data.secondary_i2c_addr, \
		reg, data)
#endif  /* #ifndef _INV_GYRO_H_ */

