/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
 *
 * File Name		: lsm330_acc.c
 * Authors		: MSH - Motion Mems BU - Application Team
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Denis Ciocca (denis.ciocca@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
* Version		: V.1.2.6
* Date			: 2013/Apr/09
 * Description		: LSM330 accelerometer driver
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
Version History.
	V 1.0.0		First Release
	V 1.0.2		I2C address bugfix
	V 1.2.0		Registers names compliant to correct datasheet
	V.1.2.1		Removed enable_interrupt_output sysfs file, manages int1
			and int2, implements int1 isr.
	V.1.2.2		Added HR_Timer and custom sysfs path
	V.1.2.3		Ch state program codes and state prog parameters defines
	V.1.2.5		Changes create_sysfs_interfaces
	V.1.2.6		Changes resume and suspend functions
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
//#include <fih/lsm330.h>
#include "lsm330.h"
#include <linux/sensors.h>
//#define DEBUG

#define LOAD_STATE_PROGRAM1     1
#define LOAD_SP1_PARAMETERS     1
#define LOAD_STATE_PROGRAM2     1
#define LOAD_SP2_PARAMETERS     1
#define CONFIG_OF               1

#define G_MAX                   23920640    /* ug */
#define I2C_RETRY_DELAY         5           /* Waiting for signals [ms] */
#define I2C_RETRIES             5           /* Number of retries */
#define I2C_AUTO_INCREMENT      0x00        /* Autoincrement i2c address */
#define MS_TO_NS(x)             (x*1000000L)

#define SENSITIVITY_2G          60           /* ug/LSB */
#define SENSITIVITY_4G          120          /* ug/LSB */
#define SENSITIVITY_6G          180          /* ug/LSB */
#define SENSITIVITY_8G          240          /* ug/LSB */
#define SENSITIVITY_16G         730          /* ug/LSB */

#define	LSM330_ACC_FS_MASK      (0x38)
#define LSM330_ACC_BW_MASK      (0xC0)

/* Output Data Rates ODR */
#define LSM330_ODR_MASK	        (0XF0)
#define LSM330_PM_OFF           (0x00)        /* OFF */
#define LSM330_ODR3_125         (0x10)        /*    3.125 Hz */
#define LSM330_ODR6_25          (0x20)        /*    6.25  Hz */
#define LSM330_ODR12_5          (0x30)        /*   12.5   Hz */
#define LSM330_ODR25            (0x40)        /*   25     Hz */
#define LSM330_ODR50            (0x50)        /*   50     Hz */
#define LSM330_ODR100           (0x60)        /*  100     Hz */
#define LSM330_ODR400           (0x70)        /*  400     Hz */
#define LSM330_ODR800           (0x80)        /*  800     Hz */
#define LSM330_ODR1600          (0x90)        /* 1600     Hz */

/* Registers configuration Mask and settings */
/* CTRLREGx */
#define LSM330_INTEN_MASK       (0x01)
#define LSM330_INTEN_OFF        (0x00)
#define LSM330_INTEN_ON	        (0x01)

/* CTRLREG1 */
#define LSM330_HIST1_MASK       (0xE0)
#define LSM330_SM1INT_PIN_MASK  (0x08)
#define LSM330_SM1INT_PININT2   (0x08)
#define LSM330_SM1INT_PININT1   (0x00)
#define LSM330_SM1_EN_MASK      (0x01)
#define LSM330_SM1_EN_ON        (0x01)
#define LSM330_SM1_EN_OFF       (0x00)
/* */

/* CTRLREG2 */
#define LSM330_HIST2_MASK       (0xE0)
#define LSM330_SM2INT_PIN_MASK	(0x08)
#define LSM330_SM2INT_PININT2   (0x08)
#define LSM330_SM2INT_PININT1   (0x00)
#define LSM330_SM2_EN_MASK      (0x01)
#define LSM330_SM2_EN_ON        (0x01)
#define LSM330_SM2_EN_OFF       (0x00)
/* */

/* CTRLREG3 */
#define LSM330_INT_ACT_MASK     (0x01 << 6)
#define LSM330_INT_ACT_H        (0x01 << 6)
#define LSM330_INT_ACT_L        (0x00)

#define LSM330_INT2_EN_MASK     (0x01 << 4)
#define LSM330_INT2_EN_ON       (0x01 << 4)
#define LSM330_INT2_EN_OFF      (0x00)

#define LSM330_INT1_EN_MASK     (0x01 << 3)
#define LSM330_INT1_EN_ON       (0x01 << 3)
#define LSM330_INT1_EN_OFF      (0x00)
/* */

/* CTRLREG4 */
#define LSM330_BDU_EN           (0x08)
#define LSM330_ALL_AXES	        (0x07)
#define LSM330_Z_AXES           (0x04)
/* */

/* STATUS REG BITS */
#define LSM330_STAT_INTSM1_BIT  (0x01 << 3)
#define LSM330_STAT_INTSM2_BIT  (0x01 << 2)

#define OUT_AXISDATA_REG        LSM330_OUTX_L
#define WHOAMI_LIS3DSH_ACC      0x3F    /*OEM use LIS3DSH*/

/*CONTROL REGISTERS*/
#define LSM330_WHO_AM_I         (0x0F)  /* WhoAmI register Address */

#define LSM330_OUTX_L           (0x28)   /* Output X LSByte */
#define LSM330_OUTX_H           (0x29)   /* Output X MSByte */
#define LSM330_OUTY_L           (0x2A)   /* Output Y LSByte */
#define LSM330_OUTY_H           (0x2B)   /* Output Y MSByte */
#define LSM330_OUTZ_L           (0x2C)   /* Output Z LSByte */
#define LSM330_OUTZ_H           (0x2D)   /* Output Z MSByte */
#define LSM330_LC_L             (0x16)   /* LSByte Long Counter Status */
#define LSM330_LC_H             (0x17)   /* MSByte Long Counter Status */

#define LSM330_INTERR_STAT      (0x18)   /* Interrupt Status */

#define LSM330_STATUS_REG       (0x27)   /* Status */

#define LSM330_CTRL_REG1        (0x21)   /* control reg 1 */
#define LSM330_CTRL_REG2        (0x22)   /* control reg 2 */
#define LSM330_CTRL_REG3        (0x23)   /* control reg 3 */
#define LSM330_CTRL_REG4        (0x20)   /* control reg 4 */
#define LSM330_CTRL_REG5        (0x24)   /* control reg 5 */
#define LSM330_CTRL_REG6        (0x25)   /* control reg 6 */

#define LSM330_OFF_X            (0x10)   /* Offset X Corr */
#define LSM330_OFF_Y            (0x11)   /* Offset Y Corr */
#define LSM330_OFF_Z            (0x12)   /* Offset Z Corr */

#define LSM330_CS_X             (0x13)   /* Const Shift X */
#define LSM330_CS_Y             (0x14)   /* Const Shift Y */
#define LSM330_CS_Z             (0x15)   /* Const Shift Z */

#define LSM330_VFC_1            (0x1B)   /* Vect Filter Coeff 1 */
#define LSM330_VFC_2            (0x1C)   /* Vect Filter Coeff 2 */
#define LSM330_VFC_3            (0x1D)   /* Vect Filter Coeff 3 */
#define LSM330_VFC_4            (0x1E)   /* Vect Filter Coeff 4 */

/* state program 1 */
#define LSM330_STATEPR1         (0X40)   /*State Program 1 16 bytes */
#define LSM330_TIM4_1           (0X50)   /*SPr1 Timer4*/
#define LSM330_TIM3_1           (0X51)   /*SPr1 Timer3*/
#define LSM330_TIM2_1           (0X52)   /*SPr1 Timer2 2bytes*/
#define LSM330_TIM1_1           (0X54)   /*SPr1 Timer1 2bytes*/

#define LSM330_THRS2_1          (0X56)   /*SPr1 Threshold1*/
#define LSM330_THRS1_1          (0X57)   /*SPr1 Threshold2*/
#define LSM330_SA_1             (0X59)   /*SPr1 Swap Axis Sign Msk*/
#define LSM330_MA_1             (0X5A)   /*SPr1 Axis Sign Msk*/
#define LSM330_SETT_1           (0X5B)   /*SPr1*/
#define LSM330_PPRP_1           (0X5C)   /*SPr1 ProgPointer ResetPointer */
#define LSM330_TC_1             (0X5D)   /*SPr1 2bytes*/
#define LSM330_OUTS_1           (0X5F)   /*SPr1*/

/* state program 2 */
#define LSM330_STATEPR2         (0X60)   /*State Program 2 16 bytes */
#define LSM330_TIM4_2           (0X70)   /*SPr2 Timer4*/
#define LSM330_TIM3_2           (0X71)   /*SPr2 Timer3*/
#define LSM330_TIM2_2           (0X72)   /*SPr2 Timer2 2bytes*/
#define LSM330_TIM1_2           (0X74)   /*SPr2 Timer1 2bytes*/

#define LSM330_THRS2_2          (0X76)   /*SPr2 Threshold1*/
#define LSM330_THRS1_2          (0X77)   /*SPr2 Threshold2*/
#define LSM330_DES_2            (0X78)   /*SPr2 Decimation*/
#define LSM330_SA_2             (0X79)   /*SPr2 Swap Axis Sign Msk*/
#define LSM330_MA_2             (0X7A)   /*SPr2 Axis Sign Msk*/
#define LSM330_SETT_2           (0X7B)   /*SPr2 */
#define LSM330_PPRP_2           (0X7C)   /*SPr2 ProgPointer ResetPointer */
#define LSM330_TC_2             (0X7D)   /*SPr2 2bytes*/
#define LSM330_OUTS_2           (0X7F)   /*SPr2*/
/*end CONTROL REGISTRES*/

/* RESUME STATE INDICES */
#define RES_LSM330_LC_L         0
#define RES_LSM330_LC_H         1

#define RES_LSM330_CTRL_REG4    2
#define RES_LSM330_CTRL_REG1    3
#define RES_LSM330_CTRL_REG2    4
#define RES_LSM330_CTRL_REG3    5
#define RES_LSM330_CTRL_REG5    6
#define RES_LSM330_CTRL_REG6    7

#define RES_LSM330_OFF_X        8
#define RES_LSM330_OFF_Y        9
#define RES_LSM330_OFF_Z        10

#define RES_LSM330_CS_X	        11
#define RES_LSM330_CS_Y	        12
#define RES_LSM330_CS_Z	        13

#define RES_LSM330_VFC_1        14
#define RES_LSM330_VFC_2        15
#define RES_LSM330_VFC_3        16
#define RES_LSM330_VFC_4        17

#define RES_LSM330_THRS3        18

#define RES_LSM330_TIM4_1       20
#define RES_LSM330_TIM3_1       21
#define RES_LSM330_TIM2_1_L     22
#define RES_LSM330_TIM2_1_H     23
#define RES_LSM330_TIM1_1_L     24
#define RES_LSM330_TIM1_1_H     25

#define RES_LSM330_THRS2_1      26
#define RES_LSM330_THRS1_1      27
#define RES_LSM330_SA_1         28
#define RES_LSM330_MA_1         29
#define RES_LSM330_SETT_1       30

#define RES_LSM330_TIM4_2       31
#define RES_LSM330_TIM3_2       32
#define RES_LSM330_TIM2_2_L     33
#define RES_LSM330_TIM2_2_H     34
#define RES_LSM330_TIM1_2_L     35
#define RES_LSM330_TIM1_2_H     36

#define RES_LSM330_THRS2_2      37
#define RES_LSM330_THRS1_2      38
#define RES_LSM330_DES_2        39
#define RES_LSM330_SA_2         40
#define RES_LSM330_MA_2         41
#define RES_LSM330_SETT_2       42

#define LSM330_RESUME_ENTRIES   43

#define LSM330_STATE_PR_SIZE    16
/* end RESUME STATE INDICES */

/* STATE PROGRAMS ENABLE CONTROLS */
#define LSM330_SM1_DIS_SM2_DIS  (0x00)
#define LSM330_SM1_EN_SM2_DIS   (0x01)
#define LSM330_SM1_DIS_SM2_EN   (0x02)
#define LSM330_SM1_EN_SM2_EN    (0x03)

/* INTERRUPTS ENABLE CONTROLS */
#define LSM330_INT1_DIS_INT2_DIS (0x00)
#define LSM330_INT1_EN_INT2_DIS  (0x01)
#define LSM330_INT1_DIS_INT2_EN  (0x02)
#define LSM330_INT1_EN_INT2_EN   (0x03)

#define LSM330_SMART_ALERT_ODR    20

struct workqueue_struct *lsm330_workqueue = 0;
struct workqueue_struct *work_help = 0;
bool check;
char gesture_status = 0x00;
int pedometer_status = 0;
int st_offset_x = 0;
int st_offset_y = 0;
int st_offset_z = 0;
int  value_XYZ[3];

struct {
    unsigned int cutoff_ms;
    unsigned int mask;
} lsm330_acc_odr_table[] = {
    {    1, LSM330_ODR1600 },
    {    3, LSM330_ODR400  },
    {   10, LSM330_ODR100  },
    {   20, LSM330_ODR50   },
    {   40, LSM330_ODR25   },
    {   80, LSM330_ODR12_5 },
    {  160, LSM330_ODR6_25 },
    {  320, LSM330_ODR3_125},
};

static struct lsm330_acc_platform_data default_lsm330_acc_pdata = {
    .fs_range = LSM330_ACC_G_4G,
    .axis_map_x = 1,
    .axis_map_y = 0,
    .axis_map_z = 2,
    .negate_x = 0,
    .negate_y = 1,
    .negate_z = 0,
    .poll_interval = 40,
    .min_interval = LSM330_ACC_MIN_POLL_PERIOD_MS,
    .gpio_int1 = LSM330_ACC_DEFAULT_INT1_GPIO,
    .gpio_int2 = LSM330_ACC_DEFAULT_INT2_GPIO,
};

static struct sensors_classdev lis3dsh_acc_cdev = {
    .name = "lis3dsh-accel",
    .vendor = "STMicroelectronics",
    .version = 1,
    .handle = SENSORS_ACCELERATION_HANDLE,
    .type = SENSOR_TYPE_ACCELEROMETER,
    .max_range = "156.8",
    .resolution = "0.01",
    .sensor_power = "0.01",
    .min_delay = 10000,
    .max_delay = 200000,
    .delay_msec = 40,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .max_latency = 0,
    .flags = 0,
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
    .sensors_set_latency = NULL,
    .sensors_flush = NULL,
};

struct sensor_regulator {
    struct regulator *vreg;
    const char *name;
    u32 min_uV;
    u32	max_uV;
};

struct sensor_regulator lis3dh_acc_vreg[] = {
    {NULL, "vdd", 1700000, 3600000},
    {NULL, "vddio", 1700000, 3600000},
};

static int int1_gpio = LSM330_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = LSM330_ACC_DEFAULT_INT2_GPIO;
module_param(int1_gpio, int, S_IRUGO);
module_param(int2_gpio, int, S_IRUGO);
MODULE_PARM_DESC(int1_gpio, "integer: gpio number being assined to interrupt PIN1");
MODULE_PARM_DESC(int2_gpio, "integer: gpio number being assined to interrupt PIN2");

//@20150620 add for FAO-573 AMD sensor implementation begin
static u8 bSmartAlertInterrupt = 0;
static u8 bSmartAlertInterruptEnabled = 0;
//@20150620 add for FAO-573 AMD sensor implementation end

struct lsm330_acc_data {
    struct i2c_client *client;
    struct lsm330_acc_platform_data *pdata;

    struct mutex lock;
    struct work_struct input_work_acc;
    struct hrtimer hr_timer_acc;
    ktime_t ktime_acc;

    struct input_dev *input_dev;
    struct input_dev *input_gesture;

#ifdef CUSTOM_SYSFS_PATH
    struct class *acc_class;
    struct device *acc_dev;
#endif
    struct sensors_classdev cdev;
    struct pinctrl *gsensor_pinctrl;
    struct pinctrl_state *gsensor_gpio_state_active;
    struct pinctrl_state *gsensor_gpio_state_suspend;

    int hw_initialized;
    /* hw_working=-1 means not tested yet */
    int hw_working;
    atomic_t enabled;
    int on_before_suspend;
    int use_smbus;

    u16 sensitivity;
    u8 stateprogs_enable_setting;
    u8 on_before_stateprogs_enable;
    u8 resume_state[LSM330_RESUME_ENTRIES];
    u8 resume_stmach_program1[LSM330_STATE_PR_SIZE];
    u8 resume_stmach_program2[LSM330_STATE_PR_SIZE];

    u32 irq1;
    struct work_struct irq1_work;
    struct workqueue_struct *irq1_work_queue;
    u32 irq2;
    struct work_struct irq2_work;
    struct workqueue_struct *irq2_work_queue;

#ifdef DEBUG
    u8 reg_addr;
#endif
};

/* sets default init values to be written in registers at probe stage */
static void lsm330_acc_set_init_register_values(struct lsm330_acc_data *acc)
{
    acc->resume_state[RES_LSM330_LC_L] = 0xFF;
    acc->resume_state[RES_LSM330_LC_H] = 0x7F;

    acc->resume_state[RES_LSM330_CTRL_REG1] = (0x00 | LSM330_SM1INT_PININT1);
    acc->resume_state[RES_LSM330_CTRL_REG2] = (0x00 | LSM330_SM2INT_PININT1);
//  acc->resume_state[RES_LSM330_CTRL_REG3] = LSM330_INT_ACT_H|0x4;
    acc->resume_state[RES_LSM330_CTRL_REG3] = LSM330_INT_ACT_H;
    if(acc->pdata->gpio_int1 >= 0)
        acc->resume_state[RES_LSM330_CTRL_REG3] = acc->resume_state[RES_LSM330_CTRL_REG3] | LSM330_INT1_EN_ON;
    if(acc->pdata->gpio_int2 >= 0)
        acc->resume_state[RES_LSM330_CTRL_REG3] = acc->resume_state[RES_LSM330_CTRL_REG3] | LSM330_INT2_EN_ON;
    acc->resume_state[RES_LSM330_CTRL_REG4] = (LSM330_ODR6_25 | LSM330_ALL_AXES);
    acc->resume_state[RES_LSM330_CTRL_REG5] = 0x00;
    acc->resume_state[RES_LSM330_CTRL_REG6] = 0x10;

    acc->resume_state[RES_LSM330_THRS3] = 0x00;
    acc->resume_state[RES_LSM330_OFF_X] = 0x00;
    acc->resume_state[RES_LSM330_OFF_Y] = 0x00;
    acc->resume_state[RES_LSM330_OFF_Z] = 0x00;

    acc->resume_state[RES_LSM330_CS_X] = 0x00;
    acc->resume_state[RES_LSM330_CS_Y] = 0x00;
    acc->resume_state[RES_LSM330_CS_Z] = 0x00;
}

static void lsm330_acc_set_init_statepr1_inst(struct lsm330_acc_data *acc)
{
/* loads custom state program1 */
#if LOAD_STATE_PROGRAM1
    acc->resume_stmach_program1[0] = 0x09;
    acc->resume_stmach_program1[1] = 0x88;
    acc->resume_stmach_program1[2] = 0x33;
    acc->resume_stmach_program1[3] = 0X0A;
    acc->resume_stmach_program1[4] = 0x29;
    acc->resume_stmach_program1[5] = 0xA1;
    acc->resume_stmach_program1[6] = 0x25;
    acc->resume_stmach_program1[7] = 0x11;
    acc->resume_stmach_program1[8] = 0x00;
    acc->resume_stmach_program1[9] = 0x00;
    acc->resume_stmach_program1[10] = 0x00;
    acc->resume_stmach_program1[11] = 0x00;
    acc->resume_stmach_program1[12] = 0x00;
    acc->resume_stmach_program1[13] = 0x00;
    acc->resume_stmach_program1[14] = 0x00;
    acc->resume_stmach_program1[15] = 0x00;

#else /* loads default state program1 */
    acc->resume_stmach_program1[0] = 0x00;
    acc->resume_stmach_program1[1] = 0x00;
    acc->resume_stmach_program1[2] = 0X00;
    acc->resume_stmach_program1[3] = 0X00;
    acc->resume_stmach_program1[4] = 0x00;
    acc->resume_stmach_program1[5] = 0x00;
    acc->resume_stmach_program1[6] = 0x00;
    acc->resume_stmach_program1[7] = 0x00;
    acc->resume_stmach_program1[8] = 0x00;
    acc->resume_stmach_program1[9] = 0x00;
    acc->resume_stmach_program1[10] = 0x00;
    acc->resume_stmach_program1[11] = 0x00;
    acc->resume_stmach_program1[12] = 0x00;
    acc->resume_stmach_program1[13] = 0x00;
    acc->resume_stmach_program1[14] = 0x00;
    acc->resume_stmach_program1[15] = 0x00;
#endif /* LOAD_STATE_PROGRAM1 */
}

static void lsm330_acc_set_init_statepr2_inst(struct lsm330_acc_data *acc)
{
/* loads custom state program2 */
#if LOAD_STATE_PROGRAM2
    acc->resume_stmach_program2[0] = 0x15;
    acc->resume_stmach_program2[1] = 0x02;
    acc->resume_stmach_program2[2] = 0X15;
    acc->resume_stmach_program2[3] = 0X02;
    acc->resume_stmach_program2[4] = 0x15;
    acc->resume_stmach_program2[5] = 0x02;
    acc->resume_stmach_program2[6] = 0x15;
    acc->resume_stmach_program2[7] = 0x02;
    acc->resume_stmach_program2[8] = 0x15;
    acc->resume_stmach_program2[9] = 0xBB;
    acc->resume_stmach_program2[10] = 0xBB;
    acc->resume_stmach_program2[11] = 0xBB;
    acc->resume_stmach_program2[12] = 0x02;
    acc->resume_stmach_program2[13] = 0x22;
    acc->resume_stmach_program2[14] = 0x15;
    acc->resume_stmach_program2[15] = 0x0B;
#else /* loads default state program2 */
    acc->resume_stmach_program2[0] = 0x00;
    acc->resume_stmach_program2[1] = 0x00;
    acc->resume_stmach_program2[2] = 0X00;
    acc->resume_stmach_program2[3] = 0X00;
    acc->resume_stmach_program2[4] = 0x00;
    acc->resume_stmach_program2[5] = 0x00;
    acc->resume_stmach_program2[6] = 0x00;
    acc->resume_stmach_program2[7] = 0x00;
    acc->resume_stmach_program2[8] = 0x00;
    acc->resume_stmach_program2[9] = 0x00;
    acc->resume_stmach_program2[10] = 0x00;
    acc->resume_stmach_program2[11] = 0x00;
    acc->resume_stmach_program2[12] = 0x00;
    acc->resume_stmach_program2[13] = 0x00;
    acc->resume_stmach_program2[14] = 0x00;
    acc->resume_stmach_program2[15] = 0x00;
#endif /* LOAD_STATE_PROGRAM2 */
}

static void lsm330_acc_set_init_statepr1_param(struct lsm330_acc_data *acc)
{
/* loads custom state prog1 parameters */
#if LOAD_SP1_PARAMETERS
//    acc->resume_state[RES_LSM330_TIM4_1] = 0x05;
//    acc->resume_state[RES_LSM330_TIM3_1] = 0x14;
    acc->resume_state[RES_LSM330_TIM2_1_L] = 0x40;

    acc->resume_state[RES_LSM330_TIM2_1_H] = 0x00;
    acc->resume_state[RES_LSM330_TIM1_1_L] = 0x04;
    acc->resume_state[RES_LSM330_TIM1_1_H] = 0x00;
    /*acc->resume_state[RES_LSM330_THRS2_1] = 0x03;
    acc->resume_state[RES_LSM330_THRS1_1] = 0x03;*/  //small

    acc->resume_state[RES_LSM330_THRS2_1] = 0x30;
    acc->resume_state[RES_LSM330_THRS1_1] = 0x32;  //middle
    /*acc->resume_state[RES_LSM330_THRS2_1] = 0x0C;
    acc->resume_state[RES_LSM330_THRS1_1] = 0x0C;*/    //big

    /* DES1 not available*/
    acc->resume_state[RES_LSM330_SA_1] = 0xFC;
    acc->resume_state[RES_LSM330_MA_1] = 0xFC;
    acc->resume_state[RES_LSM330_SETT_1] = 0x23;
#else /* loads default state prog1 parameters */
    acc->resume_state[RES_LSM330_TIM4_1] = 0x00;
    acc->resume_state[RES_LSM330_TIM3_1] = 0x00;
    acc->resume_state[RES_LSM330_TIM2_1_L] = 0x00;
    acc->resume_state[RES_LSM330_TIM2_1_H] = 0x00;
    acc->resume_state[RES_LSM330_TIM1_1_L] = 0x00;
    acc->resume_state[RES_LSM330_TIM1_1_H] = 0x00;
    acc->resume_state[RES_LSM330_THRS2_1] = 0x00;
    acc->resume_state[RES_LSM330_THRS1_1] = 0x00;
    /* DES1 not available*/
    acc->resume_state[RES_LSM330_SA_1] = 0x00;
    acc->resume_state[RES_LSM330_MA_1] = 0x00;
    acc->resume_state[RES_LSM330_SETT_1] = 0x00;
#endif
}

static void lsm330_acc_set_init_statepr2_param(struct lsm330_acc_data *acc)
{
/* loads custom state prog2 parameters */
#if LOAD_SP2_PARAMETERS
    acc->resume_state[RES_LSM330_TIM4_2] = 0x00;
    acc->resume_state[RES_LSM330_TIM3_2] = 0x00;
    acc->resume_state[RES_LSM330_TIM2_2_L] = 0x2D;
    acc->resume_state[RES_LSM330_TIM2_2_H] = 0x00;
    acc->resume_state[RES_LSM330_TIM1_2_L] = 0x5A;
    acc->resume_state[RES_LSM330_TIM1_2_H] = 0x00;
    acc->resume_state[RES_LSM330_THRS2_2] = 0x00;
    acc->resume_state[RES_LSM330_THRS1_2] = 0x03;
    acc->resume_state[RES_LSM330_DES_2] = 0x00;
    acc->resume_state[RES_LSM330_SA_2] = 0x00;
    acc->resume_state[RES_LSM330_MA_2] = 0x03;
    acc->resume_state[RES_LSM330_SETT_2] = 0x21;
#else /* loads default state prog2 parameters */
    acc->resume_state[RES_LSM330_TIM4_2] = 0x00;
    acc->resume_state[RES_LSM330_TIM3_2] = 0x00;
    acc->resume_state[RES_LSM330_TIM2_2_L] = 0x00;
    acc->resume_state[RES_LSM330_TIM2_2_H] = 0x00;
    acc->resume_state[RES_LSM330_TIM1_2_L] = 0x00;
    acc->resume_state[RES_LSM330_TIM1_2_H] = 0x00;
    acc->resume_state[RES_LSM330_THRS2_2] = 0x00;
    acc->resume_state[RES_LSM330_THRS1_2] = 0x00;
    acc->resume_state[RES_LSM330_DES_2] = 0x00;
    acc->resume_state[RES_LSM330_SA_2] = 0x00;
    acc->resume_state[RES_LSM330_MA_2] = 0x00;
    acc->resume_state[RES_LSM330_SETT_2] = 0x00;
#endif
}

static int lsm330_acc_i2c_read(struct lsm330_acc_data *acc, u8 * buf, int len)
{
    int err;
    int tries = 0;

    struct i2c_msg msgs[] = {
        {
            .addr = acc->client->addr,
            .flags = acc->client->flags & I2C_M_TEN,
            .len = 1,
            .buf = buf,
        },
        {
            .addr = acc->client->addr,
            .flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
            .len = len,
            .buf = buf,
        },
    };

    do {
        err = i2c_transfer(acc->client->adapter, msgs, 2);
        if (err != 2)
            msleep_interruptible(I2C_RETRY_DELAY);
    } while ((err != 2) && (++tries < I2C_RETRIES));
    if (err != 2) {
        printk(KERN_INFO "[G-sensor]read transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}

static int lsm330_acc_i2c_write(struct lsm330_acc_data *acc, u8 * buf, int len)
{
    int err;
    int tries = 0;

    struct i2c_msg msgs[] = {
        {
            .addr = acc->client->addr,
            .flags = acc->client->flags & I2C_M_TEN,
            .len = len + 1,
            .buf = buf,
        },
    };

    do {
        err = i2c_transfer(acc->client->adapter, msgs, 1);
        if (err != 1)
            msleep_interruptible(I2C_RETRY_DELAY);
    } while ((err != 1) && (++tries < I2C_RETRIES));

    if (err != 1) {
        printk(KERN_INFO "[G-sensor]write transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }
    return err;
}

static int lsm330_acc_i2c_update(struct lsm330_acc_data *acc, u8 reg_address, u8 mask, u8 new_bit_values)
{
    int err = -1;
    u8 rdbuf[1] = { reg_address };
    u8 wrbuf[2] = { reg_address , 0x00 };

    u8 init_val;
    u8 updated_val;
    err = lsm330_acc_i2c_read(acc, rdbuf, 1);
    if (!(err < 0)) {
        init_val = rdbuf[0];
        updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
        wrbuf[1] = updated_val;
        err = lsm330_acc_i2c_write(acc, wrbuf, 1);
    }
    return err;
}

static int lsm330_acc_hw_init(struct lsm330_acc_data *acc)
{
    int i;
    int err = -1;
    u8 buf[17];

    pr_info("%s: hw init start\n", LSM330_ACC_DEV_NAME);

    buf[0] = LSM330_WHO_AM_I;
    err = lsm330_acc_i2c_read(acc, buf, 1);
    if (err < 0) {
    printk(KERN_INFO "[G-sensor]Error reading WHO_AM_I: is device available/working?\n");
        goto err_firstread;
    } else
        acc->hw_working = 1;

    if (buf[0] != WHOAMI_LIS3DSH_ACC) {
        printk(KERN_INFO "[G-sensor]device unknown. Expected: 0x%02x, Replies: 0x%02x\n", WHOAMI_LIS3DSH_ACC, buf[0]);
        err = -1; /* choose the right coded error */
        goto err_unknown_device;
    }

    buf[0] = (I2C_AUTO_INCREMENT | LSM330_LC_L);
    buf[1] = acc->resume_state[RES_LSM330_LC_L];
    buf[2] = acc->resume_state[RES_LSM330_LC_H];
    err = lsm330_acc_i2c_write(acc, buf, 2);
    if (err < 0)
        goto err_resume_state;

    buf[0] = (I2C_AUTO_INCREMENT | LSM330_TIM4_1);
    buf[1] = acc->resume_state[RES_LSM330_TIM4_1];
    buf[2] = acc->resume_state[RES_LSM330_TIM3_1];
    buf[3] = acc->resume_state[RES_LSM330_TIM2_1_L];
    buf[4] = acc->resume_state[RES_LSM330_TIM2_1_H];
    buf[5] = acc->resume_state[RES_LSM330_TIM1_1_L];
    buf[6] = acc->resume_state[RES_LSM330_TIM1_1_H];
    buf[7] = acc->resume_state[RES_LSM330_THRS2_1];
    buf[8] = acc->resume_state[RES_LSM330_THRS1_1];
    err = lsm330_acc_i2c_write(acc, buf, 8);
    if (err < 0)
        goto err_resume_state;

    buf[0] = (I2C_AUTO_INCREMENT | LSM330_SA_1);
    buf[1] = acc->resume_state[RES_LSM330_SA_1];
    buf[2] = acc->resume_state[RES_LSM330_MA_1];
    buf[3] = acc->resume_state[RES_LSM330_SETT_1];
    err = lsm330_acc_i2c_write(acc, buf, 3);
    if (err < 0)
        goto err_resume_state;

    buf[0] = (I2C_AUTO_INCREMENT | LSM330_TIM4_2);
    buf[1] = acc->resume_state[RES_LSM330_TIM4_2];
    buf[2] = acc->resume_state[RES_LSM330_TIM3_2];
    buf[3] = acc->resume_state[RES_LSM330_TIM2_2_L];
    buf[4] = acc->resume_state[RES_LSM330_TIM2_2_H];
    buf[5] = acc->resume_state[RES_LSM330_TIM1_2_L];
    buf[6] = acc->resume_state[RES_LSM330_TIM1_2_H];
    buf[7] = acc->resume_state[RES_LSM330_THRS2_2];
    buf[8] = acc->resume_state[RES_LSM330_THRS1_2];
    buf[9] = acc->resume_state[RES_LSM330_DES_2];
    buf[10] = acc->resume_state[RES_LSM330_SA_2];
    buf[11] = acc->resume_state[RES_LSM330_MA_2];
    buf[12] = acc->resume_state[RES_LSM330_SETT_2];
    err = lsm330_acc_i2c_write(acc, buf, 12);
    if (err < 0)
        goto err_resume_state;

    /*state program 1 */
    buf[0] = (I2C_AUTO_INCREMENT | LSM330_STATEPR1);
    for (i = 1; i <= LSM330_STATE_PR_SIZE; i++) {
        buf[i] = acc->resume_stmach_program1[i-1];
        pr_debug("i=%d,sm pr1 buf[%d]=0x%02x\n", i, i, buf[i]);
    };
    err = lsm330_acc_i2c_write(acc, buf, LSM330_STATE_PR_SIZE);
    if (err < 0)
        goto err_resume_state;

    /*state program 2 */
    buf[0] = (I2C_AUTO_INCREMENT | LSM330_STATEPR2);
    for(i = 1; i <= LSM330_STATE_PR_SIZE; i++){
        buf[i] = acc->resume_stmach_program2[i-1];
        pr_debug("i=%d,sm pr2 buf[%d]=0x%02x\n", i, i, buf[i]);
    };
    err = lsm330_acc_i2c_write(acc, buf, LSM330_STATE_PR_SIZE);
    if (err < 0)
        goto err_resume_state;

    buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG5);
    buf[1] = acc->resume_state[RES_LSM330_CTRL_REG5];
    buf[2] = acc->resume_state[RES_LSM330_CTRL_REG6];
    err = lsm330_acc_i2c_write(acc, buf, 2);
    if (err < 0)
        goto err_resume_state;

    buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG1);
    buf[1] = acc->resume_state[RES_LSM330_CTRL_REG1];
    buf[2] = acc->resume_state[RES_LSM330_CTRL_REG2];
    buf[3] = acc->resume_state[RES_LSM330_CTRL_REG3];
    err = lsm330_acc_i2c_write(acc, buf, 3);
    if (err < 0)
        goto err_resume_state;

    buf[0] = (LSM330_CTRL_REG4);
    buf[1] = acc->resume_state[RES_LSM330_CTRL_REG4];
    err = lsm330_acc_i2c_write(acc, buf, 1);
    if (err < 0)
        goto err_resume_state;

    acc->hw_initialized = 1;
    pr_info("%s: hw init done\n", LSM330_ACC_DEV_NAME);
    return 0;

err_firstread:
    acc->hw_working = 0;
err_unknown_device:
err_resume_state:
    acc->hw_initialized = 0;
    printk(KERN_INFO "[G-sensor]hw init error 0x%02x,0x%02x: %d\n", buf[0],buf[1], err);
    return err;
}

static void lsm330_acc_device_power_off(struct lsm330_acc_data *acc)
{
    int err;
    //printk(KERN_INFO "[G-sensor]%s enter\n", __FUNCTION__);
    err = lsm330_acc_i2c_update(acc, LSM330_CTRL_REG4,LSM330_ODR_MASK, LSM330_PM_OFF);
    if (err < 0)
        printk(KERN_INFO "[G-sensor]soft power off failed: %d\n", err);

    if (acc->pdata->power_off) {
        if(acc->pdata->gpio_int1)
            disable_irq_wake(acc->irq1);
        if(acc->pdata->gpio_int2)
            disable_irq_wake(acc->irq2);
        acc->pdata->power_off();
        acc->hw_initialized = 0;
    }
    if (acc->hw_initialized) {
        if(acc->pdata->gpio_int1 >= 0)
            disable_irq_wake(acc->irq1);
        if(acc->pdata->gpio_int2 >= 0)
            disable_irq_wake(acc->irq2);
        acc->hw_initialized = 0;
    }
    //printk(KERN_INFO "[G-sensor]%s exit\n", __FUNCTION__);
}

static int lsm330_acc_device_power_on(struct lsm330_acc_data *acc)
{
    int err = -1;
    //printk(KERN_INFO "[G-sensor]%s enter\n", __FUNCTION__);
    if (acc->pdata->power_on) {
        err = acc->pdata->power_on();
        if (err < 0) {
            printk(KERN_INFO "[G-sensor]power_on failed: %d\n", err);
            return err;
        }
        if(acc->pdata->gpio_int1 >= 0)
            enable_irq_wake(acc->irq1);
        if(acc->pdata->gpio_int2 >= 0)
            enable_irq_wake(acc->irq2);
    }

    if (!acc->hw_initialized) {
        err = lsm330_acc_hw_init(acc);
        if (acc->hw_working == 1 && err < 0) {
            lsm330_acc_device_power_off(acc);
            return err;
        }
    }

    if (acc->hw_initialized) {
        if(acc->pdata->gpio_int1 >= 0)
            enable_irq_wake(acc->irq1);
        if(acc->pdata->gpio_int2 >= 0)
            enable_irq_wake(acc->irq2);
    }
    //printk(KERN_INFO "[G-sensor]%s exit\n", __FUNCTION__);
    if(err < 0)
        return err;
    else
        return 0;
}

static irqreturn_t lsm330_acc_isr1(int irq, void *dev)
{
    struct lsm330_acc_data *acc = dev;

    disable_irq_wake(irq);
    queue_work(acc->irq1_work_queue, &acc->irq1_work);
    //printk(KERN_INFO "%s: isr1 queued\n", LSM330_ACC_DEV_NAME);

    return IRQ_HANDLED;
}

static irqreturn_t lsm330_acc_isr2(int irq, void *dev)
{
    struct lsm330_acc_data *acc = dev;

    disable_irq_wake(irq);
    queue_work(acc->irq2_work_queue, &acc->irq2_work);
    pr_debug("%s: isr2 queued\n", LSM330_ACC_DEV_NAME);

    return IRQ_HANDLED;
}

static void lsm330_acc_irq1_work_func(struct work_struct *work)
{
    int err = -1;
    u8 rbuf[2], status;
    struct lsm330_acc_data *acc;

    acc = container_of(work, struct lsm330_acc_data, irq1_work);
    /* TODO  add interrupt service procedure.ie:lsm330_acc_get_int_source(acc); */
    pr_info("%s: IRQ1 triggered\n", LSM330_ACC_DEV_NAME);
    /*  */
    rbuf[0] = LSM330_INTERR_STAT;
    err = lsm330_acc_i2c_read(acc, rbuf, 1);
    pr_debug("%s: INTERR_STAT_REG: 0x%02x\n",LSM330_ACC_DEV_NAME, rbuf[0]);
    status = rbuf[0];
    if(status & LSM330_STAT_INTSM1_BIT) {
        rbuf[0] = LSM330_OUTS_1;
        err = lsm330_acc_i2c_read(acc, rbuf, 1);
        pr_debug("%s: OUTS_1: 0x%02x\n",LSM330_ACC_DEV_NAME, rbuf[0]);
    }
//    if(status & LSM330_STAT_INTSM2_BIT) {
//        rbuf[0] = LSM330_OUTS_2;
//        err = lsm330_acc_i2c_read(acc, rbuf, 1);
//        pr_debug("%s: OUTS_2: 0x%02x\n",LSM330_ACC_DEV_NAME, rbuf[0]);
//    }

    if(bSmartAlertInterruptEnabled == 1)
    {
    	bSmartAlertInterruptEnabled = 0;
    	return;
    }
    
    /*
    input_report_key(acc->input_gesture, KEY_F4, 1);
    input_sync(acc->input_gesture);
    input_report_key(acc->input_gesture, KEY_F4, 0);
    input_sync(acc->input_gesture);
    */

    //@20150620 add for FAO-573 AMD sensor implementation begin
    bSmartAlertInterrupt = 1;
    printk(KERN_INFO "[G-sensor] Smart alert interrupt triggered\n");
    //@20150620 add for FAO-573 AMD sensor implementation end

    enable_irq_wake(acc->irq1);
//    pr_debug("%s: IRQ1 re-enabled\n", LSM330_ACC_DEV_NAME);
}

static void lsm330_acc_irq2_work_func(struct work_struct *work)
{
    struct lsm330_acc_data *acc;

    acc = container_of(work, struct lsm330_acc_data, irq2_work);
    pr_debug("%s: IRQ2 triggered\n", LSM330_ACC_DEV_NAME);
    /* TODO  add interrupt service procedure. ie:lsm330_acc_get_stat_source(acc); */
    /* ; */
    pr_debug("%s: IRQ2 served\n", LSM330_ACC_DEV_NAME);
//exit:
    enable_irq_wake(acc->irq2);
    pr_debug("%s: IRQ2 re-enabled\n", LSM330_ACC_DEV_NAME);
}

static int lsm330_acc_register_masked_update(struct lsm330_acc_data *acc,u8 reg_address, u8 mask, u8 new_bit_values, int resume_index)
{
    u8 config[2] = {0};
    u8 init_val, updated_val;
    int err;
    int step = 0;

    config[0] = reg_address;
    err = lsm330_acc_i2c_read(acc, config, 1);
    if (err < 0)
        goto error;
    init_val = config[0];
    acc->resume_state[resume_index] = init_val;
    step = 1;
    updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
    config[0] = reg_address;
    config[1] = updated_val;
    err = lsm330_acc_i2c_write(acc, config, 1);
    if (err < 0)
        goto error;
    acc->resume_state[resume_index] = updated_val;

    return err;
error:
    dev_err(&acc->client->dev, "register 0x%02x update failed at step %d, error: %d\n",config[0], step, err);
    return err;
}

static int lsm330_acc_update_fs_range(struct lsm330_acc_data *acc, u8 new_fs_range)
{
    int err=-1;
    u16 sensitivity;

    switch (new_fs_range) {
        case LSM330_ACC_G_2G:
            sensitivity = SENSITIVITY_2G;
            break;
        case LSM330_ACC_G_4G:
            sensitivity = SENSITIVITY_4G;
            break;
        case LSM330_ACC_G_6G:
            sensitivity = SENSITIVITY_6G;
            break;
        case LSM330_ACC_G_8G:
            sensitivity = SENSITIVITY_8G;
            break;
        case LSM330_ACC_G_16G:
            sensitivity = SENSITIVITY_16G;
            break;
        default:
            dev_err(&acc->client->dev, "invalid g range requested: %u\n", new_fs_range);
            return -EINVAL;
    }

    if (atomic_read(&acc->enabled)) {
        /* Updates configuration register 1,* which contains g range setting */
        err = lsm330_acc_register_masked_update(acc, LSM330_CTRL_REG5,LSM330_ACC_FS_MASK, new_fs_range, RES_LSM330_CTRL_REG5);
        if(err < 0) {
            dev_err(&acc->client->dev, "update g range failed\n");
            return err;
        } else
            acc->sensitivity = sensitivity;
    }

    if(err < 0)
        printk(KERN_INFO "[G-sensor]update g range not executed because the device is off\n");
    return err;
}

static int lsm330_acc_update_bw_range(struct lsm330_acc_data *acc, u8 new_bw_range)
{
    int err=-1;
    //printk(KERN_INFO "[Gsensor]%s new_bw_range %u\n", __func__, new_bw_range);
    if (atomic_read(&acc->enabled)) {
        /* Updates configuration register 1,* which contains g range setting */
        err = lsm330_acc_register_masked_update(acc, LSM330_CTRL_REG5,LSM330_ACC_BW_MASK, new_bw_range, RES_LSM330_CTRL_REG5);
        if(err < 0) {
            dev_err(&acc->client->dev, "update g range failed\n");
            return err;
        }
    }

    return err;
}

static int lsm330_acc_update_odr(struct lsm330_acc_data *acc,int poll_interval_ms)
{
    int err = -1;
    int i;
    u8 new_odr;
    //printk(KERN_INFO "[G-sensor]%s\n",__func__);
    /* Following, looks for the longest possible odr interval scrolling the
     * odr_table vector from the end (shortest interval) backward (longest
     * interval), to support the poll_interval requested by the system.
     * It must be the longest interval lower then the poll interval.*/
    for (i = ARRAY_SIZE(lsm330_acc_odr_table) - 1; i >= 0; i--) {
        if (lsm330_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
            break;
    }
    new_odr = lsm330_acc_odr_table[i].mask;
    /* If device is currently enabled, we need to write new
     *  configuration out to it */
    if (atomic_read(&acc->enabled)) {
        err = lsm330_acc_register_masked_update(acc,LSM330_CTRL_REG4, LSM330_ODR_MASK, new_odr,RES_LSM330_CTRL_REG4);
        acc->ktime_acc = ktime_set(0, MS_TO_NS(poll_interval_ms));
    }

    if(err < 0)
        dev_err(&acc->client->dev, "update odr failed %d\n", poll_interval_ms);
    return err;
}

#ifdef DEBUG
static int lsm330_acc_register_write(struct lsm330_acc_data *acc, u8 *buf,u8 reg_address, u8 new_value)
{
    int err = -1;

    /* Sets configuration register at reg_address
     *  NOTE: this is a straight overwrite  */
    buf[0] = reg_address;
    buf[1] = new_value;
    err = lsm330_acc_i2c_write(acc, buf, 1);
    if (err < 0){
        printk(KERN_INFO "[G-sensor]write register error\n");
        return err;
    }
    return err;
}

static int lsm330_acc_register_read(struct lsm330_acc_data *acc, u8 *buf, u8 reg_address)
{
    int err = -1;
    buf[0] = (reg_address);
    err = lsm330_acc_i2c_read(acc, buf, 1);
    //printk(KERN_INFO "[G-sensor]read register\n");
    return err;
}

static int lsm330_acc_register_update(struct lsm330_acc_data *acc, u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
    int err = -1;
    u8 init_val;
    u8 updated_val;
    err = lsm330_acc_register_read(acc, buf, reg_address);
    if (!(err < 0)) {
        init_val = buf[0];
        updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
        err = lsm330_acc_register_write(acc, buf, reg_address,updated_val);
    }
    return err;
}
#endif


static int lsm330_acc_get_data(struct lsm330_acc_data *acc, int *xyz)
{
    int err = -1;
    /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
    u8 acc_data[6];
    /* x,y,z hardware data */
    s32 hw_d[3] = { 0 };

    acc_data[0] = (I2C_AUTO_INCREMENT | OUT_AXISDATA_REG);
    err = lsm330_acc_i2c_read(acc, acc_data, 6);
    if (err < 0)
        return err;

    hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]));
    hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]));
    hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]));

    hw_d[0] = hw_d[0] * acc->sensitivity;
    hw_d[1] = hw_d[1] * acc->sensitivity;
    hw_d[2] = hw_d[2] * acc->sensitivity;

    xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x]) : (hw_d[acc->pdata->axis_map_x]));
    xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y]) : (hw_d[acc->pdata->axis_map_y]));
    xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z]) : (hw_d[acc->pdata->axis_map_z]));
    value_XYZ[0] = xyz[0];
    value_XYZ[1] = xyz[1];
    value_XYZ[2] = xyz[2];
//    pr_err("%s read x=%d, y=%d, z=%d\n", LSM330_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);

    return err;
}

//static void lsm330_acc_report_values(struct lsm330_acc_data *acc, int *xyz)
//{
//ktime_t timestamp;
//timestamp = ktime_get_boottime();
//
//
//    input_report_abs(acc->input_dev, ABS_X, (xyz[0] - st_offset_x));
//    input_report_abs(acc->input_dev, ABS_Y, (xyz[1] - st_offset_y));
//    input_report_abs(acc->input_dev, ABS_Z, (xyz[2] - st_offset_z));
//    //printk(KERN_INFO "[Gsensor]x = %d y = %d z =%d\n", xyz[0], xyz[1], xyz[2]);
//
//input_event(acc->input_dev,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
//input_event(acc->input_dev,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
//    input_sync(acc->input_dev);
//}

static int lsm330_acc_enable(struct lsm330_acc_data *acc)
{
    int err;
    //printk(KERN_INFO "[G-sensor]%s gesture_status = 0x%x", __func__, gesture_status);
    if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
        err = lsm330_acc_device_power_on(acc);
        if (err < 0) {
            atomic_set(&acc->enabled, 0);
            printk(KERN_INFO "[G-sensor]%s power on error\n", __func__);
            return err;
        }
        err = lsm330_acc_update_odr(acc,acc->pdata->poll_interval);
        hrtimer_start(&acc->hr_timer_acc,acc->ktime_acc,HRTIMER_MODE_REL);
    }else{
        err = lsm330_acc_update_odr(acc,acc->pdata->poll_interval);
        hrtimer_start(&acc->hr_timer_acc,acc->ktime_acc, HRTIMER_MODE_REL);
    }

    return 0;
}

static int lsm330_acc_disable(struct lsm330_acc_data *acc)
{
    int err1, err2;
    //printk(KERN_INFO "[G-sensor]%s ", __func__);
    if(gesture_status == 0x00) {
        if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
            err1 = cancel_work_sync(&acc->input_work_acc);
            //printk(KERN_INFO "[G-sensor]%s : cancel_work_sync() = %d\n", __func__, err1);
            err2 = hrtimer_cancel(&acc->hr_timer_acc);
            //printk(KERN_INFO "[G-sensor]%s : hrtimer_cancel() = %d\n", __func__, err2);
            lsm330_acc_device_power_off(acc);
        }
    }else{
        err1 = cancel_work_sync(&acc->input_work_acc);
        //printk(KERN_INFO "[G-sensor]%s : cancel_work_sync() = %d\n", __func__, err1);
        err2 = hrtimer_cancel(&acc->hr_timer_acc);
        //printk(KERN_INFO "[G-sensor]%s : hrtimer_cancel() = %d\n", __func__, err2);
    }
    //printk(KERN_INFO "[G-sensor]%s : end\n", __func__);
    return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,struct device_attribute *attr,char *buf)
{
    int val;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    val = acc->pdata->poll_interval;
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
    int err;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    unsigned long interval_ms;

    if (strict_strtoul(buf, 10, &interval_ms)){
        printk(KERN_INFO "[G-sensor]%s strtoul error\n", __func__);
        return -EINVAL;
    }
    if (!interval_ms){
        printk(KERN_INFO "[G-sensor]%s !interval_ms\n", __func__);
        return -EINVAL;
    }
    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s gesture_status = 0x%x lock gesture_status && 0x02 = 0x%x  gesture_status && 0x04 =0x%x\n", __func__, gesture_status, gesture_status & 0x02, gesture_status & 0x04);
    if(!(gesture_status & 0x02) || !(gesture_status & 0x04)){
        //printk(KERN_INFO "[G-sensor]%s start lsm330_acc_update_odr\n", __func__);
        err = lsm330_acc_update_odr(acc, interval_ms);
        if(err >= 0){
            acc->pdata->poll_interval = interval_ms;
            acc->cdev.delay_msec = acc->pdata->poll_interval;
        }
    }
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    return size;
}

static ssize_t attr_get_range(struct device *dev,struct device_attribute *attr, char *buf)
{
    u8 val;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    int range = 2;
    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    val = acc->pdata->fs_range ;
    switch(val) {
        case LSM330_ACC_G_2G:
            range = 2;
            break;
        case LSM330_ACC_G_4G:
            range = 4;
            break;
        case LSM330_ACC_G_6G:
            range = 6;
            break;
        case LSM330_ACC_G_8G:
            range = 8;
            break;
        case LSM330_ACC_G_16G:
            range = 16;
            break;
    }
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
    int err;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    unsigned long val;
    u8 range;
    if (strict_strtoul(buf, 10, &val))
        return -EINVAL;

    switch(val) {
        case 2:
            range = LSM330_ACC_G_2G;
            break;
        case 4:
            range = LSM330_ACC_G_4G;
            break;
        case 6:
            range = LSM330_ACC_G_6G;
            break;
        case 8:
            range = LSM330_ACC_G_8G;
            break;
        case 16:
            range = LSM330_ACC_G_16G;
            break;
        default:
            return -1;
    }

    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    err = lsm330_acc_update_fs_range(acc, range);
    if(err >= 0)
    {
        acc->pdata->fs_range = range;
    }
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    return size;
}

static ssize_t attr_set_bw(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int err;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    unsigned long val;
    u8 bw;
    if (strict_strtoul(buf, 10, &val))
        return -EINVAL;

    switch(val) {
        case 50:
            bw = LSM330_ACC_HZ_50HZ;
            break;
        case 200:
            bw = LSM330_ACC_HZ_200HZ;
            break;
        case 400:
            bw = LSM330_ACC_HZ_400HZ;
            break;
        case 800:
            bw = LSM330_ACC_HZ_800HZ;
            break;
        default:
            return -1;
    }

     mutex_lock(&acc->lock);
     //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
     err = lsm330_acc_update_bw_range(acc, bw);
     mutex_unlock(&acc->lock);
     //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
     return size;
}

static ssize_t attr_get_enable(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    int val = atomic_read(&acc->enabled);
    return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 10, &val))
        return -EINVAL;
    //printk(KERN_INFO "[G-Sensor]%s enable = %ld x_offset = %d y_offset = %d z_offset = %d\n", __func__, val, st_offset_x, st_offset_y, st_offset_z);
    if (val) {
        mutex_lock(&acc->lock);
        //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
        gesture_status |= 0x01;
        mutex_unlock(&acc->lock);
        //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
        lsm330_acc_enable(acc);
    } else {
        mutex_lock(&acc->lock);
        gesture_status &= 0xFE;
        mutex_unlock(&acc->lock);
        //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
        lsm330_acc_disable(acc);
    }
    return size;
}

static int lsm330_acc_state_progrs_off(struct lsm330_acc_data *acc)
{
    int err = -1;

    if(acc->pdata->gpio_int1 >= 0)
        disable_irq_wake(acc->irq1);
    if(acc->pdata->gpio_int2 >= 0)
        disable_irq_wake(acc->irq2);
    err = lsm330_acc_update_odr(acc,acc->pdata->poll_interval);
    if (err < 0) {
        dev_err(&acc->client->dev, "Cannot update ODR\n");
        err = -EBUSY;
    }
    return err;
}

static int lsm330_acc_state_progrs_enable_control(struct lsm330_acc_data *acc, u8 settings)
{
    u8 val1, val2;
    int err = -1;
    //settings = settings & 0x03;

    switch ( settings ) {
        case LSM330_SM1_DIS_SM2_DIS:
            val1 = LSM330_SM1_EN_OFF;
            val2 = LSM330_SM2_EN_OFF;
            break;
        case LSM330_SM1_DIS_SM2_EN:
            val1 = LSM330_SM1_EN_OFF;
            val2 = LSM330_SM2_EN_ON;
            break;
        case LSM330_SM1_EN_SM2_DIS:
            val1 = LSM330_SM1_EN_ON;
            val2 = LSM330_SM2_EN_OFF;
            break;
        case LSM330_SM1_EN_SM2_EN:
            val1 = LSM330_SM1_EN_ON;
            val2 = LSM330_SM2_EN_ON;
            break;
        default :
            printk(KERN_INFO "[G-sensor]invalid state program setting : 0x%02x\n",settings);
            return err;
    }
    err = lsm330_acc_register_masked_update(acc,LSM330_CTRL_REG1, LSM330_SM1_EN_MASK, val1,RES_LSM330_CTRL_REG1);
    if (err < 0 ){
        printk(KERN_INFO "[G-sensor]lsm330_acc_register_masked_update error1\n");
        return err;
    }

    err = lsm330_acc_register_masked_update(acc,LSM330_CTRL_REG2, LSM330_SM2_EN_MASK, val2,RES_LSM330_CTRL_REG2);
    if (err < 0 ){
        printk(KERN_INFO "[G-sensor]lsm330_acc_register_masked_update error2\n");
        return err;
    }
    acc->stateprogs_enable_setting = settings;

    //printk(KERN_INFO "[G-sensor]state program setting : 0x%02x\n",acc->stateprogs_enable_setting);
    return err;
}

static ssize_t attr_set_enable_state_prog(struct device *dev,struct device_attribute *attr,	const char *buf, size_t size)
{
    int err = -1;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    long val=0;
    long onoff=0;
    u8 buf_temp[5];

    if (strict_strtoul(buf, 16, &val))
        return -EINVAL;

    if ( val < 0x00 || val > LSM330_SM1_EN_SM2_EN){
        printk(KERN_INFO "[G-sensor]invalid state program setting, val: %ld\n",val);
        return -EINVAL;
    }
    //printk(KERN_INFO "[G-sensor]%s = %ld\n", __FUNCTION__, val);
    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    if (val){
        acc->on_before_stateprogs_enable =  atomic_read(&acc->enabled);
        gesture_status |= 0x02;
        bSmartAlertInterruptEnabled = 1;
        onoff = acc->stateprogs_enable_setting | 0x01;
        if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
            err = lsm330_acc_device_power_on(acc);
                if (err < 0) {
                    atomic_set(&acc->enabled, 0);
                    mutex_unlock(&acc->lock);
                    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
                    return err;
                }
        }else {
            if(acc->pdata->gpio_int1 >= 0)
                enable_irq_wake(acc->irq1);
            if(acc->pdata->gpio_int2 >= 0)
                enable_irq_wake(acc->irq2);
        }
//        buf_temp[0] = (I2C_AUTO_INCREMENT | LSM330_VFC_1);
//        buf_temp[1] = 0x7d;
//        buf_temp[2] = 0x40;
//        buf_temp[3] = 0x20;
//        buf_temp[4] = 0x10;
//        err = lsm330_acc_i2c_write(acc, buf_temp, 4);
//        if (err < 0)
//            printk(KERN_INFO "[G-sensor]LSM330_VFC_1 write error\n");
        buf_temp[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG5);
        buf_temp[1] = 0xC0;
        acc->sensitivity = SENSITIVITY_2G;
        err = lsm330_acc_i2c_write(acc, buf_temp, 1);
        if (err < 0)
            printk(KERN_INFO "[G-sensor]LSM330_CTRL_REG5 write error\n");
        if(acc->pdata->poll_interval >=LSM330_SMART_ALERT_ODR)
            err = lsm330_acc_update_odr(acc, LSM330_SMART_ALERT_ODR);
    }else{
        gesture_status &=0xFD;
        bSmartAlertInterruptEnabled = 0;
        onoff = acc->stateprogs_enable_setting & 0x02;
        err = lsm330_acc_update_fs_range(acc, acc->pdata->fs_range);
        if (err < 0) {
            dev_err(&acc->client->dev, "update_fs_range failed\n");
            err = -EBUSY;
        }
    }
    err = lsm330_acc_state_progrs_enable_control(acc, onoff);
//    if ((!val) && (gesture_status == 0x00)){
//        if(!acc->on_before_stateprogs_enable){
    pr_info("[%s]gesture_status = %d \n",__func__,gesture_status);
    if (!val){
        if(gesture_status == 0x00){
            if (atomic_cmpxchg(&acc->enabled, 1, 0))
                lsm330_acc_device_power_off(acc);
        }else
            err = lsm330_acc_state_progrs_off(acc);
    }
    //printk(KERN_INFO "[G-sensor]gesture_status = %d\n", gesture_status);
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    if (err < 0) {
        printk(KERN_INFO "[G-sensor]%s error happened\n", __func__);
        return err;
    }
    return size;
}

static ssize_t attr_get_enable_state_prog(struct device *dev,struct device_attribute *attr,char *buf)
{
    u8 val;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    val = acc->stateprogs_enable_setting;
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    return sprintf(buf, "0x%02x\n", val);
}


//@20150620 add for FAO-573 AMD sensor implementation begin
static ssize_t attr_set_alert_state(struct device *dev,struct device_attribute *attr,	const char *buf, size_t size)
{
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    long val=0;

    if (strict_strtoul(buf, 16, &val))
        return -EINVAL;
        
    mutex_lock(&acc->lock);
    bSmartAlertInterrupt = val;
    mutex_unlock(&acc->lock);

    return size;
}

static ssize_t attr_get_alert_state(struct device *dev,struct device_attribute *attr,char *buf)
{
    u8 val;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    mutex_lock(&acc->lock);
    val = bSmartAlertInterrupt;
    mutex_unlock(&acc->lock);
    return sprintf(buf, "%x\n", val);
}
//@20150620 add for FAO-573 AMD sensor implementation end

#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
    int rc;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    u8 x[2];
    unsigned long val;

    if (strict_strtoul(buf, 16, &val)){
        printk(KERN_INFO "[G-sensor]strict_strtoul Error\n");
        return -EINVAL;
    }
    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    x[0] = acc->reg_addr;
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    x[1] = val;
    rc = lsm330_acc_i2c_write(acc, x, 1);
    /*TODO: error need to be managed */
    return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,char *buf)
{
    ssize_t ret;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    int rc;
    u8 data;

    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    data = acc->reg_addr;
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    rc = lsm330_acc_i2c_read(acc, &data, 1);
    /*TODO: error need to be managed */
    ret = sprintf(buf, "0x%02x\n", data);
    return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    unsigned long val;
    if (strict_strtoul(buf, 16, &val))
        return -EINVAL;
    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    acc->reg_addr = val;
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    return size;
}
#endif

static ssize_t attr_set_data(struct device *dev, struct device_attribute *attr,const char *buf,size_t size)
{
    unsigned long val;
    if (kstrtoul(buf, 10, &val))
        return -EINVAL;
    pr_info("[****]attr_set_data = %lu \n",val);
    if(val)
        check = true;
    else
        check = false;
    return size;
}

static ssize_t attr_get_data(struct device *dev, struct device_attribute *attr,char *buf)
{
    ssize_t ret;
    /*TODO: error need to be managed */
    printk(KERN_INFO "Raw_data x = %d y = %d z = %d\n", (value_XYZ[0]), (value_XYZ[1]), (value_XYZ[2]));
    ret = sprintf(buf, "%d %d %d\n", (value_XYZ[0]), (value_XYZ[1]), (value_XYZ[2]));

    return ret;
}

static ssize_t attr_set_value(struct device *dev, struct device_attribute *attr,const char *buf,size_t size)
{
    //ssize_t ret;
    //TODO: error need to be managed
    printk(KERN_INFO "x = %d y = %d z = %d\n", (value_XYZ[0] - st_offset_x), (value_XYZ[1] - st_offset_y), (value_XYZ[2] - st_offset_z));
    //ret = sprintf(buf, "%d %d %d\n", (value_XYZ[0] - st_offset_x), (value_XYZ[1] - st_offset_y), (value_XYZ[2] - st_offset_z));
    return size;
}

static ssize_t attr_get_value(struct device *dev, struct device_attribute *attr,char *buf)
{
    ssize_t ret;
    /*TODO: error need to be managed */
    printk(KERN_INFO "x = %d y = %d z = %d\n", (value_XYZ[0] - st_offset_x), (value_XYZ[1] - st_offset_y), (value_XYZ[2] - st_offset_z));
    ret = sprintf(buf, "%d %d %d\n", (value_XYZ[0] - st_offset_x), (value_XYZ[1] - st_offset_y), (value_XYZ[2] - st_offset_z));
    return ret;
}


static ssize_t lsm330_offset_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    printk(KERN_INFO "%s :%d %d %d\n", __func__, st_offset_x, st_offset_y, st_offset_z);
    return sprintf(buf, "%d %d %d\n", st_offset_x, st_offset_y, st_offset_z);
}

static ssize_t lsm330_offset_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    signed long offset_x, offset_y, offset_z;
    sscanf(buf, "%ld %ld %ld", &offset_x, &offset_y, &offset_z);
    printk(KERN_INFO "%s :%ld %ld %ld\n", __func__, offset_x, offset_y, offset_z);
    st_offset_x = offset_x;
    st_offset_y = offset_y;
    st_offset_z = offset_z;
    return count;
}

static ssize_t attr_get_counter(struct device *dev,struct device_attribute *attr,char *buf)
{
    ssize_t ret;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    int rc;
    /*Long Counter*/
    u8 LC_data[2];
    int LC_InitValue = 0x7FFF;
    int data = 0;

    LC_data[0] = (I2C_AUTO_INCREMENT | LSM330_LC_L);
    rc = lsm330_acc_i2c_read(acc, LC_data, 2);
    if (rc <0)
        return rc;
    //calcuate
    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    data = LC_InitValue - ((LC_data[1] << 8) | LC_data[0]);
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
  
    ret = sprintf(buf, "%d\n", data);
    return ret;
}

static ssize_t attr_set_counter(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
    int err = -1;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    long val=0;
    long onoff=0;
    u8 buf_temp[5];

    if (strict_strtoul(buf, 16, &val))
        return -EINVAL;

    if ( val < 0x00 || val > 0x01){
        printk(KERN_INFO "[G-sensor]invalid state program setting, val: %ld\n",val);
        return -EINVAL;
    }
    //printk(KERN_INFO "[G-sensor]%s = %ld\n", __FUNCTION__, val);
    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);

    if (val){
        gesture_status |= 0x04;
        onoff = acc->stateprogs_enable_setting | 0x02;
        if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
            err = lsm330_acc_device_power_on(acc);
            if (err < 0) {
                atomic_set(&acc->enabled, 0);
                mutex_unlock(&acc->lock);
                printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
                return err;
            }
        }
        buf_temp[0] = (I2C_AUTO_INCREMENT | LSM330_VFC_1);
        buf_temp[1] = 0x7d;
        buf_temp[2] = 0x72;
        buf_temp[3] = 0x4c;
        buf_temp[4] = 0x26;
        err = lsm330_acc_i2c_write(acc, buf_temp, 4);
        if (err < 0)
            printk(KERN_INFO "[G-sensor]LSM330_VFC_1\n");

        buf_temp[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG5);
        buf_temp[1] = 0x08;

        err = lsm330_acc_i2c_write(acc, buf_temp, 1);
        if (err < 0)
            printk(KERN_INFO "[G-sensor]LSM330_CTRL_REG5 write error\n");

        err = lsm330_acc_update_odr(acc, 10);
        if(err >= 0)
        {
            acc->pdata->poll_interval = 10;
        }
    }else{
        gesture_status &= 0xFB;
        onoff = acc->stateprogs_enable_setting & 0x01;
        buf_temp[0] = (I2C_AUTO_INCREMENT | LSM330_LC_L);
        buf_temp[1] = acc->resume_state[RES_LSM330_LC_L];
        buf_temp[2] = acc->resume_state[RES_LSM330_LC_H];
        err = lsm330_acc_i2c_write(acc, buf_temp, 2);
        if (err < 0)
            printk(KERN_INFO "[G-sensor]%s reset error\n", __func__);
    }
    err = lsm330_acc_state_progrs_enable_control(acc, onoff);
    if ((!val) && (gesture_status == 0x00)){
        if (atomic_cmpxchg(&acc->enabled, 1, 0))
            lsm330_acc_device_power_off(acc);
    }
    //printk(KERN_INFO "[G-sensor]gesture_status = %d\n", gesture_status);
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    if (err < 0)
        return err;
    return size;
}

static ssize_t attr_get_mode(struct device *dev,struct device_attribute *attr,	char *buf)
{
    //printk(KERN_INFO "%s :%d\n", __func__, pedometer_status);
    return sprintf(buf, "%d\n", pedometer_status);
}

static ssize_t attr_set_mode(struct device *dev,struct device_attribute *attr,	const char *buf, size_t size)
{
    int err = -1;
    struct lsm330_acc_data *acc = dev_get_drvdata(dev);
    long val=0;
    u8 buf_temp[17];

    if (strict_strtoul(buf, 16, &val))
        return -EINVAL;
    if ( val < 0x00 || val > 0x01){
        printk(KERN_INFO "[G-sensor]invalid state program setting, val: %ld\n",val);
        return -EINVAL;
    }
    //printk(KERN_INFO "[G-sensor]%s = %ld\n", __FUNCTION__, val);
    mutex_lock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s lock\n", __func__);

    if(val == 0x00) {
        buf_temp[0] = (I2C_AUTO_INCREMENT | LSM330_TIM4_2);
        buf_temp[1] = 0x00;
        buf_temp[2] = 0x00;
        buf_temp[3] = 0x2D;
        buf_temp[4] = 0x00;
        buf_temp[5] = 0x5A;
        buf_temp[6] = 0x00;
        buf_temp[7] = 0x00;
        buf_temp[8] = 0x03;
        buf_temp[9] = 0x00;
        buf_temp[10] = 0x00;
        buf_temp[11] = 0x03;
        buf_temp[12] = 0x21;
        err = lsm330_acc_i2c_write(acc, buf_temp, 12);
        if (err < 0)
            printk(KERN_INFO "[G-sensor]%s write LSM330_TIM4_2 error\n", __func__);
    } else {
        buf_temp[0] = (I2C_AUTO_INCREMENT | LSM330_TIM4_2);
        buf_temp[1] = 0x00;
        buf_temp[2] = 0x00;
        buf_temp[3] = 0x19;
        buf_temp[4] = 0x00;
        buf_temp[5] = 0x52;
        buf_temp[6] = 0x00;
        buf_temp[7] = 0x00;
        buf_temp[8] = 0x0B;
        buf_temp[9] = 0x00;
        buf_temp[10] = 0x00;
        buf_temp[11] = 0x03;
        buf_temp[12] = 0x21;
        err = lsm330_acc_i2c_write(acc, buf_temp, 12);
        if (err < 0)
            printk(KERN_INFO "[G-sensor]%s write LSM330_TIM4_2 error\n", __func__);
    }
    sscanf(buf, "%d", &pedometer_status);
    //printk(KERN_INFO "[G-sensor]gesture_status = %d\n", gesture_status);
    mutex_unlock(&acc->lock);
    //printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    if (err < 0)
        return err;
    return size;
}

static struct device_attribute attributes[] = {
    __ATTR(pollrate_ms, 0664, attr_get_polling_rate,attr_set_polling_rate),
    __ATTR(range, 0664, attr_get_range, attr_set_range),
    __ATTR(bandwidth, 0664, attr_get_range , attr_set_bw),
    __ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
    __ATTR(smart_alert_enable_int, 0664, attr_get_enable_state_prog,attr_set_enable_state_prog),
    __ATTR(step_count, 0664, attr_get_counter,attr_set_counter),
    __ATTR(set_pedometer_mode, 0664, attr_get_mode,attr_set_mode),
    /*FQC Calibration Add{*/
    __ATTR(data, 0664, attr_get_data, attr_set_data),
    __ATTR(offset, 0664, lsm330_offset_show, lsm330_offset_store),
    __ATTR(value, 0664, attr_get_value, attr_set_value),
    /*}FQC Calibration Add*/
    //@20150620 add for FAO-573 AMD sensor implementation begin
    __ATTR(smart_alert_value, 0664, attr_get_alert_state,attr_set_alert_state),
    //@20150620 add for FAO-573 AMD sensor implementation end
    
#ifdef DEBUG
    __ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
    __ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct lsm330_acc_data *acc)
{
    int i;
#ifdef CUSTOM_SYSFS_PATH
    acc->acc_class = class_create(THIS_MODULE, CUSTOM_SYSFS_CLASS_NAME_ACC);
    if (acc->acc_class == NULL)
        goto custom_class_error;
    acc->acc_dev = device_create(acc->acc_class, NULL, 0, "%s", "acc");
    if (acc->acc_dev == NULL)
        goto custom_class_error;
    for (i = 0; i < ARRAY_SIZE(attributes); i++)
        if (device_create_file(acc->acc_dev, attributes + i))
            goto error;
#else
    for (i = 0; i < ARRAY_SIZE(attributes); i++)
        if (device_create_file(&acc->client->dev, attributes + i))
            goto error;
#endif
    return 0;

error:
    for ( ; i >= 0; i--)
#ifdef CUSTOM_SYSFS_PATH
        device_remove_file(acc->acc_dev, attributes + i);
#else
        device_remove_file(&acc->client->dev, attributes + i);
#endif

#ifdef CUSTOM_SYSFS_PATH
custom_class_error:
#endif
    printk(KERN_INFO "[G-sensor]%s:Unable to create interface\n", __func__);
    return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(attributes); i++)
        device_remove_file(dev, attributes + i);
    return 0;
}

int lsm330_acc_input_open(struct input_dev *input)
{
#if 0
    struct lsm330_acc_data *acc = input_get_drvdata(input);
    printk(KERN_INFO "[G-sensor]%s\n", __func__);
    mutex_lock(&acc->lock);
    printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    gesture_status |= 0x01;
    mutex_unlock(&acc->lock);
    printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    return lsm330_acc_enable(acc);
#endif
	return 0;
}

void lsm330_acc_input_close(struct input_dev *dev)
{
#if 0
    struct lsm330_acc_data *acc = input_get_drvdata(dev);
    printk(KERN_INFO "[G-sensor]%s\n", __func__);
    mutex_lock(&acc->lock);printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    gesture_status &= 0xFE;
    mutex_unlock(&acc->lock);printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    lsm330_acc_disable(acc);
#endif
}

static int lsm330_acc_validate_pdata(struct lsm330_acc_data *acc)
{
    acc->pdata->poll_interval = max(acc->pdata->poll_interval,acc->pdata->min_interval);

    if (acc->pdata->axis_map_x > 2 || acc->pdata->axis_map_y > 2 || acc->pdata->axis_map_z > 2) {
        dev_err(&acc->client->dev, "invalid axis_map value x:%u y:%u z%u\n", acc->pdata->axis_map_x,acc->pdata->axis_map_y, acc->pdata->axis_map_z);
        return -EINVAL;
    }

    /* Only allow 0 and 1 for negation boolean flag */
    if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1 || acc->pdata->negate_z > 1) {
        dev_err(&acc->client->dev, "invalid negate value x:%u y:%u z:%u\n", acc->pdata->negate_x,acc->pdata->negate_y, acc->pdata->negate_z);
        return -EINVAL;
    }

    /* Enforce minimum polling interval */
    if (acc->pdata->poll_interval < acc->pdata->min_interval) {
        dev_err(&acc->client->dev, "minimum poll interval violated\n");
        return -EINVAL;
    }

    return 0;
}

static int lsm330_acc_input_init(struct lsm330_acc_data *acc)
{
    int err;

    acc->input_dev = input_allocate_device();
    if (!acc->input_dev) {
        err = -ENOMEM;
        dev_err(&acc->client->dev, "input device allocation failed\n");
        goto err0;
    }
    acc->input_gesture = input_allocate_device();
    if (!acc->input_gesture) {
        err = -ENOMEM;
        dev_err(&acc->client->dev, "gesture input device allocation failed\n");
        goto err0;
    }

    acc->input_dev->open = lsm330_acc_input_open;
    acc->input_dev->close = lsm330_acc_input_close;
    acc->input_dev->name = "accelerometer";//LSM330_ACC_DEV_NAME;

    acc->input_dev->id.bustype = BUS_I2C;
    acc->input_dev->dev.parent = &acc->client->dev;

    acc->input_gesture->name = "gsensor_gesture";
    acc->input_gesture->id.bustype = BUS_HOST;
    acc->input_gesture->dev.parent = &acc->client->dev;

    input_set_drvdata(acc->input_dev, acc);
    input_set_drvdata(acc->input_gesture, acc);

    set_bit(EV_ABS, acc->input_dev->evbit);
    /*next is used for interruptA sources data if the case */
    set_bit(ABS_MISC, acc->input_dev->absbit);
    /*next is used for interruptB sources data if the case */
    //set_bit(REL_WHEEL, acc->input_dev->absbit);
    acc->input_gesture->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) ;

    input_set_events_per_packet(acc->input_dev,100);
    input_set_capability(acc->input_dev, EV_ABS, ABS_MISC);

    input_set_capability(acc->input_gesture, EV_KEY, KEY_F4);
    input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, 0, 0);
    input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, 0, 0);
    input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, 0, 0);
    /*next is used for interruptA sources data if the case */
    //input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
    /*next is used for interruptB sources data if the case */
    //input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);

    err = input_register_device(acc->input_dev);
    if (err) {
        dev_err(&acc->client->dev,"unable to register input device %s\n",acc->input_dev->name);
        goto err1;
    }

    err = input_register_device(acc->input_gesture);
    if (err) {
        dev_err(&acc->client->dev,"unable to register input device %s\n",acc->input_gesture->name);
        goto err1;
    }

    return 0;

err1:
    input_free_device(acc->input_gesture);
    input_free_device(acc->input_dev);
err0:
    return err;
}

static void lsm330_acc_input_cleanup(struct lsm330_acc_data *acc)
{
    input_unregister_device(acc->input_dev);
    input_free_device(acc->input_dev);
}

static int lis3dsh_acc_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
    struct lsm330_acc_data *acc = container_of(sensors_cdev, struct lsm330_acc_data, cdev);
    int err;
    if (enable) {
        err = lsm330_acc_enable(acc);
        if (err < 0) {
            dev_err(&acc->client->dev, "enable error\n");
            return err;
//        }else {
//            mutex_lock(&acc->lock);
//            gesture_status |= 0x01;
//            mutex_unlock(&acc->lock);
//            err = lsm330_acc_update_odr(acc,acc->pdata->poll_interval);
//            if (err < 0) {
//                dev_err(&acc->client->dev, "Cannot update ODR\n");
//                err = -EBUSY;
//            }
       }
    } else {
        err = lsm330_acc_disable(acc);
        if (err < 0) {
            dev_err(&acc->client->dev, "enable error\n");
            return err;
        }
//        mutex_lock(&acc->lock);
//        gesture_status &= 0xFE;
//        mutex_unlock(&acc->lock);
    }
    return err;
}

static int lis3dsh_acc_poll_delay_set(struct sensors_classdev *sensors_cdev, unsigned int delay_msec)
{
    struct lsm330_acc_data *acc = container_of(sensors_cdev,struct lsm330_acc_data, cdev);
    int err;

    dev_dbg(&acc->client->dev, "set poll delay =%d\n", delay_msec);
    mutex_lock(&acc->lock);
    acc->pdata->poll_interval = delay_msec;
    acc->cdev.delay_msec = acc->pdata->poll_interval;
    /*
     * Device may not power on,
     * only set register when device is enabled.
     */
    if (atomic_read(&acc->enabled)) {
        err = lsm330_acc_update_odr(acc, delay_msec);
        if (err < 0) {
            dev_err(&acc->client->dev, "Cannot update ODR\n");
            err = -EBUSY;
        }
    }
    mutex_unlock(&acc->lock);
    return err;
}

static void poll_function_work_acc(struct work_struct *input_work_acc)
{
    struct lsm330_acc_data *acc;
    int xyz[3] = { 0 };
    int err;
    ktime_t timestamp;
    acc = container_of((struct work_struct *)input_work_acc,struct lsm330_acc_data, input_work_acc);

    timestamp = ktime_get_boottime();
    err = lsm330_acc_get_data(acc, xyz);
    input_report_abs(acc->input_dev, ABS_X, (xyz[0] - st_offset_x));
    input_report_abs(acc->input_dev, ABS_Y, (xyz[1] - st_offset_y));
    input_report_abs(acc->input_dev, ABS_Z, (xyz[2] - st_offset_z));
    //printk(KERN_INFO "[Gsensor]x = %d y = %d z =%d\n", xyz[0], xyz[1], xyz[2]);
    input_event(acc->input_dev,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
    input_event(acc->input_dev,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
    input_sync(acc->input_dev);

}

enum hrtimer_restart poll_function_read_acc(struct hrtimer *timer)
{
    struct lsm330_acc_data *acc;
    acc = container_of((struct hrtimer *)timer,struct lsm330_acc_data, hr_timer_acc);

    if(check)
        queue_work(work_help, &acc->input_work_acc);
    else
        queue_work(lsm330_workqueue, &acc->input_work_acc);
    acc->ktime_acc = ktime_set(0, MS_TO_NS(acc->pdata->poll_interval));
	  hrtimer_forward_now(&acc->hr_timer_acc,acc->ktime_acc );
    return HRTIMER_RESTART;
}

static int gsensor_pinctrl_select(bool on,struct lsm330_acc_data *sensor)
{
    struct pinctrl_state *pins_state;
    int ret;

    pins_state = on ? sensor->gsensor_gpio_state_active : sensor->gsensor_gpio_state_suspend;
    if (!IS_ERR_OR_NULL(pins_state)) {
        ret = pinctrl_select_state(sensor->gsensor_pinctrl, pins_state);
        if (ret) {
            printk(KERN_INFO "can not set %s pins\n", on ? "gsensor_active" : "gsensor_suspend");
            return ret;
        }
    } else
        printk(KERN_INFO "not a valid '%s' pinstate\n",on ? "gsensor_active" : "gsensor_suspend");

    return 0;
}

static int gsensor_pinctrl_init(struct i2c_client *client,struct lsm330_acc_data *sensor)
{
    int retval;

    /* Get pinctrl if target uses pinctrl */
    sensor->gsensor_pinctrl = devm_pinctrl_get(&(client->dev));
    if (IS_ERR_OR_NULL(sensor->gsensor_pinctrl)) {
        printk(KERN_INFO "Target does not use pinctrl\n");
        retval = PTR_ERR(sensor->gsensor_pinctrl);
        sensor->gsensor_pinctrl = NULL;
        return retval;
    }

    sensor->gsensor_gpio_state_active = pinctrl_lookup_state(sensor->gsensor_pinctrl, "gsensor_active");
    if (IS_ERR_OR_NULL(sensor->gsensor_gpio_state_active)) {
        printk(KERN_INFO "Can not get ts default pinstate\n");
        retval = PTR_ERR(sensor->gsensor_gpio_state_active);
        sensor->gsensor_pinctrl = NULL;
        return retval;
    }

    sensor->gsensor_gpio_state_suspend = pinctrl_lookup_state(sensor->gsensor_pinctrl, "gsensor_suspend");
    if (IS_ERR_OR_NULL(sensor->gsensor_gpio_state_suspend)) {
        printk(KERN_INFO "Can not get ts sleep pinstate\n");
        retval = PTR_ERR(sensor->gsensor_gpio_state_suspend);
        sensor->gsensor_pinctrl = NULL;
        return retval;
    }

    return 0;
}
#ifdef CONFIG_OF
static int lsm330_parse_dt(struct device *dev, struct lsm330_acc_platform_data *pdata)
{
    struct device_node *np = dev->of_node;
    u32 temp_val;
    int rc;
/*
    rc = of_property_read_u32(np, "st,min-interval", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read min-interval\n");
        return rc;
    } else {
        pdata->min_interval = temp_val;
    }

    rc = of_property_read_u32(np, "st,init-interval", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read init-interval\n");
        return rc;
    } else {
        pdata->init_interval = temp_val;
    }
*/
    rc = of_property_read_u32(np, "st,axis-map-x", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read axis-map_x\n");
        return rc;
    } else {
        pdata->axis_map_x = (u8)temp_val;
    }

    rc = of_property_read_u32(np, "st,axis-map-y", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read axis_map_y\n");
        return rc;
    } else {
        pdata->axis_map_y = (u8)temp_val;
    }

    rc = of_property_read_u32(np, "st,axis-map-z", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read axis-map-z\n");
        return rc;
    } else {
        pdata->axis_map_z = (u8)temp_val;
    }

    rc = of_property_read_u32(np, "st,g-range", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read g-range\n");
        return rc;
    } else {
        switch (temp_val) {
            case 2:
                pdata->fs_range = LSM330_ACC_G_2G;
                break;
            case 4:
                pdata->fs_range = LSM330_ACC_G_4G;
                break;
            case 8:
                pdata->fs_range = LSM330_ACC_G_8G;
                break;
            case 16:
                pdata->fs_range = LSM330_ACC_G_16G;
                break;
            default:
                pdata->fs_range = LSM330_ACC_G_2G;
                break;
        }
    }
    pr_err("axis_map = %d,%d,%d range= %d\n",pdata->axis_map_x,pdata->axis_map_y,pdata->axis_map_z,temp_val);
    pdata->negate_x = of_property_read_bool(np, "st,negate-x");
    pdata->negate_y = of_property_read_bool(np, "st,negate-y");
    pdata->negate_z = of_property_read_bool(np, "st,negate-z");
    pdata->gpio_int1 = of_get_named_gpio_flags(dev->of_node,"st,gpio-int1", 0, NULL);
    pdata->gpio_int2 = of_get_named_gpio_flags(dev->of_node,"st,gpio-int2", 0, NULL);
    pr_err("negate:%d,%d,%d int1:%d\n",pdata->negate_x,pdata->negate_y,pdata->negate_z,pdata->gpio_int1);
    return 0;
}
#else
static int lis3dh_parse_dt(struct device *dev,struct lis3dh_acc_platform_data *pdata)
{
    return -EINVAL;
}
#endif
static int lsm330_acc_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    struct lsm330_acc_data *acc;
    struct regulator *vcc, *vcc_i2c;
//    struct device_node *np=client->dev.of_node;
//    u32 irq_flag;
    u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK ;
    int err = -1;

    dev_info(&client->dev, "probe start.\n");

    acc = kzalloc(sizeof(struct lsm330_acc_data), GFP_KERNEL);
    if (acc == NULL) {
        err = -ENOMEM;
        dev_err(&client->dev,"failed to allocate memory for module data: %d\n", err);
        goto exit_check_functionality_failed;
    }

    vcc = regulator_get(&client->dev, "vcc");
    if (IS_ERR(vcc)) {
        err = PTR_ERR(vcc);
        printk(KERN_INFO "Regulator get failed vcc rc=%d\n", err);
    }
    err = regulator_enable(vcc);
    if (err)
        printk(KERN_INFO "Regulator vdd enable failed rc=%d\n", err);

    vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
    if (IS_ERR(vcc_i2c)) {
        err = PTR_ERR(vcc_i2c);
        printk(KERN_INFO "Regulator get failed vcc_i2c rc=%d\n", err);
    }
    err = regulator_enable(vcc_i2c);
    if (err)
        printk(KERN_INFO "Regulator vdd enable failed rc=%d\n", err);

    /* Support for both I2C and SMBUS adapter interfaces. */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_warn(&client->dev, "client not i2c capable\n");
        if (i2c_check_functionality(client->adapter, smbus_func)){
            acc->use_smbus = 1;
            dev_warn(&client->dev, "client using SMBUS\n");
        } else {
            err = -ENODEV;
            dev_err(&client->dev, "client nor SMBUS capable\n");
            acc->use_smbus = 0;
            goto exit_check_functionality_failed;
        }
    } else {
        acc->use_smbus = 0;
    }

    check=false;
    if(lsm330_workqueue == 0)
         lsm330_workqueue = create_freezable_workqueue("lsm330_workqueue");
//        lsm330_workqueue = create_workqueue("lsm330_workqueue");

    hrtimer_init(&acc->hr_timer_acc, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
    acc->hr_timer_acc.function = &poll_function_read_acc;

    work_help = alloc_workqueue("w_help",WQ_HIGHPRI, 0);
    if (!work_help) {
        dev_err(&client->dev, "Cannot create workqueue!\n");
    }

    mutex_init(&acc->lock);
    mutex_lock(&acc->lock);
    printk(KERN_INFO "[G-sensor]%s lock\n", __func__);
    acc->client = client;
    i2c_set_clientdata(client, acc);

    acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
    if (acc->pdata == NULL) {
        err = -ENOMEM;
        dev_err(&client->dev,"failed to allocate memory for pdata: %d\n",err);
        goto err_mutexunlock;
    }

    if(client->dev.platform_data == NULL) {
        default_lsm330_acc_pdata.gpio_int1 = int1_gpio;
        default_lsm330_acc_pdata.gpio_int2 = int2_gpio;
        memcpy(acc->pdata, &default_lsm330_acc_pdata, sizeof(*acc->pdata));
        dev_info(&client->dev, "using default platform_data\n");
    } else {
        memcpy(acc->pdata, client->dev.platform_data, sizeof(*acc->pdata));
    }

    if (client->dev.of_node) {
        err = lsm330_parse_dt(&client->dev, acc->pdata);
        if (err) {
            dev_err(&client->dev, "Failed to parse device tree\n");
            err = -EINVAL;
            goto exit_kfree_pdata;
        }
    }

    err = lsm330_acc_validate_pdata(acc);
    if (err < 0) {
        dev_err(&client->dev, "failed to validate platform data\n");
        goto exit_kfree_pdata;
    }

    if (acc->pdata->init) {
        err = acc->pdata->init();
        if (err < 0) {
            dev_err(&client->dev, "init failed: %d\n", err);
            goto err_pdata_init;
        }
    }

    err = gsensor_pinctrl_init(client,acc);
    if (!err && acc->gsensor_pinctrl) {
        err = gsensor_pinctrl_select(true,acc);
        if (err < 0)
            printk(KERN_INFO "[G-Sensor] - pinctrl_select fail\n");
    }

    /* resume state init config */
    memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));
    lsm330_acc_set_init_register_values(acc);
    //init state program1 and params
    lsm330_acc_set_init_statepr1_param(acc);
    lsm330_acc_set_init_statepr1_inst(acc);
    //init state program2  and params
    lsm330_acc_set_init_statepr2_param(acc);
    lsm330_acc_set_init_statepr2_inst(acc);

    err = lsm330_acc_device_power_on(acc);
    if (err < 0) {
        dev_err(&client->dev, "power on failed: %d\n", err);
        goto err_pdata_init;
    }

    atomic_set(&acc->enabled, 1);

    err = gpio_request(acc->pdata->gpio_int1, "gsensor-int1");
    if (err){
        printk("[G-Sensor] - gpio_request interrupt1 failed!\n");
    }
    err = gpio_direction_input(acc->pdata->gpio_int1);
    if (err){
        printk("[G-Sensor] - gpio_tlmm_config interrupt1 failed!\n");
    }
    err = gpio_request(acc->pdata->gpio_int2, "gsensor-int2");
    if (err){
        printk("[G-Sensor] - gpio_request interrupt2 failed!\n");
    }
    err = gpio_direction_input(acc->pdata->gpio_int2);
    if (err){
        printk("[G-Sensor] - gpio_tlmm_config interrupt2 failed!\n");
    }
    if(acc->pdata->gpio_int1 >= 0){
        acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
        pr_info("%s: %s has set irq1 to irq: %d mapped on gpio:%d\n",LSM330_ACC_DEV_NAME, __func__, acc->irq1, acc->pdata->gpio_int1);
    }
    if(acc->pdata->gpio_int2 >= 0){
        acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
        pr_info("%s: %s has set irq2 to irq: %d mapped on gpio:%d\n",LSM330_ACC_DEV_NAME, __func__, acc->irq2,acc->pdata->gpio_int2);
    }
    err = lsm330_acc_update_fs_range(acc, acc->pdata->fs_range);
    if (err < 0) {
        dev_err(&client->dev, "update_fs_range failed\n");
        goto  err_power_off;
    }

    err = lsm330_acc_update_odr(acc, acc->pdata->poll_interval);
    if (err < 0) {
        dev_err(&client->dev, "update_odr failed\n");
        goto  err_power_off;
    }

    err = lsm330_acc_input_init(acc);
    if (err < 0) {
        dev_err(&client->dev, "input init failed\n");
        goto err_power_off;
    }


    err = create_sysfs_interfaces(acc);
    if (err < 0) {
        dev_err(&client->dev,"device LSM330_ACC_DEV_NAME sysfs register failed\n");
        goto err_input_cleanup;
    }
    acc->cdev = lis3dsh_acc_cdev;
    acc->cdev.sensors_enable = lis3dsh_acc_enable_set;
    acc->cdev.sensors_poll_delay = lis3dsh_acc_poll_delay_set;
    acc->cdev.delay_msec = acc->pdata->poll_interval;
    acc->cdev.sensors_set_latency = lis3dsh_acc_poll_delay_set;
    err = sensors_classdev_register(&acc->input_dev->dev, &acc->cdev);
    if (err) {
        dev_err(&client->dev,"class device create failed: %d\n", err);
        goto err_remove_sysfs_int;
    }

#ifdef CUSTOM_SYSFS_PATH
    dev_set_drvdata(acc->acc_dev, acc);
#endif
    lsm330_acc_device_power_off(acc);

    /* As default, do not report information */
    atomic_set(&acc->enabled, 0);

    if(acc->pdata->gpio_int1 >= 0){
        INIT_WORK(&acc->irq1_work, lsm330_acc_irq1_work_func);
        acc->irq1_work_queue = create_singlethread_workqueue("lsm330_acc_wq1");
        if (!acc->irq1_work_queue) {
            err = -ENOMEM;
            dev_err(&client->dev,"cannot create work queue1: %d\n", err);
            goto err_remove_sysfs_int;
        }
        err = request_irq(acc->irq1, lsm330_acc_isr1,IRQF_TRIGGER_RISING, "lsm330_acc_irq1", acc);
        if (err < 0) {
            dev_err(&client->dev, "request irq1 failed: %d\n", err);
            goto err_destoyworkqueue1;
        }
//        enable_irq_wake(acc->irq1);
    }

    if(acc->pdata->gpio_int2 >= 0){
        INIT_WORK(&acc->irq2_work, lsm330_acc_irq2_work_func);
        acc->irq2_work_queue = create_singlethread_workqueue("lsm330_acc_wq2");
        if (!acc->irq2_work_queue) {
            err = -ENOMEM;
            dev_err(&client->dev,"cannot create work queue2: %d\n", err);
            goto err_free_irq1;
        }
        err = request_irq(acc->irq2, lsm330_acc_isr2,IRQF_TRIGGER_RISING, "lsm330_acc_irq2", acc);
        if (err < 0) {
            dev_err(&client->dev, "request irq2 failed: %d\n", err);
            goto err_destoyworkqueue2;
        }
//        enable_irq_wake(acc->irq2);
    }

    INIT_WORK(&acc->input_work_acc, poll_function_work_acc);

    mutex_unlock(&acc->lock);
    printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    dev_info(&client->dev, "%s: probed\n", LSM330_ACC_DEV_NAME);

    return 0;

err_destoyworkqueue2:
    if(acc->pdata->gpio_int2 >= 0)
        destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
    free_irq(acc->irq1, acc);
err_destoyworkqueue1:
    if(acc->pdata->gpio_int1 >= 0)
        destroy_workqueue(acc->irq1_work_queue);
err_remove_sysfs_int:
    remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
    lsm330_acc_input_cleanup(acc);
err_power_off:
    lsm330_acc_device_power_off(acc);
err_pdata_init:
    if (acc->pdata->exit)
        acc->pdata->exit();
exit_kfree_pdata:
    kfree(acc->pdata);
err_mutexunlock:
    mutex_unlock(&acc->lock);
    printk(KERN_INFO "[G-sensor]%s unlock\n", __func__);
    if(!lsm330_workqueue) {
        flush_workqueue(lsm330_workqueue);
        destroy_workqueue(lsm330_workqueue);
    }
		if(!work_help){
flush_workqueue(work_help);
destroy_workqueue(work_help);
    }
//err_freedata:
    kfree(acc);
exit_check_functionality_failed:
    pr_err("%s: Driver Init failed\n", LSM330_ACC_DEV_NAME);
    return err;
}

static int lsm330_acc_remove(struct i2c_client *client)
{
    struct lsm330_acc_data *acc = i2c_get_clientdata(client);
    int retval;

    if(acc->pdata->gpio_int1 >= 0){
        free_irq(acc->irq1, acc);
        gpio_free(acc->pdata->gpio_int1);
        destroy_workqueue(acc->irq1_work_queue);
    }

    if(acc->pdata->gpio_int2 >= 0){
        free_irq(acc->irq2, acc);
        gpio_free(acc->pdata->gpio_int2);
        destroy_workqueue(acc->irq2_work_queue);
    }

    if (acc->gsensor_pinctrl) {
        retval = gsensor_pinctrl_select(false,acc);
        if (retval < 0)
            pr_err("Cannot get idle pinctrl state\n");
    }

    lsm330_acc_device_power_off(acc);
    lsm330_acc_input_cleanup(acc);
    remove_sysfs_interfaces(&client->dev);

    if (acc->pdata->exit)
        acc->pdata->exit();

    if(!lsm330_workqueue) {
        flush_workqueue(lsm330_workqueue);
        destroy_workqueue(lsm330_workqueue);
    }
    if(!work_help){
        flush_workqueue(work_help);
        destroy_workqueue(work_help);
    }
								    
    kfree(acc->pdata);
    kfree(acc);

    return 0;
}

#ifdef CONFIG_PM
static int lsm330_acc_resume(struct i2c_client *client)
{
    struct lsm330_acc_data *acc = i2c_get_clientdata(client);
    if (acc->on_before_suspend)
        return lsm330_acc_enable(acc);
    printk(KERN_INFO "[G-sensor]resume\n");
    return 0;
}

static int lsm330_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct lsm330_acc_data *acc = i2c_get_clientdata(client);
    acc->on_before_suspend = atomic_read(&acc->enabled);
    return lsm330_acc_disable(acc);
//    printk(KERN_INFO "[G-sensor]suspend\n");
//    return 0;
}
#else /* CONFIG_PM */
#define lsm330_acc_suspend	NULL
#define lsm330_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm330_acc_id[]= 
    { { LSM330_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, lsm330_acc_id);

static const struct of_device_id gsensor_device_of_match[]  = {
    { .compatible = "fih,gsensor", },
    {},
};

static struct i2c_driver lsm330_acc_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = LSM330_ACC_DEV_NAME,
        .of_match_table = gsensor_device_of_match,
     },
    .probe = lsm330_acc_probe,
    .remove = lsm330_acc_remove,
    .suspend = lsm330_acc_suspend,
    .resume = lsm330_acc_resume,
    .id_table = lsm330_acc_id,
};

static int __init lsm330_acc_init(void)
{
    pr_info("%s accelerometer driver: init\n", LSM330_ACC_DEV_NAME);
    return i2c_add_driver(&lsm330_acc_driver);
}

static void __exit lsm330_acc_exit(void)
{
    pr_info("%s accelerometer driver exit\n", LSM330_ACC_DEV_NAME);
    i2c_del_driver(&lsm330_acc_driver);
    return;
}

module_init(lsm330_acc_init);
module_exit(lsm330_acc_exit);

MODULE_DESCRIPTION("lsm330 accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");

