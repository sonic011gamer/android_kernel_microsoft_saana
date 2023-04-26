/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name		: l3gd20_gyr_sysfs.c
* Authors		: MEMS Motion Sensors Products Div- Application Team
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Denis Ciocca (denis.ciocca@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.2.1 sysfs
* Date			: 2012/Jul/10
* Description		: L3GD20 digital output gyroscope sensor API
*
********************************************************************************
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
********************************************************************************
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS	  | DESCRIPTION
* 1.0		| 2010/May/02	| Carmine Iascone | First Release
* 1.1.3		| 2011/Jun/24	| Matteo Dameno	  | Corrects ODR Bug
* 1.1.4		| 2011/Sep/02	| Matteo Dameno	  | SMB Bus Mng,
*		|		|		  | forces BDU setting
* 1.1.5		| 2011/Sep/24	| Matteo Dameno	  | Introduces FIFO Feat.
* 1.1.5.2	| 2011/Nov/11	| Matteo Dameno	  | enable gpio_int to be
*		|		|		  | passed as parameter at
*		|		|		  | module loading time;
*		|		|		  | corrects polling
*		|		|		  | bug at end of probing;
* 1.1.5.3	| 2011/Dec/20	| Matteo Dameno	  | corrects error in
*		|		|		  | I2C SADROOT; Modifies
*		|		|		  | resume suspend func.
* 1.1.5.4	| 2012/Jan/09	| Matteo Dameno	  | moved under input/misc;
* 1.1.5.5	| 2012/Mar/30	| Matteo Dameno	  | moved watermark, use_smbus,
*		|		|		  | fifomode @ struct foo_status
*		|		|		  | sysfs range input format
*		|		|		  | changed to decimal
* 1.2		| 2012/Jul/10	| Denis Ciocca	  | input_poll_dev removal
* 1.2.1		| 2012/Jul/10	| Denis Ciocca	  | added high resolution timers
*******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include "l3gd20.h"
#include <linux/sensors.h>
#include <linux/delay.h>
/* Maximum polled-device-reported rot speed value value in dps */
#define FS_MAX            32768
#define MS_TO_NS(x)       (x*1000000L)

/* l3gd20 gyroscope registers */
#define WHO_AM_I          (0x0F)

#define SENSITIVITY_250   8750    /* udps/LSB */
#define SENSITIVITY_500   17500   /* udps/LSB */
#define SENSITIVITY_2000  70000   /* udps/LSB */

#define CTRL_REG1         (0x20)  /* CTRL REG1 */
#define CTRL_REG2         (0x21)  /* CTRL REG2 */
#define CTRL_REG3         (0x22)  /* CTRL_REG3 */
#define CTRL_REG4         (0x23)  /* CTRL_REG4 */
#define CTRL_REG5         (0x24)  /* CTRL_REG5 */
#define	REFERENCE         (0x25)  /* REFERENCE REG */
#define	FIFO_CTRL_REG     (0x2E)  /* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG      (0x2F)  /* FIFO SOURCE REGISTER */
#define	OUT_X_L           (0x28)  /* 1st AXIS OUT REG of 6 */
#define AXISDATA_REG      OUT_X_L

/* CTRL_REG1 */
#define ALL_ZEROES        (0x00)
#define PM_OFF            (0x00)
#define PM_NORMAL         (0x08)
#define ENABLE_ALL_AXES   (0x07)
#define ENABLE_NO_AXES    (0x00)
#define BW00              (0x00)
#define BW01              (0x10)
#define BW10              (0x20)
#define BW11              (0x30)
#define ODR095            (0x00)  /* ODR =  95Hz */
#define ODR190            (0x40)  /* ODR = 190Hz */
#define ODR380            (0x80)  /* ODR = 380Hz */
#define ODR760            (0xC0)  /* ODR = 760Hz */

/* CTRL_REG3 bits */
#define	I2_DRDY           (0x08)
#define	I2_WTM            (0x04)
#define	I2_OVRUN          (0x02)
#define	I2_EMPTY          (0x01)
#define	I2_NONE           (0x00)
#define	I2_MASK           (0x0F)

/* CTRL_REG4 bits */
#define	FS_MASK           (0x30)
#define	BDU_ENABLE        (0x80)

/* CTRL_REG5 bits */
#define	FIFO_ENABLE       (0x40)
#define HPF_ENALBE        (0x11)

/* FIFO_CTRL_REG bits */
#define	FIFO_MODE_MASK        (0xE0)
#define	FIFO_MODE_BYPASS      (0x00)
#define	FIFO_MODE_FIFO        (0x20)
#define	FIFO_MODE_STREAM      (0x40)
#define	FIFO_MODE_STR2FIFO    (0x60)
#define	FIFO_MODE_BYPASS2STR  (0x80)
#define	FIFO_WATERMARK_MASK   (0x1F)

#define FIFO_STORED_DATA_MASK (0x1F)

#define I2C_AUTO_INCREMENT    (0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1         0
#define	RES_CTRL_REG2         1
#define	RES_CTRL_REG3         2
#define	RES_CTRL_REG4         3
#define	RES_CTRL_REG5         4
#define	RES_FIFO_CTRL_REG     5
#define	RESUME_ENTRIES        6
/*#define DEBUG 1*/
/** Registers Contents */
#define WHOAMI_L3GD20_GYR     (0xD4) /* Expected content for WAI register*/
#define CAL_SAMPLE_NUM         100
#define ZERO_RATE_LEVEL        357 // 25/0.07 =357
#define ZERO_VARIETY_RANGE     100

static int int1_gpio = L3GD20_GYR_DEFAULT_INT1_GPIO;
static int int2_gpio = L3GD20_GYR_DEFAULT_INT2_GPIO;
struct pinctrl *gyro_pinctrl;
struct pinctrl_state *gyro_gpio_state_active;
struct pinctrl_state *gyro_gpio_state_suspend;
/* module_param(int1_gpio, int, S_IRUGO); */
module_param(int2_gpio, int, S_IRUGO);

struct sensor_regulator {
    struct regulator *vreg;
    const char *name;
    u32  min_uV;
    u32  max_uV;
};

struct sensor_regulator l3gd20_vreg[] = {
    {NULL, "vcc", 2100000, 3600000},
    {NULL, "vcc_i2c", 1800000, 1800000},
};

/*
 * L3GD20 gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * s32
 */

struct l3gd20_gyr_triple {
    s16 x, /* x-axis angular rate data. */
        y, /* y-axis angluar rate data. */
        z; /* z-axis angular rate data. */
};


struct l3gd20_gyr_triple_large {
    int32_t x, /* x-axis angular rate data. */
            y, /* y-axis angluar rate data. */
            z; /* z-axis angular rate data. */
};

struct output_rate {
    int poll_rate_ms;
    u8  mask;
};

static const struct output_rate odr_table[] = {
    { 2, ODR760|BW10},
    { 3, ODR380|BW01},
    { 6, ODR190|BW00},
    {11, ODR095|BW00},
};

static struct l3gd20_gyr_platform_data default_l3gd20_gyr_pdata = {
    .fs_range = L3GD20_GYR_FS_2000DPS,
    .axis_map_x = 0,
    .axis_map_y = 1,
    .axis_map_z = 2,
    .negate_x = 1,
    .negate_y = 1,
    .negate_z = 0,
    .poll_interval = 100,
    .min_interval = L3GD20_GYR_MIN_POLL_PERIOD_MS, /* 2ms */
    .gpio_int1 = L3GD20_GYR_DEFAULT_INT1_GPIO,
    .gpio_int2 = L3GD20_GYR_DEFAULT_INT2_GPIO,	/* int for fifo */
};

struct workqueue_struct *l3gd20_gyr_workqueue = 0;
struct workqueue_struct *l3gd20_gyr_workqueue_1 = 0;

struct l3gd20_gyr_status {
    struct i2c_client *client;
    struct l3gd20_gyr_platform_data *pdata;
    struct mutex lock;
    struct input_dev *input_dev;
    int hw_initialized;
    atomic_t enabled;
    int use_smbus;
    u8 reg_addr;
    u8 resume_state[RESUME_ENTRIES];
    u32 sensitivity;
    /* interrupt related */
    int irq2;
    struct work_struct irq2_work;
    struct workqueue_struct *irq2_work_queue;
    bool polling_enabled;
    /* fifo related */
    u8 watermark;
    u8 fifomode;
    struct hrtimer hr_timer;
    ktime_t ktime;
    struct work_struct polling_task;
    struct sensors_classdev cdev;
    bool  high_q;
    bool  cal_zero;
    unsigned int  zero_offset_range;
    unsigned int  zero_variety;
    bool zero_static;
    struct l3gd20_gyr_triple_large sum;
    struct l3gd20_gyr_triple_large sum_small;
    struct l3gd20_gyr_triple cal;
    int Valid_Sample;
};

static struct sensors_classdev sensors_cdev = {
    .name = "l3gd20-gyro",
    .vendor = "STMicro",
    .version = 1,
    .handle = SENSORS_GYROSCOPE_HANDLE,
    .type = SENSOR_TYPE_GYROSCOPE,
    .max_range = "35.0",
    .resolution = "0.06",
    .sensor_power = "3.6",
    .min_delay = 10000,
    .max_delay = 200000,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .delay_msec = 100,
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
    .sensors_set_latency = NULL,
    .sensors_flush = NULL,
};

static int gyro_pinctrl_select(bool on);
 
static int l3gd20_config_regulator(struct i2c_client *client, bool on)
{
    int rc = 0, i;
    int num_reg = sizeof(l3gd20_vreg) / sizeof(struct sensor_regulator);

    if (on) {
        for (i = 0; i < num_reg; i++) {
            l3gd20_vreg[i].vreg = regulator_get(&client->dev,l3gd20_vreg[i].name);
            if (IS_ERR(l3gd20_vreg[i].vreg)) {
                rc = PTR_ERR(l3gd20_vreg[i].vreg);
                pr_err("%s:regulator get failed rc=%d\n",__func__, rc);
                l3gd20_vreg[i].vreg = NULL;
                goto error_vdd;
            }
            if (regulator_count_voltages(l3gd20_vreg[i].vreg) > 0) {
                rc = regulator_set_voltage(l3gd20_vreg[i].vreg,l3gd20_vreg[i].min_uV, l3gd20_vreg[i].max_uV);
                if (rc) {
                    pr_err("%s:set_voltage failed rc=%d\n",__func__, rc);
                    regulator_put(l3gd20_vreg[i].vreg);
                    l3gd20_vreg[i].vreg = NULL;
                    goto error_vdd;
                }
            }
            rc = regulator_enable(l3gd20_vreg[i].vreg);
            if (rc) {
                pr_err("%s: regulator_enable failed rc =%d\n",__func__,rc);
                if (regulator_count_voltages(l3gd20_vreg[i].vreg) > 0) {
                    regulator_set_voltage(l3gd20_vreg[i].vreg,0, l3gd20_vreg[i].max_uV);
                }
                regulator_put(l3gd20_vreg[i].vreg);
                l3gd20_vreg[i].vreg = NULL;
                goto error_vdd;
            }
        }
        return rc;
    } else {
        i = num_reg;
    }
error_vdd:
    while (--i >= 0) {
        if (!IS_ERR_OR_NULL(l3gd20_vreg[i].vreg)) {
            if (regulator_count_voltages(l3gd20_vreg[i].vreg) > 0) {
                regulator_set_voltage(l3gd20_vreg[i].vreg, 0,l3gd20_vreg[i].max_uV);
            }
            regulator_disable(l3gd20_vreg[i].vreg);
            regulator_put(l3gd20_vreg[i].vreg);
            l3gd20_vreg[i].vreg = NULL;
        }
    }
    return rc;
}

#define I2C_RETRY_DELAY         5           /* Waiting for signals [ms] */
#define I2C_RETRIES             5           /* Number of retries */

static int l3gd20_gyr_i2c_read(struct l3gd20_gyr_status *stat, u8 *buf,int len)
{
    int err;
    int tries = 0;
    struct i2c_msg msgs[] = {
        {
            .addr = stat->client->addr,
            .flags = stat->client->flags & I2C_M_TEN,
            .len = 1,
            .buf = buf,
        },
        {
            .addr = stat->client->addr,
            .flags = (stat->client->flags & I2C_M_TEN) | I2C_M_RD,
            .len = len,
            .buf = buf,
        },
    };

    if (len > 1)
        buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

    do {
        err = i2c_transfer(stat->client->adapter, msgs, 2);
        if (err != 2)
            msleep_interruptible(I2C_RETRY_DELAY);
    } while ((err != 2) && (++tries < I2C_RETRIES));
    if (err != 2) {
        pr_err("[Gyro]read transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}

static int l3gd20_gyr_i2c_write(struct l3gd20_gyr_status *stat,u8 *buf,int len)
{
    int err;
    int tries = 0;
    struct i2c_msg msgs[] = {
        {
            .addr = stat->client->addr,
            .flags = stat->client->flags & I2C_M_TEN,
            .len = len + 1,
            .buf = buf,
        },
    };

    if (len > 1)
        buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

    do {
        err = i2c_transfer(stat->client->adapter, msgs, 1);
        if (err != 1)
            msleep_interruptible(I2C_RETRY_DELAY);
    } while ((err != 1) && (++tries < I2C_RETRIES));

    if (err != 1) {
        pr_err("[Gyro]write transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }
    return err;
}

static int l3gd20_gyr_register_write(struct l3gd20_gyr_status *stat,u8 *buf, u8 reg_address, u8 new_value)
{
    int err;
    /* Sets configuration register at reg_address
     *  NOTE: this is a straight overwrite  */
    buf[0] = reg_address;
    buf[1] = new_value;
    err = l3gd20_gyr_i2c_write(stat, buf, 1);
    if (err < 0)
        return err;

    return err;
}

static int l3gd20_gyr_register_read(struct l3gd20_gyr_status *stat,u8 *buf, u8 reg_address)
{
    int err = -1;
    buf[0] = (reg_address);
    err = l3gd20_gyr_i2c_read(stat, buf, 1);
    return err;
}

static int l3gd20_gyr_register_update(struct l3gd20_gyr_status *stat,u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
    int err = -1;
    u8 init_val;
    u8 updated_val;
    err = l3gd20_gyr_register_read(stat, buf, reg_address);
    if (!(err < 0)) {
        init_val = buf[0];
        updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
        err = l3gd20_gyr_register_write(stat, buf, reg_address,updated_val);
    }
    return err;
}


static int l3gd20_gyr_update_watermark(struct l3gd20_gyr_status *stat,u8 watermark)
{
    int res = 0;
    u8 buf[2];
    u8 new_value;

    mutex_lock(&stat->lock);
    new_value = (watermark % 0x20);
    res = l3gd20_gyr_register_update(stat, buf, FIFO_CTRL_REG,FIFO_WATERMARK_MASK, new_value);
    if (res < 0) {
        dev_err(&stat->client->dev, "failed to update watermark\n");
        return res;
    }
    dev_dbg(&stat->client->dev, "%s new_value:0x%02x,watermark:0x%02x\n",__func__, new_value, watermark);

    stat->resume_state[RES_FIFO_CTRL_REG] = ((FIFO_WATERMARK_MASK & new_value) | (~FIFO_WATERMARK_MASK & stat->resume_state[RES_FIFO_CTRL_REG]));
    stat->watermark = new_value;
    mutex_unlock(&stat->lock);
    return res;
}

static int l3gd20_gyr_update_fifomode(struct l3gd20_gyr_status *stat,u8 fifomode)
{
    int res;
    u8 buf[2];
    u8 new_value;

    new_value = fifomode;
    res = l3gd20_gyr_register_update(stat, buf, FIFO_CTRL_REG,FIFO_MODE_MASK, new_value);
    if (res < 0) {
        dev_err(&stat->client->dev, "failed to update fifoMode\n");
        return res;
    }
    //dev_dbg(&stat->client->dev, "new_value:0x%02x,prev fifomode:0x%02x\n",__func__, new_value, stat->fifomode);
    stat->resume_state[RES_FIFO_CTRL_REG] =	((FIFO_MODE_MASK & new_value) | (~FIFO_MODE_MASK & stat->resume_state[RES_FIFO_CTRL_REG]));
    stat->fifomode = new_value;

    return res;
}

static int l3gd20_gyr_fifo_reset(struct l3gd20_gyr_status *stat)
{
    u8 oldmode;
    int res;

    oldmode = stat->fifomode;
    res = l3gd20_gyr_update_fifomode(stat, FIFO_MODE_BYPASS);
    if (res < 0)
        return res;
    res = l3gd20_gyr_update_fifomode(stat, oldmode);
    if (res >= 0)
        dev_dbg(&stat->client->dev, "%s fifo reset to: 0x%02x\n",__func__, oldmode);

    return res;
}

static int l3gd20_gyr_fifo_hwenable(struct l3gd20_gyr_status *stat,u8 enable)
{
    int res;
    u8 buf[2];
    u8 set = 0x00;
    if (enable)
        set = FIFO_ENABLE;

    res = l3gd20_gyr_register_update(stat, buf, CTRL_REG5,FIFO_ENABLE, set);
    if (res < 0) {
        dev_err(&stat->client->dev, "fifo_hw switch to:0x%02x failed\n",set);
        return res;
    }
    stat->resume_state[RES_CTRL_REG5] =	((FIFO_ENABLE & set) | (~FIFO_ENABLE & stat->resume_state[RES_CTRL_REG5]));
    dev_dbg(&stat->client->dev, "%s set to:0x%02x\n", __func__, set);
    return res;
}

static int l3gd20_gyr_manage_int2settings(struct l3gd20_gyr_status *stat,u8 fifomode)
{
    int res;
    u8 buf[2];
    bool enable_fifo_hw;
    bool recognized_mode = false;
    u8 int2bits = I2_NONE;
/*
if (stat->polling_enabled) {
fifomode = FIFO_MODE_BYPASS;
dbg_warn(&stat->client->dev, "in polling mode, fifo mode forced to BYPASS mode\n");
}
*/
    switch (fifomode) {
        case FIFO_MODE_FIFO:
            recognized_mode = true;
            if (stat->polling_enabled) {
                int2bits = I2_NONE;
                enable_fifo_hw = false;
            } else {
                int2bits = (I2_WTM | I2_OVRUN);
                enable_fifo_hw = true;
            }
            res = l3gd20_gyr_register_update(stat, buf, CTRL_REG3,I2_MASK, int2bits);
            if (res < 0) {
                dev_err(&stat->client->dev, "%s : failed to update CTRL_REG3:0x%02x\n",__func__, fifomode);
                goto err_mutex_unlock;
            }
            stat->resume_state[RES_CTRL_REG3] = ((I2_MASK & int2bits) | (~(I2_MASK) & stat->resume_state[RES_CTRL_REG3]));
            /* enable_fifo_hw = true; */
        break;

        case FIFO_MODE_BYPASS:
            recognized_mode = true;
            if (stat->polling_enabled)
                int2bits = I2_NONE;
            else
                int2bits = I2_DRDY;
            res = l3gd20_gyr_register_update(stat, buf, CTRL_REG3,I2_MASK, int2bits);
            if (res < 0) {
                dev_err(&stat->client->dev, "%s : failed to update to CTRL_REG3:0x%02x\n",__func__, fifomode);
                goto err_mutex_unlock;
            }
            stat->resume_state[RES_CTRL_REG3] =	((I2_MASK & int2bits) |	(~I2_MASK & stat->resume_state[RES_CTRL_REG3]));
            enable_fifo_hw = false;
        break;

        default:
            recognized_mode = false;
            res = l3gd20_gyr_register_update(stat, buf, CTRL_REG3, I2_MASK, I2_NONE);
            if (res < 0) {
                dev_err(&stat->client->dev, "%s : failed to update CTRL_REG3:0x%02x\n",	__func__, fifomode);
                goto err_mutex_unlock;
            }
            enable_fifo_hw = false;
            stat->resume_state[RES_CTRL_REG3] = ((I2_MASK & 0x00) |(~I2_MASK & stat->resume_state[RES_CTRL_REG3]));
        break;
    }
    if (recognized_mode) {
        res = l3gd20_gyr_update_fifomode(stat, fifomode);
        if (res < 0) {
            dev_err(&stat->client->dev, "%s : failed to set fifoMode\n", __func__);
            goto err_mutex_unlock;
        }
    }
    res = l3gd20_gyr_fifo_hwenable(stat, enable_fifo_hw);

err_mutex_unlock:
    return res;
}

static int l3gd20_gyr_update_fs_range(struct l3gd20_gyr_status *stat,u8 new_fs)
{
    int res ;
    u8 buf[2];
    u32 sensitivity;

    switch(new_fs) {
        case L3GD20_GYR_FS_250DPS:
            sensitivity = SENSITIVITY_250;
        break;
        case L3GD20_GYR_FS_500DPS:
            sensitivity = SENSITIVITY_500;
        break;
        case L3GD20_GYR_FS_2000DPS:
            sensitivity = SENSITIVITY_2000;
        break;
        default:
            dev_err(&stat->client->dev, "invalid g range requested: %u\n", new_fs);
            return -EINVAL;
    }

    buf[0] = CTRL_REG4;
    res = l3gd20_gyr_register_update(stat, buf, CTRL_REG4,FS_MASK, new_fs);
    if (res < 0) {
        dev_err(&stat->client->dev, "%s : failed to update fs:0x%02x\n",__func__, new_fs);
        return res;
    }
    stat->resume_state[RES_CTRL_REG4] =	((FS_MASK & new_fs) |	(~FS_MASK & stat->resume_state[RES_CTRL_REG4]));
    stat->sensitivity = sensitivity;
    return res;
}

static int l3gd20_gyr_update_odr(struct l3gd20_gyr_status *stat,unsigned int poll_interval_ms)
{
    int err = -1;
    int i;
    u8 config[2]={0};

    for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
        if ((odr_table[i].poll_rate_ms <= poll_interval_ms) || (i == 0))
            break;
    }

    config[1] = odr_table[i].mask;
    config[1] |= (ENABLE_ALL_AXES + PM_NORMAL);

    /* If device is currently enabled, we need to write new
     *  configuration out to it */
    if (atomic_read(&stat->enabled)) {
        config[0] = CTRL_REG1;
        err = l3gd20_gyr_i2c_write(stat, config, 1);
        if (err < 0)
            return err;
        stat->resume_state[RES_CTRL_REG1] = config[1];
        stat->ktime = ktime_set(0, MS_TO_NS(poll_interval_ms));
        err = l3gd20_gyr_i2c_write(stat, config, 1);
    }
    return err;
}

static int check_data(struct l3gd20_gyr_status *stat){
    struct l3gd20_gyr_triple_large result;
    int32_t variety = (int32_t)stat->zero_variety;
    int32_t sample_s = CAL_SAMPLE_NUM * CAL_SAMPLE_NUM;
    int ret = 0;
    result.x = (stat->sum_small.x / CAL_SAMPLE_NUM) - ((stat->sum.x * stat->sum.x) / sample_s);
    result.y = (stat->sum_small.y / CAL_SAMPLE_NUM) - ((stat->sum.y * stat->sum.y) / sample_s);
    result.z = (stat->sum_small.z / CAL_SAMPLE_NUM) - ((stat->sum.z * stat->sum.z) / sample_s);
    pr_debug("[CHECK] stat->sum_small !! x=%d, y=%d z=%d\n",stat->sum_small.x,stat->sum_small.y,stat->sum_small.x);

    if((abs(result.x) < variety)&&(abs(result.y) < variety)&&(abs(result.z) < variety))
        ret = 1;
    return ret;
}
static int do_zero_cali(struct l3gd20_gyr_status *stat,struct l3gd20_gyr_triple *data)
{
    s16 range = (s16) stat->zero_offset_range;
    pr_debug("[do_zero_cali] range=%d Valid_Sample=%d , stat->cal_zero =%d \n", range, stat->Valid_Sample,stat->cal_zero);
    pr_debug("[do_zero_cali] Calibrating !! data->x=%d\n", data->x);
    pr_debug("[do_zero_cali] Calibrating !! data->y=%d\n", data->y);
    pr_debug("[do_zero_cali] Calibrating !! data->z=%d\n", data->z);
    if((data->x < range)&&(data->x > -range)&&
       (data->y < range)&&(data->y > -range)&&
       (data->z < range)&&(data->z > -range)){
            stat->Valid_Sample++;
            stat->sum.x += data->x;
            stat->sum.y += data->y;
            stat->sum.z += data->z;
            stat->sum_small.x += data->x*data->x;
            stat->sum_small.y += data->y*data->y;
            stat->sum_small.z += data->z*data->z;
            if(stat->Valid_Sample ==  CAL_SAMPLE_NUM){
                if(check_data(stat)){
                    stat->cal_zero = false;
                    stat->zero_static = true;
                    stat->cal.x = stat->sum.x / CAL_SAMPLE_NUM;
                    stat->cal.y = stat->sum.y / CAL_SAMPLE_NUM;
                    stat->cal.z = stat->sum.z / CAL_SAMPLE_NUM;
                    pr_debug("[do_zero_cali] Calibration Success !! sum.x=%d,sum.y=%d,sum.z=%d\n",stat->sum.x,stat->sum.y,stat->sum.z);
                    pr_debug("[do_zero_cali] Calibration Success !! cal.x=%d,cal.y=%d,cal.z=%d\n",stat->cal.x,stat->cal.y,stat->cal.z);
                }else {
                    stat->sum.x = 0;
                    stat->sum.y = 0;
                    stat->sum.z = 0;
                    stat->sum_small.x = 0;
                    stat->sum_small.y = 0;
                    stat->sum_small.z = 0;
                    stat->Valid_Sample = 0;
                }
            }
    }else {
        //all paramters should be reset here for else case
        stat->Valid_Sample = 0;
        stat->sum_small.x = 0;
        stat->sum_small.y = 0;
        stat->sum_small.z = 0;
        stat->sum.x = 0;
        stat->sum.y = 0;
        stat->sum.z = 0;
    }
    return 0;
}

/* gyroscope data readout */
static int l3gd20_gyr_get_data(struct l3gd20_gyr_status *stat,struct l3gd20_gyr_triple *data)
{
    int err;
    unsigned char gyro_out[6];
    /* y,p,r hardware data */
    s16 hw_d[3] = { 0 };
    gyro_out[0] = (AXISDATA_REG);
    err = l3gd20_gyr_i2c_read(stat, gyro_out, 6);

    if (err < 0)
        return err;

    hw_d[0] = (s16)((gyro_out[1]) << 8) | gyro_out[0];
    hw_d[1] = (s16)((gyro_out[3]) << 8) | gyro_out[2];
    hw_d[2] = (s16)((gyro_out[5]) << 8) | gyro_out[4];

    data->x = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x]):(hw_d[stat->pdata->axis_map_x]));
    data->y = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y]):(hw_d[stat->pdata->axis_map_y]));
    data->z = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z]):(hw_d[stat->pdata->axis_map_z]));
    if(stat->cal_zero)
        do_zero_cali(stat,data);
   
#ifdef DEBUG
    /* dev_info(&stat->client->dev, "gyro_out: x = %d, y = %d, z = %d\n", data->x, data->y, data->z); */
#endif
    return err;
}

static void l3gd20_gyr_report_values(struct l3gd20_gyr_status *stat, struct l3gd20_gyr_triple *data)
{
    ktime_t timestamp;
    timestamp = ktime_get_boottime();
    input_report_abs(stat->input_dev, ABS_RX, data->x - stat->cal.x);
    input_report_abs(stat->input_dev, ABS_RY, data->y - stat->cal.y);
    input_report_abs(stat->input_dev, ABS_RZ, data->z - stat->cal.z);
    input_event(stat->input_dev,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
    input_event(stat->input_dev,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
    input_sync(stat->input_dev);
}

static int l3gd20_gyr_hw_init(struct l3gd20_gyr_status *stat)
{
    int err;
    u8 buf[6];
    buf[0] = (CTRL_REG1);
    buf[1] = stat->resume_state[RES_CTRL_REG1];
    buf[2] = stat->resume_state[RES_CTRL_REG2];
    buf[3] = stat->resume_state[RES_CTRL_REG3];
    buf[4] = stat->resume_state[RES_CTRL_REG4];
    buf[5] = stat->resume_state[RES_CTRL_REG5];

    err = l3gd20_gyr_i2c_write(stat, buf, 5);
    if (err < 0)
        return err;

    buf[0] = (FIFO_CTRL_REG);
    buf[1] = stat->resume_state[RES_FIFO_CTRL_REG];
    err = l3gd20_gyr_i2c_write(stat, buf, 1);
    if (err < 0)
        return err;
    stat->hw_initialized = 1;
    return err;
}

static void l3gd20_gyr_device_power_off(struct l3gd20_gyr_status *stat)
{
    int err;
    u8 buf[2];

    dev_info(&stat->client->dev, "power off\n");

    buf[0] = (CTRL_REG1);
    buf[1] = (PM_OFF);
    err = l3gd20_gyr_i2c_write(stat, buf, 1);
    if (err < 0)
        dev_err(&stat->client->dev, "soft power off failed\n");

    if (stat->pdata->power_off) {
        /*disable_irq_nosync(acc->irq1);*/
        disable_irq_nosync(stat->irq2);
        stat->pdata->power_off();
        stat->hw_initialized = 0;
    }

    if (stat->hw_initialized) {
        /*if(stat->pdata->gpio_int1 >= 0)*/
        /*disable_irq_nosync(stat->irq1);*/
        if (stat->pdata->gpio_int2 > 0) {
            disable_irq_nosync(stat->irq2);
            dev_info(&stat->client->dev,"power off: irq2 disabled\n");
        }
        stat->hw_initialized = 0;
    }
}
static int l3gd20_gyr_device_power_on(struct l3gd20_gyr_status *stat)
{
    int err;

    if (stat->pdata->power_on) {
        err = stat->pdata->power_on();
        if (err < 0)
            return err;
        if (stat->pdata->gpio_int2 > 0)
            enable_irq(stat->irq2);
    }
    if (!stat->hw_initialized) {
        err = l3gd20_gyr_hw_init(stat);
        if (err < 0) {
            l3gd20_gyr_device_power_off(stat);
            return err;
        }
    }

    if (stat->hw_initialized) {
        /*if (stat->pdata->gpio_int1) {
              enable_irq(stat->irq1);
              dev_info(&stat->client->dev,"power on: irq1 enabled\n");
        }*/
        dev_dbg(&stat->client->dev, "stat->pdata->gpio_int2 = %d\n",stat->pdata->gpio_int2);
        if (stat->pdata->gpio_int2 > 0) {
            enable_irq(stat->irq2);
            dev_info(&stat->client->dev,"power on: irq2 enabled\n");
        }
    }
    return 0;
}

static int l3gd20_gyr_enable(struct l3gd20_gyr_status *stat)
{
    int err;
    if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
        err = l3gd20_gyr_device_power_on(stat);
        if (err < 0) {
            atomic_set(&stat->enabled, 0);
            return err;
        }
        stat->Valid_Sample = 0;
        stat->sum.x = 0;
        stat->sum.y = 0;
        stat->sum.z = 0;
        stat->sum_small.x = 0;
        stat->sum_small.y = 0;
        stat->sum_small.z = 0;
        stat->cal_zero = true;
	if (stat->polling_enabled) {
            l3gd20_gyr_update_odr(stat, stat->pdata->poll_interval);
            hrtimer_start(&(stat->hr_timer), stat->ktime, HRTIMER_MODE_REL);
        }
    }

    return 0;
}

static int l3gd20_gyr_disable(struct l3gd20_gyr_status *stat)
{
    dev_dbg(&stat->client->dev, "%s: stat->enabled = %d\n", __func__,atomic_read(&stat->enabled));

    if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
        l3gd20_gyr_device_power_off(stat);
        hrtimer_cancel(&stat->hr_timer);
//        l3gd20_config_regulator(stat->client, 0);
//        udelay(10);
//        gyro_pinctrl_select(false);
        dev_dbg(&stat->client->dev, "%s: cancel_hrtimer ", __func__);
    }
    return 0;
}

static ssize_t attr_polling_rate_show(struct device *dev,struct device_attribute *attr,char *buf)
{
    int val;
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    mutex_lock(&stat->lock);
    val = stat->pdata->poll_interval;
    mutex_unlock(&stat->lock);
    return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
    int err;
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    unsigned long interval_ms;

    if (strict_strtoul(buf, 10, &interval_ms))
        return -EINVAL;
    if (!interval_ms)
        return -EINVAL;

    mutex_lock(&stat->lock);
    err = l3gd20_gyr_update_odr(stat, interval_ms);
    if(err >= 0)
        stat->pdata->poll_interval = interval_ms;
    mutex_unlock(&stat->lock);
    return size;
}

static ssize_t attr_range_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    int range = 0;
    u8 val;
    mutex_lock(&stat->lock);
    val = stat->pdata->fs_range;

    switch (val) {
        case L3GD20_GYR_FS_250DPS:
            range = 250;
        break;
        case L3GD20_GYR_FS_500DPS:
            range = 500;
        break;
        case L3GD20_GYR_FS_2000DPS:
            range = 2000;
        break;
    }
    mutex_unlock(&stat->lock);
    /* return sprintf(buf, "0x%02x\n", val); */
    return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    unsigned long val;
    u8 range;
    int err;
    if (strict_strtoul(buf, 10, &val))
        return -EINVAL;
    switch (val) {
        case 250:
            range = L3GD20_GYR_FS_250DPS;
        break;
        case 500:
            range = L3GD20_GYR_FS_500DPS;
        break;
        case 2000:
            range = L3GD20_GYR_FS_2000DPS;
        break;
        default:
            dev_err(&stat->client->dev, "invalid range request: %lu,discarded\n", val);
            return -EINVAL;
    }
    mutex_lock(&stat->lock);
    err = l3gd20_gyr_update_fs_range(stat, range);
    if (err >= 0)
        stat->pdata->fs_range = range;
    mutex_unlock(&stat->lock);
    dev_info(&stat->client->dev, "range set to: %lu dps\n", val);
    return size;
}

static ssize_t attr_enable_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    int val = atomic_read(&stat->enabled);
    return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 10, &val))
        return -EINVAL;

    if (val){
        l3gd20_gyr_enable(stat);
        if(val == 2)
            stat->cal_zero = false;
    }else
        l3gd20_gyr_disable(stat);

    return size;
}

static ssize_t attr_polling_mode_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    int val = 0;
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);

    mutex_lock(&stat->lock);
    if (stat->polling_enabled)
        val = 1;
    mutex_unlock(&stat->lock);
    return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_mode_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 10, &val))
        return -EINVAL;

    mutex_lock(&stat->lock);
    if (val) {
        stat->polling_enabled = true;
        l3gd20_gyr_manage_int2settings(stat, stat->fifomode);
        dev_info(dev, "polling mode enabled\n");
        if (atomic_read(&stat->enabled)) {
            hrtimer_start(&(stat->hr_timer), stat->ktime, HRTIMER_MODE_REL);
        }
    } else {
        if (stat->polling_enabled) {
            hrtimer_cancel(&stat->hr_timer);
        }
        stat->polling_enabled = false;
        l3gd20_gyr_manage_int2settings(stat, stat->fifomode);
        dev_info(dev, "polling mode disabled\n");
    }
    mutex_unlock(&stat->lock);
    return size;
}

static ssize_t attr_watermark_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    unsigned long watermark;
    int res;

    if (strict_strtoul(buf, 16, &watermark))
        return -EINVAL;

    res = l3gd20_gyr_update_watermark(stat, watermark);
    if (res < 0)
        return res;

    return size;
}

static ssize_t attr_watermark_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    int val = stat->watermark;
    return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_fifomode_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    unsigned long fifomode;
    int res;

    if (strict_strtoul(buf, 16, &fifomode))
        return -EINVAL;
    /* if (!fifomode) return -EINVAL; */

    dev_dbg(dev, "%s, got value:0x%02x\n", __func__, (u8)fifomode);

    mutex_lock(&stat->lock);
    res = l3gd20_gyr_manage_int2settings(stat, (u8) fifomode);
    mutex_unlock(&stat->lock);

    if (res < 0)
        return res;
    return size;
}

static ssize_t attr_fifomode_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    u8 val = stat->fifomode;
    return sprintf(buf, "0x%02x\n", val);
}

#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
    int rc;
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    u8 x[2];
    unsigned long val;

    if (strict_strtoul(buf, 16, &val))
        return -EINVAL;
    mutex_lock(&stat->lock);
    x[0] = stat->reg_addr;
    mutex_unlock(&stat->lock);
    x[1] = val;
    rc = l3gd20_gyr_i2c_write(stat, x, 1);
    return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,char *buf)
{
    ssize_t ret;
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    int rc;
    u8 data;

    mutex_lock(&stat->lock);
    data = stat->reg_addr;
    mutex_unlock(&stat->lock);
    rc = l3gd20_gyr_i2c_read(stat, &data, 1);
    ret = sprintf(buf, "0x%02x\n", data);
    return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
    unsigned long val;
    if (strict_strtoul(buf, 16, &val))
        return -EINVAL;
    mutex_lock(&stat->lock);
    stat->reg_addr = val;
    mutex_unlock(&stat->lock);
    return size;
}
#endif /* DEBUG */

static ssize_t attr_get_data(struct device *dev, struct device_attribute *attr,char *buf)
{
    ssize_t ret;
    struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);

    struct l3gd20_gyr_triple data_out;

    l3gd20_gyr_get_data(stat, &data_out);
    /*TODO: error need to be managed */
    pr_info( "x = %d y = %d z = %d\n", data_out.x, data_out.y, data_out.z);
    ret = sprintf(buf, "%d %d %d \n",  data_out.x, data_out.y, data_out.z);
    return ret;
}

static ssize_t attr_set_data(struct device *dev, struct device_attribute *attr, const char *buf,size_t count)
{
    struct l3gd20_gyr_status *sensor = dev_get_drvdata(dev);
    unsigned long val;
    if (kstrtoul(buf, 10, &val))
    return -EINVAL;

    pr_info("[****]attr_set_high_q = %lu \n",val);
    if(val)
        sensor->high_q = true;
    else
        sensor->high_q = false;
    return count;
}

static ssize_t attr_get_need_sensitivity_calibration(struct device *dev, struct device_attribute *attr,char *buf)
{
    ssize_t ret=1;
    pr_info( "[attr_get_need_sensitivity_calibration] ret = %d\n",ret );
    return sprintf(buf, "%d\n", ret);
}

static ssize_t attr_set_need_sensitivity_calibration(struct device *dev, struct device_attribute *attr, const char *buf,size_t count)
{
    struct l3gd20_gyr_status *sensor = dev_get_drvdata(dev);
    unsigned long val;
    if (kstrtoul(buf, 10, &val))
        return -EINVAL;
    pr_info("[****]attr_set_cal_zero = %lu \n",val);
    if(val)
        sensor->cal_zero = true;
    else
        sensor->cal_zero = false;
    return count;
}

static ssize_t attr_get_static_calibration(struct device *dev, struct device_attribute *attr,char *buf)
{
    ssize_t ret=0;
    struct l3gd20_gyr_status *sensor = dev_get_drvdata(dev);
    if(sensor->zero_static){
        ret = 1;
        sensor->zero_static = false;
    }
    pr_err( "[attr_get_need_static_calibration] ret = %d\n",ret );
    return sprintf(buf, "%d\n", ret);
}

static ssize_t attr_set_static_calibration(struct device *dev, struct device_attribute *attr, const char *buf,size_t count)
{
    struct l3gd20_gyr_status *sensor = dev_get_drvdata(dev);
    unsigned long val;
    if (kstrtoul(buf, 10, &val))
        return -EINVAL;
    pr_err( "[attr_set_static_calibration] ret = %lu, \n",val);
    if(val == 0){
        sensor->zero_static = false;
    }
    return count;
}

static ssize_t attr_get_zero_offset_range(struct device *dev, struct device_attribute *attr,char *buf)
{
    struct l3gd20_gyr_status *sensor = dev_get_drvdata(dev);
    pr_info( "[attr_get_zero_offset_range] ret = %u\n",sensor->zero_offset_range );

    return sprintf(buf, "%u\n", sensor->zero_offset_range);
}

static ssize_t attr_set_zero_offset_range(struct device *dev, struct device_attribute *attr, const char *buf,size_t count)
{
    struct l3gd20_gyr_status *sensor = dev_get_drvdata(dev);
    unsigned int val;
    if (kstrtouint(buf, 10, &val))
        return -EINVAL;
    pr_info("[****]attr_set_zero_offset_range = %u \n",val);
    sensor->zero_offset_range = val;

    return count;
}

static ssize_t attr_get_zero_variety(struct device *dev, struct device_attribute *attr,char *buf)
{
    struct l3gd20_gyr_status *sensor = dev_get_drvdata(dev);
    pr_info( "[attr_get_zero_variety] ret = %u\n",sensor->zero_variety );
    return sprintf(buf, "%u\n", sensor->zero_variety);
}

static ssize_t attr_set_zero_variety(struct device *dev, struct device_attribute *attr, const char *buf,size_t count)
{
    struct l3gd20_gyr_status *sensor = dev_get_drvdata(dev);
    unsigned int val;
    if (kstrtouint(buf, 10, &val))
        return -EINVAL;
    pr_info("[****]attr_set_zero_variety = %u \n",val);
    sensor->zero_variety = val;
    return count;
}


static struct device_attribute attributes[] = {
    __ATTR(pollrate_ms, 0666, attr_polling_rate_show,attr_polling_rate_store),
    __ATTR(zero_offset_range, 0666, attr_get_zero_offset_range, attr_set_zero_offset_range),
    __ATTR(zero_variety, 0666, attr_get_zero_variety, attr_set_zero_variety),
    __ATTR(range, 0666, attr_range_show, attr_range_store),
    __ATTR(enable_device, 0666, attr_enable_show, attr_enable_store),
    __ATTR(enable_polling, 0666, attr_polling_mode_show,attr_polling_mode_store),
    __ATTR(fifo_samples, 0666, attr_watermark_show, attr_watermark_store),
    __ATTR(fifo_mode, 0666, attr_fifomode_show, attr_fifomode_store),
#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
    __ATTR(value, 0664, attr_get_data,attr_set_data),
    __ATTR(need_sensitivity_calibration, 0664, attr_get_need_sensitivity_calibration,attr_set_need_sensitivity_calibration),
    __ATTR(static_calibration, 0664, attr_get_static_calibration,attr_set_static_calibration),
};

static int create_sysfs_interfaces(struct device *dev)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(attributes); i++)
        if (device_create_file(dev, attributes + i))
            goto error;
    return 0;

error:
    for (; i >= 0; i--)
        device_remove_file(dev, attributes + i);
    dev_err(dev, "%s:Unable to create interface\n", __func__);
    return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(attributes); i++)
        device_remove_file(dev, attributes + i);
    return 0;
}

static void l3gd20_gyr_report_triple(struct l3gd20_gyr_status *stat)
{
    int err;
    struct l3gd20_gyr_triple data_out;

    err = l3gd20_gyr_get_data(stat, &data_out);
    if (err < 0)
        dev_err(&stat->client->dev, "get_gyroscope_data failed\n");
    else
        l3gd20_gyr_report_values(stat, &data_out);
}

static void l3gd20_gyr_irq2_fifo(struct l3gd20_gyr_status *stat)
{
    int err;
    u8 buf[2];
    u8 int_source;
    u8 samples;
    u8 workingmode;
    u8 stored_samples;

    mutex_lock(&stat->lock);
    workingmode = stat->fifomode;
    dev_dbg(&stat->client->dev, "%s : fifomode:0x%02x\n", __func__,workingmode);

    switch (workingmode) {
        case FIFO_MODE_BYPASS:
          {
            dev_dbg(&stat->client->dev, "%s : fifomode:0x%02x\n", __func__,stat->fifomode);
            l3gd20_gyr_report_triple(stat);
            break;
          }
        case FIFO_MODE_FIFO:
            samples = (stat->watermark)+1;
            dev_dbg(&stat->client->dev,"%s : FIFO_SRC_REG init samples:%d\n",__func__, samples);
            err = l3gd20_gyr_register_read(stat, buf, FIFO_SRC_REG);
            if (err < 0)
                dev_err(&stat->client->dev,"error reading fifo source reg\n");
            int_source = buf[0];
            dev_dbg(&stat->client->dev, "%s :FIFO_SRC_REG content:0x%02x\n",__func__, int_source);
            stored_samples = int_source & FIFO_STORED_DATA_MASK;
            dev_dbg(&stat->client->dev, "%s : fifomode:0x%02x\n", __func__,stat->fifomode);
            dev_dbg(&stat->client->dev, "%s : samples:%d stored:%d\n",__func__, samples, stored_samples);
            for (; samples > 0; samples--) {
#ifdef DEBUG
                input_report_abs(stat->input_dev, ABS_MISC, 1);
                input_sync(stat->input_dev);
#endif
                dev_dbg(&stat->client->dev, "%s : current sample:%d\n",__func__, samples);
                l3gd20_gyr_report_triple(stat);
#ifdef DEBUG
                input_report_abs(stat->input_dev, ABS_MISC, 0);
                input_sync(stat->input_dev);
#endif
            }
            l3gd20_gyr_fifo_reset(stat);
        break;
    }
#ifdef DEBUG
    input_report_abs(stat->input_dev, ABS_MISC, 3);
    input_sync(stat->input_dev);
#endif
    mutex_unlock(&stat->lock);
}

static irqreturn_t l3gd20_gyr_isr2(int irq, void *dev)
{
    struct l3gd20_gyr_status *stat = dev;

    disable_irq_nosync(irq);
#ifdef DEBUG
    input_report_abs(stat->input_dev, ABS_MISC, 2);
    input_sync(stat->input_dev->input);
#endif
    queue_work(stat->irq2_work_queue, &stat->irq2_work);
    pr_debug("%s %s: isr2 queued\n", L3GD20_GYR_DEV_NAME, __func__);

    return IRQ_HANDLED;
}

static void l3gd20_gyr_irq2_work_func(struct work_struct *work)
{
    struct l3gd20_gyr_status *stat = container_of(work, struct l3gd20_gyr_status, irq2_work);
    /* TODO  add interrupt service procedure.
       ie:l3gd20_gyr_irq2_XXX(stat); */
    l3gd20_gyr_irq2_fifo(stat);
    pr_debug("%s %s: IRQ2 served\n", L3GD20_GYR_DEV_NAME, __func__);
    /* exit: */
    enable_irq(stat->irq2);
}

int l3gd20_gyr_input_open(struct input_dev *input)
{
    struct l3gd20_gyr_status *stat = input_get_drvdata(input);
    dev_dbg(&stat->client->dev, "%s\n", __func__);
    return l3gd20_gyr_enable(stat);
}

void l3gd20_gyr_input_close(struct input_dev *dev)
{
    struct l3gd20_gyr_status *stat = input_get_drvdata(dev);
    dev_dbg(&stat->client->dev, "%s\n", __func__);
    l3gd20_gyr_disable(stat);
}

static int l3gd20_gyr_validate_pdata(struct l3gd20_gyr_status *stat)
{
    /* checks for correctness of minimal polling period */
    stat->pdata->min_interval =	max((unsigned int) L3GD20_GYR_MIN_POLL_PERIOD_MS,stat->pdata->min_interval);
    stat->pdata->poll_interval = max(stat->pdata->poll_interval,stat->pdata->min_interval);
    /* Enforce minimum polling interval */
    if (stat->pdata->poll_interval < stat->pdata->min_interval) {
        dev_err(&stat->client->dev,"minimum poll interval violated\n");
        return -EINVAL;
    }
    return 0;
}

static int l3gd20_gyr_input_init(struct l3gd20_gyr_status *stat)
{
    int err = -1;

    dev_dbg(&stat->client->dev, "%s\n", __func__);

    stat->input_dev = input_allocate_device();
    if (!stat->input_dev) {
        err = -ENOMEM;
        dev_err(&stat->client->dev,"input device allocation failed\n");
        goto err0;
    }

    stat->input_dev->open = l3gd20_gyr_input_open;
    stat->input_dev->close = l3gd20_gyr_input_close;
    stat->input_dev->name = L3GD20_GYR_DEV_NAME;
    stat->input_dev->id.bustype = BUS_I2C;
    stat->input_dev->dev.parent = &stat->client->dev;
    set_bit(EV_ABS, stat->input_dev->evbit);

#ifdef DEBUG
    set_bit(EV_KEY, stat->input_dev->keybit);
    set_bit(KEY_LEFT, stat->input_dev->keybit);
    input_set_abs_params(stat->input_dev, ABS_MISC, 0, 1, 0, 0);
#endif
    input_set_capability(stat->input_dev, EV_ABS, ABS_MISC);
    input_set_abs_params(stat->input_dev, ABS_RX, -FS_MAX-1, FS_MAX, 0, 0);
    input_set_abs_params(stat->input_dev, ABS_RY, -FS_MAX-1, FS_MAX, 0, 0);
    input_set_abs_params(stat->input_dev, ABS_RZ, -FS_MAX-1, FS_MAX, 0, 0);
    input_set_events_per_packet(stat->input_dev, 100);
    input_set_drvdata(stat->input_dev, stat);
    err = input_register_device(stat->input_dev);
    if (err) {
        dev_err(&stat->client->dev,"unable to register input polled device %s\n",stat->input_dev->name);
        goto err1;
    }
    return 0;

err1:
    input_free_device(stat->input_dev);
err0:
    return err;
}

static void l3gd20_gyr_input_cleanup(struct l3gd20_gyr_status *stat)
{
    input_unregister_device(stat->input_dev);
    input_free_device(stat->input_dev);
}

static void poll_function_work(struct work_struct *polling_task)
{
    struct l3gd20_gyr_status *stat;
    struct l3gd20_gyr_triple data_out;
    int err;
		ktime_t timestamp;

    stat = container_of((struct work_struct *)polling_task,struct l3gd20_gyr_status, polling_task);
    err = l3gd20_gyr_get_data(stat, &data_out);
    //l3gd20_gyr_report_values(stat, &data_out);
    timestamp = ktime_get_boottime();
    input_report_abs(stat->input_dev, ABS_RX, data_out.x - stat->cal.x);
    input_report_abs(stat->input_dev, ABS_RY, data_out.y - stat->cal.y);
    input_report_abs(stat->input_dev, ABS_RZ, data_out.z - stat->cal.z);
    input_event(stat->input_dev,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
    input_event(stat->input_dev,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
    input_sync(stat->input_dev);

//    stat->ktime = ktime_set(0,stat->pdata->poll_interval * NSEC_PER_MSEC);
//    hrtimer_start(&stat->hr_timer, stat->ktime, HRTIMER_MODE_REL);
}

enum hrtimer_restart poll_function_read(struct hrtimer *timer)
{
    struct l3gd20_gyr_status *stat;
    ktime_t ktime;

    stat = container_of((struct hrtimer *)timer,struct l3gd20_gyr_status, hr_timer);
    if(stat->high_q)
        queue_work(l3gd20_gyr_workqueue_1, &stat->polling_task);
    else
        queue_work(l3gd20_gyr_workqueue, &stat->polling_task);

    ktime = ktime_set(0,stat->pdata->poll_interval * NSEC_PER_MSEC);
    hrtimer_forward_now(&stat->hr_timer,stat->ktime);
    return HRTIMER_RESTART;
}

static int gyro_pinctrl_init(struct i2c_client *client)
{
    int retval;

    /* Get pinctrl if target uses pinctrl */
    gyro_pinctrl = devm_pinctrl_get(&(client->dev));
    if (IS_ERR_OR_NULL(gyro_pinctrl)) {
        pr_info( "Target does not use pinctrl\n");
        retval = PTR_ERR(gyro_pinctrl);
        gyro_pinctrl = NULL;
        return retval;
    }

    gyro_gpio_state_active = pinctrl_lookup_state(gyro_pinctrl, "gyro_active");
    if (IS_ERR_OR_NULL(gyro_gpio_state_active)) {
        pr_info( "Can not get ts default pinstate\n");
        retval = PTR_ERR(gyro_gpio_state_active);
        gyro_pinctrl = NULL;
        return retval;
    }

    gyro_gpio_state_suspend = pinctrl_lookup_state(gyro_pinctrl, "gyro_suspend");
    if (IS_ERR_OR_NULL(gyro_gpio_state_suspend)) {
        pr_info( "Can not get ts sleep pinstate\n");
        retval = PTR_ERR(gyro_gpio_state_suspend);
        gyro_pinctrl = NULL;
        return retval;
    }

    return 0;
}

static int gyro_pinctrl_select(bool on)
{
    struct pinctrl_state *pins_state;
    int ret;

    pins_state = on ? gyro_gpio_state_active : gyro_gpio_state_suspend;
    if (!IS_ERR_OR_NULL(pins_state)) {
        ret = pinctrl_select_state(gyro_pinctrl, pins_state);
        if (ret) {
            pr_info( "can not set %s pins\n",	on ? "gyro_active" : "gyro_suspend");
            return ret;
        }
    } else
        pr_info( "not a valid '%s' pinstate\n",	on ? "gyro_active" : "gyro_suspend");

    return 0;
}

static int l3gd20_gry_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
    struct l3gd20_gyr_status *stat = container_of(sensors_cdev, struct l3gd20_gyr_status, cdev);
    int err;
    if (enable) {
        err = l3gd20_gyr_enable(stat);
        if (err < 0) {
            dev_err(&stat->client->dev, "enable error\n");
            return err;
       }
    } else {
        err = l3gd20_gyr_disable(stat);
        if (err < 0) {
            dev_err(&stat->client->dev, "enable error\n");
            return err;
        }
    }
    return err;
}

static int l3gd20_gry_poll_delay_set(struct sensors_classdev *sensors_cdev, unsigned int delay_msec)
{
    struct l3gd20_gyr_status *stat = container_of(sensors_cdev,struct l3gd20_gyr_status, cdev);
    int err;

    dev_dbg(&stat->client->dev, "set poll delay =%d\n", delay_msec);
    mutex_lock(&stat->lock);
    stat->pdata->poll_interval = delay_msec;
    stat->cdev.delay_msec = stat->pdata->poll_interval;
    /*
     * Device may not power on,
     * only set register when device is enabled.
     */
    if (atomic_read(&stat->enabled)) {
        err = l3gd20_gyr_update_odr(stat, delay_msec);
        if (err < 0) {
            dev_err(&stat->client->dev, "Cannot update ODR\n");
            err = -EBUSY;
        }
    }
    mutex_unlock(&stat->lock);
    return err;
}

static int l3gd20_gyr_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
    struct l3gd20_gyr_status *stat;
//    struct regulator *vcc, *vcc_i2c;
    int err, enable_pin,rc;
    u32 temp_val;
    u8 buf[2];
    struct device_node *np=client->dev.of_node;
    u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |	I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK ;

    pr_err("***********{l3gd20_gyr_probe} \n");
    l3gd20_config_regulator(client, 1);
//    vcc = regulator_get(&client->dev, "vcc");
//    if (IS_ERR(vcc)) {
//        err = PTR_ERR(vcc);
//        pr_info( "Regulator get failed vcc rc=%d\n", err);
//    }
//    err = regulator_enable(vcc);
//    if (err)
//        pr_info( "Regulator vdd enable failed rc=%d\n", err);
//    vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
//    if (IS_ERR(vcc_i2c)) {
//        err = PTR_ERR(vcc_i2c);
//        pr_info( "Regulator get failed vcc_i2c rc=%d\n", err);
//    }
//    err = regulator_enable(vcc_i2c);
//    if (err)
//        pr_info( "Regulator vdd enable failed rc=%d\n", err);
  err = -1;
    err = gyro_pinctrl_init(client);
    if (!err && gyro_pinctrl) {
        err = gyro_pinctrl_select(true);
        if (err < 0)
            pr_info( "[Gyro]l3gd20_gyr_probe gyro_pinctrl_select fail\n");
    }

    stat = kzalloc(sizeof(*stat), GFP_KERNEL);
    if (stat == NULL) {
        dev_err(&client->dev,"failed to allocate memory for module data\n");
        err = -ENOMEM;
        goto err0;
    }

    /* Support for both I2C and SMBUS adapter interfaces. */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_warn(&client->dev, "client not i2c capable\n");
        if (i2c_check_functionality(client->adapter, smbus_func)) {
            stat->use_smbus = 1;
            dev_warn(&client->dev, "client using SMBUS\n");
        } else {
            err = -ENODEV;
            dev_err(&client->dev, "client nor SMBUS capable\n");
            stat->use_smbus = 0;
            goto err0;
        }
    }
    dev_info(&client->dev, "probe start.\n");

    if(l3gd20_gyr_workqueue == 0)
        l3gd20_gyr_workqueue = create_workqueue("l3gd20_gyr_workqueue");
    if(l3gd20_gyr_workqueue_1 == 0)
        l3gd20_gyr_workqueue_1 = alloc_workqueue("l3gd20_data_work1", WQ_HIGHPRI, 0);

    hrtimer_init(&stat->hr_timer, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
    stat->hr_timer.function = &poll_function_read;

    mutex_init(&stat->lock);
    mutex_lock(&stat->lock);
    stat->client = client;
    stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
    if (stat->pdata == NULL) {
        dev_err(&client->dev,"failed to allocate memory for pdata: %d\n", err);
        goto err1;
    }

    if (client->dev.platform_data == NULL) {
        default_l3gd20_gyr_pdata.gpio_int1 = int1_gpio;
        default_l3gd20_gyr_pdata.gpio_int2 = int2_gpio;
        memcpy(stat->pdata, &default_l3gd20_gyr_pdata,sizeof(*stat->pdata));
        dev_info(&client->dev, "using default plaform_data\n");
    } else {
        memcpy(stat->pdata, client->dev.platform_data,sizeof(*stat->pdata));
    }

    stat->pdata->gpio_int1 = of_get_named_gpio(np, "gyro,irq-gpio1", 0);
    stat->pdata->gpio_int2 = of_get_named_gpio(np, "gyro,irq-gpio2", 0);
    enable_pin = of_get_named_gpio(np, "gyro,data-enable", 0);
    stat->pdata->gpio_int1 = 0;
    stat->pdata->gpio_int2 = 0;

    rc = of_property_read_u32(np, "gyro,axis-map-x", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(&client->dev, "Unable to read axis-map_x\n");
        return rc;
    } else {
        stat->pdata->axis_map_x = (u8)temp_val;
    }
    rc = of_property_read_u32(np, "gyro,axis-map-y", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(&client->dev, "Unable to read axis_map_y\n");
        return rc;
    } else {
        stat->pdata->axis_map_y = (u8)temp_val;
    }

    rc = of_property_read_u32(np, "gyro,axis-map-z", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(&client->dev, "Unable to read axis-map-z\n");
        return rc;
    } else {
        stat->pdata->axis_map_z = (u8)temp_val;
    }

    stat->pdata->negate_x = of_property_read_bool(np, "gyro,negate-x");
    stat->pdata->negate_y = of_property_read_bool(np, "gyro,negate-y");
    stat->pdata->negate_z = of_property_read_bool(np, "gyro,negate-z");

    err = gpio_request(enable_pin, "gyro-enable_data");
    if (err){
        printk("[Gyro] - enable pin failed!\n");
    }
    err = gpio_direction_output(enable_pin, 0);
    if (err){
        printk("[Gyro] - gpio_direction_input enable pin failed!\n");
    }
//    err = gpio_request(stat->pdata->gpio_int1, "gyro-int1");
//    if (err){
//        printk("[Gyro] - gpio_request interrupt1 failed!\n");
//    }
//    err = gpio_direction_input(stat->pdata->gpio_int1);
//    if (err){
//        printk("[Gyro] - gpio_tlmm_config interrupt1 failed!\n");
//    }

//    err = gpio_request(stat->pdata->gpio_int2, "gyro-int2");
//    if (err){
//        printk("[Gyro] - gpio_request interrupt2 failed!\n");
//    }
//    err = gpio_direction_input(stat->pdata->gpio_int2);
//    if (err){
//        printk("[Gyro] - gpio_tlmm_config interrupt2 failed!\n");
//    }

    err = l3gd20_gyr_validate_pdata(stat);
    if (err < 0) {
        dev_err(&client->dev, "failed to validate platform data\n");
        goto err1_1;
    }

    i2c_set_clientdata(client, stat);

    if (stat->pdata->init) {
        err = stat->pdata->init();
        if (err < 0) {
            dev_err(&client->dev, "init failed: %d\n", err);
            goto err1_1;
        }
    }

    err = l3gd20_gyr_register_read(stat, buf, WHO_AM_I);
    pr_info( "[Gyro]WHO AM I = 0x%02x\n", buf[0]);
    if (err < 0)
        dev_err(&stat->client->dev,"error reading who am I reg\n");

    memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));
    stat->resume_state[RES_CTRL_REG1] = ALL_ZEROES | ENABLE_ALL_AXES | PM_NORMAL;
    stat->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
    stat->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
    stat->resume_state[RES_CTRL_REG4] = ALL_ZEROES | BDU_ENABLE;
    stat->resume_state[RES_CTRL_REG5] = ALL_ZEROES;
    stat->resume_state[RES_FIFO_CTRL_REG] = ALL_ZEROES;

    stat->polling_enabled = true;
    dev_info(&client->dev, "polling mode enabled\n");

    err = l3gd20_gyr_device_power_on(stat);
    if (err < 0) {
        dev_err(&client->dev, "power on failed: %d\n", err);
        goto err2;
    }

    atomic_set(&stat->enabled, 1);
    err = l3gd20_gyr_update_fs_range(stat, stat->pdata->fs_range);
    if (err < 0) {
        dev_err(&client->dev, "update_fs_range failed\n");
        goto err2;
    }

    err = l3gd20_gyr_update_odr(stat, stat->pdata->poll_interval);
    if (err < 0) {
        dev_err(&client->dev, "update_odr failed\n");
        goto err2;
    }

    err = l3gd20_gyr_input_init(stat);
    if (err < 0)
        goto err3;

    err = create_sysfs_interfaces(&stat->input_dev->dev);
    if (err < 0) {
        dev_err(&client->dev,"%s device register failed\n", L3GD20_GYR_DEV_NAME);
        goto err4;
    }
    stat->cdev = sensors_cdev;
    stat->cdev.delay_msec = stat->pdata->poll_interval;
    stat->cdev.sensors_enable = l3gd20_gry_enable_set;
    stat->cdev.sensors_poll_delay = l3gd20_gry_poll_delay_set;
    stat->cdev.sensors_set_latency = l3gd20_gry_poll_delay_set;
    err = sensors_classdev_register(&stat->input_dev->dev, &stat->cdev);
    if (err) {
        dev_err(&client->dev, "class device create failed: %d\n", err);
        err = -EINVAL;
        goto err6;
    }

/* As default, do not report information */
    atomic_set(&stat->enabled, 0);
    if (stat->pdata->gpio_int2 > 0) {
        stat->irq2 = gpio_to_irq(stat->pdata->gpio_int2);
        dev_info(&client->dev, "%s: %s has set irq2 to irq: %d mapped on gpio:%d\n",L3GD20_GYR_DEV_NAME, __func__, stat->irq2,stat->pdata->gpio_int2);
        INIT_WORK(&stat->irq2_work, l3gd20_gyr_irq2_work_func);
        stat->irq2_work_queue =	create_singlethread_workqueue("l3gd20_gyr_irq2_wq");
        if (!stat->irq2_work_queue) {
            err = -ENOMEM;
            dev_err(&client->dev, "cannot create work queue2: %d\n", err);
            goto err5;
        }
        err = request_irq(stat->irq2, l3gd20_gyr_isr2,IRQF_TRIGGER_HIGH, "l3gd20_gyr_irq2", stat);

        if (err < 0) {
            dev_err(&client->dev, "request irq2 failed: %d\n", err);
            goto err6;
        }
        disable_irq_nosync(stat->irq2);
    }
    mutex_unlock(&stat->lock);
    stat->high_q = false;
    stat->cal_zero = true;
    stat->zero_offset_range = ZERO_RATE_LEVEL;
    stat->zero_variety =  ZERO_VARIETY_RANGE;
    stat->zero_static = false;
    stat->Valid_Sample=0;
    stat->cal.x = 0 ;
    stat->cal.y = 0 ;
    stat->cal.z = 0 ;
    stat->sum_small.x = 0;
    stat->sum_small.y = 0;
    stat->sum_small.z = 0;
    stat->sum.x = 0 ;
    stat->sum.y = 0 ;
    stat->sum.z = 0 ;
    INIT_WORK(&stat->polling_task, poll_function_work);
    dev_info(&client->dev, "%s probed: device created successfully\n",L3GD20_GYR_DEV_NAME);
    l3gd20_gyr_device_power_off(stat);
//    gyro_pinctrl_select(false);
//    l3gd20_config_regulator(client, 0);
    return 0;

/*err7:
	free_irq(stat->irq2, stat);
*/
err6:
    destroy_workqueue(stat->irq2_work_queue);
err5:
    l3gd20_gyr_device_power_off(stat);
    remove_sysfs_interfaces(&stat->input_dev->dev);
err4:
    l3gd20_gyr_input_cleanup(stat);
err3:
    l3gd20_gyr_device_power_off(stat);
err2:
    if (stat->pdata->exit)
        stat->pdata->exit();
err1_1:
    mutex_unlock(&stat->lock);
    kfree(stat->pdata);
err1:
    destroy_workqueue(l3gd20_gyr_workqueue);
    destroy_workqueue(l3gd20_gyr_workqueue_1);
    kfree(stat);
err0:
    pr_err("%s: Driver Initialization failed\n",L3GD20_GYR_DEV_NAME);
    return err;
}

static int l3gd20_gyr_remove(struct i2c_client *client)
{
    struct l3gd20_gyr_status *stat = i2c_get_clientdata(client);
    int retval;
    dev_info(&stat->client->dev, "driver removing\n");

    cancel_work_sync(&stat->polling_task);
    if(!l3gd20_gyr_workqueue) {
        flush_workqueue(l3gd20_gyr_workqueue);
        destroy_workqueue(l3gd20_gyr_workqueue);
    }
    if(!l3gd20_gyr_workqueue_1) {
        flush_workqueue(l3gd20_gyr_workqueue_1);
        destroy_workqueue(l3gd20_gyr_workqueue_1);
    }

    /*
    if (stat->pdata->gpio_int1 >= 0)
    {
        free_irq(stat->irq1, stat);
        gpio_free(stat->pdata->gpio_int1);
        destroy_workqueue(stat->irq1_work_queue);
    }
    */
    if (stat->pdata->gpio_int2 > 0) {
        free_irq(stat->irq2, stat);
        gpio_free(stat->pdata->gpio_int2);
        destroy_workqueue(stat->irq2_work_queue);
    }
    if (gyro_pinctrl) {
        retval = gyro_pinctrl_select(false);
        if (retval < 0)
            pr_err("Cannot get idle pinctrl state\n");
    }
    l3gd20_gyr_disable(stat);
    l3gd20_gyr_input_cleanup(stat);

    remove_sysfs_interfaces(&client->dev);

    kfree(stat->pdata);
    kfree(stat);
    return 0;
}
#define SLEEP
static int l3gd20_gyr_suspend(struct device *dev)
{
    int err = 0;

#ifdef CONFIG_PM
    struct i2c_client *client = to_i2c_client(dev);
    struct l3gd20_gyr_status *stat = i2c_get_clientdata(client);
    u8 buf[2];
    dev_dbg(&client->dev, "%s\n", __func__);
    if (atomic_read(&stat->enabled)) {
        mutex_lock(&stat->lock);
        if (stat->polling_enabled) {
            dev_info(&stat->client->dev, "polling mode disabled\n");
            hrtimer_cancel(&stat->hr_timer);
        }
#ifdef SLEEP
        err = l3gd20_gyr_register_update(stat, buf, CTRL_REG1,0x0F, (ENABLE_NO_AXES | PM_NORMAL));
#else
        err = l3gd20_gyr_register_update(stat, buf, CTRL_REG1,0x08, PM_OFF);
#endif /*SLEEP*/
        mutex_unlock(&stat->lock);
    }
#endif /*CONFIG_PM*/
    return err;
}

static int l3gd20_gyr_resume(struct device *dev)
{
    int err = 0;
#ifdef CONFIG_PM
    struct i2c_client *client = to_i2c_client(dev);
    struct l3gd20_gyr_status *stat = i2c_get_clientdata(client);
    u8 buf[2];
    dev_dbg(&client->dev, "%s\n", __func__);
    if (atomic_read(&stat->enabled)) {
        mutex_lock(&stat->lock);
//        if (stat->polling_enabled) {
//            dev_info(&stat->client->dev, "polling mode enabled\n");
//            hrtimer_init(&stat->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
//        }
#ifdef SLEEP
        err = l3gd20_gyr_register_update(stat, buf, CTRL_REG1,0x0F, (ENABLE_ALL_AXES | PM_NORMAL));
#else
        err = l3gd20_gyr_register_update(stat, buf, CTRL_REG1,0x08, PM_NORMAL);
#endif
        if (stat->polling_enabled) {
            dev_info(&stat->client->dev, "polling mode enabled\n");
            //hrtimer_init(&stat->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
            l3gd20_gyr_update_odr(stat, stat->pdata->poll_interval);
            hrtimer_start(&(stat->hr_timer), stat->ktime, HRTIMER_MODE_REL);
        }
        mutex_unlock(&stat->lock);
    }
#endif /*CONFIG_PM*/
    return err;
}

static const struct of_device_id sensor_device_of_match[]  = {
    { .compatible = "fih,gyro", },
    {},
};

static const struct i2c_device_id l3gd20_gyr_id[] = {
    { "l3gd20" , 0 },
    {},
};
MODULE_DEVICE_TABLE(i2c, l3gd20_gyr_id);

static const struct dev_pm_ops l3gd20_gyr_pm = {
    .suspend = l3gd20_gyr_suspend,
    .resume = l3gd20_gyr_resume,
};

static struct i2c_driver l3gd20_gyr_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "l3gd20",
        .of_match_table = sensor_device_of_match,
        .pm = &l3gd20_gyr_pm,
    },
    .probe = l3gd20_gyr_probe,
    .remove = l3gd20_gyr_remove,
    .id_table = l3gd20_gyr_id,
};

static int __init l3gd20_gyr_init(void)
{
    pr_info("%s: gyroscope sysfs driver init\n", L3GD20_GYR_DEV_NAME);
    return i2c_add_driver(&l3gd20_gyr_driver);
}

static void __exit l3gd20_gyr_exit(void)
{
    pr_info("%s exit\n", L3GD20_GYR_DEV_NAME);
    i2c_del_driver(&l3gd20_gyr_driver);
    return;
}

module_init(l3gd20_gyr_init);
module_exit(l3gd20_gyr_exit);

MODULE_DESCRIPTION("l3gd20 gyroscope driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");
