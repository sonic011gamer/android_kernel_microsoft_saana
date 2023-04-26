/*!
 * @section LICENSE
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename    bmm050_driver.c
 * @date        2015/11/23
 * @id          "d2ea96c"
 * @version     v2.8.1
 *
 * @brief       BMM050 Linux Driver
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/math64.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif


#include "bmm050.h"
#include "bs_log.h"

/* sensor specific */
#define SENSOR_NAME "bmm050"
#define DRIVER_VERSION "0.0.27.0"

#define SENSOR_CHIP_ID_BMM (0x32)
#define CHECK_CHIP_ID_TIME_MAX   5
#define MSC_TIME                6

#define BMM_REG_NAME(name) BMM050_##name
#define BMM_VAL_NAME(name) BMM050_##name
#define BMM_CALL_API(name) bmm050_##name

#define BMM_I2C_WRITE_DELAY_TIME 5
#define CONFIG_BMM_USE_PLATFORM_DATA 1
#define BMM_DEFAULT_REPETITION_XY BMM_VAL_NAME(REGULAR_REPXY)
#define BMM_DEFAULT_REPETITION_Z BMM_VAL_NAME(REGULAR_REPZ)
#define BMM_DEFAULT_ODR BMM_VAL_NAME(REGULAR_DR)
/* generic */
#define BMM_MAX_RETRY_I2C_XFER (10)
#define BMM_MAX_RETRY_WAKEUP (5)
#define BMM_MAX_RETRY_WAIT_DRDY (100)

#define BMM_DELAY_MIN (1)
#define BMM_DELAY_DEFAULT (200)

#define MAG_VALUE_MAX (32767)
#define MAG_VALUE_MIN (-32768)

#define BYTES_PER_LINE (16)

#define BMM_SELF_TEST 1
#define BMM_ADV_TEST 2

#define BMM_OP_MODE_UNKNOWN (-1)

/*! Bosch sensor unknown place*/
#define BOSCH_SENSOR_PLACE_UNKNOWN (-1)
/*! Bosch sensor remapping table size P0~P7*/
#define MAX_AXIS_REMAP_TAB_SZ 8

int ecompass_err_count = 0;

#ifdef CONFIG_BMM_USE_PLATFORM_DATA
struct bosch_sensor_specific {
	char *name;
	/* 0 to 7 */
	int place;
	int irq;
	struct regulator *vcc_i2c;
	struct regulator *mux_vcc;
	int (*irq_gpio_cfg)(void);
};
#endif
struct bosch_sensor_specific bmm050_platform;
/*!
 * we use a typedef to hide the detail,
 * because this type might be changed
 */
struct bosch_sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis */
	/* if an target OS axis is remapped from (-)x,
	 * src is 0, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)y,
	 * src is 1, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)z,
	 * src is 2, sign_* is (-)1 */
	int src_x:3;
	int src_y:3;
	int src_z:3;

	int sign_x:2;
	int sign_y:2;
	int sign_z:2;
};

struct bosch_sensor_data {
	union {
		int16_t v[3];
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		};
	};
};

static const u8 odr_map[] = {10, 2, 6, 8, 15, 20, 25, 30};
static const long op_mode_maps[] = {
	BMM_VAL_NAME(NORMAL_MODE),
	BMM_VAL_NAME(FORCED_MODE),
	BMM_VAL_NAME(SUSPEND_MODE),
	BMM_VAL_NAME(SLEEP_MODE)
};


struct bmm_client_data {
	struct bmm050 device;
	struct sensors_classdev cdev;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_handler;
#endif

	atomic_t delay;
	/* whether the system in suspend state */
	atomic_t in_suspend;

	struct bmm050_mdata_s32 value;
	u8 enable:1;
	s8 op_mode:4;
	u8 odr;
	u8 sampling_frequency;
	u8 rept_xy;
	u8 rept_z;

	s16 result_test;

	struct mutex mutex_power_mode;

	/* controls not only reg, but also workqueue */
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;
	struct mutex mutex_odr;
	struct mutex mutex_rept_xy;
	struct mutex mutex_rept_z;

	struct mutex mutex_value;
#ifdef CONFIG_BMM_USE_PLATFORM_DATA
	struct bosch_sensor_specific *bst_pd;
#endif

	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct work_struct report_data_work;
	int is_timer_running;
	struct hrtimer timer;
	ktime_t work_delay_kt;
	u8 timer_read;
};

static struct i2c_client *bmm_client;
/* i2c operation for API */
static void bmm_delay(u32 msec);
static int bmm_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);
static int bmm_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);

static void bmm_dump_reg(struct i2c_client *client);
static int bmm_wakeup(struct i2c_client *client);
static int bmm_check_chip_id(struct i2c_client *client);

static int bmm_pre_suspend(struct i2c_client *client);
static int bmm_post_resume(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmm_early_suspend(struct early_suspend *handler);
static void bmm_late_resume(struct early_suspend *handler);
#endif
extern int fih_ecompass_proc_init(void);

static struct sensors_classdev sensors_cdev = {
		.name = "bmm050-mag",
		.vendor = "bosch",
		.version = 1,
		.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
		.type = SENSOR_TYPE_MAGNETIC_FIELD,
		.max_range = "1228.8",	/* 16g */
		.resolution = "0.0488228125",	/* 15.6mg */
		.sensor_power = "0.13",	/* typical value */
	  .min_delay = 20000,
	  .max_delay = 200000,
//		.max_latency = POLL_INTERVAL_MAX_MS,
		.fifo_reserved_event_count = 0,
		.fifo_max_event_count = 0,
		.enabled = 0,
		.delay_msec = 100, /* in millisecond */
		.sensors_enable = NULL,
		.sensors_poll_delay = NULL,
		.sensors_self_test = NULL,
};


static int bmm_restore_hw_cfg(struct i2c_client *client);

static void bmm_delay(u32 msec)
{
	if (msec <= 20)
		usleep_range(msec * 1000, msec * 1000);
	else
		msleep(msec);
}

static const struct bosch_sensor_axis_remap
bst_axis_remap_tab_dft[MAX_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,    1,    2,     1,      1,      1 }, /* P0 */
	{  1,    0,    2,     1,     -1,      1 }, /* P1 */
	{  0,    1,    2,    -1,     -1,      1 }, /* P2 */
	{  1,    0,    2,    -1,      1,      1 }, /* P3 */

	{  0,    1,    2,    -1,      1,     -1 }, /* P4 */
	{  1,    0,    2,    -1,     -1,     -1 }, /* P5 */
	{  0,    1,    2,     1,     -1,     -1 }, /* P6 */
	{  1,    0,    2,     1,      1,     -1 }, /* P7 */
};

static void bst_remap_sensor_data(struct bosch_sensor_data *data,
		const struct bosch_sensor_axis_remap *remap)
{
	struct bosch_sensor_data tmp;

	tmp.x = data->v[remap->src_x] * remap->sign_x;
	tmp.y = data->v[remap->src_y] * remap->sign_y;
	tmp.z = data->v[remap->src_z] * remap->sign_z;

	memcpy(data, &tmp, sizeof(*data));
}

uint64_t bmm_get_alarm_timestamp(void)
{
	uint64_t ts_ap;
	struct timespec tmp_time;
	get_monotonic_boottime(&tmp_time);
	ts_ap = (uint64_t)tmp_time.tv_sec * 1000000000 + tmp_time.tv_nsec;
	return ts_ap;
}

static void bst_remap_sensor_data_dft_tab(struct bosch_sensor_data *data,
		int place)
{
	/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= MAX_AXIS_REMAP_TAB_SZ))
		return;

	bst_remap_sensor_data(data, &bst_axis_remap_tab_dft[place]);
}

static void bmm_remap_sensor_data(struct bmm050_mdata_s32 *val,
		struct bmm_client_data *client_data)
{
#ifdef CONFIG_BMM_USE_PLATFORM_DATA
	struct bosch_sensor_data bsd;
	int place;

	if ((NULL == client_data->bst_pd) || (BOSCH_SENSOR_PLACE_UNKNOWN == client_data->bst_pd->place))
		place = bmm050_platform.place;
	else
		place = client_data->bst_pd->place;

	bsd.x = val->datax;
	bsd.y = val->datay;
	bsd.z = val->dataz;

	bst_remap_sensor_data_dft_tab(&bsd, place);

	val->datax = bsd.x;
	val->datay = bsd.y;
	val->dataz = bsd.z;
#else
	(void)val;
	(void)client_data;
#endif
}

static int bmm_check_chip_id(struct i2c_client *client)
{
	int err = -1;
	u8 chip_id = 0;
	u8 read_count = 0;

	while (read_count++ < CHECK_CHIP_ID_TIME_MAX) {
		bmm_i2c_read(client, BMM_REG_NAME(CHIP_ID), &chip_id, 1);
		PINFO("read chip id result: %#x", chip_id);

		if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMM) {
			bmm_delay(1);
		} else {
			err = 0;
			break;
		}
	}

	return err;
}

static inline int bmm_get_forced_drdy_time(int rept_xy, int rept_z)
{
	return  (145 * rept_xy + 500 * rept_z + 980 + (1000 - 1)) / 1000;
}


static void bmm_dump_reg(struct i2c_client *client)
{
#ifdef DEBUG
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		snprintf(dbg_buf_str + i * 3, 32, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	printk(KERN_DEBUG "%s\n", dbg_buf_str);

	bmm_i2c_read(client, BMM_REG_NAME(CHIP_ID), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		snprintf(dbg_buf_str + i * 3, 32, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	printk(KERN_DEBUG "%s\n", dbg_buf_str);
#endif
}

static int bmm_wakeup(struct i2c_client *client)
{
	int err = 0;
	int try_times = BMM_MAX_RETRY_WAKEUP;
	const u8 value = 0x01;
	u8 dummy;

	PINFO("waking up the chip...");

	bmm_delay(BMM_I2C_WRITE_DELAY_TIME);
	while (try_times) {
		err = bmm_i2c_write(client,
				BMM_REG_NAME(POWER_CNTL), (u8 *)&value, 1);
		bmm_delay(BMM_I2C_WRITE_DELAY_TIME);
		dummy = 0;
		err = bmm_i2c_read(client, BMM_REG_NAME(POWER_CNTL), &dummy, 1);
		if (value == dummy)
			break;

		try_times--;
	}

	PINFO("wake up result: %s, tried times: %d",
			(try_times > 0) ? "succeed" : "fail",
			BMM_MAX_RETRY_WAKEUP - try_times + 1);

	err = (try_times > 0) ? 0 : -1;

	return err;
}

/*i2c read routine for API*/
static int bmm_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMM_USE_BASIC_I2C_FUNC
	s32 dummy;
	if (NULL == client) {
		printk("BBox::UEC; 8::18\n");
		ecompass_err_count++;
		return -ENODEV;
	}

	while (0 != len--) {
#ifdef BMM_SMBUS
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			PERR("i2c bus read error");
			printk("BBox::UEC; 8::18\n");
			ecompass_err_count++;
			return -EIO;
		}
		*data = (u8)(dummy & 0xff);
#else
		dummy = i2c_master_send(client, (char *)&reg_addr, 1);
		if (dummy < 0) {
			printk("BBox::UEC; 8::18\n");
			ecompass_err_count++;
			return -EIO;
		}

		dummy = i2c_master_recv(client, (char *)data, 1);
		if (dummy < 0) {
			printk("BBox::UEC; 8::18\n");
			ecompass_err_count++;
			return -EIO;
		}
#endif
		reg_addr++;
		data++;
	}
	return 0;
#else
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < BMM_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			bmm_delay(BMM_I2C_WRITE_DELAY_TIME);
	}

	if (BMM_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		printk("BBox::UEC; 8::18\n");
		ecompass_err_count++;
		return -EIO;
	}

	return 0;
#endif
}

/*i2c write routine for */
static int bmm_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMM_USE_BASIC_I2C_FUNC
	s32 dummy;

#ifndef BMM_SMBUS
	u8 buffer[2];
#endif

	if (NULL == client) {
		printk("BBox::UEC; 8::19\n");
		ecompass_err_count++;
		return -ENODEV;
	}

	while (0 != len--) {
#ifdef BMM_SMBUS
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(client, (char *)buffer, 2);
#endif
		reg_addr++;
		data++;
		if (dummy < 0) {
			PERR("error writing i2c bus");
			ecompass_err_count++;
			printk("BBox::UEC; 8::19\n");
			return -EIO;
		}

	}
	return 0;
#else
	u8 buffer[2];
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buffer,
		},
	};

	while (0 != len--) {
		buffer[0] = reg_addr;
		buffer[1] = *data;
		for (retry = 0; retry < BMM_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, msg,
						ARRAY_SIZE(msg)) > 0) {
				break;
			} else
				bmm_delay(BMM_I2C_WRITE_DELAY_TIME);
		}
		if (BMM_MAX_RETRY_I2C_XFER <= retry) {
			PERR("I2C xfer error");
			printk("BBox::UEC; 8::19\n");
			ecompass_err_count++;
			return -EIO;
		}
		reg_addr++;
		data++;
	}

	return 0;
#endif
}

static int bmm_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err = 0;
	err = bmm_i2c_read(bmm_client, reg_addr, data, len);
	return err;
}

static int bmm_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err = 0;
	err = bmm_i2c_write(bmm_client, reg_addr, data, len);
	return err;
}

/* this function exists for optimization of speed,
 * because it is frequently called */
static inline int bmm_set_forced_mode(struct i2c_client *client)
{
	int err = 0;

	/* FORCED_MODE */
	const u8 value = 0x02;
	err = bmm_i2c_write(client, BMM_REG_NAME(CONTROL), (u8 *)&value, 1);

	return err;
}

static void bmm_work_func(struct work_struct *work)
{
	struct bmm_client_data *client_data =
		container_of((struct delayed_work *)work,
			struct bmm_client_data, work);
	struct i2c_client *client = client_data->client;
	unsigned long delay =
		msecs_to_jiffies(atomic_read(&client_data->delay));
	int err = 0;
	ktime_t ts;

	ts = ktime_get_boottime();

	mutex_lock(&client_data->mutex_value);

	mutex_lock(&client_data->mutex_op_mode);
	if (BMM_VAL_NAME(NORMAL_MODE) != client_data->op_mode)
		bmm_set_forced_mode(client);

	mutex_unlock(&client_data->mutex_op_mode);

	err = BMM_CALL_API(read_mdataXYZ_s32)(&client_data->value);
	if(err != 0)
		printk("BBox::UEC; 8::69\n");
	bmm_remap_sensor_data(&client_data->value, client_data);
	input_report_abs(client_data->input, ABS_X, client_data->value.datax);
	input_report_abs(client_data->input, ABS_Y, client_data->value.datay);
	input_report_abs(client_data->input, ABS_Z, client_data->value.dataz);
	input_event(client_data->input, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(ts).tv_sec);
	input_event(client_data->input, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(ts).tv_nsec);
	mutex_unlock(&client_data->mutex_value);

	input_sync(client_data->input);

	schedule_delayed_work(&client_data->work, delay);
}


static int bmm_set_odr(struct i2c_client *client, u8 odr)
{
	int err = 0;

	err = BMM_CALL_API(set_datarate)(odr);
	bmm_delay(BMM_I2C_WRITE_DELAY_TIME);

	return err;
}

static int bmm_get_odr(struct i2c_client *client, u8 *podr)
{
	int err = 0;
	u8 value;

	err = BMM_CALL_API(get_datarate)(&value);
	if (!err)
		*podr = value;

	return err;
}

static ssize_t bmm_show_chip_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 16, "%d\n", SENSOR_CHIP_ID_BMM);
}

static ssize_t bmm_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	u8 op_mode = 0xff;
	u8 power_mode;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_op_mode);
		BMM_CALL_API(get_functional_state)(&op_mode);
		mutex_unlock(&client_data->mutex_op_mode);
	} else {
		op_mode = BMM_VAL_NAME(SUSPEND_MODE);
	}

	mutex_unlock(&client_data->mutex_power_mode);

	PDEBUG("op_mode: %d", op_mode);

	ret = snprintf(buf, 16, "%d\n", op_mode);

	return ret;
}


static inline int bmm_get_op_mode_idx(u8 op_mode)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(op_mode_maps); i++) {
		if (op_mode_maps[i] == op_mode)
			break;
	}

	if (i < ARRAY_SIZE(op_mode_maps))
		return i;
	else
		return -EIO;
}


static inline int bmm_get_op_mode(struct bmm_client_data *client_data)
{
	u8 op_mode = 0xff;
	u8 power_mode;

	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_op_mode);
		BMM_CALL_API(get_functional_state)(&op_mode);
		mutex_unlock(&client_data->mutex_op_mode);
	} else {
		op_mode = BMM_VAL_NAME(SUSPEND_MODE);
	}

	return op_mode;
}


static int bmm_set_op_mode(struct bmm_client_data *client_data, int op_mode)
{
	int err = 0;

	err = BMM_CALL_API(set_functional_state)(
			op_mode);

	if (BMM_VAL_NAME(SUSPEND_MODE) == op_mode)
		atomic_set(&client_data->in_suspend, 1);
	else
		atomic_set(&client_data->in_suspend, 0);

	return err;
}

static ssize_t bmm_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err = 0;
	int i;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	struct i2c_client *client = client_data->client;
	long op_mode;

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;

	mutex_lock(&client_data->mutex_power_mode);

	i = bmm_get_op_mode_idx(op_mode);
	if (i != -1) {
		mutex_lock(&client_data->mutex_op_mode);
		if (op_mode != client_data->op_mode) {
			if (BMM_VAL_NAME(FORCED_MODE) == op_mode) {
				/* special treat of forced mode
				 * for optimization */
				err = bmm_set_forced_mode(client);
			} else {
				err = bmm_set_op_mode(client_data, op_mode);
			}

			if (!err) {
				if (BMM_VAL_NAME(FORCED_MODE) == op_mode)
					client_data->op_mode =
						BMM_OP_MODE_UNKNOWN;
				else
					client_data->op_mode = op_mode;
			}
		}
		mutex_unlock(&client_data->mutex_op_mode);
	} else {
		err = -EINVAL;
	}

	mutex_unlock(&client_data->mutex_power_mode);

	if (err)
		return err;
	else
		return count;
}

static ssize_t bmm_show_enable_timer(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);

	int err;
	mutex_lock(&client_data->mutex_enable);
	err = snprintf(buf, 64, "en_timer %d timer value %d\n",
		client_data->is_timer_running, client_data->timer_read);
	mutex_unlock(&client_data->mutex_enable);
	return err;
}

static void bmm_set_enable(struct device *dev, int timer)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);

	mutex_lock(&client_data->mutex_enable);
	if (timer) {
		if (0 == client_data->is_timer_running) {
			hrtimer_start(&client_data->timer,
						ns_to_ktime(timer * 1000000),
				HRTIMER_MODE_REL);
			client_data->is_timer_running = 1;
		}
	} else {
		if (1 == client_data->is_timer_running) {
			hrtimer_cancel(&client_data->timer);
			client_data->is_timer_running = 0;
	}
	}
	client_data->timer_read = timer;
	mutex_unlock(&client_data->mutex_enable);

}

static ssize_t bmm_store_enable_timer(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long timer;
	int error;
	/*here use the timer from the hal*/
	error = kstrtoul(buf, 10, &timer);
	if (error)
		return error;
	bmm_set_enable(dev, timer);

	return count;

}
static ssize_t bmm_show_odr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	struct i2c_client *client = client_data->client;
	int err;
	u8 power_mode;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_odr);
		err = bmm_get_odr(client, &data);
		mutex_unlock(&client_data->mutex_odr);
	} else {
		err = -EIO;
	}
	mutex_unlock(&client_data->mutex_power_mode);

	if (!err) {
		if (data < ARRAY_SIZE(odr_map))
			err = snprintf(buf, 16, "%d\n", odr_map[data]);
		else
			err = -EINVAL;
	}

	return err;
}

static ssize_t bmm_store_odr(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	unsigned char data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	struct i2c_client *client = client_data->client;
	u8 power_mode;
	int i;

	err = kstrtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		for (i = 0; i < ARRAY_SIZE(odr_map); i++) {
			if (odr_map[i] == data)
				break;
		}

		if (i < ARRAY_SIZE(odr_map)) {
			mutex_lock(&client_data->mutex_odr);
			err = bmm_set_odr(client, i);
			if (!err)
				client_data->odr = i;

			mutex_unlock(&client_data->mutex_odr);
		} else {
			err = -EINVAL;
		}
	} else {
		err = -EIO;
	}

	mutex_unlock(&client_data->mutex_power_mode);
	if (err)
		return err;

	return count;
}

static ssize_t bmm_show_rept_xy(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 power_mode;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_rept_xy);
		err = BMM_CALL_API(get_repetitions_XY)(&data);
		mutex_unlock(&client_data->mutex_rept_xy);
	} else {
		err = -EIO;
	}

	mutex_unlock(&client_data->mutex_power_mode);

	if (err)
		return err;

	return snprintf(buf, 16, "%d\n", data);
}

static ssize_t bmm_store_rept_xy(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 data;
	u8 power_mode;

	err = kstrtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_rept_xy);
		err = BMM_CALL_API(set_repetitions_XY)(data);
		if (!err) {
			bmm_delay(BMM_I2C_WRITE_DELAY_TIME);
			client_data->rept_xy = data;
		}
		mutex_unlock(&client_data->mutex_rept_xy);
	} else {
		err = -EIO;
	}
	mutex_unlock(&client_data->mutex_power_mode);

	if (err)
		return err;

	return count;
}

static ssize_t bmm_show_rept_z(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 power_mode;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_rept_z);
		err = BMM_CALL_API(get_repetitions_Z)(&data);
		mutex_unlock(&client_data->mutex_rept_z);
	} else {
		err = -EIO;
	}

	mutex_unlock(&client_data->mutex_power_mode);

	if (err)
		return err;

	return snprintf(buf, 16, "%d\n", data);
}

static ssize_t bmm_store_rept_z(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 data;
	u8 power_mode;

	err = kstrtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_rept_z);
		err = BMM_CALL_API(set_repetitions_Z)(data);
		if (!err) {
			bmm_delay(BMM_I2C_WRITE_DELAY_TIME);
			client_data->rept_z = data;
		}
		mutex_unlock(&client_data->mutex_rept_z);
	} else {
		err = -EIO;
	}
	mutex_unlock(&client_data->mutex_power_mode);

	if (err)
		return err;

	return count;
}


static ssize_t bmm_show_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int count;
	struct bmm050_mdata_s32 value = {0, 0, 0, 0, 0};

	BMM_CALL_API(read_mdataXYZ_s32)(&value);
	if (value.drdy) {
		bmm_remap_sensor_data(&value, client_data);
		client_data->value = value;
	} else
		PDEBUG("data not ready");

	count = snprintf(buf, 96, "%d %d %d\n",
			client_data->value.datax,
			client_data->value.datay,
			client_data->value.dataz);
	PDEBUG("%d %d %d",
			client_data->value.datax,
			client_data->value.datay,
			client_data->value.dataz);

	return count;
}


static ssize_t bmm_show_value_raw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bmm050_mdata value;
	int count;

	BMM_CALL_API(get_raw_xyz)(&value);

	count = snprintf(buf, 128, "%hd %hd %hd\n",
			value.datax,
			value.datay,
			value.dataz);

	return count;
}


static ssize_t bmm_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;

	mutex_lock(&client_data->mutex_enable);
	err = snprintf(buf, 16, "%d\n", client_data->enable);
	mutex_unlock(&client_data->mutex_enable);
	return err;
}

static ssize_t bmm_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	data = data ? 1 : 0;
	mutex_lock(&client_data->mutex_enable);
	if (data != client_data->enable) {
		if (data) {
			schedule_delayed_work(
					&client_data->work,
					msecs_to_jiffies(atomic_read(
							&client_data->delay)));
		} else {
			cancel_delayed_work_sync(&client_data->work);
		}

		client_data->enable = data;
	}
	mutex_unlock(&client_data->mutex_enable);

	return count;
}

static ssize_t bmm_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, 16, "%d\n", atomic_read(&client_data->delay));

}

static ssize_t bmm_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	if (data == 0) {
		err = -EINVAL;
		return err;
	}

	if (data < BMM_DELAY_MIN)
		data = BMM_DELAY_MIN;

	atomic_set(&client_data->delay, data);

	return count;
}

static ssize_t bmm_show_test(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;

	err = snprintf(buf, 16, "%d\n", client_data->result_test);
	return err;
}

static ssize_t bmm_store_test(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	struct i2c_client *client = client_data->client;
	u8 dummy;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	/* the following code assumes the work thread is not running */
	if (BMM_SELF_TEST == data) {
		/* self test */
		err = bmm_set_op_mode(client_data, BMM_VAL_NAME(SLEEP_MODE));
		bmm_delay(3);
		err = BMM_CALL_API(set_selftest)(1);
		bmm_delay(3);
		err = BMM_CALL_API(get_self_test_XYZ)(&dummy);
		client_data->result_test = dummy;
	} else if (BMM_ADV_TEST == data) {
		/* advanced self test */
		err = BMM_CALL_API(perform_advanced_selftest)(
				&client_data->result_test);
	} else {
		err = -EINVAL;
	}

	if (!err) {
		BMM_CALL_API(soft_reset)();
		bmm_delay(BMM_I2C_WRITE_DELAY_TIME);
		bmm_restore_hw_cfg(client);
	}

	if (err)
		count = -1;

	return count;
}


static ssize_t bmm_show_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err = 0;
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	struct i2c_client *client = client_data->client;

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		snprintf(dbg_buf_str + i * 3, 32, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	memcpy(buf, dbg_buf_str, BYTES_PER_LINE * 3);

	for (i = 0; i < BYTES_PER_LINE * 3 - 1; i++)
		dbg_buf_str[i] = '-';

	dbg_buf_str[i] = '\n';
	memcpy(buf + BYTES_PER_LINE * 3, dbg_buf_str, BYTES_PER_LINE * 3);


	bmm_i2c_read(client, BMM_REG_NAME(CHIP_ID), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		snprintf(dbg_buf_str + i * 3, 32, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	memcpy(buf + BYTES_PER_LINE * 3 + BYTES_PER_LINE * 3,
			dbg_buf_str, 64 * 3);

	err = BYTES_PER_LINE * 3 + BYTES_PER_LINE * 3 + 64 * 3;
	return err;
}


static ssize_t bmm_show_place(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_BMM_USE_PLATFORM_DATA
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
#endif
	int place = bmm050_platform.place;

#ifdef CONFIG_BMM_USE_PLATFORM_DATA
	if (NULL != client_data->bst_pd)
		place = client_data->bst_pd->place;
#endif
	return snprintf(buf, 16, "%d\n", place);
}

static ssize_t bmm_driver_version_show(struct device *dev
		, struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int ret;

	if (client_data == NULL) {
		printk(KERN_ERR "Invalid client_data pointer");
		return -ENODEV;
	}

	ret = snprintf(buf, 128, "Driver version: %s\n",
			DRIVER_VERSION);

	return ret;
}


static DEVICE_ATTR(chip_id, S_IRUGO,
		bmm_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_show_op_mode, bmm_store_op_mode);
static DEVICE_ATTR(odr, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_show_odr, bmm_store_odr);
static DEVICE_ATTR(rept_xy, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_show_rept_xy, bmm_store_rept_xy);
static DEVICE_ATTR(rept_z, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_show_rept_z, bmm_store_rept_z);
static DEVICE_ATTR(value, S_IRUGO,
		bmm_show_value, NULL);
static DEVICE_ATTR(value_raw, S_IRUGO,
		bmm_show_value_raw, NULL);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_show_enable, bmm_store_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_show_delay, bmm_store_delay);
static DEVICE_ATTR(test, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_show_test, bmm_store_test);
static DEVICE_ATTR(reg, S_IRUGO,
		bmm_show_reg, NULL);
static DEVICE_ATTR(place, S_IRUGO,
		bmm_show_place, NULL);
static DEVICE_ATTR(driver_version, S_IRUGO,
		bmm_driver_version_show, NULL);
static DEVICE_ATTR(enable_timer, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_show_enable_timer, bmm_store_enable_timer);
static struct attribute *bmm_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_odr.attr,
	&dev_attr_rept_xy.attr,
	&dev_attr_rept_z.attr,
	&dev_attr_value.attr,
	&dev_attr_value_raw.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_test.attr,
	&dev_attr_reg.attr,
	&dev_attr_place.attr,
	&dev_attr_driver_version.attr,
	&dev_attr_enable_timer.attr,
	NULL
};


static struct attribute_group bmm_attribute_group = {
	.attrs = bmm_attributes
};


static int bmm_input_init(struct bmm_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_capability(dev, EV_MSC, MSC_GESTURE);
	input_set_capability(dev, EV_MSC, MSC_RAW);
	input_set_capability(dev, EV_MSC, MSC_SCAN);
	input_set_capability(dev, EV_MSC, MSC_TIME);
	input_set_drvdata(dev, client_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}

	err = sysfs_create_link(dev->dev.kobj.parent,
				&dev->dev.kobj,
				dev->name);
	if (err < 0) {
		printk(KERN_INFO "[Gyro] %s Can't create soft link in hall driver\n", __func__);
	}

	client_data->input = dev;

	return 0;
}

static void bmm_input_destroy(struct bmm_client_data *client_data)
{
	struct input_dev *dev = client_data->input;

	sysfs_remove_link(client_data->input->dev.kobj.parent, client_data->input->name);
	input_unregister_device(dev);
	input_free_device(dev);
}

static int bmm_restore_hw_cfg(struct i2c_client *client)
{
	int err = 0;
	u8 value;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);
	int op_mode;

	mutex_lock(&client_data->mutex_op_mode);
	err = bmm_set_op_mode(client_data, BMM_VAL_NAME(SLEEP_MODE));

	if (bmm_get_op_mode_idx(client_data->op_mode) != -1)
		err = bmm_set_op_mode(client_data, client_data->op_mode);

	op_mode = client_data->op_mode;
	mutex_unlock(&client_data->mutex_op_mode);

	if (BMM_VAL_NAME(SUSPEND_MODE) == op_mode)
		return err;

	PINFO("app did not close this sensor before suspend");

	mutex_lock(&client_data->mutex_odr);
	BMM_CALL_API(set_datarate)(client_data->odr);
	bmm_delay(BMM_I2C_WRITE_DELAY_TIME);
	mutex_unlock(&client_data->mutex_odr);

	mutex_lock(&client_data->mutex_rept_xy);
	err = bmm_i2c_write(client, BMM_REG_NAME(NO_REPETITIONS_XY),
			&client_data->rept_xy, 1);
	bmm_delay(BMM_I2C_WRITE_DELAY_TIME);
	err = bmm_i2c_read(client, BMM_REG_NAME(NO_REPETITIONS_XY), &value, 1);
	PINFO("BMM_NO_REPETITIONS_XY: %02x", value);
	mutex_unlock(&client_data->mutex_rept_xy);

	mutex_lock(&client_data->mutex_rept_z);
	err = bmm_i2c_write(client, BMM_REG_NAME(NO_REPETITIONS_Z),
			&client_data->rept_z, 1);
	bmm_delay(BMM_I2C_WRITE_DELAY_TIME);
	err = bmm_i2c_read(client, BMM_REG_NAME(NO_REPETITIONS_Z), &value, 1);
	PINFO("BMM_NO_REPETITIONS_Z: %02x", value);
	mutex_unlock(&client_data->mutex_rept_z);

	mutex_lock(&client_data->mutex_op_mode);
	if (BMM_OP_MODE_UNKNOWN == client_data->op_mode) {
		bmm_set_forced_mode(client);
		PINFO("set forced mode after hw_restore");
		bmm_delay(bmm_get_forced_drdy_time(client_data->rept_xy,
					client_data->rept_z));
	}
	mutex_unlock(&client_data->mutex_op_mode);


	PINFO("register dump after init");
	bmm_dump_reg(client);

	return err;
}
#if 0
static int bmm050_pinctrl_init(struct i2c_client *client)
{
	int retval;
	struct bmm_client_data *data = i2c_get_clientdata(client);

	/* Get pinctrl if target uses pinctrl */
	data->ts_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->ts_pinctrl)) {
		printk(KERN_INFO "Target does not use pinctrl\n");
		retval = PTR_ERR(data->ts_pinctrl);
		data->ts_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_active
		= pinctrl_lookup_state(data->ts_pinctrl, "bmm050_active");
	if (IS_ERR_OR_NULL(data->gpio_state_active)) {
		printk(KERN_INFO "Can not get ts default pinstate\n");
		retval = PTR_ERR(data->gpio_state_active);
		data->ts_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_suspend
		= pinctrl_lookup_state(data->ts_pinctrl, "bmm050_suspend");
	if (IS_ERR_OR_NULL(data->gpio_state_suspend)) {
		printk(KERN_INFO "Can not get ts sleep pinstate\n");
		retval = PTR_ERR(data->gpio_state_suspend);
		data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int bmm050_pinctrl_select(struct bmm_client_data *data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? data->gpio_state_active
		: data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(data->ts_pinctrl, pins_state);
		if (ret) {
			printk(KERN_INFO "can not set %s pins\n",
				on ? "bmm050_active" : "bmm050_suspend");
			return ret;
		}
	} else
		printk(KERN_INFO "not a valid '%s' pinstate\n",
				on ? "bmm050_active" : "bmm050_suspend");

	return 0;
}
#endif
void bmm050_config_gpio(struct i2c_client *client)
{
	int ret;
	u32 temp_val;
	int rc;
	struct device_node *np = client->dev.of_node;
	
	bmm050_platform.mux_vcc = regulator_get(&client->dev, "mux_vcc");
	if(IS_ERR(bmm050_platform.mux_vcc))
		printk(KERN_INFO "[E-compass]regulator_get mux_vcc fail\n");

	bmm050_platform.vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
	if(IS_ERR(bmm050_platform.vcc_i2c))
		printk(KERN_INFO "[E-compass]regulator_get vcc_i2c fail\n");

	ret = regulator_enable(bmm050_platform.mux_vcc);
	if (ret)
		printk(KERN_INFO "[E-compass]regulator_enable mux_vcc fail\n");

	ret = regulator_enable(bmm050_platform.vcc_i2c);
	if (ret)
		printk(KERN_INFO "[E-compass]regulator_enable vcc_i2c fail\n");
#if 0
	err = bmm050_pinctrl_init(client);
	if (!err && data->ts_pinctrl) {
		err = bmm050_pinctrl_select(data, true);
		if (err < 0)
			PERR("[Sensor]bmm050_pinctrl_select fail\n");
	}
#endif
	bmm050_platform.name = "bmm050";
	//bmm050_platform.place = 8;
	rc = of_property_read_u32(np, "bosch,place", &temp_val);
	if (rc && (rc != -EINVAL)) {
		PERR("%s:Unable to read sensor place paramater\n",__func__);
	}
	if (temp_val > 7 || temp_val < 0) {
		PERR("%s:Invalid place parameter, use default value 0\n",__func__);
		bmm050_platform.place = 8;
	} else {
		bmm050_platform.place = temp_val;
	}
	PERR( "bmm050_platform.place = %d\n",bmm050_platform.place);
#if 0
	bmm050_platform.irq = of_get_named_gpio_flags(np, "bosch,irq-gpio", 0, NULL);
	err = gpio_request(bmm050_platform.irq, "bmm050_irq");
	if (err) {
		PERR("%s: Failed to get gpio %d (code: %d)",
				__func__, bmm050_platform.irq, err);
	}
	err = gpio_direction_input(bmm050_platform.irq);
	if (err) {
		PERR("%s: Failed to set gpio %d direction",
				__func__, bmm050_platform.irq);
	}
#endif
}

void bmm050_gpio_free(struct i2c_client *client)
{
	int ret;
	ret = regulator_disable(bmm050_platform.vcc_i2c);
	if (ret)
		printk(KERN_INFO "[E-compass]regulator_disable vcc_i2c fail\n");
	regulator_put(bmm050_platform.vcc_i2c);

	ret = regulator_disable(bmm050_platform.mux_vcc);
	if (ret)
		printk(KERN_INFO "[E-compass]regulator_disable mux_vcc fail\n");
	regulator_put(bmm050_platform.mux_vcc);
}

static void  bmm050_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmm_client_data *client_data = i2c_get_clientdata(client);
	int err=0;
	mutex_lock(&client_data->mutex_enable);
	if (enable != client_data->enable) {
		if (enable) {
			err = bmm_set_op_mode(client_data,BMM050_NORMAL_MODE);
			schedule_delayed_work(
					&client_data->work,
					msecs_to_jiffies(atomic_read(
							&client_data->delay)));
		} else {
			cancel_delayed_work_sync(&client_data->work);
			err = bmm_set_op_mode(client_data,BMM050_SUSPEND_MODE);
		}
		client_data->op_mode = bmm_get_op_mode(client_data);
		client_data->enable = enable;
	}
	mutex_unlock(&client_data->mutex_enable);
}

static int bmm050_cdev_enable(struct sensors_classdev *sensors_cdev,
				unsigned int enable)
{
	struct bmm_client_data *data = container_of(sensors_cdev,
					struct bmm_client_data, cdev);

	bmm050_set_enable(&data->client->dev, enable);
	return 0;
}

static int bmm050_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_ms)
{
	struct bmm_client_data *data = container_of(sensors_cdev,
					struct bmm_client_data, cdev);

	/*if (delay_ms < POLL_INTERVAL_MIN_MS)
		delay_ms = POLL_INTERVAL_MIN_MS;
	if (delay_ms > POLL_INTERVAL_MAX_MS)
		delay_ms = POLL_INTERVAL_MAX_MS;*/
	atomic_set(&data->delay, (unsigned int) delay_ms);

	return 0;
}

static struct workqueue_struct *reportdata_wq;
static void bmm050_work_fun(struct work_struct *work)
{
	struct  bmm_client_data *client_data =
		container_of(work,
				struct bmm_client_data, report_data_work);
	struct i2c_client *client = client_data->client;
	uint64_t time = 0;
	struct timespec ts;
	int err = 0;

	err = BMM_CALL_API(read_mdataXYZ_s32)(&client_data->value);
	if(err != 0)
		printk("BBox::UEC; 8::69\n");
	time = bmm_get_alarm_timestamp();
	ts = ns_to_timespec(time);
	bmm_remap_sensor_data(&client_data->value, client_data);
	input_event(client_data->input, EV_MSC, MSC_TIME,
	ts.tv_sec);
	input_event(client_data->input, EV_MSC, MSC_TIME,
	ts.tv_nsec);
	input_event(client_data->input, EV_MSC,
		MSC_GESTURE, client_data->value.datax);
	input_event(client_data->input, EV_MSC,
		MSC_RAW, client_data->value.datay);
	input_event(client_data->input, EV_MSC,
		MSC_SCAN, client_data->value.dataz);
	input_sync(client_data->input);


	bmm_set_forced_mode(client);

}

static enum hrtimer_restart reportdata_timer_fun(
	struct hrtimer *hrtimer)
{
	struct bmm_client_data *client_data =
		container_of(hrtimer, struct bmm_client_data, timer);
	int32_t delay = 0;
	delay = client_data->timer_read;
	queue_work(reportdata_wq, &(client_data->report_data_work));
	client_data->work_delay_kt = ns_to_ktime(delay*1000000);
	hrtimer_forward(hrtimer, ktime_get(), client_data->work_delay_kt);

	return HRTIMER_RESTART;
}

static int bmm_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct bmm_client_data *client_data = NULL;
	int dummy;
	struct bosch_sensor_specific *pdata;
	PINFO("function entrance");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		err = -EIO;
		printk("BBox::UEC; 8::26\n");
		goto exit_err_clean;
	}

	if (NULL == bmm_client) {
		bmm_client = client;
	} else {
		PERR("this driver does not support multiple clients");
		err = -EBUSY;
		printk("BBox::UEC; 8::26\n");
		return err;
	}

	client_data = kzalloc(sizeof(struct bmm_client_data), GFP_KERNEL);
	if (NULL == client_data) {
		PERR("no memory available");
		err = -ENOMEM;
		printk("BBox::UEC; 8::26\n");
		goto exit_err_clean;
	}

	i2c_set_clientdata(client, client_data);
	client_data->client = client;

	bmm050_config_gpio(client);
	pdata = &bmm050_platform;
	/* wake up the chip */
	dummy = bmm_wakeup(client);
	if (dummy < 0) {
		PERR("Cannot wake up %s, I2C xfer error", SENSOR_NAME);
		err = -EIO;
		printk("BBox::UEC; 8::26\n");
		goto exit_err_clean;
	}

	PINFO("register dump after waking up");
	bmm_dump_reg(client);
	/* check chip id */
	err = bmm_check_chip_id(client);
	if (!err) {
		PNOTICE("Bosch Sensortec Device %s detected, i2c_addr: %#x",
				SENSOR_NAME, client->addr);
	} else {
		PERR("Bosch Sensortec Device not found, chip id mismatch");
		err = -1;
		printk("BBox::UEC; 8::26\n");
		goto exit_err_clean;
	}

	mutex_init(&client_data->mutex_power_mode);
	mutex_init(&client_data->mutex_op_mode);
	mutex_init(&client_data->mutex_enable);
	mutex_init(&client_data->mutex_odr);
	mutex_init(&client_data->mutex_rept_xy);
	mutex_init(&client_data->mutex_rept_z);
	mutex_init(&client_data->mutex_value);

	/* input device init */
	err = bmm_input_init(client_data);
	if (err < 0) {
		printk("BBox::UEC; 8::26\n");
		goto exit_err_clean;
	}

	/* sysfs node creation */
	err = sysfs_create_group(&client_data->input->dev.kobj,
			&bmm_attribute_group);

	if (err < 0) {
		printk("BBox::UEC; 8::26\n");
		goto exit_err_sysfs;
	}

#ifdef CONFIG_BMM_USE_PLATFORM_DATA
	if (NULL != client->dev.platform_data) {
		client_data->bst_pd = kzalloc(sizeof(*client_data->bst_pd),
				GFP_KERNEL);

		if (NULL != client_data->bst_pd) {
			memcpy(client_data->bst_pd, client->dev.platform_data,
					sizeof(*client_data->bst_pd));
			PINFO("platform data of bmm %s: place: %d, irq: %d",
					client_data->bst_pd->name,
					client_data->bst_pd->place,
					client_data->bst_pd->irq);
		}
	}
#endif

	/* workqueue init */
	INIT_DELAYED_WORK(&client_data->work, bmm_work_func);
	atomic_set(&client_data->delay, BMM_DELAY_DEFAULT);

	/* h/w init */
	client_data->device.bus_read = bmm_i2c_read_wrapper;
	client_data->device.bus_write = bmm_i2c_write_wrapper;
	client_data->device.delay_msec = bmm_delay;
	BMM_CALL_API(init)(&client_data->device);

	bmm_dump_reg(client);

	PDEBUG("trimming_reg x1: %d y1: %d x2: %d y2: %d xy1: %d xy2: %d",
			client_data->device.dig_x1,
			client_data->device.dig_y1,
			client_data->device.dig_x2,
			client_data->device.dig_y2,
			client_data->device.dig_xy1,
			client_data->device.dig_xy2);

	PDEBUG("trimming_reg z1: %d z2: %d z3: %d z4: %d xyz1: %d",
			client_data->device.dig_z1,
			client_data->device.dig_z2,
			client_data->device.dig_z3,
			client_data->device.dig_z4,
			client_data->device.dig_xyz1);

	client_data->enable = 0;
	/* now it's power on which is considered as resuming from suspend */
	client_data->op_mode = BMM_VAL_NAME(SUSPEND_MODE);
	client_data->odr = BMM_DEFAULT_ODR;
	client_data->sampling_frequency =
		odr_map[client_data->odr];
	client_data->rept_xy = BMM_DEFAULT_REPETITION_XY;
	client_data->rept_z = BMM_DEFAULT_REPETITION_Z;

	err = bmm_set_op_mode(client_data, BMM_VAL_NAME(SUSPEND_MODE));
	if (err) {
		PERR("fail to init h/w of %s", SENSOR_NAME);
		err = -EIO;
		printk("BBox::UEC; 8::26\n");
		goto exit_err_sysfs;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	client_data->early_suspend_handler.level =
		EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	client_data->early_suspend_handler.suspend = bmm_early_suspend;
	client_data->early_suspend_handler.resume = bmm_late_resume;
	register_early_suspend(&client_data->early_suspend_handler);
#endif
	INIT_WORK(&client_data->report_data_work,
	bmm050_work_fun);
	reportdata_wq = create_singlethread_workqueue("bmm050_wq");
	if (NULL == reportdata_wq)
		PERR("fail to create the reportdta_wq %d", -ENOMEM);
	hrtimer_init(&client_data->timer, CLOCK_MONOTONIC,
		HRTIMER_MODE_REL);
	client_data->timer.function = reportdata_timer_fun;
	client_data->work_delay_kt = ns_to_ktime(20000000);
	client_data->is_timer_running = 0;

	client_data->cdev = sensors_cdev;
	client_data->cdev.delay_msec = 100;
	client_data->cdev.sensors_enable = bmm050_cdev_enable;
	client_data->cdev.sensors_poll_delay = bmm050_cdev_poll_delay;

	err = sensors_classdev_register(&client_data->input->dev, &client_data->cdev);
	if (err) {
		dev_err(&client->dev, "create class device file failed!\n");
		err = -EINVAL;
		printk("BBox::UEC; 8::26\n");
		goto exit_err_sysfs;
	}
	fih_ecompass_proc_init();
	PNOTICE("sensor %s probed successfully", SENSOR_NAME);

	PDEBUG("i2c_client: %p client_data: %p i2c_device: %p input: %p",
			client, client_data, &client->dev, client_data->input);

	return 0;

exit_err_sysfs:
	if (err)
		bmm_input_destroy(client_data);

exit_err_clean:
	bmm050_gpio_free(client);
	if (err) {
		if (client_data != NULL) {
#ifdef CONFIG_BMM_USE_PLATFORM_DATA
			if (NULL != client_data->bst_pd) {
				kfree(client_data->bst_pd);
				client_data->bst_pd = NULL;
			}
#endif
			kfree(client_data);
			client_data = NULL;
		}

		bmm_client = NULL;
	}

	printk(KERN_INFO "[E-compass]%s Exit\n", __func__);
	return err;
}

static int bmm_pre_suspend(struct i2c_client *client)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);
	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		cancel_delayed_work_sync(&client_data->work);
		PDEBUG("cancel work");
	}
	if (client_data->is_timer_running)
		hrtimer_cancel(&client_data->timer);
	mutex_unlock(&client_data->mutex_enable);

	return err;
}

static int bmm_post_resume(struct i2c_client *client)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		schedule_delayed_work(&client_data->work,
				msecs_to_jiffies(
					atomic_read(&client_data->delay)));
	}
	if (client_data->is_timer_running)
		hrtimer_start(&client_data->timer,
					ns_to_ktime(1000000),
			HRTIMER_MODE_REL);
	mutex_unlock(&client_data->mutex_enable);

	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmm_early_suspend(struct early_suspend *handler)
{
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)container_of(handler,
			struct bmm_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;
	u8 power_mode;
	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		bmm_pre_suspend(client);
		bmm_set_op_mode(client_data, BMM_VAL_NAME(SUSPEND_MODE));
	}
	mutex_unlock(&client_data->mutex_power_mode);

}

static void bmm_late_resume(struct early_suspend *handler)
{
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)container_of(handler,
			struct bmm_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;
	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_power_mode);

	bmm_restore_hw_cfg(client);
	/* post resume operation */
	bmm_post_resume(client);

	mutex_unlock(&client_data->mutex_power_mode);
}
#else
static int bmm_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);
	u8 power_mode;

	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		err = bmm_pre_suspend(client);
		err = bmm_set_op_mode(client_data, BMM_VAL_NAME(SUSPEND_MODE));
	}

	mutex_unlock(&client_data->mutex_power_mode);

	return err;
}

static int bmm_resume(struct i2c_client *client)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);

	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_power_mode);
	err = bmm_restore_hw_cfg(client);
	/* post resume operation */
	bmm_post_resume(client);
	mutex_unlock(&client_data->mutex_power_mode);

	return err;
}
#endif

void bmm_shutdown(struct i2c_client *client)
{
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);
	mutex_lock(&client_data->mutex_power_mode);
	bmm_set_op_mode(client_data, BMM_VAL_NAME(SUSPEND_MODE));
	mutex_unlock(&client_data->mutex_power_mode);
}

static int bmm_remove(struct i2c_client *client)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);

	if (NULL != client_data) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&client_data->early_suspend_handler);
#endif

			mutex_lock(&client_data->mutex_op_mode);
			if (BMM_VAL_NAME(NORMAL_MODE) == client_data->op_mode) {
				cancel_delayed_work_sync(&client_data->work);
				PDEBUG("cancel work");
			}
			mutex_unlock(&client_data->mutex_op_mode);
		if (client_data->is_timer_running)
			hrtimer_cancel(&client_data->timer);
			err = bmm_set_op_mode(client_data, BMM_VAL_NAME(SUSPEND_MODE));
		bmm_delay(BMM_I2C_WRITE_DELAY_TIME);

			sysfs_remove_group(&client_data->input->dev.kobj,
					&bmm_attribute_group);
			bmm_input_destroy(client_data);

	#ifdef CONFIG_BMM_USE_PLATFORM_DATA
				if (NULL != client_data->bst_pd) {
					kfree(client_data->bst_pd);
					client_data->bst_pd = NULL;
				}
	#endif
			kfree(client_data);

			bmm_client = NULL;
		}
		bmm050_gpio_free(client);

	return err;
}

static const struct i2c_device_id bmm_id[] = {
	{SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, bmm_id);

static struct of_device_id bmm050_match_table[] = {
	{ .compatible = "bosch,bmm050",},
	{ },
};

static struct i2c_driver bmm_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
		.of_match_table = bmm050_match_table,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = bmm_id,
	.probe = bmm_probe,
	.remove = bmm_remove,
	.shutdown = bmm_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = bmm_suspend,
	.resume = bmm_resume,
#endif
};

static int __init BMM_init(void)
{
	return i2c_add_driver(&bmm_driver);
}

static void __exit BMM_exit(void)
{
	i2c_del_driver(&bmm_driver);
}

MODULE_AUTHOR("contact@bosch.sensortec.com");
MODULE_DESCRIPTION("BMM MAGNETIC SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

module_init(BMM_init);
module_exit(BMM_exit);
