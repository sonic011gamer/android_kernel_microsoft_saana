/*!
 * @section LICENSE
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bmg160_driver.c
 * @date     2015/11/17 13:44
 * @id       "836294d"
 * @version  1.5.9
 *
 * @brief    BMG160 Linux Driver
 */
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
#include <linux/math64.h>
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
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/platform_device.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "bmg160.h"
#include "bs_log.h"

/* sensor specific */
#define SENSOR_NAME "bmg160"

#define SENSOR_CHIP_ID_BMG (0x0f)
#define CHECK_CHIP_ID_TIME_MAX   5
#define DRIVER_VERSION "0.0.28.0"
#define BMG_USE_FIFO          1
#define BMG_USE_BASIC_I2C_FUNC     1
#define BMG_REG_NAME(name) BMG160_##name
#define BMG_VAL_NAME(name) BMG160_##name
#define BMG_CALL_API(name) bmg160_##name
#define MSC_TIME                6

#define BMG_I2C_WRITE_DELAY_TIME 1

/* generic */
#define BMG_MAX_RETRY_I2C_XFER (100)
#define BMG_MAX_RETRY_WAKEUP (5)
#define BMG_MAX_RETRY_WAIT_DRDY (100)

#define BMG_DELAY_MIN (1)
#define BMG_DELAY_DEFAULT (200)

#define BMG_VALUE_MAX (32767)
#define BMG_VALUE_MIN (-32768)

#define BYTES_PER_LINE (16)

#define BMG_SELF_TEST 0

#define BMG_SOFT_RESET_VALUE                0xB6


#ifdef BMG_USE_FIFO
#define MAX_FIFO_F_LEVEL 100
#define MAX_FIFO_F_BYTES 8
#define BMG160_FIFO_DAT_SEL_X                     1
#define BMG160_FIFO_DAT_SEL_Y                     2
#define BMG160_FIFO_DAT_SEL_Z                     3
#endif

/*!
 * @brief:BMI058 feature
 *  macro definition
*/
#ifdef CONFIG_SENSORS_BMI058
/*! BMI058 X AXIS definition*/
#define BMI058_X_AXIS	BMG160_Y_AXIS
/*! BMI058 Y AXIS definition*/
#define BMI058_Y_AXIS	BMG160_X_AXIS

#define C_BMI058_One_U8X	1
#define C_BMI058_Two_U8X	2
#endif

/*! Bosch sensor unknown place*/
#define BOSCH_SENSOR_PLACE_UNKNOWN (-1)
/*! Bosch sensor remapping table size P0~P7*/
#define MAX_AXIS_REMAP_TAB_SZ 8


struct bosch_sensor_specific {
	char *name;
	/* 0 to 7 */
	int place;
	int irq;
	struct regulator *vcc_i2c;
	struct regulator *mux_vcc;
	int (*irq_gpio_cfg)(void);
};

struct bosch_sensor_specific bmg160_platform;

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

struct bmg_client_data {
	struct bmg160_t device;
	struct sensors_classdev cdev;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_handler;
#endif

	atomic_t delay;
	uint8_t debug_level;
	struct bmg160_data_t value;
	u8 enable:1;
	unsigned int fifo_count;
	unsigned char fifo_datasel;

	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	uint64_t timestamp;
	uint64_t base_time;
	uint64_t fifo_time;
	uint64_t gyro_count;
	uint64_t time_odr;
	/* controls not only reg, but also workqueue */
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;
	struct bosch_sensor_specific *bst_pd;
	struct work_struct report_data_work;
	int is_timer_running;
	struct hrtimer timer;
	ktime_t work_delay_kt;
};

static struct i2c_client *bmg_client;
/* i2c operation for API */
static int bmg_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);
static int bmg_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);

//static void bmg_dump_reg(struct i2c_client *client);
static int bmg_check_chip_id(struct i2c_client *client);

static int bmg_pre_suspend(struct i2c_client *client);
static int bmg_post_resume(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmg_early_suspend(struct early_suspend *handler);
static void bmg_late_resume(struct early_suspend *handler);
#endif

static void bmg160_delay(BMG160_U16 msec)
{
	if (msec <= 20)
		usleep_range(msec * 1000, msec * 1000);
	else
		msleep(msec);
}

/*!
* BMG160 sensor remapping function
* need to give some parameter in BSP files first.
*/
static const struct bosch_sensor_axis_remap
	bst_axis_remap_tab_dft[MAX_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,	 1,    2,	  1,	  1,	  1 }, /* P0 */
	{  1,	 0,    2,	  1,	 -1,	  1 }, /* P1 */
	{  0,	 1,    2,	 -1,	 -1,	  1 }, /* P2 */
	{  1,	 0,    2,	 -1,	  1,	  1 }, /* P3 */

	{  0,	 1,    2,	 -1,	  1,	 -1 }, /* P4 */
	{  1,	 0,    2,	 -1,	 -1,	 -1 }, /* P5 */
	{  0,	 1,    2,	  1,	 -1,	 -1 }, /* P6 */
	{  1,	 0,    2,	  1,	  1,	 -1 }, /* P7 */
};
static struct sensors_classdev sensors_cdev = {
		.name = "bmg160-gyro",
		.vendor = "bosch",
		.version = 1,
		.handle = SENSORS_GYROSCOPE_HANDLE,
		.type = SENSOR_TYPE_GYROSCOPE,
		.max_range = "156.8",	/* 16g */
		.resolution = "0.153125",	/* 15.6mg */
		.sensor_power = "0.13",	/* typical value */
    .min_delay = 10000,
    .max_delay = 200000,
		.fifo_reserved_event_count = 0,
		.fifo_max_event_count = 0,
		.enabled = 0,
		.delay_msec = 200, /* in millisecond */
		.sensors_enable = NULL,
		.sensors_poll_delay = NULL,
		.sensors_self_test = NULL,
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

static void bst_remap_sensor_data_dft_tab(struct bosch_sensor_data *data,
			int place)
{
/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= MAX_AXIS_REMAP_TAB_SZ))
		return;
	bst_remap_sensor_data(data, &bst_axis_remap_tab_dft[place]);
}

static void bmg160_remap_sensor_data(struct bmg160_data_t *val,
		struct bmg_client_data *client_data)
{
	struct bosch_sensor_data bsd;
	int place;

	if ((NULL == client_data->bst_pd) || (BOSCH_SENSOR_PLACE_UNKNOWN
			 == client_data->bst_pd->place))
		place = bmg160_platform.place;
	else
		place = client_data->bst_pd->place;

#ifdef CONFIG_SENSORS_BMI058
/*x,y need to be invesed becase of HW Register for BMI058*/
	bsd.y = val->datax;
	bsd.x = val->datay;
	bsd.z = val->dataz;
#else
	bsd.x = val->datax;
	bsd.y = val->datay;
	bsd.z = val->dataz;
#endif

	bst_remap_sensor_data_dft_tab(&bsd, place);

	val->datax = bsd.x;
	val->datay = bsd.y;
	val->dataz = bsd.z;

}

static int bmg_check_chip_id(struct i2c_client *client)
{
	int err = -1;
	u8 chip_id = 0;
	u8 read_count = 0;

	while (read_count++ < CHECK_CHIP_ID_TIME_MAX) {
		bmg_i2c_read(client, BMG_REG_NAME(CHIP_ID_ADDR), &chip_id, 1);
		PINFO("read chip id result: %#x", chip_id);

		if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMG) {
			bmg160_delay(1);
		} else {
			err = 0;
			break;
		}
	}
	return err;
}


/*static void bmg_dump_reg(struct i2c_client *client)
{
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&client->dev, "%s\n", dbg_buf_str);

	bmg_i2c_read(client, BMG_REG_NAME(CHIP_ID_ADDR), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&client->dev, "%s\n", dbg_buf_str);
}*/

/*i2c read routine for API*/
static int bmg_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMG_USE_BASIC_I2C_FUNC
	s32 dummy;
	if (NULL == client)
		return -ENODEV;

	while (0 != len--) {
#ifdef BMG_SMBUS
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			dev_err(&client->dev, "i2c bus read error");
			return -EIO;
		}
		*data = (u8)(dummy & 0xff);
#else
		dummy = i2c_master_send(client, (char *)&reg_addr, 1);
		if (dummy < 0)
			return -EIO;

		dummy = i2c_master_recv(client, (char *)data, 1);
		if (dummy < 0)
			return -EIO;
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

	for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			bmg160_delay(BMG_I2C_WRITE_DELAY_TIME);
	}

	if (BMG_MAX_RETRY_I2C_XFER <= retry) {
		dev_err(&client->dev, "I2C xfer error");
		return -EIO;
	}

	return 0;
#endif
}

#ifdef BMG_USE_FIFO
static int bmg_i2c_burst_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u16 len)
{
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

	for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			bmg160_delay(BMG_I2C_WRITE_DELAY_TIME);
	}

	if (BMG_MAX_RETRY_I2C_XFER <= retry) {
		dev_err(&client->dev, "I2C xfer error");
		return -EIO;
	}

	return 0;
}
#endif

/*i2c write routine for */
static int bmg_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMG_USE_BASIC_I2C_FUNC
	s32 dummy;

#ifndef BMG_SMBUS
	u8 buffer[2];
#endif

	if (NULL == client)
		return -ENODEV;

	while (0 != len--) {
#ifdef BMG_SMBUS
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(client, (char *)buffer, 2);
#endif
		reg_addr++;
		data++;
		if (dummy < 0) {
			dev_err(&client->dev, "error writing i2c bus");
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
		for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, msg,
						ARRAY_SIZE(msg)) > 0) {
				break;
			} else {
				bmg160_delay(BMG_I2C_WRITE_DELAY_TIME);
			}
		}
		if (BMG_MAX_RETRY_I2C_XFER <= retry) {
			dev_err(&client->dev, "I2C xfer error");
			return -EIO;
		}
		reg_addr++;
		data++;
	}

	return 0;
#endif
}

static int bmg_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err;
	err = bmg_i2c_read(bmg_client, reg_addr, data, len);
	return err;
}

static int bmg_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err;
	err = bmg_i2c_write(bmg_client, reg_addr, data, len);
	return err;
}


static void bmg_work_func(struct work_struct *work)
{
	struct bmg_client_data *client_data =
		container_of((struct delayed_work *)work,
			struct bmg_client_data, work);

	unsigned long delay =
		msecs_to_jiffies(atomic_read(&client_data->delay));
	struct bmg160_data_t gyro_data;

	BMG_CALL_API(get_dataXYZ)(&gyro_data);
	/*remapping for BMG160 sensor*/
	bmg160_remap_sensor_data(&gyro_data, client_data);

	input_report_abs(client_data->input, ABS_X, gyro_data.datax);
	input_report_abs(client_data->input, ABS_Y, gyro_data.datay);
	input_report_abs(client_data->input, ABS_Z, gyro_data.dataz);
	input_sync(client_data->input);

	schedule_delayed_work(&client_data->work, delay);
}

static struct workqueue_struct *reportdata_wq;

uint64_t bmg160_get_alarm_timestamp(void)
{
	uint64_t ts_ap;
	struct timespec tmp_time;
	get_monotonic_boottime(&tmp_time);
	ts_ap = (uint64_t)tmp_time.tv_sec * 1000000000 + tmp_time.tv_nsec;
	return ts_ap;
}
#define ABS(x) ((x) > 0 ? (x) : -(x))

static void bmg160_work_func(struct work_struct *work)
{
	struct	bmg_client_data *bmg160 =
		container_of(work,
				struct bmg_client_data, report_data_work);
	int i;
	struct bmg160_data_t gyro_lsb;
	unsigned char fifo_framecount;
	signed char fifo_data_out[MAX_FIFO_F_LEVEL * MAX_FIFO_F_BYTES] = {0};
	unsigned char f_len = 0;
	uint64_t del;
	uint64_t time_internal;
	struct timespec ts;
	int64_t drift_time = 0;
	static uint64_t time_odr;
	static uint32_t data_cnt;
	static uint32_t pre_data_cnt;
	static int64_t sample_drift_offset;
	if (bmg160->fifo_datasel)
		/*Select one axis data output for every fifo frame*/
		f_len = 2;
	else
		/*Select X Y Z axis data output for every fifo frame*/
		f_len = 6;
	if (BMG_CALL_API(get_fifo_framecount)(&fifo_framecount) < 0) {
		PERR("bm160_get_fifo_framecount err\n");
		return;
	}
	if (fifo_framecount == 0)
		return;
	if (fifo_framecount > MAX_FIFO_F_LEVEL)
			fifo_framecount = MAX_FIFO_F_LEVEL;
	if (bmg_i2c_burst_read(bmg160->client, BMG160_FIFO_DATA_ADDR,
			fifo_data_out, fifo_framecount * f_len) < 0) {
			PERR("bmg160 read fifo err\n");
			return;
	}
	bmg160->fifo_time = bmg160_get_alarm_timestamp();
	if (bmg160->gyro_count == 0)
		bmg160->base_time = bmg160->timestamp =
		bmg160->fifo_time - (fifo_framecount-1) * bmg160->time_odr;

	bmg160->gyro_count += fifo_framecount;
	del = bmg160->fifo_time - bmg160->base_time;
	time_internal = div64_u64(del, bmg160->gyro_count);
	data_cnt++;
	if (data_cnt == 1)
		time_odr = bmg160->time_odr;
	if (time_internal > time_odr) {
		if (time_internal - time_odr > div64_u64 (time_odr, 200))
			time_internal = time_odr + div64_u64(time_odr, 200);
	} else {
		if (time_odr - time_internal > div64_u64(time_odr, 200))
			time_internal = time_odr - div64_u64(time_odr, 200);
	}

	/* Select X Y Z axis data output for every frame */
	for (i = 0; i < fifo_framecount; i++) {
		if (bmg160->debug_level & 0x01)
			printk(KERN_INFO "bmg time =%llu fifo_time = %llu time_internal = %llu bmg->count= %llu count = %d",
		bmg160->timestamp, bmg160->fifo_time,
		time_internal, bmg160->gyro_count, fifo_framecount);
		ts = ns_to_timespec(bmg160->timestamp);
		gyro_lsb.datax =
		((unsigned char)fifo_data_out[i * f_len + 1] << 8
				| (unsigned char)fifo_data_out[i * f_len + 0]);
		gyro_lsb.datay =
		((unsigned char)fifo_data_out[i * f_len + 3] << 8
				| (unsigned char)fifo_data_out[i * f_len + 2]);
		gyro_lsb.dataz =
		((unsigned char)fifo_data_out[i * f_len + 5] << 8
				| (unsigned char)fifo_data_out[i * f_len + 4]);
		bmg160_remap_sensor_data(&gyro_lsb, bmg160);
		input_event(bmg160->input, EV_MSC, MSC_TIME,
		ts.tv_sec);
		input_event(bmg160->input, EV_MSC, MSC_TIME,
		ts.tv_nsec);
		input_event(bmg160->input, EV_MSC,
			MSC_GESTURE, gyro_lsb.datax);
		input_event(bmg160->input, EV_MSC,
			MSC_RAW, gyro_lsb.datay);
		input_event(bmg160->input, EV_MSC,
			MSC_SCAN, gyro_lsb.dataz);
		input_sync(bmg160->input);
		bmg160->timestamp += time_internal - sample_drift_offset;
	}
	drift_time = bmg160->timestamp - bmg160->fifo_time;
	if (data_cnt % 20 == 0) {
		if (ABS(drift_time) > div64_u64(time_odr, 5)) {
			sample_drift_offset =
		div64_s64(drift_time, bmg160->gyro_count - pre_data_cnt);
			pre_data_cnt = bmg160->gyro_count;
			time_odr = time_internal;
		}
	}
}


static enum hrtimer_restart reportdata_timer_fun(
	struct hrtimer *hrtimer)
{
	struct bmg_client_data *client_data =
		container_of(hrtimer, struct bmg_client_data, timer);
	int32_t delay = 0;
	delay = 10;
	queue_work(reportdata_wq, &(client_data->report_data_work));
	client_data->work_delay_kt = ns_to_ktime(delay*1000000);
	hrtimer_forward(hrtimer, ktime_get(), client_data->work_delay_kt);

	return HRTIMER_RESTART;
}

static ssize_t bmg_show_enable_timer(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, 16, "%d\n", client_data->is_timer_running);
}

static ssize_t bmg_store_enable_timer(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (data) {
		if (0 == client_data->is_timer_running) {
			hrtimer_start(&client_data->timer,
			ns_to_ktime(10000000),
			HRTIMER_MODE_REL);
		client_data->is_timer_running = 1;
		client_data->base_time = 0;
		client_data->timestamp = 0;
		client_data->gyro_count = 0;
	}
	} else {
		if (1 == client_data->is_timer_running) {
			hrtimer_cancel(&client_data->timer);
			client_data->is_timer_running = 0;
			client_data->base_time = 0;
			client_data->timestamp = 0;
			client_data->gyro_count = 0;
	}
	}
	return count;
}

static ssize_t bmg160_show_debug_level(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	err = snprintf(buf, 8, "%d\n", client_data->debug_level);
	return err;
}
static ssize_t bmg160_store_debug_level(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int32_t ret = 0;
	unsigned long data;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	ret = kstrtoul(buf, 16, &data);
	if (ret)
		return ret;
	client_data->debug_level = (uint8_t)data;
	return count;
}

static int bmg_set_soft_reset(struct i2c_client *client)
{
	int err = 0;
	unsigned char data = BMG_SOFT_RESET_VALUE;
	err = bmg_i2c_write(client, BMG160_BGW_SOFTRESET_ADDR, &data, 1);
	return err;
}

static ssize_t bmg_show_chip_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 16, "%d\n", SENSOR_CHIP_ID_BMG);
}

static ssize_t bmg_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	u8 op_mode = 0xff;

	mutex_lock(&client_data->mutex_op_mode);
	BMG_CALL_API(get_mode)(&op_mode);
	mutex_unlock(&client_data->mutex_op_mode);

	ret = snprintf(buf, 16, "%d\n", op_mode);

	return ret;
}

static ssize_t bmg_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	long op_mode;

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;

	mutex_lock(&client_data->mutex_op_mode);

	err = BMG_CALL_API(set_mode)(op_mode);

	mutex_unlock(&client_data->mutex_op_mode);

	if (err)
		return err;
	else
		return count;
}



static ssize_t bmg_show_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int count;

	struct bmg160_data_t value_data;
	BMG_CALL_API(get_dataXYZ)(&value_data);
	/*BMG160 sensor raw data remapping*/
	bmg160_remap_sensor_data(&value_data, client_data);

	count = snprintf(buf, 96, "%hd %hd %hd\n",
				value_data.datax,
				value_data.datay,
				value_data.dataz);

	return count;
}

static ssize_t bmg_show_range(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char range = 0;
	BMG_CALL_API(get_range_reg)(&range);
	err = snprintf(buf, 16, "%d\n", range);
	return err;
}

static ssize_t bmg_store_range(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long range;
	err = kstrtoul(buf, 10, &range);
	if (err)
		return err;
	BMG_CALL_API(set_range_reg)(range);
	return count;
}

/*
decimation    odr     filter bandwidth     bits
20	100HZ		32HZ		7
10	200Hz		64HZ		6
20	100HZ		12HZ		5
10	200hz		23HZ		4
5	400HZ		47HZ		3
2	1000HZ		116HZ		2
0	2000HZ		230HZ		1
0	2000HZ		Unfiltered(523HZ)	0
*/

static const uint64_t odr_map[8] = {
500000, 500000, 1000000, 2500000, 5000000, 10000000, 5000000, 10000000};

static ssize_t bmg_show_bandwidth(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char bandwidth = 0;
	BMG_CALL_API(get_bw)(&bandwidth);
	err = snprintf(buf, 16, "%d\n", bandwidth);
	return err;
}

static ssize_t bmg_store_bandwidth(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	unsigned long bandwidth;
	u8 op_mode = 0xff;
	err = kstrtoul(buf, 10, &bandwidth);
	if (err)
		return err;
	/*
	set bandwidth only in the op_mode=0
	*/
	err = BMG_CALL_API(get_mode)(&op_mode);
	if (op_mode == 0) {
		err += BMG_CALL_API(set_bw)(bandwidth);
	} else {
		err += BMG_CALL_API(set_mode)(0);
		err += BMG_CALL_API(set_bw)(bandwidth);
		bmg160_delay(1);
		err += BMG_CALL_API(set_mode)(2);
		bmg160_delay(3);
	}

	if (err)
		PERR("set failed");
	client_data->time_odr = odr_map[bandwidth];
	client_data->base_time = 0;
	client_data->gyro_count = 0;
	return count;
}


static ssize_t bmg_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int err;

	mutex_lock(&client_data->mutex_enable);
	err = snprintf(buf, 16, "%d\n", client_data->enable);
	mutex_unlock(&client_data->mutex_enable);
	return err;
}

static ssize_t bmg_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

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

static ssize_t bmg_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, 16, "%d\n", atomic_read(&client_data->delay));

}

static ssize_t bmg_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	if (data == 0) {
		err = -EINVAL;
		return err;
	}

	if (data < BMG_DELAY_MIN)
		data = BMG_DELAY_MIN;

	atomic_set(&client_data->delay, data);

	return count;
}


static ssize_t bmg_store_fastoffset_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long fastoffset_en;
	err = kstrtoul(buf, 10, &fastoffset_en);
	if (err)
		return err;
	if (fastoffset_en) {

#ifdef CONFIG_SENSORS_BMI058
		BMG_CALL_API(set_fast_offset_en_ch)(BMI058_X_AXIS, 1);
		BMG_CALL_API(set_fast_offset_en_ch)(BMI058_Y_AXIS, 1);
#else
		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_X_AXIS, 1);
		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_Y_AXIS, 1);
#endif

		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_Z_AXIS, 1);
		BMG_CALL_API(enable_fast_offset)();
	}
	return count;
}

static ssize_t bmg_store_slowoffset_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long slowoffset_en;
	err = kstrtoul(buf, 10, &slowoffset_en);
	if (err)
		return err;
	if (slowoffset_en) {
		BMG_CALL_API(set_slow_offset_th)(3);
		BMG_CALL_API(set_slow_offset_dur)(0);
#ifdef CONFIG_SENSORS_BMI058
		BMG_CALL_API(set_slow_offset_en_ch)(BMI058_X_AXIS, 1);
		BMG_CALL_API(set_slow_offset_en_ch)(BMI058_Y_AXIS, 1);
#else
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_X_AXIS, 1);
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Y_AXIS, 1);
#endif
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Z_AXIS, 1);
	} else {
#ifdef CONFIG_SENSORS_BMI058
	BMG_CALL_API(set_slow_offset_en_ch)(BMI058_X_AXIS, 0);
	BMG_CALL_API(set_slow_offset_en_ch)(BMI058_Y_AXIS, 0);
#else
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_X_AXIS, 0);
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Y_AXIS, 0);
#endif
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Z_AXIS, 0);
	}

	return count;
}

static ssize_t bmg_show_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char selftest;
	BMG_CALL_API(selftest)(&selftest);
	err = snprintf(buf, 16, "%d\n", selftest);
	return err;
}

static ssize_t bmg_show_sleepdur(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char sleepdur;
	BMG_CALL_API(get_sleepdur)(&sleepdur);
	err = snprintf(buf, 16, "%d\n", sleepdur);
	return err;
}

static ssize_t bmg_store_sleepdur(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long sleepdur;
	err = kstrtoul(buf, 10, &sleepdur);
	if (err)
		return err;
	BMG_CALL_API(set_sleepdur)(sleepdur);
	return count;
}

static ssize_t bmg_show_autosleepdur(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char autosleepdur;
	BMG_CALL_API(get_autosleepdur)(&autosleepdur);
	err = snprintf(buf, 16, "%d\n", autosleepdur);
	return err;
}

static ssize_t bmg_store_autosleepdur(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long autosleepdur;
	unsigned char bandwidth;
	err = kstrtoul(buf, 10, &autosleepdur);
	if (err)
		return err;
	BMG_CALL_API(get_bw)(&bandwidth);
	BMG_CALL_API(set_autosleepdur)(autosleepdur, bandwidth);
	return count;
}

static ssize_t bmg_show_place(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int place = bmg160_platform.place;
	if (NULL != client_data->bst_pd)
		place = client_data->bst_pd->place;

	return snprintf(buf, 16, "%d\n", place);
}


#ifdef BMG_DEBUG
static ssize_t bmg_store_softreset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long softreset;
	err = kstrtoul(buf, 10, &softreset);
	if (err)
		return err;
	BMG_CALL_API(set_soft_reset)();
	return count;
}

static ssize_t bmg_show_dumpreg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	u8 reg[0x40];
	int i;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	for (i = 0; i < 0x40; i++) {
		bmg_i2c_read(client_data->client, i, reg+i, 1);

		count += snprintf(&buf[count], 48, "0x%x: 0x%x\n", i, reg[i]);
	}
	return count;
}
#endif

#ifdef BMG_USE_FIFO
static ssize_t bmg_show_fifo_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_mode;
	BMG_CALL_API(get_fifo_mode)(&fifo_mode);
	err = snprintf(buf, 16, "%d\n", fifo_mode);
	return err;
}

static ssize_t bmg_store_fifo_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long fifo_mode;
	err = kstrtoul(buf, 10, &fifo_mode);
	if (err)
		return err;
	BMG_CALL_API(set_fifo_mode)(fifo_mode);
	return count;
}

static ssize_t bmg_show_fifo_framecount(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_framecount;
	BMG_CALL_API(get_fifo_framecount)(&fifo_framecount);
	err = snprintf(buf, 32, "%d\n", fifo_framecount);
	return err;
}

static ssize_t bmg_store_fifo_framecount(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	client_data->fifo_count = (unsigned int) data;

	return count;
}

static ssize_t bmg_show_fifo_overrun(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_overrun;
	BMG_CALL_API(get_fifo_overrun)(&fifo_overrun);
	err = snprintf(buf, 16, "%d\n", fifo_overrun);
	return err;
}

static ssize_t bmg_show_fifo_data_frame(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char f_len = 0;
	unsigned char fifo_framecount;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	if (client_data->fifo_datasel)
		/*Select one axis data output for every fifo frame*/
		f_len = 2;
	else
		/*Select X Y Z axis data output for every fifo frame*/
		f_len = 6;

	if (BMG_CALL_API(get_fifo_framecount)(&fifo_framecount) < 0) {
		PERR("bm160_get_fifo_framecount err\n");
		return -EINVAL;
	}
	if (fifo_framecount == 0)
		return 0;

	bmg_i2c_burst_read(client_data->client, BMG160_FIFO_DATA_ADDR,
			buf, fifo_framecount * f_len);
	return fifo_framecount * f_len;
}

/*!
 * @brief show fifo_data_sel axis definition(Android definition, not sensor HW reg).
 * 0--> x, y, z axis fifo data for every frame
 * 1--> only x axis fifo data for every frame
 * 2--> only y axis fifo data for every frame
 * 3--> only z axis fifo data for every frame
 */
static ssize_t bmg_show_fifo_data_sel(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_data_sel;
	struct i2c_client *client = to_i2c_client(dev);
	struct bmg_client_data *client_data = i2c_get_clientdata(client);
	signed char place = bmg160_platform.place;

	BMG_CALL_API(get_fifo_data_sel)(&fifo_data_sel);

	/*remapping fifo_dat_sel if define virtual place in BSP files*/
	if ((NULL != client_data->bst_pd) &&
		(BOSCH_SENSOR_PLACE_UNKNOWN != client_data->bst_pd->place)) {
		place = client_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place > 0) && (place < MAX_AXIS_REMAP_TAB_SZ)) {
			if (BMG160_FIFO_DAT_SEL_X == fifo_data_sel)
				/* BMG160_FIFO_DAT_SEL_X: 1, Y:2, Z:3;
				*bst_axis_remap_tab_dft[i].src_x:0, y:1, z:2
				*so we need to +1*/
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_x + 1;

			else if (BMG160_FIFO_DAT_SEL_Y == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_y + 1;
		}

	}

	err = snprintf(buf, 16, "%d\n", fifo_data_sel);
	return err;
}

/*!
 * @brief store fifo_data_sel axis definition(Android definition, not sensor HW reg).
 * 0--> x, y, z axis fifo data for every frame
 * 1--> only x axis fifo data for every frame
 * 2--> only y axis fifo data for every frame
 * 3--> only z axis fifo data for every frame
 */
static ssize_t bmg_store_fifo_data_sel(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	int err;
	unsigned long fifo_data_sel;

	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	signed char place;

	err = kstrtoul(buf, 10, &fifo_data_sel);
	if (err)
		return err;

	/*save fifo_data_sel(android axis definition)*/
	client_data->fifo_datasel = (unsigned char) fifo_data_sel;

	/*remaping fifo_dat_sel if define virtual place*/
	if ((NULL != client_data->bst_pd) &&
		(BOSCH_SENSOR_PLACE_UNKNOWN != client_data->bst_pd->place)) {
		place = client_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place > 0) && (place < MAX_AXIS_REMAP_TAB_SZ)) {
			/*Need X Y axis revesal sensor place: P1, P3, P5, P7 */
			/* BMG160_FIFO_DAT_SEL_X: 1, Y:2, Z:3;
			  * but bst_axis_remap_tab_dft[i].src_x:0, y:1, z:2
			  * so we need to +1*/
			if (BMG160_FIFO_DAT_SEL_X == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_x + 1;

			else if (BMG160_FIFO_DAT_SEL_Y == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_y + 1;
		}
	}

	if (BMG_CALL_API(set_fifo_data_sel)(fifo_data_sel) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bmg_show_fifo_tag(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_tag;
	BMG_CALL_API(get_fifo_tag)(&fifo_tag);
	err = snprintf(buf, 16, "%d\n", fifo_tag);
	return err;
}

static ssize_t bmg_store_fifo_tag(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	int err;
	unsigned long fifo_tag;
	err = kstrtoul(buf, 10, &fifo_tag);
	if (err)
		return err;
	BMG_CALL_API(set_fifo_tag)(fifo_tag);
	return count;
}
#endif

static ssize_t bmg160_driver_version_show(struct device *dev
		, struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
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
		bmg_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_op_mode, bmg_store_op_mode);
static DEVICE_ATTR(value, S_IRUGO,
		bmg_show_value, NULL);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_range, bmg_store_range);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_bandwidth, bmg_store_bandwidth);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_enable, bmg_store_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_delay, bmg_store_delay);
static DEVICE_ATTR(fastoffset_en, S_IWUSR|S_IWGRP,
		NULL, bmg_store_fastoffset_en);
static DEVICE_ATTR(slowoffset_en, S_IWUSR|S_IWGRP,
		NULL, bmg_store_slowoffset_en);
static DEVICE_ATTR(selftest, S_IRUGO,
		bmg_show_selftest, NULL);
static DEVICE_ATTR(sleepdur, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_sleepdur, bmg_store_sleepdur);
static DEVICE_ATTR(autosleepdur, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_autosleepdur, bmg_store_autosleepdur);
static DEVICE_ATTR(place, S_IRUGO,
		bmg_show_place, NULL);
static DEVICE_ATTR(enable_timer, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_enable_timer, bmg_store_enable_timer);
static DEVICE_ATTR(debug_level, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg160_show_debug_level, bmg160_store_debug_level);
static DEVICE_ATTR(driver_version, S_IRUGO,
		bmg160_driver_version_show, NULL);
#ifdef BMG_DEBUG
static DEVICE_ATTR(softreset, S_IWUSR|S_IWGRP,
		NULL, bmg_store_softreset);
static DEVICE_ATTR(regdump, S_IRUGO,
		bmg_show_dumpreg, NULL);
#endif
#ifdef BMG_USE_FIFO
static DEVICE_ATTR(fifo_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_fifo_mode, bmg_store_fifo_mode);
static DEVICE_ATTR(fifo_framecount, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_fifo_framecount, bmg_store_fifo_framecount);
static DEVICE_ATTR(fifo_overrun, S_IRUGO,
		bmg_show_fifo_overrun, NULL);
static DEVICE_ATTR(fifo_data_frame, S_IRUGO,
		bmg_show_fifo_data_frame, NULL);
static DEVICE_ATTR(fifo_data_sel, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_fifo_data_sel, bmg_store_fifo_data_sel);
static DEVICE_ATTR(fifo_tag, S_IRUGO|S_IWUSR|S_IWGRP,
		bmg_show_fifo_tag, bmg_store_fifo_tag);
#endif

static struct attribute *bmg_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_fastoffset_en.attr,
	&dev_attr_slowoffset_en.attr,
	&dev_attr_selftest.attr,
	&dev_attr_sleepdur.attr,
	&dev_attr_autosleepdur.attr,
	&dev_attr_place.attr,
	&dev_attr_enable_timer.attr,
	&dev_attr_debug_level.attr,
	&dev_attr_driver_version.attr,
#ifdef BMG_DEBUG
	&dev_attr_softreset.attr,
	&dev_attr_regdump.attr,
#endif
#ifdef BMG_USE_FIFO
	&dev_attr_fifo_mode.attr,
	&dev_attr_fifo_framecount.attr,
	&dev_attr_fifo_overrun.attr,
	&dev_attr_fifo_data_frame.attr,
	&dev_attr_fifo_data_sel.attr,
	&dev_attr_fifo_tag.attr,
#endif
	NULL
};

static struct attribute_group bmg_attribute_group = {
	.attrs = bmg_attributes
};


static int bmg_input_init(struct bmg_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
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

static void bmg_input_destroy(struct bmg_client_data *client_data)
{
	struct input_dev *dev = client_data->input;

	sysfs_remove_link(client_data->input->dev.kobj.parent, client_data->input->name);
	input_unregister_device(dev);
	input_free_device(dev);
}

#if 0
static int bmg160_pinctrl_init(struct i2c_client *client)
{
	int retval;
    struct bmg_client_data *data = i2c_get_clientdata(client);

	/* Get pinctrl if target uses pinctrl */
	data->ts_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->ts_pinctrl)) {
		printk(KERN_INFO "Target does not use pinctrl\n");
		retval = PTR_ERR(data->ts_pinctrl);
		data->ts_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_active
		= pinctrl_lookup_state(data->ts_pinctrl, "bmg160_active");
	if (IS_ERR_OR_NULL(data->gpio_state_active)) {
		printk(KERN_INFO "Can not get ts default pinstate\n");
		retval = PTR_ERR(data->gpio_state_active);
		data->ts_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_suspend
		= pinctrl_lookup_state(data->ts_pinctrl, "bmg160_suspend");
	if (IS_ERR_OR_NULL(data->gpio_state_suspend)) {
		printk(KERN_INFO "Can not get ts sleep pinstate\n");
		retval = PTR_ERR(data->gpio_state_suspend);
		data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int bmg160_pinctrl_select(struct bmg_client_data *data,
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
				on ? "bmg160_active" : "bmg160_suspend");
			return ret;
		}
	} else
		printk(KERN_INFO "not a valid '%s' pinstate\n",
				on ? "bmg160_active" : "bmg160_suspend");

	return 0;
}
#endif

void bmg160_config_gpio(struct i2c_client *client)
{
	int ret;
	u32 temp_val;
	int rc;
	struct device_node *np = client->dev.of_node;
	bmg160_platform.mux_vcc = regulator_get(&client->dev, "mux_vcc");
	if(IS_ERR(bmg160_platform.mux_vcc))
		printk(KERN_INFO "[Gyro]regulator_get mux_vcc fail\n");

	bmg160_platform.vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
	if(IS_ERR(bmg160_platform.vcc_i2c))
		printk(KERN_INFO "[Gyro]regulator_get vcc_i2c fail\n");

	ret = regulator_enable(bmg160_platform.mux_vcc);
	if (ret)
		printk(KERN_INFO "[Gyro]regulator_enable mux_vcc fail\n");

	ret = regulator_enable(bmg160_platform.vcc_i2c);
	if (ret)
		printk(KERN_INFO "[Gyro]regulator_enable vcc_i2c fail\n");
#if 0
	err = bmg160_pinctrl_init(client);
	if (!err && data->ts_pinctrl) {
		err = bmg160_pinctrl_select(data, true);
		if (err < 0)
			PERR("[Sensor]bmg160_pinctrl_select fail\n");
	}
#endif
	bmg160_platform.name = "bmg160";
	//bmg160_platform.place = 1;
	rc = of_property_read_u32(np, "bosch,place", &temp_val);
	if (rc && (rc != -EINVAL)) {
		PERR("%s:Unable to read sensor place paramater\n",__func__);
	}
	if (temp_val > 7 || temp_val < 0) {
		PERR("%s:Invalid place parameter, use default value 0\n",__func__);
		bmg160_platform.place = 8;
	} else {
		bmg160_platform.place = temp_val;
	}
	PERR( "[Gyro]bmg160_platform.place = %d\n",bmg160_platform.place);
#if 0
	bmg160_platform.irq1 = of_get_named_gpio_flags(np, "bosch,irq1-gpio", 0, NULL);
	bmg160_platform.irq2 = of_get_named_gpio_flags(np, "bosch,irq2-gpio", 0, NULL);
	err = gpio_request(bmg160_platform.irq1, "bmg160_irq");
	if (err) {
		PERR("%s: Failed to get gpio %d (code: %d)",
				__func__, bmg160_platform.irq1, err);
	}
	err = gpio_direction_input(bmg160_platform.irq1);
	if (err) {
		PERR("%s: Failed to set gpio %d direction",
				__func__, bmg160_platform.irq1);
	}
	err = gpio_request(bmg160_platform.irq2, "bmg160_irq");
	if (err) {
		PERR("%s: Failed to get gpio %d (code: %d)",
				__func__, bmg160_platform.irq2, err);
	}
	err = gpio_direction_input(bmg160_platform.irq2);
	if (err) {
		PERR("%s: Failed to set gpio %d direction",
				__func__, bmg160_platform.irq1);
	}
#endif
}

void bmg160_gpio_free(struct i2c_client *client)
{
	int ret;
	printk(KERN_INFO "[Gyro]%s Enter\n", __func__);
	ret = regulator_disable(bmg160_platform.vcc_i2c);
	if (ret)
		printk(KERN_INFO "[Gyro]regulator_disable vcc_i2c fail\n");
	regulator_put(bmg160_platform.vcc_i2c);

	ret = regulator_disable(bmg160_platform.mux_vcc);
	if (ret)
		printk(KERN_INFO "[Gyro]regulator_disable mux_vcc fail\n");
	regulator_put(bmg160_platform.mux_vcc);
}


static void  bmg160_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmg_client_data *client_data = i2c_get_clientdata(client);
	int err=0;
	mutex_lock(&client_data->mutex_enable);
	if (enable != client_data->enable) {
		if (enable) {
		mutex_lock(&client_data->mutex_op_mode);
	  err = BMG_CALL_API(set_mode)(BMG160_MODE_NORMAL);
	  mutex_unlock(&client_data->mutex_op_mode);
	
			schedule_delayed_work(
					&client_data->work,
					msecs_to_jiffies(atomic_read(
							&client_data->delay)));
		} else {
			cancel_delayed_work_sync(&client_data->work);
			mutex_lock(&client_data->mutex_op_mode);
	  err = BMG_CALL_API(set_mode)(BMG160_MODE_SUSPEND);
	  mutex_unlock(&client_data->mutex_op_mode);
		}

		client_data->enable = enable;
	}
	mutex_unlock(&client_data->mutex_enable);
}

static int bmg160_cdev_enable(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	struct bmg_client_data *data = container_of(sensors_cdev,
					struct bmg_client_data, cdev);

	bmg160_set_enable(&data->client->dev, enable);
	return 0;
}

static int bmg160_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_ms)
{
	struct bmg_client_data *data = container_of(sensors_cdev,
					struct bmg_client_data, cdev);

/*	if (delay_ms < POLL_INTERVAL_MIN_MS)
		delay_ms = POLL_INTERVAL_MIN_MS;
	if (delay_ms > POLL_INTERVAL_MAX_MS)
		delay_ms = POLL_INTERVAL_MAX_MS;*/
	atomic_set(&data->delay, (unsigned int) delay_ms);

	return 0;
}

static int bmg_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct bmg_client_data *client_data = NULL;

	PINFO("function entrance");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		err = -EIO;
		goto exit_err_clean;
	}

	if (NULL == bmg_client) {
		bmg_client = client;
	} else {
		PERR("this driver does not support multiple clients");
		err = -EINVAL;
		goto exit_err_clean;
	}

	client_data = kzalloc(sizeof(struct bmg_client_data), GFP_KERNEL);
	if (NULL == client_data) {
		PERR("no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	i2c_set_clientdata(client, client_data);
	client_data->client = client;

	bmg160_config_gpio(client);
	client_data->bst_pd = &bmg160_platform;
	/* do soft reset */
	bmg160_delay(5);

	err = bmg_set_soft_reset(client);

	if (err < 0) {
		PERR("erro soft reset!\n");
		err = -EINVAL;
		goto exit_err_clean;
	}
	bmg160_delay(30);

	/* check chip id */
	err = bmg_check_chip_id(client);
	if (!err) {
		PINFO("Bosch Sensortec Device %s detected", SENSOR_NAME);
	} else {
		PERR("Bosch Sensortec Device not found, chip id mismatch");
		err = -1;
		goto exit_err_clean;
	}

	mutex_init(&client_data->mutex_op_mode);
	mutex_init(&client_data->mutex_enable);

	/* input device init */
	err = bmg_input_init(client_data);
	if (err < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	err = sysfs_create_group(&client_data->input->dev.kobj,
			&bmg_attribute_group);

	if (err < 0)
		goto exit_err_sysfs;

	if (NULL != client->dev.platform_data) {
		client_data->bst_pd = kzalloc(sizeof(*client_data->bst_pd),
				GFP_KERNEL);

		if (NULL != client_data->bst_pd) {
			memcpy(client_data->bst_pd, client->dev.platform_data,
					sizeof(*client_data->bst_pd));
			PINFO("%s sensor driver set place: p%d",
					SENSOR_NAME,
					client_data->bst_pd->place);
		}
	}

	/* workqueue init */
	INIT_DELAYED_WORK(&client_data->work, bmg_work_func);
	atomic_set(&client_data->delay, BMG_DELAY_DEFAULT);

	/* h/w init */
	client_data->device.bus_read = bmg_i2c_read_wrapper;
	client_data->device.bus_write = bmg_i2c_write_wrapper;
	client_data->device.delay_msec = bmg160_delay;
	BMG_CALL_API(init)(&client_data->device);
	//printk(KERN_INFO "[Gyro]%s BMG_CALL_API OK\n", __func__);
	//bmg_dump_reg(client);
	//printk(KERN_INFO "[Gyro]%s bmg_dump_reg OK\n", __func__);
	client_data->enable = 0;
	client_data->fifo_datasel = 0;
	client_data->fifo_count = 0;

	/* now it's power on which is considered as resuming from suspend */
	err = BMG_CALL_API(set_fifo_mode)(2);/*open fifo mode*/
	if (err)
		PERR("set fifo mode failed\n");
	err = BMG_CALL_API(set_mode)(
			BMG_VAL_NAME(MODE_SUSPEND));

	if (err < 0)
		goto exit_err_sysfs;

	/*workqueue init*/
	INIT_WORK(&client_data->report_data_work,
	bmg160_work_func);
	reportdata_wq = create_singlethread_workqueue("bmg160_wq");
	if (NULL == reportdata_wq)
		PERR("fail to create the reportdta_wq %d", -ENOMEM);
	hrtimer_init(&client_data->timer, CLOCK_MONOTONIC,
		HRTIMER_MODE_REL);
	client_data->timer.function = reportdata_timer_fun;
	client_data->work_delay_kt = ns_to_ktime(10000000);
	client_data->is_timer_running = 0;
	client_data->time_odr = 500000;

#ifdef CONFIG_HAS_EARLYSUSPEND
	client_data->early_suspend_handler.suspend = bmg_early_suspend;
	client_data->early_suspend_handler.resume = bmg_late_resume;
	register_early_suspend(&client_data->early_suspend_handler);
#endif

	client_data->cdev = sensors_cdev;
	client_data->cdev.delay_msec = 100;
	client_data->cdev.sensors_enable = bmg160_cdev_enable;
	client_data->cdev.sensors_poll_delay = bmg160_cdev_poll_delay;

	err = sensors_classdev_register(&client_data->input->dev, &client_data->cdev);
	if (err) {
		dev_err(&client->dev, "create class device file failed!\n");
		err = -EINVAL;
		goto exit_err_sysfs;
	}

	PINFO("sensor %s probed successfully", SENSOR_NAME);

	dev_dbg(&client->dev,
		"i2c_client: %p client_data: %p i2c_device: %p input: %p",
		client, client_data, &client->dev, client_data->input);

	return 0;

exit_err_sysfs:
	if (err)
		bmg_input_destroy(client_data);

exit_err_clean:
	bmg160_gpio_free(client);
	if (err) {
		if (client_data != NULL) {
			kfree(client_data);
			client_data = NULL;
		}

		bmg_client = NULL;
	}

	return err;
}

static int bmg_pre_suspend(struct i2c_client *client)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);
	PINFO("function entrance");

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		cancel_delayed_work_sync(&client_data->work);
		PINFO("cancel work");
	}
	mutex_unlock(&client_data->mutex_enable);
	if (client_data->is_timer_running) {
		hrtimer_cancel(&client_data->timer);
		client_data->base_time = 0;
		client_data->timestamp = 0;
		client_data->fifo_time = 0;
		client_data->gyro_count = 0;
	}
	return err;
}

static int bmg_post_resume(struct i2c_client *client)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	PINFO("function entrance");
	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		schedule_delayed_work(&client_data->work,
				msecs_to_jiffies(
					atomic_read(&client_data->delay)));
	}
	mutex_unlock(&client_data->mutex_enable);
	if (client_data->is_timer_running) {
		hrtimer_start(&client_data->timer,
					ns_to_ktime(client_data->time_odr),
			HRTIMER_MODE_REL);
		client_data->base_time = 0;
		client_data->timestamp = 0;
		client_data->is_timer_running = 1;
		client_data->gyro_count = 0;
	}
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmg_early_suspend(struct early_suspend *handler)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)container_of(handler,
			struct bmg_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;

	PINFO("function entrance");

	mutex_lock(&client_data->mutex_op_mode);
	if (client_data->enable) {
		err = bmg_pre_suspend(client);
		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
	}
	mutex_unlock(&client_data->mutex_op_mode);
}

static void bmg_late_resume(struct early_suspend *handler)
{

	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)container_of(handler,
			struct bmg_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;

	PINFO("function entrance");

	mutex_lock(&client_data->mutex_op_mode);

	if (client_data->enable)
		err = BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_NORMAL));

	/* post resume operation */
	bmg_post_resume(client);

	mutex_unlock(&client_data->mutex_op_mode);
}
#else
static int bmg_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	PINFO("function entrance");

	mutex_lock(&client_data->mutex_op_mode);
	if (client_data->enable) {
		err = bmg_pre_suspend(client);
		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
	}
	mutex_unlock(&client_data->mutex_op_mode);
	return err;
}

static int bmg_resume(struct i2c_client *client)
{

	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	PINFO("function entrance");

	mutex_lock(&client_data->mutex_op_mode);

	if (client_data->enable)
		err = BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_NORMAL));

	/* post resume operation */
	bmg_post_resume(client);

	mutex_unlock(&client_data->mutex_op_mode);
	return err;
}
#endif

void bmg_shutdown(struct i2c_client *client)
{
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	mutex_lock(&client_data->mutex_op_mode);
	BMG_CALL_API(set_mode)(
		BMG_VAL_NAME(MODE_DEEPSUSPEND));
	mutex_unlock(&client_data->mutex_op_mode);
}

static int bmg_remove(struct i2c_client *client)
{
	int err = 0;
	u8 op_mode;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	if (NULL != client_data) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&client_data->early_suspend_handler);
#endif
		mutex_lock(&client_data->mutex_op_mode);
		BMG_CALL_API(get_mode)(&op_mode);
		if (BMG_VAL_NAME(MODE_NORMAL) == op_mode) {
			cancel_delayed_work_sync(&client_data->work);
			PINFO("cancel work");
		}
		mutex_unlock(&client_data->mutex_op_mode);

		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
		bmg160_delay(BMG_I2C_WRITE_DELAY_TIME);

		sysfs_remove_group(&client_data->input->dev.kobj,
				&bmg_attribute_group);
		bmg_input_destroy(client_data);
		kfree(client_data);

			bmg_client = NULL;
		}
		bmg160_gpio_free(client);

	return err;
}

static const struct i2c_device_id bmg_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bmg_id);

static struct of_device_id bmg160_match_table[] = {
	{ .compatible = "bosch,bmg160",},
	{ },
};

static struct i2c_driver bmg_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
		.of_match_table = bmg160_match_table,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = bmg_id,
	.probe = bmg_probe,
	.remove = bmg_remove,
	.shutdown = bmg_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = bmg_suspend,
	.resume = bmg_resume,
#endif
};

static int __init BMG_init(void)
{
	return i2c_add_driver(&bmg_driver);
}

static void __exit BMG_exit(void)
{
	i2c_del_driver(&bmg_driver);
}

MODULE_AUTHOR("contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMG GYROSCOPE SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

module_init(BMG_init);
module_exit(BMG_exit);
