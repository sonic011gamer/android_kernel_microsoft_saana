/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"
#include <linux/gpio.h>

#define FLASH_NAME "qcom,nt50573_e1m"

#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define NT50573_E1M_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define NT50573_E1M_DBG(fmt, args...) pr_err(fmt, ##args)
#endif


static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver nt50573_e1m_i2c_driver;

static struct msm_camera_i2c_reg_array nt50573_e1m_init_array[] = {
	{0x01, 0x01},
};

static struct msm_camera_i2c_reg_array nt50573_e1m_off_array[] = {
	{0x01, 0x00},
};

static struct msm_camera_i2c_reg_array nt50573_e1m_release_array[] = {
	{0x01, 0x00},
};

static struct msm_camera_i2c_reg_array nt50573_e1m_low_array[] = {
	{0x07, 0x04},
	{0x05, 0x46},
	//{0x06, 0x01},
	{0x08, 0x0A},
	{0x01, 0x09},
};

static struct msm_camera_i2c_reg_array nt50573_e1m_high_array[] = {
	{0x02, 0x00},
	{0x07, 0x05},
	{0x03, 0x70},
	//{0x04, 0x07},
	{0x08, 0x0A},
	{0x01, 0x0D},
};

static void __exit msm_flash_nt50573_e1m_i2c_remove(struct i2c_client *client)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;

	i2c_del_driver(& nt50573_e1m_i2c_driver);

	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}
	return;
}

static const struct of_device_id nt50573_e1m_i2c_trigger_dt_match[] = {
	{.compatible = "qcom,nt50573_e1m", .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, nt50573_e1m_i2c_trigger_dt_match);
static const struct i2c_device_id nt50573_e1m_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};
        
#if 0
static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value > LED_OFF) {
		if(fctrl.func_tbl->flash_led_low)
			fctrl.func_tbl->flash_led_low(&fctrl);
	} else {
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
};

static struct led_classdev msm_torch_led = {
	.name			= "torch-light",
	.brightness_set	= msm_led_torch_brightness_set,
	.brightness		= LED_OFF,
};

static int32_t msm_nt50573_e1m_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;
	msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);
	rc = led_classdev_register(dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};
#endif

int msm_flash_nt50573_e1m_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	uint16_t value = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	NT50573_E1M_DBG("%s:%d called\n", __func__, __LINE__); 

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);

	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
			fctrl->flash_i2c_client, 0x0C, &value, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0){
		pr_err("%s:%d, Read Device ID fail\n", __func__, __LINE__);
	}else{
		pr_err("%s:%d, Device ID = 0x%x\n", __func__, __LINE__, value);
	}

	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	pr_err("%s:%d, start init_setting command\n",  __func__, __LINE__);
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d, failed\n", __func__, __LINE__);
	}
	fctrl->led_state = MSM_CAMERA_LED_INIT;
	
	return 0;
}

int msm_flash_nt50573_e1m_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	NT50573_E1M_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}else{
		pr_err("%s:%d, led_state = %d\n", __func__, __LINE__, fctrl->led_state);
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->release_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);

	return 0;
}

int msm_flash_nt50573_e1m_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	NT50573_E1M_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	if (fctrl->led_state == MSM_CAMERA_LED_OFF) {
		return 0;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);

	pr_err("%s:%d, start off_setting command\n", __func__, __LINE__);
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		pr_err("%s:%d, rc = %d\n", __func__, __LINE__, rc);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

int msm_flash_nt50573_e1m_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	NT50573_E1M_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}else{
		pr_err("%s:%d, led_state = %d\n", __func__, __LINE__, fctrl->led_state);
	}
	
	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

int msm_flash_nt50573_e1m_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	NT50573_E1M_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}else{
		pr_err("%s:%d, led_state = %d\n", __func__, __LINE__, fctrl->led_state);
	}
	
	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

static struct platform_driver nt50573_e1m_platform_driver;
static int msm_flash_nt50573_e1m_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc = 0;

	NT50573_E1M_DBG("%s, Enter\n", __func__);

	if (!id) {
		pr_err("msm_flash_nt50573_e1m_i2c_probe: id is NULL");
		id = nt50573_e1m_i2c_id;
	}

	rc = msm_flash_i2c_init_res(client, id);
	if (rc < 0) {
		pr_err("%s: msm_flash_i2c_init_res failed\n", __func__);
		platform_driver_unregister(&nt50573_e1m_platform_driver);
	}else{
		struct msm_camera_sensor_board_info *flashdata = NULL;
		struct msm_camera_power_ctrl_t *power_info = NULL;

		flashdata = fctrl.flashdata;
		power_info = &flashdata->power_info;

		rc = msm_camera_request_gpio_table(
			power_info->gpio_conf->cam_gpio_req_tbl,
			power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
		if (rc < 0) {
			pr_err("%s: request gpio failed\n", __func__);
			return rc;
		}

		if (fctrl.pinctrl_info.use_pinctrl == true) {
			pr_err("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
			rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
					fctrl.pinctrl_info.gpio_state_active);
			if (rc)
				pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
		}

		flashdata = fctrl.flashdata;
		power_info = &flashdata->power_info;
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
		msleep(2);

		rc = msm_flash_i2c_probe_only_check_i2c(client, id);
		if (rc < 0) {
			pr_err("%s: msm_flash_i2c_probe_only_check_i2c failed\n", __func__);
			printk("BBox::UEC;10::1\n"); /// + BBS log
			platform_driver_unregister(&nt50573_e1m_platform_driver);
		}

		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_LOW);
	}

	return rc;
 
	NT50573_E1M_DBG("%s, Exit, rc = %d\n", __func__, rc);
	return rc;
}

static const struct of_device_id nt50573_e1m_trigger_dt_match[] = {
	{.compatible = "qcom,nt50573_e1m", .data = &fctrl},
	{}
};

static struct i2c_driver nt50573_e1m_i2c_driver = {
	.id_table = nt50573_e1m_i2c_id,
	.probe  = msm_flash_nt50573_e1m_i2c_probe,
	.remove = __exit_p(msm_flash_nt50573_e1m_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = nt50573_e1m_i2c_trigger_dt_match,
	},
};

static int msm_flash_nt50573_e1m_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;

	match = of_match_device(nt50573_e1m_trigger_dt_match, &pdev->dev);
	if (!match){
		pr_err("msm_flash_nt50573_e1m_platform_probe fail");
		return -EFAULT;
	}

	return msm_flash_probe(pdev, match->data);
}

static struct platform_driver nt50573_e1m_platform_driver = {
	.probe = msm_flash_nt50573_e1m_platform_probe,
	.driver = {
		.name = "qcom,nt50573_e1m", ///"qcom,led-flash",///peter test
		.owner = THIS_MODULE,
		.of_match_table = nt50573_e1m_trigger_dt_match,
	},
};

static int __init msm_flash_nt50573_e1m_init_module(void)
{
	int32_t rc = 0;
	pr_err("%s, called\n", __func__);
	rc = platform_driver_register(&nt50573_e1m_platform_driver);
	if (fctrl.pdev != NULL && rc == 0) {
		pr_err("%s, nt50573_e1m platform_driver_register success, rc = %d", __func__, rc);
		return rc;
	} else if (rc != 0) {
		pr_err("%s, nt50573_e1m platform_driver_register failed, rc = %d", __func__, rc);
		return rc;
	} else {
		rc = i2c_add_driver(&nt50573_e1m_i2c_driver);
		if (!rc)
			pr_err("%s, nt50573_e1m i2c_add_driver success, rc = %d", __func__, rc);
	}
	return rc;
}

static void __exit msm_flash_nt50573_e1m_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&nt50573_e1m_platform_driver);
	else
		i2c_del_driver(&nt50573_e1m_i2c_driver);

}


static struct msm_camera_i2c_client nt50573_e1m_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting nt50573_e1m_init_setting = {
	.reg_setting = nt50573_e1m_init_array,
	.size = ARRAY_SIZE(nt50573_e1m_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting nt50573_e1m_off_setting = {
	.reg_setting = nt50573_e1m_off_array,
	.size = ARRAY_SIZE(nt50573_e1m_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting nt50573_e1m_release_setting = {
	.reg_setting = nt50573_e1m_release_array,
	.size = ARRAY_SIZE(nt50573_e1m_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting nt50573_e1m_low_setting = {
	.reg_setting = nt50573_e1m_low_array,
	.size = ARRAY_SIZE(nt50573_e1m_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting nt50573_e1m_high_setting = {
	.reg_setting = nt50573_e1m_high_array,
	.size = ARRAY_SIZE(nt50573_e1m_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t nt50573_e1m_regs = {
	.init_setting = &nt50573_e1m_init_setting,
	.off_setting = &nt50573_e1m_off_setting,
	.low_setting = &nt50573_e1m_low_setting,
	.high_setting = &nt50573_e1m_high_setting,
	.release_setting = &nt50573_e1m_release_setting,
};

static struct msm_flash_fn_t nt50573_e1m_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_nt50573_e1m_led_init,
	.flash_led_release = msm_flash_nt50573_e1m_led_release,
	.flash_led_off = msm_flash_nt50573_e1m_led_off,
	.flash_led_low = msm_flash_nt50573_e1m_led_low,
	.flash_led_high = msm_flash_nt50573_e1m_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &nt50573_e1m_i2c_client,
	.reg_setting = &nt50573_e1m_regs,
	.func_tbl = &nt50573_e1m_func_tbl,
};

module_init(msm_flash_nt50573_e1m_init_module);
module_exit(msm_flash_nt50573_e1m_exit_module);
MODULE_DESCRIPTION("NT50573_E1M FLASH");
MODULE_LICENSE("GPL v2");
