/*
 * Simple driver for Texas Instruments max77387 LED Flash driver chip
 * Copyright (C) 2012 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/export.h>
#include "msm_led_flash.h"
#include "../cci/msm_cci.h"

static struct msm_led_flash_ctrl_t fctrl;

//SW4-RL-PortingFlashLEDdriver_MAX77387-00+{_20150206
static struct i2c_driver max77387_i2c_driver;
#define FLASH_NAME "flash,max77387"
//SW4-RL-PortingFlashLEDdriver_MAX77387-00+}_20150206

static int is_torch_mode;	//SW4-RL-Camera-fixTorchModeFlickerIssue-00+_20150702

//SW4-RL-Camera-settingDifferentCurrent_for_EVT/DVT-00+{_20150707
extern uint32_t torch_Current;
extern uint32_t flash_Current;
//SW4-RL-Camera-settingDifferentCurrent_for_EVT/DVT-00+}_20150707

static struct msm_camera_i2c_reg_array max77387_init_array[] = {
	{0x08, 0xCA},
	//{0x05, 0x10},
	//{0x07, 0x20},
	{0x0A, 0xC0},
	{0x0D, 0xFF},
	{0x0E, 0x80},
};

static struct msm_camera_i2c_reg_array max77387_off_array[] = {
	{0x08, 0x00},
};

static struct msm_camera_i2c_reg_array max77387_release_array[] = {
	{0x08, 0x00},
};

static struct msm_camera_i2c_reg_array max77387_low_array[] = {
	{0x08, 0x80},
	{0x15, 0x40},	//Set DCDC to 0 	//B[7:6]=10 OVP_TH=5.1V , B[7:6]=01 OVP_TH = 4.8V, B[7:6]=00 OVP_TH = 4.5V
	{0x16, 0xC0},	// Set DCDC_CNT2 bit6 and bit 7 to (1,1)
	{0x04, 0x00}, //Flash_1 Disable
	{0x05, 0x00}, //Flash_2 Disable
	//{0x06, 0x8E}, //ITORCH1	// 27.37mA
	{0x06, 0xA6}, //ITORCH1	// 75mA
	{0x08, 0xAB}, 
};

static struct msm_camera_i2c_reg_array max77387_high_array[] = {
	{0x08, 0x40},
	{0x15, 0x40},	//B[7:6]=10 OVP_TH=5.1V , B[7:6]=01 OVP_TH = 4.8V, B[7:6]=00 OVP_TH = 4.5V
	//{0x04, 0x88},	// 0x88h = 0x10001000b  b7:FLASH1_EN, b[5:0]: FLASH1[5:0]  (EX: 000000 = 15.625mA , 000001 = 31.25mA ....  111111 = 1000mA)		// 125mA
	//{0x04, 0x93},	// 0x88h = 0x10001000b  b7:FLASH1_EN, b[5:0]: FLASH1[5:0]  (EX: 000000 = 15.625mA , 000001 = 31.25mA ....  111111 = 1000mA)		// 296.875mA
	//{0x04, 0x9A},	// 0x88h = 0x10001000b  b7:FLASH1_EN, b[5:0]: FLASH1[5:0]  (EX: 000000 = 15.625mA , 000001 = 31.25mA ....  111111 = 1000mA)		// 406.25mA
	//{0x04, 0xA0},	// 0x88h = 0x10001000b  b7:FLASH1_EN, b[5:0]: FLASH1[5:0]  (EX: 000000 = 15.625mA , 000001 = 31.25mA ....  111111 = 1000mA)		// 500mA
	{0x04, 0xA6},	// 0x88h = 0x10001000b  b7:FLASH1_EN, b[5:0]: FLASH1[5:0]  (EX: 000000 = 15.625mA , 000001 = 31.25mA ....  111111 = 1000mA)		//593.75mA
	//{0x04, 0xB0},	// 0x88h = 0x10001000b  b7:FLASH1_EN, b[5:0]: FLASH1[5:0]  (EX: 000000 = 15.625mA , 000001 = 31.25mA ....  111111 = 1000mA)		// 750mA
	//{0x04, 0xBF},	// 0x88h = 0x10001000b  b7:FLASH1_EN, b[5:0]: FLASH1[5:0]  (EX: 000000 = 15.625mA , 000001 = 31.25mA ....  111111 = 1000mA)		// 1000mA
	{0x05, 0x00},	//FLASH2_EN Disable
	{0x06, 0x00}, //TORCH_1 Disable
	{0x07, 0x00}, //TORCH_2 Disable
	{0x08, 0x45},
}; 

static void __exit msm_flash_max77387_i2c_remove(void)
{
	i2c_del_driver(& max77387_i2c_driver);
	return;
}

static const struct of_device_id max77387_trigger_dt_match[] = {
	{.compatible = "flash,max77387", .data = &fctrl},
	{}
};

//SW4-RL-PortingFlashLEDdriver_MAX77387-00*{_20150206
MODULE_DEVICE_TABLE(of, max77387_trigger_dt_match);

static const struct i2c_device_id max77387_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};
static struct platform_driver max77387_platform_driver;
static int msm_flash_max77387_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	if (!id) {
		pr_err("msm_flash_max77387_i2c_probe");
		id = max77387_i2c_id;
	}

	ret = msm_flash_i2c_probe(client, id);

	if(ret < 0)
	{
		pr_err("msm_flash_max77387_i2c_probe: msm_flash_i2c_probe fail");
		platform_driver_unregister(&max77387_platform_driver);
	}

	//SW4-RL-Camera-settingDifferentCurrent_for_EVT/DVT-00+{_20150707
	max77387_low_array[5].reg_data = (uint16_t)torch_Current;
	max77387_high_array[2].reg_data = (uint16_t)flash_Current;
	//SW4-RL-Camera-settingDifferentCurrent_for_EVT/DVT-00+}_20150707

	return ret;
}

static struct i2c_driver max77387_i2c_driver = {
	.id_table = max77387_i2c_id,
	.probe  = msm_flash_max77387_i2c_probe,
	.remove = __exit_p(msm_flash_max77387_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = max77387_trigger_dt_match,
	},
};
//SW4-RL-PortingFlashLEDdriver_MAX77387-00*}_20150206

static int msm_flash_max77387_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;

	match = of_match_device(max77387_trigger_dt_match, &pdev->dev);
	if (!match){
		pr_err("msm_flash_max77387_platform_probe fail");
		return -EFAULT;
	}

	return msm_flash_probe(pdev, match->data);
}

static struct platform_driver max77387_platform_driver = {
	.probe = msm_flash_max77387_platform_probe,
	.driver = {
		.name = "qcom,led-flash",
		.owner = THIS_MODULE,
		.of_match_table = max77387_trigger_dt_match,
	},
};

static int __init msm_flash_max77387_init_module(void)
{
	int32_t rc = 0;
	pr_err("called\n");
	rc = platform_driver_register(&max77387_platform_driver);
	if (fctrl.pdev != NULL && rc == 0) {
		pr_err("max77387 platform_driver_register success");
		return rc;
	} else if (rc != 0) {
		pr_err("max77387 platform_driver_register failed");
		return rc;
	} else {
		rc = i2c_add_driver(&max77387_i2c_driver);
		if (!rc)
			pr_err("max77387 i2c_add_driver success");
	}
	return rc;
}

//SW4-RL-Camera-FlashLED_Control-00+{_20150702
int msm_flash_led_max77387_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	is_torch_mode= 0;
	pr_err("called\n");

	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	pr_err("start init_setting command\n");
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	fctrl->led_state = MSM_CAMERA_LED_INIT;

	return rc;
}
//SW4-RL-Camera-FlashLED_Control-00+}_20150702

//SW4-RK-Camera-DisableFlashWhenStopPreview+{_20150622
int msm_flash_led_max77387_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	is_torch_mode = 0;
	pr_err("called\n");

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
	rc = fctrl->func_tbl->flash_led_off(fctrl);

	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}
	return 0;
}

int msm_flash_led_max77387_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	is_torch_mode = 0;
	pr_err("called\n");

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
	pr_err("start off_setting command\n");
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	return rc;
}
//SW4-RK-Camera-DisableFlashWhenStopPreview+}_20150622

//SW4-RL-Camera-FlashLED_Control-00+{_20150702
int msm_flash_led_max77387_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	//SW4-RL-read STATUS_1 and OVP_TH-00+{_20151127
	uint16_t value = 0;

	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
			fctrl->flash_i2c_client, 0x02, &value, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0){
		pr_err("read STATUS1 fail\n");
	}else{
		pr_err("STATUS1 0x02 = 0x%x\n", value);
	}

	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
			fctrl->flash_i2c_client, 0x15, &value, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0){
		pr_err("OVP_TH\n");
	}else{
		pr_err("OVP_TH (reg[0x15], B[7:6]) = 0x%x\n", value&0xC0);
	}
	//SW4-RL-read STATUS_1 and OVP_TH-00+}_20151127

	pr_err("called, is_torch_mode = %d\n", is_torch_mode);
	if (!is_torch_mode){
		is_torch_mode = 1;

		if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
			pr_err("%s:%d invalid led state\n", __func__, __LINE__);
			return -EINVAL;
		}
		pr_err("start low_setting command\n");
		if (fctrl->flash_i2c_client && fctrl->reg_setting) {
			rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
				fctrl->flash_i2c_client,
				fctrl->reg_setting->low_setting);
			if (rc < 0)
				pr_err("%s:%d failed\n", __func__, __LINE__);
		}
	}
	return rc;
}

int msm_flash_led_max77387_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	uint16_t value = 0;	//SW4-RL-00+_20151127
	is_torch_mode = 0;

	//SW4-RL-read STATUS_1 and OVP_TH-00+{_20151127
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
			fctrl->flash_i2c_client, 0x02, &value, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0){
		pr_err("read STATUS1 fail\n");
	}else{
		pr_err("STATUS1 = 0x%x\n", value);
	}

	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
			fctrl->flash_i2c_client, 0x15, &value, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0){
		pr_err("read OVP_TH fail\n");
	}else{
		pr_err("OVP_TH (reg[0x15], B[7:6]) = 0x%x\n", value&0xC0);
	}
	//SW4-RL-read STATUS_1 and OVP_TH-00+}_20151127

	pr_err("called\n");

	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
	pr_err("start high_setting command\n");
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
//SW4-RL-Camera-FlashLED_Control-00+}_20150702

static void __exit msm_flash_max77387_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&max77387_platform_driver);
	else
		i2c_del_driver(&max77387_i2c_driver);
}

/*ChunJeng Add for follow Qcom structure Start*/
static struct msm_camera_i2c_client max77387_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting max77387_init_setting = {
	.reg_setting = max77387_init_array,
	.size = ARRAY_SIZE(max77387_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting max77387_off_setting = {
	.reg_setting = max77387_off_array,
	.size = ARRAY_SIZE(max77387_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting max77387_release_setting = {
	.reg_setting = max77387_release_array,
	.size = ARRAY_SIZE(max77387_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting max77387_low_setting = {
	.reg_setting = max77387_low_array,
	.size = ARRAY_SIZE(max77387_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting max77387_high_setting = {
	.reg_setting = max77387_high_array,
	.size = ARRAY_SIZE(max77387_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t max77387_regs = {
	.init_setting = &max77387_init_setting,
	.off_setting = &max77387_off_setting,
	.low_setting = &max77387_low_setting,
	.high_setting = &max77387_high_setting,
	.release_setting = &max77387_release_setting,
};

static struct msm_flash_fn_t max77387_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_led_max77387_init,
	.flash_led_release = msm_flash_led_max77387_release,
	.flash_led_off = msm_flash_led_max77387_off,
	.flash_led_low = msm_flash_led_max77387_low,
	.flash_led_high = msm_flash_led_max77387_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &max77387_i2c_client,
	.reg_setting = &max77387_regs,
	.func_tbl = &max77387_func_tbl,
};
/*ChunJeng ChunJeng Add for follow Qcom structure End*/

module_init(msm_flash_max77387_init_module);	//SW4-RL-PortingFlashLEDdriver_MAX77387-00*_20150206
module_exit(msm_flash_max77387_exit_module); //SW4-RL-PortingFlashLEDdriver_MAX77387-00*_20150206

MODULE_DESCRIPTION("Flash LED IC driver for max77387");
MODULE_LICENSE("GPL v2");
