/*
 * RT4501 Backlight Driver
 *
 * Copyright (C) 2013 Richtek Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/platform_data/rt4501_bl.h>	//SW4-HL-Display-BringUpNT35521-00*_20150224

//HL+{_20150610
/* Black Box */
#define BBOX_BACKLIGHT_I2C_FAIL do {printk("BBox;%s: I2C fail\n", __func__); printk("BBox::UEC;1::1\n");} while (0);
#define BBOX_BACKLIGHT_GPIO_FAIL do {printk("BBox;%s: GPIO fail\n", __func__); printk("BBox::UEC;1::1\n");} while (0);
//HL+}_20150610

extern int fih_apr_get_rere (void);

static unsigned char rt4501_init_data[] =
{ 0x87, /* reg 0x02 */
  0x40, /* reg 0x03 */
};

struct rt4501_chip_data
{
	struct device *dev;
	struct rt4501_platform_data *pdata;
	struct regmap *regmap;
};

struct i2c_client *rt4501_i2c_client;

static int reg_read(struct regmap *map, unsigned int reg, unsigned int *val)
{
	int ret;
	int retry = 0;

	do {
		ret = regmap_read(map, reg, val);
		if (ret < 0) {
			pr_err("%s: fail, reg=%d\n", __func__, reg);
			retry++;
			msleep(10);
		}
	} while ((ret < 0) && (retry <= 3));

	if (ret < 0) {
		BBOX_BACKLIGHT_I2C_FAIL
	}

	return ret;
}

static int reg_write(struct regmap *map, unsigned int reg, unsigned int val)
{
	int ret;
	int retry = 0;

	do {
		ret = regmap_write(map, reg, val);
		if (ret < 0) {
			pr_err("%s: fail, reg=%d, val=0x%X\n", __func__, reg, val);
			retry++;
			msleep(10);
		}
	} while ((ret < 0) && (retry <= 3));

	if (ret < 0) {
		BBOX_BACKLIGHT_I2C_FAIL
	}

	return ret;
}

static int reg_update_bits(struct regmap *map, unsigned int reg, unsigned int mask, unsigned int val)
{
	int ret;
	int retry = 0;

	do {
		ret = regmap_update_bits(map, reg, mask, val);
		if (ret < 0) {
			pr_err("%s: fail, reg=%d, mask=0x%X, val=0x%X\n", __func__,
				reg, mask, val);
			retry++;
			msleep(10);
		}
	} while ((ret < 0) && (retry <= 3));

	if (ret < 0) {
		BBOX_BACKLIGHT_I2C_FAIL
	}

	return ret;
}



// set enalbe
static int rt4501_chip_data_enable(int enable)
{
	struct	i2c_client *client = rt4501_i2c_client;
	struct	rt4501_chip_data *pchip;
	struct	rt4501_platform_data *pdata;
	int ret = 0;
	unsigned int reg_val;

	pr_debug("\n\n******************** [HL] %s, +++, enable = %d **********************\n\n", __func__, enable);

	if (!client) {
		pr_err("%s: client is NULLn", __func__);
		return -1;
	}

	pchip = i2c_get_clientdata(client);
	if (!pchip) {
		pr_err("%s: pchip is NULL\n", __func__);
		return -1;
	}

	pdata = pchip->pdata;
	if (!pdata) {
		pr_err("%s: pdata is NULL\n", __func__);
		return -1;
	}

	// if enable != 0 : set enable,  if enable = 0 : set disable
	if(enable)
	{
		//set enable
		if (!pdata->enable)
		{
			ret = reg_read(pchip->regmap, RT4501_CONFIG, &reg_val);
			if(ret < 0)
			{
				goto out;
			}

			reg_val |= 0x01;
			ret = reg_update_bits(pchip->regmap, RT4501_CONFIG, 0x01, reg_val);
			if(ret < 0)
			{
				goto out;
			}
		}
	}
	else
	{
		//set disable
		if (pdata->enable)
		{
			ret = reg_update_bits(pchip->regmap, RT4501_CONFIG, 0x01, 0x00);
			if(ret < 0)
			{
				goto out;
			}
		}
	}

	pdata->enable = enable;

	pr_debug("\n\n******************** [HL] %s, --- **********************\n\n", __func__);

	return ret;

out:
	pr_debug("\n\n******************** [HL] %s ---, i2c failed to access register return ret = %d **********************\n\n", __func__, ret);

	BBOX_BACKLIGHT_I2C_FAIL
	return ret;
}

//init register map
static int rt4501_init_reg(void)
{
	struct	i2c_client *client = rt4501_i2c_client;
	struct	rt4501_chip_data *pchip;
	struct	rt4501_platform_data *pdata;
	int ret = 0;

	pr_debug("\n\n******************** [HL] %s, +++, rt4501_init_data[0] = 0x%x, rt4501_init_data[1] = 0x%x **********************\n\n", __func__, rt4501_init_data[0], rt4501_init_data[1]);

	if (!client) {
		pr_err("%s: client is NULLn", __func__);
		return -1;
	}

	pchip = i2c_get_clientdata(client);
	if (!pchip) {
		pr_err("%s: pchip is NULL\n", __func__);
		return -1;
	}

	pdata = pchip->pdata;
	if (!pdata) {
		pr_err("%s: pdata is NULL\n", __func__);
		return -1;
	}

	ret = reg_write(pchip->regmap, RT4501_CONFIG, rt4501_init_data[0]);
	if (ret < 0)
		goto out;

	ret = reg_write(pchip->regmap, RT4501_TIMING, rt4501_init_data[1]);
	if (ret < 0)
		goto out;

	pr_debug("\n\n******************** [HL] %s, --- **********************\n\n", __func__);

	return ret;

out:
	pr_debug("\n\n******************** [HL] %s ---, i2c failed to access register return ret = %d **********************\n\n", __func__, ret);

	BBOX_BACKLIGHT_I2C_FAIL
	return ret;
}

static int rt4501_set_brightness(int brightness)
{
	struct	i2c_client *client = rt4501_i2c_client;
	struct	rt4501_chip_data *pchip;
	struct	rt4501_platform_data *pdata;
	int ret = 0;

	pr_debug("\n\n******************** [HL] %s, +++, brightness = %d **********************\n\n", __func__, brightness);

	if (!client) {
		pr_err("%s: client is NULLn", __func__);
		return -1;
	}

	pchip = i2c_get_clientdata(client);
	if (!pchip) {
		pr_err("%s: pchip is NULL\n", __func__);
		return -1;
	}

	pdata = pchip->pdata;
	if (!pdata) {
		pr_err("%s: pdata is NULL\n", __func__);
		return -1;
	}

	ret = reg_write(pchip->regmap, RT4501_BACKLIGHT_CTRL, brightness);
	if (ret < 0)
		goto out;

	if( brightness != 0)
	{
		ret = rt4501_chip_data_enable(1); //enalbe
		if (ret < 0)
			goto out;
	}
	else
	{
		ret = rt4501_chip_data_enable(0); //disable
		if (ret < 0)
			goto out;
	}

	pdata->brightness = brightness;

	pr_debug("\n\n******************** [HL] %s, --- **********************\n\n", __func__);

	return ret;

out:
	pr_debug("\n\n******************** [HL] %s ---, i2c failed to access register return ret = %d **********************\n\n", __func__, ret);

	BBOX_BACKLIGHT_I2C_FAIL
	return ret;
}

//SW4-HL-Display-EnablePWMOutput-00*{_20150605
// Add return parameter - rc; if backlight is set from 0 to any value, then rc = 1; others, rc = 0
int rt4501_set_backlight_level(int brightness)	//SW4-HL-FixBacklightAlwaysShowMaximumBrightnessWhenResume-00*_20150311
{
	struct	i2c_client *client = rt4501_i2c_client;
	struct	rt4501_chip_data *pchip;
	struct	rt4501_platform_data *pdata;
	int ret = 0;
	int rc = 0;

	pr_debug("\n\n******************** [HL] %s, +++, brightness = %d **********************\n\n", __func__, brightness);

	if (!client) {
		pr_err("%s: client is NULLn", __func__);
		return rc;
	}

	pchip = i2c_get_clientdata(client);
	if (!pchip) {
		pr_err("%s: pchip is NULL\n", __func__);
		return rc;
	}

	pdata = pchip->pdata;
	if (!pdata) {
		pr_err("%s: pdata is NULL\n", __func__);
		return rc;
	}

	if (pdata->old_bl != brightness)
	{
		if ((pdata->old_bl == 0) && (brightness != 0))
		{
			pr_debug("\n\n******************** [HL] %s, (old_bl == 0) && (brightness != 0) **********************\n\n", __func__);
			gpio_direction_output(pdata->gpio_enable_pin, 1);
			pdata->enable = 1;
			rt4501_init_reg();
			rc = 1;
		}
		else
		{
			rc = 0;
		}

		//Check whether bit 6 (PWM_EN) is 1 or not; If 1, PWM output is enabled; If 0, PWM output is disabled.
		if ((rt4501_init_data[0] & 0x40) != 0x40)
		{
			ret = rt4501_set_brightness(brightness);
			if (ret < 0)
				goto out;
		}

		if ((brightness == 0) || ((pdata->old_bl == 0) && (brightness != 0)))
		{
			pr_debug("%s: level=%d\n", __func__, brightness);
		}

		if ((pdata->old_bl != 0) && (brightness == 0))
		{
			pr_debug("\n\n******************** [HL] %s, (old_bl != 0) && (brightness == 0) **********************\n\n", __func__);
			gpio_direction_output(pdata->gpio_enable_pin, 0);
			pdata->enable = 0;
			rc = 2;
		}

		pdata->old_bl = brightness;
	}
	else
	{
		//No set backlight since old_bl equals to brightness
		pr_debug("\n\n******************** [HL] %s, No set backlight since old_bl equals to brightness **********************\n\n", __func__);
	}

	pr_debug("\n\n******************** [HL] %s, --- **********************\n\n", __func__);

	return rc;

out:
	pr_debug("\n\n******************** [HL] %s ---, i2c failed to access register return ret = %d **********************\n\n", __func__, ret);

	BBOX_BACKLIGHT_I2C_FAIL
	return rc;
}
//SW4-HL-Display-EnablePWMOutput-00*}_20150605
//SW4-HL-Display-BringUpNT35521-00+_20150224
EXPORT_SYMBOL(rt4501_set_backlight_level);

//SW4-HL-Display-FTM-FixLCMCommandNotWorkInFAO-00+{_20150313
void rt4501_get_backlight_level(int *level)
{
 	struct	i2c_client *client = rt4501_i2c_client;
	struct	rt4501_chip_data *pchip;
	struct	rt4501_platform_data *pdata;

	pr_debug("\n\n******************** [HL] %s +++ **********************\n\n", __func__);

	if (!client) {
		pr_err("%s: client is NULLn", __func__);
		return;
	}

	pchip = i2c_get_clientdata(client);
	if (!pchip) {
		pr_err("%s: pchip is NULL\n", __func__);
		return;
	}

	pdata = pchip->pdata;
	if (!pdata) {
		pr_err("%s: pdata is NULL\n", __func__);
		return;
	}

	*level = pdata->brightness;

	pr_debug("\n\n******************** [HL] %s --- **********************\n\n", __func__);
}

static ssize_t rt4501_bl_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int bl_val=0;
	int ret = 0;

	rt4501_get_backlight_level(&bl_val);
	pr_debug("bl_val = %d\n",bl_val);
	ret += sprintf(buf + ret, "%d\n", bl_val);
	return ret;
}
static ssize_t rt4501_bl_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int reg_val =0;

	pr_debug("%s\n", __func__);
	if (sscanf(buf, "%d", &reg_val) <= 0) {
		pr_err("[%s] get user-space data failed\n", __func__);
		return -EINVAL;
	}
	pr_debug("reg_val = %d\n",reg_val);

	rt4501_set_backlight_level(reg_val);

	return count;
}

static DEVICE_ATTR(level, 0644, rt4501_bl_show,  rt4501_bl_store);
//SW4-HL-Display-FTM-FixLCMCommandNotWorkInFAO-00+}_20150313

static int rt_parse_dt(struct rt4501_platform_data *pdata, struct device *dev)
{
	#ifdef CONFIG_OF
	u32 tmp = 0;
	struct device_node *np = dev->of_node;
	pdata->gpio_enable_pin = of_get_named_gpio(np, "rt,en_pin", 0);

	if (of_property_read_bool(np, "rt,ovp_sel"))
		rt4501_init_data[0] |= RT4501_OVPSEL_MASK;
	else
		rt4501_init_data[0] &= ~RT4501_OVPSEL_MASK;

	//SW4-HL-Display-EnablePWMOutput-00*{_20150605
	//Parse the item 'rc,pwm_en' exists or not only when panel is hx8394f
	if(strstr(saved_command_line, "fih,mdss_dsi_hx8394f_720p_video")!=NULL)
	{
		if (of_property_read_bool(np, "rt,pwm_en"))
			rt4501_init_data[0] |= RT4501_PWMEN_MASK;
		else
			rt4501_init_data[0] &= ~RT4501_PWMEN_MASK;
	}
	//SW4-HL-Display-EnablePWMOutput-00*}_20150605

	if (of_property_read_bool(np, "rt,pwm_pol"))
		rt4501_init_data[0] |= RT4501_PWMPOL_MASK;
	else
		rt4501_init_data[0] &= ~RT4501_PWMPOL_MASK;

	if (of_property_read_bool(np, "rt,max_curr"))
		rt4501_init_data[0] |= RT4501_MAXCURR_MASK;
	else
		rt4501_init_data[0] &= ~RT4501_MAXCURR_MASK;

	if (of_property_read_u32(np, "rt,up_ramprate", &tmp) >= 0)
	{
		rt4501_init_data[1] &= ~RT4501_UPRAMP_MASK;
		rt4501_init_data[1] |= (tmp << RT4501_UPRAMP_SHFT);
	}
	if (of_property_read_u32(np, "rt,down_ramprate", &tmp) >= 0)
	{
		rt4501_init_data[1] &= ~RT4501_DNRAMP_MASK;
		rt4501_init_data[1] |= (tmp << RT4501_DNRAMP_SHFT);
	}
	#endif /* #ifdef CONFIG_OF */
	return 0;
}

static const struct regmap_config rt4501_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = RT4501_MAX_REG,
};

static int skt_restart(void)
{
  unsigned int rere = fih_apr_get_rere();
  if (rere != 0x21534b54) {
          return (0);
  }

  pr_info("rt4501_bl: SKT Silence Reboot\n");
  return (1);
}

static int rt4501_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rt4501_chip_data *pchip;
	struct rt4501_platform_data *pdata;
	int ret = 0;

	pr_debug("\n\n******************** [HL] %s, +++ **********************\n\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check...\n");
		return -EOPNOTSUPP;
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct rt4501_chip_data),
			     GFP_KERNEL);
	if (!pchip)
	{
		dev_err(&client->dev, "%s: Failed to alloc mem for rt4501_chip_data\n", __func__);
		return -ENOMEM;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct rt4501_platform_data),
			     GFP_KERNEL);
	if (!pdata)
	{
		dev_err(&client->dev, "%s: Failed to alloc mem for rt4501_platform_data\n", __func__);
		return -ENOMEM;
	}

	rt_parse_dt(pdata, &client->dev);

	//chip data initialize
	pchip->pdata = pdata;
	pchip->dev = &client->dev;

	pchip->regmap = devm_regmap_init_i2c(client, &rt4501_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "fail : allocate register map: %d\n",
			ret);
		return ret;
	}
	i2c_set_clientdata(client, pchip);

	rt4501_i2c_client = client;

	//request gpio
	ret = gpio_request(pdata->gpio_enable_pin,"rt4501 enable pin");
	if(ret < 0)
	{
		BBOX_BACKLIGHT_GPIO_FAIL
		return -EINVAL;
	}

	if (0 < skt_restart()) {
		/* no enable backlight when skt silence boot */
		gpio_direction_output(pdata->gpio_enable_pin, 0);
	} else {
		gpio_direction_output(pdata->gpio_enable_pin, 1);
	}

	pdata->enable = 1;

	//start mutex
	mutex_init(&pdata->io_lock);

	//init register map
	rt4501_init_reg();

	//SW4-HL-Display-FTM-FixLCMCommandNotWorkInFAO-00+{_20150313
	ret = device_create_file(&client->dev, &dev_attr_level);
	if (ret)
	{
		dev_err(&client->dev, "Failed to register devive %s:(%d)\n", dev_attr_level.attr.name, ret);
	}
	//SW4-HL-Display-FTM-FixLCMCommandNotWorkInFAO-00+}_20150313

	dev_info(&client->dev, "RT4501 backlight probe OK.\n");

	pr_debug("\n\n******************** [HL] %s, --- **********************\n\n", __func__);

	return 0;
}

static int rt4501_remove(struct i2c_client *client)
{
	struct rt4501_chip_data *pchip = i2c_get_clientdata(client);
	struct	rt4501_platform_data *pdata;

	pdata = pchip->pdata;
	if (!pdata) {
		pr_err("%s: pdata is NULL\n", __func__);
		return -1;
	}

	if(pdata)
	{
		mutex_destroy(&pdata->io_lock);
		gpio_free(pdata->gpio_enable_pin);
	}

	//SW4-HL-Display-FTM-FixLCMCommandNotWorkInFAO-00+_20150313
	device_remove_file(&client->dev, &dev_attr_level);

	return 0;
}

//device tree id table
static struct of_device_id rt_match_table[] = {
	{ .compatible = "rt,rt4501", },
	{},
};

static const struct i2c_device_id rt_id[] = {
	{ RT4501_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver rt4501_driver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name = RT4501_DEV_NAME,
		.of_match_table = rt_match_table,//device tree
		//SW4-HL-Display-BringUpNT35521-00+{_20150224
		//.owner = THIS_MODULE,
		//.pm		= &pm_ops,
		//SW4-HL-Display-BringUpNT35521-00+}_20150224
	},
	.probe = rt4501_probe,
	.remove = rt4501_remove,
	.id_table = rt_id,
};


static int __init rt4501_init(void)
{
	return i2c_add_driver(&rt4501_driver);
}

static void __exit rt4501_exit(void)
{
	i2c_del_driver(&rt4501_driver);
}
module_init(rt4501_init);
module_exit(rt4501_exit);

MODULE_DESCRIPTION("Richtek Backlight Driver for RT4501");
MODULE_AUTHOR("Richtek Technology Corp.");
MODULE_VERSION(RT4501_DRV_VER);
MODULE_LICENSE("GPL");
