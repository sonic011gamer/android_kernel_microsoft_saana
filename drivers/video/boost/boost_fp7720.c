/*
 * FP7720 Boost Driver
 *
 * Copyright (C) 2015 FIH Corp.
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
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/platform_data/boost_fp7720.h>

/* Black Box */
#define BBOX_BACKLIGHT_I2C_FAIL do {printk("BBox;%s: I2C fail\n", __func__); printk("BBox::UEC;1::1\n");} while (0);
//#define BBOX_BACKLIGHT_GPIO_FAIL do {printk("BBox;%s: GPIO fail\n", __func__); printk("BBox::UEC;1::1\n");} while (0);

/*static unsigned char fp7720_init_data[] =
{ 0x87,
  0x40,
};*/

struct fp7720_chip_data
{
	struct device *dev;
	struct fp7720_platform_data *pdata;
	struct regmap *regmap;
};

struct i2c_client *fp7720_i2c_client;

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

//init register map
/*static int fp7720_init_reg(void)
{
	struct	i2c_client *client = fp7720_i2c_client;
	struct	fp7720_chip_data *pchip;
	struct	fp7720_platform_data *pdata;
	int ret = 0;

	pr_debug("\n\n******************** [HL] %s, +++, fp7720_init_data[0] = 0x%x, fp7720_init_data[1] = 0x%x **********************\n\n", __func__, fp7720_init_data[0], fp7720_init_data[1]);

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

	ret = reg_write(pchip->regmap, FP7720_CONFIG, fp7720_init_data[0]);
	if (ret < 0)
		goto out;

	ret = reg_write(pchip->regmap, FP7720_TIMING, fp7720_init_data[1]);
	if (ret < 0)
		goto out;

	pr_debug("\n\n******************** [HL] %s, --- **********************\n\n", __func__);

	return ret;

out:
	pr_debug("\n\n******************** [HL] %s ---, i2c failed to access register return ret = %d **********************\n\n", __func__, ret);

	BBOX_BACKLIGHT_I2C_FAIL
	return ret;
}*/

int fp7720_set_avdd_voltage(void)
{
	struct	i2c_client *client = fp7720_i2c_client;
	struct	fp7720_chip_data *pchip;
	struct	fp7720_platform_data *pdata;
	int ret = 0;
	int i = 0;
	unsigned int reg_val;

	pr_debug("\n\n*** [HL] %s, +++, ***n\n", __func__);

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

	ret = reg_write(pchip->regmap, FP7720_AVDD, FP7720_5_8_V);
	if (ret < 0)
		goto out;

	reg_read(pchip->regmap, FP7720_AVDD, &reg_val);
	pr_debug("\n\n*** [HL] %s: AVDD reg_val = 0x%x ***\n\n", __func__, reg_val);
	if (reg_val != FP7720_5_8_V)
	{
		for (i = 0; i < 3; i ++)
		{
			mdelay(50);
			ret = reg_write(pchip->regmap, FP7720_AVDD, FP7720_5_8_V);
			if (ret < 0)
				continue;
			reg_read(pchip->regmap, FP7720_AVDD, &reg_val);
			if (reg_val != FP7720_5_8_V)
				continue;
			else
				break;
		}
	}

	pr_debug("\n\n*** [HL] %s, --- ***n\n", __func__);

	return ret;

out:
	pr_debug("\n\n*** [HL] %s ---, i2c failed to access register return ret = %d ***n\n", __func__, ret);

	BBOX_BACKLIGHT_I2C_FAIL
	return ret;
}
EXPORT_SYMBOL(fp7720_set_avdd_voltage);

int fp7720_set_avee_voltage(void)
{
	struct	i2c_client *client = fp7720_i2c_client;
	struct	fp7720_chip_data *pchip;
	struct	fp7720_platform_data *pdata;
	int ret = 0;
	int i = 0;
	unsigned int reg_val;

	pr_debug("\n\n*** [HL] %s, +++, ***n\n", __func__);

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

	ret = reg_write(pchip->regmap, FP7720_AVEE, FP7720_5_8_V);
	if (ret < 0)
		goto out;

	reg_read(pchip->regmap, FP7720_AVEE, &reg_val);
	pr_debug("\n\n*** [HL] %s: AVEE reg_val = 0x%x ***\n\n", __func__, reg_val);
	if (reg_val != FP7720_5_8_V)
	{
		for (i = 0; i < 3; i ++)
		{
			mdelay(50);
			ret = reg_write(pchip->regmap, FP7720_AVEE, FP7720_5_8_V);
			if (ret < 0)
				continue;
			reg_read(pchip->regmap, FP7720_AVEE, &reg_val);
			if (reg_val != FP7720_5_8_V)
				continue;
			else
				break;
		}
	}

	pr_debug("\n\n*** [HL] %s, --- ***n\n", __func__);

	return ret;

out:
	pr_debug("\n\n*** [HL] %s ---, i2c failed to access register return ret = %d ***n\n", __func__, ret);

	BBOX_BACKLIGHT_I2C_FAIL
	return ret;
}
EXPORT_SYMBOL(fp7720_set_avee_voltage);


/*static int rt_parse_dt(struct fp7720_platform_data *pdata, struct device *dev)
{
	#ifdef CONFIG_OF
	u32 tmp = 0;
	struct device_node *np = dev->of_node;
	pdata->gpio_enable_pin = of_get_named_gpio(np, "rt,en_pin", 0);

	if (of_property_read_bool(np, "rt,ovp_sel"))
		fp7720_init_data[0] |= FP7720_OVPSEL_MASK;
	else
		fp7720_init_data[0] &= ~FP7720_OVPSEL_MASK;

	#endif
	return 0;
}*/

static const struct regmap_config fp7720_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FP7720_MAX_REG,
};

static int fp7720_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fp7720_chip_data *pchip;
	struct fp7720_platform_data *pdata;
	int ret = 0;

	pr_debug("\n\n******************** [HL] %s, +++ **********************\n\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check...\n");
		return -EOPNOTSUPP;
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct fp7720_chip_data),
			     GFP_KERNEL);
	if (!pchip)
	{
		dev_err(&client->dev, "%s: Failed to alloc mem for fp7720_chip_data\n", __func__);
		return -ENOMEM;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct fp7720_platform_data),
			     GFP_KERNEL);
	if (!pdata)
	{
		dev_err(&client->dev, "%s: Failed to alloc mem for fp7720_platform_data\n", __func__);
		return -ENOMEM;
	}

	//rt_parse_dt(pdata, &client->dev);

	//chip data initialize
	pchip->pdata = pdata;
	pchip->dev = &client->dev;

	pchip->regmap = devm_regmap_init_i2c(client, &fp7720_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "fail : allocate register map: %d\n",
			ret);
		return ret;
	}
	i2c_set_clientdata(client, pchip);

	fp7720_i2c_client = client;

	//start mutex
	mutex_init(&pdata->io_lock);

	//init register map
	//fp7720_init_reg();

	dev_info(&client->dev, "FP7720 backlight probe OK.\n");

	pr_debug("\n\n******************** [HL] %s, --- **********************\n\n", __func__);

	return 0;
}

static int fp7720_remove(struct i2c_client *client)
{
	struct fp7720_chip_data *pchip = i2c_get_clientdata(client);
	struct	fp7720_platform_data *pdata;

	pdata = pchip->pdata;
	if (!pdata) {
		pr_err("%s: pdata is NULL\n", __func__);
		return -1;
	}

	if(pdata)
	{
		mutex_destroy(&pdata->io_lock);
	}

	return 0;
}

//device tree id table
static struct of_device_id fp7720_match_table[] = {
	{ .compatible = "fp,fp7720", },
	{},
};

static const struct i2c_device_id fp7720_id[] = {
	{ FP7720_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver fp7720_driver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name = FP7720_DEV_NAME,
		.of_match_table = fp7720_match_table,//device tree
	},
	.probe = fp7720_probe,
	.remove = fp7720_remove,
	.id_table = fp7720_id,
};


static int __init fp7720_init(void)
{
	return i2c_add_driver(&fp7720_driver);
}

static void __exit fp7720_exit(void)
{
	i2c_del_driver(&fp7720_driver);
}
module_init(fp7720_init);
module_exit(fp7720_exit);

MODULE_DESCRIPTION("Fiti Power Boost Converter Driver for FP7720");
MODULE_AUTHOR("OEM Corp.");
MODULE_VERSION(FP7720_DRV_VER);
MODULE_LICENSE("GPL");

