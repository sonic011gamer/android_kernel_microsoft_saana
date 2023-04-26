/*
 * NT50358 Boost Driver
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
#include <linux/platform_data/boost_nt50358.h>

/* Black Box */
#define BBOX_BACKLIGHT_I2C_FAIL do {printk("BBox;%s: I2C fail\n", __func__); printk("BBox::UEC;1::1\n");} while (0);
//#define BBOX_BACKLIGHT_GPIO_FAIL do {printk("BBox;%s: GPIO fail\n", __func__); printk("BBox::UEC;1::1\n");} while (0);

/*static unsigned char nt50358_init_data[] =
{ 0x87,
  0x40,
};*/

struct nt50358_chip_data
{
	struct device *dev;
	struct nt50358_platform_data *pdata;
	struct regmap *regmap;
};

struct i2c_client *nt50358_i2c_client;

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
/*static int nt50358_init_reg(void)
{
	struct	i2c_client *client = nt50358_i2c_client;
	struct	nt50358_chip_data *pchip;
	struct	nt50358_platform_data *pdata;
	int ret = 0;

	pr_debug("\n\n******************** [HL] %s, +++, nt50358_init_data[0] = 0x%x, nt50358_init_data[1] = 0x%x **********************\n\n", __func__, nt50358_init_data[0], nt50358_init_data[1]);

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

	ret = reg_write(pchip->regmap, NT50358_CONFIG, nt50358_init_data[0]);
	if (ret < 0)
		goto out;

	ret = reg_write(pchip->regmap, NT50358_TIMING, nt50358_init_data[1]);
	if (ret < 0)
		goto out;

	pr_debug("\n\n******************** [HL] %s, --- **********************\n\n", __func__);

	return ret;

out:
	pr_debug("\n\n******************** [HL] %s ---, i2c failed to access register return ret = %d **********************\n\n", __func__, ret);

	BBOX_BACKLIGHT_I2C_FAIL
	return ret;
}*/

int nt50358_set_avdd_voltage(void)
{
	struct	i2c_client *client = nt50358_i2c_client;
	struct	nt50358_chip_data *pchip;
	struct	nt50358_platform_data *pdata;
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

	ret = reg_write(pchip->regmap, NT50358_SET_AVDD_VOLTAGE, pdata->avdd_voltage);
	if (ret < 0)
		goto out;

	reg_read(pchip->regmap, NT50358_SET_AVDD_VOLTAGE, &reg_val);
	pr_debug("\n\n*** [HL] %s: AVDD reg_val = 0x%x ***\n\n", __func__, reg_val);
	if (reg_val != pdata->avdd_voltage)
	{
		for (i = 0; i < 3; i ++)
		{
			mdelay(50);
			ret = reg_write(pchip->regmap, NT50358_SET_AVDD_VOLTAGE, pdata->avdd_voltage);
			if (ret < 0)
				continue;
			reg_read(pchip->regmap, NT50358_SET_AVDD_VOLTAGE, &reg_val);
			if (reg_val != pdata->avdd_voltage)
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
EXPORT_SYMBOL(nt50358_set_avdd_voltage);

int nt50358_set_avee_voltage(void)
{
	struct	i2c_client *client = nt50358_i2c_client;
	struct	nt50358_chip_data *pchip;
	struct	nt50358_platform_data *pdata;
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

	ret = reg_write(pchip->regmap, NT50358_SET_AVEE_VOLTAGE, pdata->avee_voltage);
	if (ret < 0)
		goto out;

	reg_read(pchip->regmap, NT50358_SET_AVEE_VOLTAGE, &reg_val);
	pr_debug("\n\n*** [HL] %s: AVEE reg_val = 0x%x ***\n\n", __func__, reg_val);
	if (reg_val != pdata->avee_voltage)
	{
		for (i = 0; i < 3; i ++)
		{
			mdelay(50);
			ret = reg_write(pchip->regmap, NT50358_SET_AVEE_VOLTAGE, pdata->avee_voltage);
			if (ret < 0)
				continue;
			reg_read(pchip->regmap, NT50358_SET_AVEE_VOLTAGE, &reg_val);
			if (reg_val != pdata->avee_voltage)
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
EXPORT_SYMBOL(nt50358_set_avee_voltage);


static int nt50358_parse_dt(struct nt50358_platform_data *pdata, struct device *dev)
{
	#ifdef CONFIG_OF
	u32 tmp = 0;
	int rc= 0;
	struct device_node *np = dev->of_node;

	rc = of_property_read_u32(np, "novatek,avdd-voltage", &tmp);
	pdata->avdd_voltage = (!rc ? tmp : NT50358_5_8_V);
	rc = of_property_read_u32(np, "novatek,avee-voltage", &tmp);
	pdata->avee_voltage = (!rc ? tmp : NT50358_5_8_V);

	pr_debug("\n\n*** [HL] %s ---, pdata->avdd_voltage = %d ***n\n", __func__, pdata->avdd_voltage);
	pr_debug("\n\n*** [HL] %s ---, pdata->avee_voltage = %d ***n\n", __func__, pdata->avee_voltage);
	#endif
	return 0;
}

static const struct regmap_config nt50358_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = NT50358_MAX_REG,
};

static int nt50358_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct nt50358_chip_data *pchip;
	struct nt50358_platform_data *pdata;
	int ret = 0;

	pr_debug("\n\n******************** [HL] %s, +++ **********************\n\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check...\n");
		return -EOPNOTSUPP;
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct nt50358_chip_data),
			     GFP_KERNEL);
	if (!pchip)
	{
		dev_err(&client->dev, "%s: Failed to alloc mem for nt50358_chip_data\n", __func__);
		return -ENOMEM;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct nt50358_platform_data),
			     GFP_KERNEL);
	if (!pdata)
	{
		dev_err(&client->dev, "%s: Failed to alloc mem for nt50358_platform_data\n", __func__);
		return -ENOMEM;
	}

	nt50358_parse_dt(pdata, &client->dev);

	//chip data initialize
	pchip->pdata = pdata;
	pchip->dev = &client->dev;

	pchip->regmap = devm_regmap_init_i2c(client, &nt50358_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "fail : allocate register map: %d\n",
			ret);
		return ret;
	}
	i2c_set_clientdata(client, pchip);

	nt50358_i2c_client = client;

	//start mutex
	mutex_init(&pdata->io_lock);

	//init register map
	//nt50358_init_reg();

	dev_info(&client->dev, "NT50358 backlight probe OK.\n");

	pr_debug("\n\n******************** [HL] %s, --- **********************\n\n", __func__);

	return 0;
}

static int nt50358_remove(struct i2c_client *client)
{
	struct nt50358_chip_data *pchip = i2c_get_clientdata(client);
	struct	nt50358_platform_data *pdata;

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
static struct of_device_id nt50358_match_table[] = {
	{ .compatible = "novatek,nt50358", },
	{},
};

static const struct i2c_device_id nt50358_id[] = {
	{ NT50358_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver nt50358_driver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name = NT50358_DEV_NAME,
		.of_match_table = nt50358_match_table,//device tree
	},
	.probe = nt50358_probe,
	.remove = nt50358_remove,
	.id_table = nt50358_id,
};


static int __init nt50358_init(void)
{
	return i2c_add_driver(&nt50358_driver);
}

static void __exit nt50358_exit(void)
{
	i2c_del_driver(&nt50358_driver);
}
module_init(nt50358_init);
module_exit(nt50358_exit);

MODULE_DESCRIPTION("NovaTek Power Boost Converter Driver for NT50358");
MODULE_AUTHOR("FIH Corp.");
MODULE_VERSION(NT50358_DRV_VER);
MODULE_LICENSE("GPL");

