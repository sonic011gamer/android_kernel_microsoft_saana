/*
 * Copyright (C) 2015 FIH Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef LINUX_BOOST_NT50358_H
#define LINUX_BOOST_NT50358_H

#define NT50358_DRV_VER 		"1.0.0"
#define NT50358_DEV_NAME 	"nt50358"

enum
{
	NT50358_SET_AVDD_VOLTAGE = 0x00,
	NT50358_SET_AVEE_VOLTAGE,
	NT50358_FAULT_BLANKING_DISCHARGED_ENABLE = 0x03,
	NT50358_MAX_REG,
};

enum
{
	NT50358_4_0_V,
	NT50358_4_1_V,
	NT50358_4_2_V,
	NT50358_4_3_V,
	NT50358_4_4_V,
	NT50358_4_5_V,
	NT50358_4_6_V,
	NT50358_4_7_V,
	NT50358_4_8_V,
	NT50358_4_9_V,
	NT50358_5_0_V,
	NT50358_5_1_V,
	NT50358_5_2_V,
	NT50358_5_3_V,
	NT50358_5_4_V,
	NT50358_5_5_V,
	NT50358_5_6_V,
	NT50358_5_7_V,
	NT50358_5_8_V,
	NT50358_5_9_V,
	NT50358_6_0_V,
};

struct nt50358_platform_data
{
	struct mutex io_lock;
	u32 avdd_voltage;
	u32 avee_voltage;
};

extern int nt50358_set_avdd_voltage(void);
extern int nt50358_set_avee_voltage(void);
#endif
