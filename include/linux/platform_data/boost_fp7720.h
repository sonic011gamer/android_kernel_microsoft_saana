/*
 * Copyright (C) 2015 FIH Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef LINUX_LEDS_FP7720_BACKLIGHT_H
#define LINUX_LEDS_FP7720_BACKLIGHT_H

#define FP7720_DRV_VER 		"1.0.0G"
#define FP7720_DEV_NAME 	"fp7720"

enum
{
	FP7720_AVDD = 0x00,
	FP7720_AVEE,
	FP7720_FAULT_BLANKING_DISCHARGED_ENABLE = 0x03,
	FP7720_MAX_REG,
};

enum
{
	FP7720_4_0_V,
	FP7720_4_1_V,
	FP7720_4_2_V,
	FP7720_4_3_V,
	FP7720_4_4_V,
	FP7720_4_5_V,
	FP7720_4_6_V,
	FP7720_4_7_V,
	FP7720_4_8_V,
	FP7720_4_9_V,
	FP7720_5_0_V,
	FP7720_5_1_V,
	FP7720_5_2_V,
	FP7720_5_3_V,
	FP7720_5_4_V,
	FP7720_5_5_V,
	FP7720_5_6_V,
	FP7720_5_7_V,
	FP7720_5_8_V,
	FP7720_5_9_V,
	FP7720_6_0_V,
};

struct fp7720_platform_data
{
	struct mutex io_lock;
};

extern int fp7720_set_avdd_voltage(void);
extern int fp7720_set_avee_voltage(void);
#endif
