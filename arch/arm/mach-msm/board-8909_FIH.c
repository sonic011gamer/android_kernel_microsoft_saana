/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include "board-dt.h"
#include "platsmp.h"
#include <fih/hwid.h> //Fihtdc lib.

static const char *msm8909_dt_match[] __initconst = {
	"qcom,msm8909-fih",	
	"qcom,msm8909",
	"qcom,apq8009",
	NULL
};

static void __init msm8909_fih_init(void)
{
	board_dt_populate(NULL);

	pr_info("msm8909_init: FIH E1M\n");

#ifdef CONFIG_FIH_HWCONFIG
	fih_hwid_setup(); /* add hwid initial */
#endif
	
}

DT_MACHINE_START(MSM8909_DT,
	"Qualcomm Technologies, Inc. MSM 8909 (Flattened Device Tree) for FIH 8909 Project")
	.init_machine	= msm8909_fih_init,
	.dt_compat	= msm8909_dt_match,
	.smp	= &msm8909_smp_ops,
MACHINE_END
