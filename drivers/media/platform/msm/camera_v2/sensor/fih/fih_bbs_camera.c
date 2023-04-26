/****************************************************************************
* File: fih_bbs_camera.c                                                    *
* Description: write camera error message to bbs log                        *
*                                                                           *
****************************************************************************/

/****************************************************************************
*                               Include File                                *
****************************************************************************/
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/io.h>
#include "fih_bbs_camera.h"

/****************************************************************************
*                               Module I2C ADDR                             *
****************************************************************************/
struct fih_bbs_camera_module_list {
	u8 i2c_addr;
	u8 type;
	char module_name[32];
};

//S5K4H8
#define FIH_I2C_ADDR_S5K4H8_CAMERA 0x5A
//#define FIH_I2C_ADDR_S5K3P3_EEPROM 0xA2
//#define FIH_I2C_ADDR_S5K3P3_ACTUATOR 0x18
//S5k5E8YX13
#define FIH_I2C_ADDR_S5k5E8YX13_CAMERA 0x5A
//S5K5E8
#define FIH_I2C_ADDR_S5K5E8_CAMERA 0x20
/*VZ1 component end*/
/****************************************************************************
*                          Private Type Declaration                         *
****************************************************************************/

struct fih_bbs_camera_module_list fih_camera_list[] = {
	{
	.i2c_addr = FIH_I2C_ADDR_S5K4H8_CAMERA,
	.type = ((FIH_BBS_CAMERA_LOCATION_FRONT << 4) | FIH_BBS_CAMERA_MODULE_IC),
	.module_name = "Camera/EEPROM (S5K4H8)",
	},
/*	{
	.i2c_addr = FIH_I2C_ADDR_S5K3P3_EEPROM,
	.type = ((FIH_BBS_CAMERA_LOCATION_MAIN << 4) | FIH_BBS_CAMERA_MODULE_EEPROM),
	.module_name = "EEPROM (S5K3P3)",
	},
	{
	.i2c_addr = FIH_I2C_ADDR_S5K3P3_ACTUATOR,
	.type = ((FIH_BBS_CAMERA_LOCATION_MAIN << 4) | FIH_BBS_CAMERA_MODULE_ACTUATOR),
	.module_name = "Actuator (S5K3P3)",
	},*/
	{
	.i2c_addr = FIH_I2C_ADDR_S5k5E8YX13_CAMERA,
	.type = ((FIH_BBS_CAMERA_LOCATION_FRONT << 4) | FIH_BBS_CAMERA_MODULE_IC),
	.module_name = "Camera/EEPROM (S5K4H8)",
	},
};


/**********************************************************************
                         Public Function                              *
**********************************************************************/

void fih_bbs_camera_msg(int module, int error_code)
{
  char param1[32]; // Camera ic, actuator, EEPROM, OIS,¡K
  char param2[64]; // I2C flash error, power up fail, ¡K

	switch (module) {
		case FIH_BBS_CAMERA_MODULE_IC: strcpy(param1, "Camera ic"); break;
		case FIH_BBS_CAMERA_MODULE_EEPROM: strcpy(param1, "EEPROM"); break;
		case FIH_BBS_CAMERA_MODULE_ACTUATOR: strcpy(param1, "Actuator"); break;
		case FIH_BBS_CAMERA_MODULE_OIS: strcpy(param1, "OIS"); break;
		default: strcpy(param1, "Unknown"); break;
  }

	switch (error_code) {
		case FIH_BBS_CAMERA_ERRORCODE_POWER_UP: strcpy(param2, "Power up fail"); break;
		case FIH_BBS_CAMERA_ERRORCODE_POWER_DW: strcpy(param2, "Power down fail"); break;
		case FIH_BBS_CAMERA_ERRORCODE_MCLK_ERR: strcpy(param2, "MCLK error"); break;
		case FIH_BBS_CAMERA_ERRORCODE_I2C_READ: strcpy(param2, "i2c read err"); break;
		case FIH_BBS_CAMERA_ERRORCODE_I2C_WRITE: strcpy(param2, "i2c write err"); break;
		case FIH_BBS_CAMERA_ERRORCODE_I2C_WRITE_SEQ: strcpy(param2, "i2c seq write err"); break;
		case FIH_BBS_CAMERA_ERRORCODE_UNKOWN: strcpy(param2, "unknow error"); break;
		default: strcpy(param2, "Unknown"); break;
	}

  printk("BBox::UPD;88::%s::%s\n", param1, param2);

  return ;
}
EXPORT_SYMBOL(fih_bbs_camera_msg);

void fih_bbs_camera_msg_by_type(int type, int error_code)
{
  char param1[32]; // Camera ic, actuator, EEPROM, OIS,¡K
  char param2[64]; // I2C flash error, power up fail, ¡K
  int isList=0;
  int i=0;

  for(i=0; i<(sizeof(fih_camera_list)/sizeof(struct fih_bbs_camera_module_list)); i++)
  {
    if(type == fih_camera_list[i].type)
    {
      strcpy(param1, fih_camera_list[i].module_name);
      isList = 1;
      break;
    }
  }

  //Do not find in list.
  if(!isList)
  {
    strcpy(param1, "Unknown ");
  }

	switch (error_code) {
		case FIH_BBS_CAMERA_ERRORCODE_POWER_UP: strcpy(param2, "Power up fail"); break;
		case FIH_BBS_CAMERA_ERRORCODE_POWER_DW: strcpy(param2, "Power down fail"); break;
		case FIH_BBS_CAMERA_ERRORCODE_MCLK_ERR: strcpy(param2, "MCLK error"); break;
		case FIH_BBS_CAMERA_ERRORCODE_I2C_READ: strcpy(param2, "i2c read err"); break;
		case FIH_BBS_CAMERA_ERRORCODE_I2C_WRITE: strcpy(param2, "i2c write err"); break;
		case FIH_BBS_CAMERA_ERRORCODE_I2C_WRITE_SEQ: strcpy(param2, "i2c seq write err"); break;
		case FIH_BBS_CAMERA_ERRORCODE_UNKOWN: strcpy(param2, "unknow error"); break;
		default: strcpy(param2, "Unknown"); break;
	}

  printk("BBox::UPD;88::%s::%s\n", param1, param2);

  return ;
}
EXPORT_SYMBOL(fih_bbs_camera_msg_by_type);

void fih_bbs_camera_msg_by_addr(int addr, int error_code)
{
  char param1[32]; // Camera ic, actuator, EEPROM, OIS,¡K
  char param2[64]; // I2C flash error, power up fail, ¡K
  int isList=0;
  int i=0;
  u8 bbs_id=99,bbs_err=99,camera=0,module=0;

  for(i=0; i<(sizeof(fih_camera_list)/sizeof(struct fih_bbs_camera_module_list)); i++)
  {
    if(addr == (fih_camera_list[i].i2c_addr>>1))
    {
      camera = fih_camera_list[i].type >> 4;
      module = fih_camera_list[i].type & 0xf;
      if(camera == FIH_BBS_CAMERA_LOCATION_MAIN)
        bbs_id = FIH_BBSUEC_MAIN_CAM_ID;
      else if(camera == FIH_BBS_CAMERA_LOCATION_FRONT)
        bbs_id = FIH_BBSUEC_FRONT_CAM_ID;
      strcpy(param1, fih_camera_list[i].module_name);
      isList = 1;
      break;
    }
  }

  //Do not find in list.
  if(!isList)
    strcpy(param1, "Unknown module");

	switch (error_code) {
		case FIH_BBS_CAMERA_ERRORCODE_POWER_UP: strcpy(param2, "Power up fail"); break;
		case FIH_BBS_CAMERA_ERRORCODE_POWER_DW: strcpy(param2, "Power down fail"); break;
		case FIH_BBS_CAMERA_ERRORCODE_MCLK_ERR: strcpy(param2, "MCLK error"); break;
		case FIH_BBS_CAMERA_ERRORCODE_I2C_READ: strcpy(param2, "i2c read err"); break;
		case FIH_BBS_CAMERA_ERRORCODE_I2C_WRITE: strcpy(param2, "i2c write err"); break;
		case FIH_BBS_CAMERA_ERRORCODE_I2C_WRITE_SEQ: strcpy(param2, "i2c seq write err"); break;
		case FIH_BBS_CAMERA_ERRORCODE_UNKOWN: strcpy(param2, "unknow error"); break;
		default: strcpy(param2, "Unknown"); break;
	}
	switch (error_code) {
		case FIH_BBS_CAMERA_ERRORCODE_POWER_UP:
		case FIH_BBS_CAMERA_ERRORCODE_POWER_DW:
			if (module == FIH_BBS_CAMERA_MODULE_IC)
				bbs_err = FIH_BBSUEC_CAMERA_ERRORCODE_SENSOR_POWER;
			else if (module == FIH_BBS_CAMERA_MODULE_ACTUATOR)
				bbs_err = FIH_BBSUEC_CAMERA_ERRORCODE_ACTUATOR_POWER;
			break;
		case FIH_BBS_CAMERA_ERRORCODE_MCLK_ERR:
			break;
		case FIH_BBS_CAMERA_ERRORCODE_I2C_READ:
		case FIH_BBS_CAMERA_ERRORCODE_I2C_WRITE:
		case FIH_BBS_CAMERA_ERRORCODE_I2C_WRITE_SEQ:
			if (module == FIH_BBS_CAMERA_MODULE_IC)
				bbs_err = FIH_BBSUEC_CAMERA_ERRORCODE_SENSOR_I2C;
			else if (module == FIH_BBS_CAMERA_MODULE_EEPROM)
				bbs_err = FIH_BBSUEC_CAMERA_ERRORCODE_EEPROM_I2C;
			else if (module == FIH_BBS_CAMERA_MODULE_ACTUATOR)
				bbs_err = FIH_BBSUEC_CAMERA_ERRORCODE_ACTUATOR_I2C;
			break;
		case FIH_BBS_CAMERA_ERRORCODE_UNKOWN:
		default:
			break;
	}

  printk("BBox::UPD;88::%s::%s\n", param1, param2);
  printk("BBox::UEC;%d::%d\n", bbs_id, bbs_err);

  return ;
}
EXPORT_SYMBOL(fih_bbs_camera_msg_by_addr);

/************************** End Of File *******************************/
