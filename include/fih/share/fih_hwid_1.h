#ifndef __FIH_HWID_1_H
#define __FIH_HWID_1_H

struct st_hwid_table {
	char r1; /* r1 is hundreds digit */
	char r2; /* r2 is tens digit     */
	char r3; /* r3 is units digit    */ /* NOT USED IN FAO */
	/* info */
	char prj; /* project */
	char rev; /* hw_rev */
	char rf;  /* rf_band */
	/* device tree */
	char dtm; /* Major number */
	char dtn; /* minor Number */
	/* driver */
	char btn; /* button */
	char uart;
	char module; /* operator */
	char sim;	/* SIM Count */
};

enum {
	/* hwid_value is equal: r1*100 + r2*10 + r3 */
	FIH_HWID_R1 = 0,	/* r1 is hundreds digit */
	FIH_HWID_R2,		/* r2 is tens digit     */
	FIH_HWID_R3,		/* r3 is units digit    */
	/* info */
	FIH_HWID_PRJ,
	FIH_HWID_REV,
	FIH_HWID_RF,
	/* device tree */
	FIH_HWID_DTM,
	FIH_HWID_DTN,
	/* driver */
	FIH_HWID_BTN,
	FIH_HWID_UART,
	/* others */
	FIH_HWID_MODULE,
	FIH_HWID_SIM,
	/* max */
	FIH_HWID_MAX
};

/******************************************************************************
 *  device tree
 *****************************************************************************/

enum {
	FIH_DTM_DEFAULT = 0,
	FIH_DTM_MAX = 255,
};

enum {
	FIH_DTN_DEFAULT = 0,
	FIH_DTN_MAX = 255,
};

/******************************************************************************
 *  driver
 *****************************************************************************/

enum {
	FIH_BTN_DEFAULT = 0,
	FIH_BTN_NONE,
	FIH_BTN_PMIC_UP_5_DN_2,
	FIH_BTN_MAX
};

enum {
	FIH_UART_DEFAULT = 0,
	FIH_UART_NONE,
	FIH_UART_TX_4_RX_5,
	FIH_UART_MAX
};

enum {
	FIH_MODULE_DEFAULT = 0,
	FIH_MODULE_WW,
	FIH_MODULE_TW,
	FIH_MODULE_CN,
	FIH_MODULE_SKT,
	FIH_MODULE_IN,
	FIH_MODULE_EU,
	FIH_MODULE_APAC,
	FIH_MODULE_LATAM,
	FIH_MODULE_MAX
};

enum {
	FIH_SIM_DEFAULT = 0,
	FIH_SIM_SINGLE,
	FIH_SIM_DUAL,
	FIH_SIM_MAX
};

#endif /* __FIH_HWID_1_H */
