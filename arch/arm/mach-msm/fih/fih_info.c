#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <fih/hwid.h>

enum SWID_BSP{
    BSP_DEFAULT = 0,
    BSP_DDR2 = 1<<0,
    BSP_DDR3 = 1<<1,
    BSP_GYRO = 1<<2,
    BSP_NAND_CONTROLLER
};

static int fih_info_proc_read_project_show(struct seq_file *m, void *v)
{
	char msg[8];

	switch (fih_hwid_fetch(FIH_HWID_PRJ)) {
		case FIH_PRJ_FAO:
		case FIH_PRJ_FAO_FOK:
		case FIH_PRJ_FAO_FAT:
		case FIH_PRJ_FAO_CHINA:
		case FIH_PRJ_FAO_FAE:
		case FIH_PRJ_DNT:
		case FIH_PRJ_FAO_FO7:
			strcpy(msg, "FAO");
		break;
    //D1M
		case FIH_PRJ_D1M_INDIA:
		case FIH_PRJ_D1M_EMEA:
		case FIH_PRJ_D1M_APAC:
		case FIH_PRJ_D1M_CHINA:
		case FIH_PRJ_D1M_LATAM:
			strcpy(msg, "D1M");
			break;
		case FIH_PRJ_E1M:
			strcpy(msg, "E1M");
			break;
                case FIH_PRJ_AT2:
                        strcpy(msg, "AT2");
                        break;
		default: strcpy(msg, "N/A"); break;
	}
	seq_printf(m, "%s\n", msg);

	return 0;
}

static int devmodel_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_info_proc_read_project_show, NULL);
};

static struct file_operations devmodel_file_ops = {
	.owner   = THIS_MODULE,
	.open    = devmodel_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int fih_info_proc_read_hw_rev_show(struct seq_file *m, void *v)
{
	char msg[8];

	switch (fih_hwid_fetch(FIH_HWID_REV)) {
		case FIH_REV_EVB:  strcpy(msg, "EVB"); break;
		case FIH_REV_EVB2: strcpy(msg, "EVB2"); break;
		case FIH_REV_EVB3: strcpy(msg, "EVB3"); break;
		case FIH_REV_EVT0:  strcpy(msg, "1.0"); break;
		case FIH_REV_EVT1: strcpy(msg, "1.1"); break;
		case FIH_REV_EVT2: strcpy(msg, "1.5"); break;
		case FIH_REV_DVT:  strcpy(msg, "2.0"); break;
		case FIH_REV_DVT2: strcpy(msg, "2.2"); break;
		case FIH_REV_DVT3: strcpy(msg, "2.3"); break;
		case FIH_REV_PVT:  strcpy(msg, "3.0"); break;
		case FIH_REV_PVT2: strcpy(msg, "3.2"); break;
		case FIH_REV_PVT3: strcpy(msg, "3.3"); break;
		case FIH_REV_MP:   strcpy(msg, "5.0"); break;
		case FIH_REV_MP2:  strcpy(msg, "5.2"); break;
		case FIH_REV_MP3:  strcpy(msg, "5.3"); break;
		default: strcpy(msg, "N/A"); break;
	}
	seq_printf(m, "%s\n", msg);

	return 0;
}
static int baseband_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_info_proc_read_hw_rev_show, NULL);
};

static struct file_operations baseband_file_ops = {
	.owner   = THIS_MODULE,
	.open    = baseband_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int fih_info_proc_read_rf_band_show(struct seq_file *m, void *v)
{
	char msg[128];

	switch (fih_hwid_fetch(FIH_HWID_RF)) {
		case FIH_BAND_DEFAULT: strcpy(msg, "NONE"); break;
		/* GSM + EDGE + UMTS + FDD-LTE */
		case FIH_BAND_G_850_900_1800_1900_W_1_2_L_1_2_3_4_5_7_9_12_17_25_26:
			strcpy(msg, "G_850_900_1800_1900^W_1_2^L_1_2_3_4_5_7_9_12_17_25_26"); break;
		/* GSM + EDGE + UMTS + FDD-LTE */
		case FIH_BAND_G_850_900_1800_1900_W_1_2_5_8_L_1_3_7_8_28:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^L_1_3_7_8_28"); break;
		case FIH_BAND_G_850_900_1800_1900_W_1_2_5_8_L_1_3_8_28:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^L_1_3_8_28"); break;
		/* FAO KOR */
		case FIH_BAND_G_850_900_1800_1900_W_1_2_5_8_L_1_3_5_7_9_17:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^L_1_3_5_7_9_17"); break;
		/* FAO IND */
		case FIH_BAND_G_900_1800_W_1_8_L_3_40:
			strcpy(msg, "G_900_1800^W_1_8^L_3_40"); break;
		/* FAO CHN */
		case FIH_BAND_G_850_900_1800_1900_W_1_2_5_8_L_1_2_3_5_7_8_20_38_39_40_41_T_34_39:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^L_1_2_3_5_7_8_20_38_39_40_41^T_34_39"); break;
		/* FAO EU */
		case FIH_BAND_G_850_900_1800_1900_W_1_2_5_8_L_1_3_7_8_20:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^L_1_3_7_8_20"); break;
		/* D1M EMEA */
		case FIH_BAND_G_850_900_1800_1900_W_1_2_5_8_L_1_3_7_20_38_40:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^L_1_3_7_20_38_40"); break;
		/* D1M APAC */
		case FIH_BAND_G_850_900_1800_1900_W_1_2_5_8_L_1_3_5_7_8_28_38_40:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^L_1_3_5_7_8_28_38_40"); break;
		/* D1M CHINA */
		case FIH_BAND_G_850_900_1800_1900_W_1_8_L_1_3_5_38_39_40_41_T_34_39:
			strcpy(msg, "G_850_900_1800_1900^W_1_8^L_1_3_5_38_39_40_41^T_34_39"); break;

                case FIH_BAND_G_850_900_1800_1900_W_1_2_4_5_8_L_1_2_3_4_5_7_8_12_17_20_28_38_40_41:
                        strcpy(msg, "G_850_900_1800_1900^W_1_2_4_5_8^L_1_2_3_4_5_7_8_12_17_20_28_38_40_41"); break;
                case FIH_BAND_G_850_900_1800_1900_W_1_2_5_8_L_1_3_5_7_8_20_28_38_40:
                        strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^L_1_3_5_7_8_20_28_38_40"); break;
                case FIH_BAND_G_850_900_1800_1900_W_1_2_4_5_8_L_2_3_4_7_12_17_28_38:
                        strcpy(msg, "G_850_900_1800_1900^W_1_2_4_5_8^L_2_3_4_7_12_17_28_38"); break;
                case FIH_BAND_G_850_900_1800_1900_W_1_2_4_5_8_L_2_3_4_5_7_12_17_28_38: //= 76, //PVT LATAM:
                        strcpy(msg, "G_850_900_1800_1900^W_1_2_4_5_8^L_2_3_4_5_7_12_17_28_38"); break;
                case FIH_BAND_G_900_1800_W_1_8_L_1_3_5_20_40_41:
                        strcpy(msg, "G_900_1800^W_1_8^L_1_3_5_20_40_41"); break;
                case FIH_BAND_L_1_3_4_8_38_39_40_41:
                       strcpy(msg, "L_1_3_4_8_38_39_40_41"); break;

		case FIH_BAND_WIFI_ONLY: strcpy(msg, "WIFI_ONLY"); break;
		default: strcpy(msg, "UNKNOWN\n"); break;
	}
	seq_printf(m, "%s\n", msg);

	return 0;
}
static int bandinfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_info_proc_read_rf_band_show, NULL);
};

static struct file_operations bandinfo_file_ops = {
	.owner   = THIS_MODULE,
	.open    = bandinfo_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int fih_info_proc_read_hwcfg_show(struct seq_file *m, void *v)
{
	struct st_hwid_table tb;

	fih_hwid_read(&tb);
	/* mpp */
	seq_printf(m, "r1=%d\n", tb.r1);
	seq_printf(m, "r2=%d\n", tb.r2);
	seq_printf(m, "r3=%d\n", tb.r3);
	/* info */
	seq_printf(m, "prj=%d\n", tb.prj);
	seq_printf(m, "rev=%d\n", tb.rev);
	seq_printf(m, "rf=%d\n", tb.rf);
	/* device tree */
	seq_printf(m, "dtm=%d\n", tb.dtm);
	seq_printf(m, "dtn=%d\n", tb.dtn);
	/* driver */
	seq_printf(m, "btn=%d\n", tb.btn);
	seq_printf(m, "uart=%d\n", tb.uart);
	/* Others */
	seq_printf(m, "module=%d\n", tb.module);
	seq_printf(m, "sim=%d\n", tb.sim);

	return 0;
}

static int hwcfg_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_info_proc_read_hwcfg_show, NULL);
};

static struct file_operations hwcfg_file_ops = {
	.owner   = THIS_MODULE,
	.open    = hwcfg_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int fih_info_proc_read_module_show(struct seq_file *m, void *v)
{
	char msg[128];

	switch (fih_hwid_fetch(FIH_HWID_MODULE)) {
		case FIH_MODULE_WW: strcpy(msg, "WW"); break;
		case FIH_MODULE_TW: strcpy(msg, "TW"); break;
		case FIH_MODULE_CN: strcpy(msg, "CN"); break;
		case FIH_MODULE_SKT: strcpy(msg, "SKT"); break;
		case FIH_MODULE_IN: strcpy(msg, "IN"); break;
		case FIH_MODULE_EU: strcpy(msg, "EU"); break;
		default: strcpy(msg, "UNKNOWN\n"); break;
	}
	seq_printf(m, "%s\n", msg);

	return 0;
}

static int module_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_info_proc_read_module_show, NULL);
};

static struct file_operations module_file_ops = {
	.owner   = THIS_MODULE,
	.open    = module_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int fih_info_proc_read_hwmodel_show(struct seq_file *m, void *v)
{
	char msg[12];

	switch (fih_hwid_fetch(FIH_HWID_PRJ)) {
		case FIH_PRJ_FAO: strcpy(msg, "FOT"); break;
		case FIH_PRJ_FAO_FOK: strcpy(msg, "FOK"); break;
		case FIH_PRJ_FAO_FAT: strcpy(msg, "FAT"); break;
		case FIH_PRJ_FAO_CHINA: strcpy(msg, "CHINA"); break;
		case FIH_PRJ_FAO_FAE: strcpy(msg, "FAU"); break;
		case FIH_PRJ_FAO_FO7: strcpy(msg, "FO7"); break;
		case FIH_PRJ_DNT:
		  {
		    if(fih_hwid_fetch(FIH_HWID_UART)&BSP_GYRO){
		      strcpy(msg, "FA9");
		    }else{
		      strcpy(msg, "FA8");
		    }
		  }
		  break;
    //D1M
    //CRITIAL: TODO
		case FIH_PRJ_D1M_INDIA: strcpy(msg, "DIN"); break;
		case FIH_PRJ_D1M_EMEA: strcpy(msg, "DEN"); break;
		case FIH_PRJ_D1M_APAC: strcpy(msg, "DAN"); break;
		case FIH_PRJ_D1M_CHINA: strcpy(msg, "DCN"); break;
		case FIH_PRJ_D1M_LATAM: strcpy(msg, "DLN"); break;
		case FIH_PRJ_E1M:
                {
                    switch(fih_hwid_fetch(FIH_HWID_RF))
                    {
                        case FIH_BAND_G_850_900_1800_1900_W_1_2_4_5_8_L_1_2_3_4_5_7_8_12_17_20_28_38_40_41:
                            strcpy(msg, "E1M_EVB");
                            break;
                        case FIH_BAND_G_850_900_1800_1900_W_1_2_5_8_L_1_3_5_7_8_20_28_38_40:
                            strcpy(msg, "E1A");
                            break;
                        case FIH_BAND_G_850_900_1800_1900_W_1_2_4_5_8_L_2_3_4_7_12_17_28_38:
                        case FIH_BAND_G_850_900_1800_1900_W_1_2_4_5_8_L_2_3_4_5_7_12_17_28_38:
                            strcpy(msg, "E1L");
                            break;
                        case FIH_BAND_G_900_1800_W_1_8_L_1_3_5_20_40_41:
                            strcpy(msg, "E1I");
                            break;
                        default:
                            strcpy(msg, "E1M");
                            break;
                    }
                    break;
                }
                case FIH_PRJ_AT2: strcpy(msg, "AT2"); break;
		default: strcpy(msg, "N/A"); break;
	}
	seq_printf(m, "%s\n", msg);

	return 0;
}

static int hwmodel_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_info_proc_read_hwmodel_show, NULL);
};

static struct file_operations hwmodel_file_ops = {
	.owner   = THIS_MODULE,
	.open    = hwmodel_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int fih_info_proc_read_fqc_xml_show(struct seq_file *m, void *v)
{
	char msg[48];

        struct st_hwid_table tb;

        fih_hwid_read(&tb);


	switch (fih_hwid_fetch(FIH_HWID_PRJ)) {
		case FIH_PRJ_FAO: strcpy(msg, "system/etc/fqc_FAO.xml"); break;
		case FIH_PRJ_FAO_FOK: strcpy(msg, "system/etc/fqc_FAO.xml"); break;
		case FIH_PRJ_FAO_FAT: strcpy(msg, "system/etc/fqc_FAO.xml"); break;
		case FIH_PRJ_FAO_CHINA: strcpy(msg, "system/etc/fqc_FAO.xml"); break;
		case FIH_PRJ_FAO_FAE: strcpy(msg, "system/etc/fqc_FAO.xml"); break;
		case FIH_PRJ_FAO_FO7: strcpy(msg, "system/etc/fqc_FAO.xml"); break;
		case FIH_PRJ_DNT: 
		  {
		    if(fih_hwid_fetch(FIH_HWID_UART)&BSP_GYRO){
		      strcpy(msg, "system/etc/fqc_FAO.xml");
		    }else{
		      strcpy(msg, "system/etc/fqc_FAO.xml");
		    }
		  } 
		  break;
		case FIH_PRJ_E1M:
                  {
                  if(1 == tb.sim)
                      {
                          strcpy(msg, "system/etc/fqc_E1M_single_sim.xml");
                      }
                  else
                      {
                          strcpy(msg, "system/etc/fqc_E1M_dual_sim.xml");
                      }
                  }
                  break;
		case FIH_PRJ_AT2: strcpy(msg, "system/etc/fqc_AT2.xml"); break;
		default: strcpy(msg, "N/A"); break;
	}
	seq_printf(m, "%s\n", msg);

	return 0;
}

static int fqc_xml_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_info_proc_read_fqc_xml_show, NULL);
};

static struct file_operations fqc_xml_file_ops = {
	.owner   = THIS_MODULE,
	.open    = fqc_xml_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int fih_info_proc_simslot_show(struct seq_file *m, void *v)
{
    int slot = 1;
//  if(fih_hwcfg_match_pcba_description("dualsim"))
    struct st_hwid_table tb;

    fih_hwid_read(&tb);

    if(2== tb.sim)
        slot=2;
    else
        slot=1;

    seq_printf(m, "%d\n", slot);
        return 0;
}

static int simslot_proc_open(struct inode *inode, struct file *file)
{
        return single_open(file, fih_info_proc_simslot_show, NULL);
};

static struct file_operations simslot_file_ops = {
        .owner   = THIS_MODULE,
        .open    = simslot_proc_open,
        .read    = seq_read,
        .llseek  = seq_lseek,
        .release = single_release
};



static struct {
		char *name;
		struct file_operations *ops;
} *p, fih_info[] = {
	{"devmodel", &devmodel_file_ops},
	{"baseband", &baseband_file_ops},
	{"bandinfo", &bandinfo_file_ops},
	{"hwcfg", &hwcfg_file_ops},
	{"MODULE", &module_file_ops},
	{"hwmodel", &hwmodel_file_ops},
        {"SIMSlot", &simslot_file_ops},
	{"fqc_xml", &fqc_xml_file_ops},
	{NULL}, };

static int __init fih_info_init(void)
{
	for (p = fih_info; p->name; p++) {
		if (proc_create(p->name, 0, NULL, p->ops) == NULL) {
			pr_err("fail to create proc/%s\n", p->name);
		}
	}

	return (0);
}

static void __exit fih_info_exit(void)
{
	for (p = fih_info; p->name; p++) {
		remove_proc_entry(p->name, NULL);
	}
}

module_init(fih_info_init);
module_exit(fih_info_exit);
