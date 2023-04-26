#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <fih/hwid.h>

extern int Touch_TestResult;
extern int TestResult_once;
extern ssize_t txtotxshort(void);
extern ssize_t fullRawCap(void);
extern ssize_t touch_selftest(void);
extern int touch_fwver_read(char *);
extern int touch_fwback_write(void);
extern int touch_fwback_read(char *);
extern int touch_fwimver_read(char *);
extern void touch_e1m_fwimver_read(char *);
extern int touch_scover_write(int);
extern int touch_upgrade_write(char *);
extern int touch_upgrade_read(char *);
extern int touch_vendor_read(char *);
extern void touch_upgrade(int);
extern void touch_tpfwver_read(char *);

extern int read_register_result(void);
extern void fts_fih_tp_rst(void);
extern void fts_fih_tp_enable(int);

#define FIH_PROC_PATH_TP_SMART_COVER  "AllHWList/tp_smart_cover"
//touch start
#define FIH_PROC_DIR        "AllHWList"
//Win add for ALT use
#define FIH_PROC_TP_ALT_RST       "AllHWList/tp_alt_rst"
#define FIH_PROC_TP_ALT_ST_COUNT       "AllHWList/tp_alt_st_count"
#define FIH_PROC_TP_ALT_ST_ENABLE       "AllHWList/tp_alt_st_enable"
#define FIH_PROC_TP_ALT_ST_DISABLE       "AllHWList/tp_alt_st_disable"
//Win add for ALT use

static int fih_touch_read_show(struct seq_file *m, void *v)
{
	if (Touch_TestResult == 1)
	{
		seq_printf(m, "%d\n", 0);
	}
	else
	{
		seq_printf(m, "%d\n", 1);
	}

	return 0;
}

static int fih_touch_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_touch_read_show, NULL);
};

static int fih_touch_proc_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
//	int ret =0;
	pr_err("F@Touch Do Touch Selftest\n");
    //E1M
    if ( fih_hwid_fetch(FIH_HWID_PRJ) == FIH_PRJ_E1M ){
        touch_selftest();
    }
    //FAO
    // else{
	// ret = txtotxshort();
	// ret = fullRawCap();
	// if(TestResult_once !=0)
	// {
		// Touch_TestResult = 0;
		// TestResult_once =0;
	// }
    // }
    
	return count;
}
static struct file_operations touch_proc_file_ops = {
	.owner   = THIS_MODULE,
	.write   = fih_touch_proc_write,
	.open    = fih_touch_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//touch start emd

//touch_fwver start
static int fih_touch_read_fwver_show(struct seq_file *m, void *v)
{
	char fwver[30]={0};
//	int ret =0;
	pr_err("F@Touch Read Touch firmware version\n");
	//FIH read TP FW start
	if ( fih_hwid_fetch(FIH_HWID_PRJ) == FIH_PRJ_E1M ){
	    touch_tpfwver_read(fwver);
	}
	// else{
	// //FAO
	    // ret = touch_fwver_read(fwver);
	// }
    
	seq_printf(m, "%s", fwver);
	return 0;
}

static int fih_touch_fwver_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_touch_read_fwver_show, NULL);
};

static struct file_operations touch_fwver_proc_file_ops = {
	.owner   = THIS_MODULE,
	.open    = fih_touch_fwver_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//touch_fwver end

//touch_fwback start
static int fih_touch_fwback_read_show(struct seq_file *m, void *v)
{
	//int ret = 0;
	char buf[10]={0};
	pr_err("F@Touch Read Touch firmware flag\n");
//	ret = touch_fwback_read(buf);
	seq_printf(m, "%s", buf);
	return 0;
}

static int fih_touch_fwback_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_touch_fwback_read_show, NULL);
};

static int fih_touch_fwback_proc_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	//int ret =0;
	pr_err("F@Touch Write Touch firmware flag to 1\n");
//	ret = touch_fwback_write();
	return count;
}

static struct file_operations touch_fwback_proc_file_ops = {
	.owner   = THIS_MODULE,
	.write   = fih_touch_fwback_proc_write,
	.open    = fih_touch_fwback_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//touch_fwback end

//touch_fwimver start
static int fih_touch_read_fwimver_show(struct seq_file *m, void *v)
{
	char fwimver[30]={0};
	//int ret =0;
	pr_err("F@Touch Read Touch firmware image version\n");
	if ( fih_hwid_fetch(FIH_HWID_PRJ) == FIH_PRJ_E1M ){
	//E1M
	    touch_e1m_fwimver_read(fwimver);
	}
	// else{
	// //FAO
	    // ret = touch_fwimver_read(fwimver);
	// }
	seq_printf(m, "%s", fwimver);
	return 0;
}

static int fih_touch_fwimver_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_touch_read_fwimver_show, NULL);
};

static struct file_operations touch_fwimver_proc_file_ops = {
	.owner   = THIS_MODULE,
	.open    = fih_touch_fwimver_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//touch_fwimver end

//touch_scover start
static int fih_touch_scover_read_show(struct seq_file *m, void *v)
{
	return 0;
}

static int fih_touch_scover_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_touch_scover_read_show, NULL);
};

static int fih_touch_scover_proc_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char *buf;
	unsigned int input = 0;

	if (count < 1)
		return -EINVAL;
	buf = kzalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	input = simple_strtoul(buf, NULL, 10);
	pr_err("F@Touch Write Touch scover flag to %d\n",input);
	//ret = touch_scover_write(input);
	return count;
}

static struct file_operations touch_scover_proc_file_ops = {
	.owner   = THIS_MODULE,
	.write   = fih_touch_scover_proc_write,
	.open    = fih_touch_scover_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//touch_scover end

//touch_upgrade start
static int fih_touch_upgrade_read_show(struct seq_file *m, void *v)
{
	char upgrade_flag[10]={0};
	//int ret =0;
	pr_err("F@Touch Read Touch upgrade flag\n");
    //FAO
	// ret = touch_upgrade_read(upgrade_flag);
    //E1M
	seq_printf(m, "%s", upgrade_flag);
	return 0;
}

static int fih_touch_upgrade_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_touch_upgrade_read_show, NULL);
};

//win add for ALT test
static int fih_touch_alt_rst_show(struct seq_file *m, void *v)
{
       return 0;
}

static int fih_touch_alt_rst_proc_open(struct inode *inode, struct file *file)
{
       return single_open(file, fih_touch_alt_rst_show, NULL);
};

static ssize_t fih_touch_alt_rst_proc_write(struct file *file, const char __user *buffer,
       size_t count, loff_t *ppos)
{
	char *buf;
	unsigned int input = 0;

	if (count < 1)
		return -EINVAL;
	buf = kzalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	input = simple_strtoul(buf, NULL, 10);

	fts_fih_tp_rst();
	return count;
}

static struct file_operations touch_alt_rst_file_ops = {
       .owner   = THIS_MODULE,
       .open    = fih_touch_alt_rst_proc_open,
       .write   = fih_touch_alt_rst_proc_write,
       .read    = seq_read,
       .llseek  = seq_lseek,
       .release = single_release
};

static int fih_touch_alt_st_count_show(struct seq_file *m, void *v)
{
       int count=0;

       count = read_register_result();
       seq_printf(m, "%d", count);

       return 0;
}

static int fih_touch_alt_st_count_proc_open(struct inode *inode, struct file *file)
{
       return single_open(file, fih_touch_alt_st_count_show, NULL);
};

static struct file_operations touch_alt_st_count_file_ops = {
       .owner   = THIS_MODULE,
       .open    = fih_touch_alt_st_count_proc_open,
       .read    = seq_read,
       .llseek  = seq_lseek,
       .release = single_release
};

static int fih_touch_alt_st_enable_show(struct seq_file *m, void *v)
{
       return 0;
}

static int fih_touch_alt_st_enable_proc_open(struct inode *inode, struct file *file)
{
       return single_open(file, fih_touch_alt_st_enable_show, NULL);
};

static ssize_t fih_touch_alt_st_enable_proc_write(struct file *file, const char __user *buffer,
       size_t count, loff_t *ppos)
{
	char *buf;
	unsigned int input = 0;

	if (count < 1)
		return -EINVAL;
	buf = kzalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	input = simple_strtoul(buf, NULL, 10);

	fts_fih_tp_enable(input);
	return count;
}
static struct file_operations touch_alt_st_enable_file_ops = {
       .owner   = THIS_MODULE,
       .open    = fih_touch_alt_st_enable_proc_open,
       .write   = fih_touch_alt_st_enable_proc_write,
       .read    = seq_read,
       .llseek  = seq_lseek,
       .release = single_release
};
//win add for ALT test 

static int fih_touch_upgrade_proc_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	char *buf;
	unsigned int input = 0;

	if (count < 1)
		return -EINVAL;
	buf = kzalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	input = simple_strtoul(buf, NULL, 10);
	
	pr_err("F@Touch Write Touch upgrade flag to %d\n",input);
	//E1M
	if ( (fih_hwid_fetch(FIH_HWID_PRJ) == FIH_PRJ_E1M )||( fih_hwid_fetch(FIH_HWID_PRJ) == FIH_PRJ_AT2 )){
	    touch_upgrade(input);
	}
	// else{
	// //FAO
	    // ret = touch_upgrade_write(input);
	// }
    
	return count;
}

static struct file_operations touch_upgrade_proc_file_ops = {
	.owner   = THIS_MODULE,
	.write   = fih_touch_upgrade_proc_write,
	.open    = fih_touch_upgrade_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//touch_upgrade end

//touch_vendor start
static int fih_touch_read_vendor_show(struct seq_file *m, void *v)
{
	char vendor[30]={0};
	//int ret =0;
	pr_err("F@Touch Read Touch vendor\n");
    //FAO
	// ret = touch_vendor_read(vendor);
    //E1M
    
	seq_printf(m, "%s", vendor);
	return 0;
}

static int fih_touch_vendor_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_touch_read_vendor_show, NULL);
};

static struct file_operations touch_vendor_proc_file_ops = {
	.owner   = THIS_MODULE,
	.open    = fih_touch_vendor_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//touch_vendor end
int fih_touch_proc_init(void)
{
	if (proc_create("touch", 0, NULL, &touch_proc_file_ops) == NULL) {
		pr_err("fail to create proc/%s\n", "touch");
		return (1);
	}
	if (proc_create("touch_fwver", 0, NULL, &touch_fwver_proc_file_ops) == NULL) {
		pr_err("fail to create proc/%s\n", "touch_fwver");
		return (1);
	}
	if (proc_create("touch_fwback", 0, NULL, &touch_fwback_proc_file_ops) == NULL) {
		pr_err("fail to create proc/%s\n", "touch_fwback");
		return (1);
	}
	if (proc_create("touch_fwimver", 0, NULL, &touch_fwimver_proc_file_ops) == NULL) {
		pr_err("fail to create proc/%s\n", "touch_fwimver");
		return (1);
	}
	if (proc_create("touch_scover", 0, NULL, &touch_scover_proc_file_ops) == NULL) {
		pr_err("fail to create proc/%s\n", "touch_scover");
		return (1);
	}
	if (proc_create(FIH_PROC_PATH_TP_SMART_COVER, 0, NULL, &touch_scover_proc_file_ops) == NULL) {
		pr_err("fail to create proc/%s\n", "touch_scover");
		return (1);
	}
	if (proc_create("touch_upgrade", 0, NULL, &touch_upgrade_proc_file_ops) == NULL) {
		pr_err("fail to create proc/%s\n", "touch_upgrade");
		return (1);
	}
	if (proc_create("touch_vendor", 0, NULL, &touch_vendor_proc_file_ops) == NULL) {
		pr_err("fail to create proc/%s\n", "touch_vendor");
		return (1);
	}
    //win add for ALT test
	if (proc_create(FIH_PROC_TP_ALT_RST, 0, NULL, &touch_alt_rst_file_ops) == NULL)
	{
	    proc_mkdir(FIH_PROC_DIR, NULL);
	    if (proc_create(FIH_PROC_TP_ALT_RST, 0, NULL, &touch_alt_rst_file_ops) == NULL)
	    {
	        pr_err("fail to create proc/%s\n", FIH_PROC_TP_ALT_RST);
	        return (1);
	    }
	}

	if (proc_create(FIH_PROC_TP_ALT_ST_COUNT, 0, NULL, &touch_alt_st_count_file_ops) == NULL)
	{
	    pr_err("fail to create proc/%s\n", FIH_PROC_TP_ALT_ST_COUNT);
	    return (1);
	}
	if (proc_create(FIH_PROC_TP_ALT_ST_ENABLE, 0, NULL, &touch_alt_st_enable_file_ops) == NULL)
	{
	    pr_err("fail to create proc/%s\n", FIH_PROC_TP_ALT_ST_ENABLE);
	    return (1);
	}
//win add for ALT test 
	return (0);
}

