#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <fih/hwid.h>

#define FIH_PROC_DIR "AllHWList"

/*FIH G-sensor*/
#define FIH_PROC_GS_COUNT "AllHWList/gsensor_count"
#define FIH_PROC_GS_ENABLE "AllHWList/gsensor_enable"

extern int gsensor_err_count;

//gsensor count start
static int gsensor_enable_read_show(struct seq_file *m, void *v)
{
    char data[10]={0};
    struct file *file=NULL;
    mm_segment_t old_fs;
    ssize_t result;

    file=filp_open("/sys/class/sensors/bma2x2-accel/enable",O_RDWR,0);
    old_fs=get_fs();
    set_fs(get_ds());
    result=file->f_op->read(file,data,sizeof(data),&file->f_pos);

	if (result >= 0)
	{
        data[strlen(data)] = '\n';
		pr_err("[G-sensor]Read g-sensor enable = %s\n", data);
	}
	else
	{
		pr_err("[G-sensor]Read g-sensor enable fail\n");
	}
    seq_printf(m, "%s\n", data);

	return 0;
}

static int gsensor_enable_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, gsensor_enable_read_show, NULL);
};

static int gsensor_enable_proc_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
    char data[10]={0};
	int ret =0;
    struct file *fd=NULL;
    mm_segment_t old_fs;

    ret = copy_from_user(data, buffer, count);
    if (ret < 0) {
        pr_err("Write g-sensor enable copy_from_user fail\n");
    }

    pr_err("[G-sensor]Write g-sensor enable flag to %s\n\n", data);

    fd=filp_open("/sys/class/sensors/bma2x2-accel/enable",O_RDWR,0);
    old_fs=get_fs();
    set_fs(get_ds());
    fd->f_op->write(fd,data,sizeof(data),&fd->f_pos);
    set_fs(old_fs);
    if(!strcmp(data, "1\n"))
        gsensor_err_count = 0;
    filp_close(fd,NULL);

	return count;
}

static struct file_operations gsensor_enable_file_ops = {
	.owner   = THIS_MODULE,
	.write   = gsensor_enable_proc_write,
	.open    = gsensor_enable_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//gsensor count end

//gsensor count start
static int gsensor_count_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d", gsensor_err_count);
	return 0;
}

static int gsensor_count_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, gsensor_count_show, NULL);
};

static struct file_operations gsensor_count_file_ops = {
	.owner   = THIS_MODULE,
	.open    = gsensor_count_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//gsensor count end

int fih_gsensor_proc_init(void)
{
    if (proc_create(FIH_PROC_GS_COUNT, 0, NULL, &gsensor_count_file_ops) == NULL)
    {
        proc_mkdir(FIH_PROC_DIR, NULL);
        if (proc_create(FIH_PROC_GS_COUNT, 0, NULL, &gsensor_count_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_GS_COUNT);
            return (1);
        }
    }

    if (proc_create(FIH_PROC_GS_ENABLE, 0, NULL, &gsensor_enable_file_ops) == NULL)
    {
        proc_mkdir(FIH_PROC_DIR, NULL);
        if (proc_create(FIH_PROC_GS_ENABLE, 0, NULL, &gsensor_enable_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_GS_ENABLE);
            return (1);
        }
    }

	return (0);
}

/*FIH E-compass*/
#define FIH_PROC_ECOMPASS_COUNT "AllHWList/ecompass_count"
#define FIH_PROC_ECOMPASS_ENABLE "AllHWList/ecompass_enable"

extern int ecompass_err_count;

//ecompass count start
static int ecompass_enable_read_show(struct seq_file *m, void *v)
{
    char data[10]={0};
    struct file *file=NULL;
    mm_segment_t old_fs;
    ssize_t result;

    file=filp_open("/sys/class/sensors/bmm050-mag/enable",O_RDWR,0);
    old_fs=get_fs();
    set_fs(get_ds());
    result=file->f_op->read(file,data,sizeof(data),&file->f_pos);

	if (result >= 0)
	{
        data[strlen(data)] = '\n';
		pr_err("[E-compass]Read e-compass enable = %s\n", data);
	}
	else
	{
		pr_err("[E-compass]Read e-compass enable fail\n");
	}
    seq_printf(m, "%s\n", data);

	return 0;
}

static int ecompass_enable_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ecompass_enable_read_show, NULL);
};

static int ecompass_enable_proc_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
    char data[10]={0};
	int ret =0;
    struct file *fd=NULL;
    mm_segment_t old_fs;

    ret = copy_from_user(data, buffer, count);
    if (ret < 0) {
        pr_err("Write e-compass enable copy_from_user fail\n");
    }

    pr_err("[E-compass]Write e-compass enable flag to %s\n\n", data);

    fd=filp_open("/sys/class/sensors/bmm050-mag/enable",O_RDWR,0);
    old_fs=get_fs();
    set_fs(get_ds());
    fd->f_op->write(fd,data,sizeof(data),&fd->f_pos);
    set_fs(old_fs);
    if(!strcmp(data, "1\n"))
        ecompass_err_count = 0;
    filp_close(fd,NULL);

	return count;
}

static struct file_operations ecompass_enable_file_ops = {
	.owner   = THIS_MODULE,
	.write   = ecompass_enable_proc_write,
	.open    = ecompass_enable_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//ecompass count end

//ecompass count start
static int ecompass_count_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d", ecompass_err_count);
	return 0;
}

static int ecompass_count_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ecompass_count_show, NULL);
};

static struct file_operations ecompass_count_file_ops = {
	.owner   = THIS_MODULE,
	.open    = ecompass_count_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//ecompass count end

int fih_ecompass_proc_init(void)
{
    if (proc_create(FIH_PROC_ECOMPASS_COUNT, 0, NULL, &ecompass_count_file_ops) == NULL)
    {
        proc_mkdir(FIH_PROC_DIR, NULL);
        if (proc_create(FIH_PROC_ECOMPASS_COUNT, 0, NULL, &ecompass_count_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_ECOMPASS_COUNT);
            return (1);
        }
    }

    if (proc_create(FIH_PROC_ECOMPASS_ENABLE, 0, NULL, &ecompass_enable_file_ops) == NULL)
    {
        proc_mkdir(FIH_PROC_DIR, NULL);
        if (proc_create(FIH_PROC_ECOMPASS_ENABLE, 0, NULL, &ecompass_enable_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_ECOMPASS_ENABLE);
            return (1);
        }
    }

	return (0);
}

/*FIH ALS*/
#define FIH_PROC_ALS_COUNT "AllHWList/als_count"
#define FIH_PROC_ALS_ENABLE "AllHWList/als_enable"

extern int als_err_count;

//als count start
static int als_enable_read_show(struct seq_file *m, void *v)
{
    char data[10]={0};
    struct file *file=NULL;
    mm_segment_t old_fs;
    ssize_t result;

    file=filp_open("/sys/class/sensors/stk3x1x-light/enable",O_RDWR,0);
    old_fs=get_fs();
    set_fs(get_ds());
    result=file->f_op->read(file,data,sizeof(data),&file->f_pos);

	if (result >= 0)
	{
        data[strlen(data)] = '\n';
		pr_err("[ALS]Read lightsensor enable = %s\n", data);
	}
	else
	{
		pr_err("[ALS]Read lightsensor enable fail\n");
	}
    seq_printf(m, "%s\n", data);

	return 0;
}

static int als_enable_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, als_enable_read_show, NULL);
};

static int als_enable_proc_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
    char data[10]={0};
	int ret =0;
    struct file *fd=NULL;
    mm_segment_t old_fs;

    ret = copy_from_user(data, buffer, count);
    if (ret < 0) {
        pr_err("Write lightsensor enable copy_from_user fail\n");
    }

    pr_err("[ALS]Write lightsensor enable flag to %s\n\n", data);

    fd=filp_open("/sys/class/sensors/stk3x1x-light/enable",O_RDWR,0);
    old_fs=get_fs();
    set_fs(get_ds());
    fd->f_op->write(fd,data,sizeof(data),&fd->f_pos);
    set_fs(old_fs);
    if(!strcmp(data, "1\n"))
        als_err_count = 0;
    filp_close(fd,NULL);

	return count;
}

static struct file_operations als_enable_file_ops = {
	.owner   = THIS_MODULE,
	.write   = als_enable_proc_write,
	.open    = als_enable_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//als count end

//als count start
static int als_count_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d", als_err_count);
	return 0;
}

static int als_count_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, als_count_show, NULL);
};

static struct file_operations als_count_file_ops = {
	.owner   = THIS_MODULE,
	.open    = als_count_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};
//als count end

int fih_als_proc_init(void)
{
    if (proc_create(FIH_PROC_ALS_COUNT, 0, NULL, &als_count_file_ops) == NULL)
    {
        proc_mkdir(FIH_PROC_DIR, NULL);
        if (proc_create(FIH_PROC_ALS_COUNT, 0, NULL, &als_count_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_ALS_COUNT);
            return (1);
        }
    }

    if (proc_create(FIH_PROC_ALS_ENABLE, 0, NULL, &als_enable_file_ops) == NULL)
    {
        proc_mkdir(FIH_PROC_DIR, NULL);
        if (proc_create(FIH_PROC_ALS_ENABLE, 0, NULL, &als_enable_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_ALS_ENABLE);
            return (1);
        }
    }

	return (0);
}
