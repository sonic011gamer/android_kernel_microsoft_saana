#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "fih_touch.h"

#define FIH_PROC_DIR   "AllHWList"
#define FIH_PROC_PATH  "AllHWList/Touch"

/*FIHTDC, touch panel, Eddie, 2012/1/13{*/
char fih_touch[32] = "unknown";
void fih_info_set_touch(char *info)
{
	strcpy(fih_touch, info);
}

/*}FIHTDC, touch panel, Eddie, 2012/1/13*/

static int fih_touch_read_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", fih_touch);
	return 0;
}

static int touchinfo_proc_open(struct inode *inode, struct file *file)
{
	pr_err("F@Touch Read Touch firmware version\n");
	return single_open(file, fih_touch_read_show, NULL);
};

static struct file_operations touch_info_file_ops = {
	.owner   = THIS_MODULE,
	.open    = touchinfo_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int __init fih_touch_init(void)
{
	//proc_mkdir(FIH_PROC_DIR, NULL);
	if (proc_create(FIH_PROC_PATH, 0, NULL, &touch_info_file_ops) == NULL) {
		pr_err("fail to create proc/%s\n", FIH_PROC_PATH);
		return (1);
	}
	return (0);
}

static void __exit fih_touch_exit(void)
{
	remove_proc_entry(FIH_PROC_PATH, NULL);
}

module_init(fih_touch_init);
module_exit(fih_touch_exit);
