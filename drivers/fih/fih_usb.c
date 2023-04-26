#include <fih/fih_usb.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/uaccess.h>

#include <linux/device.h>
#include <scsi/scsi.h>
#include <linux/file.h>
#include <linux/fs.h>

#include <linux/switch.h>
#include <linux/reboot.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>

#define FIH_PROC_FS_NEW

#ifdef FIH_PROC_FS
#include <linux/proc_fs.h>
#endif

#ifdef FIH_SYS_FS
#include <linux/slab.h>
#endif

#ifdef FIH_PROC_FS_NEW
#include <linux/proc_fs.h>
#endif

#undef fih_debug
#define fih_debug(x, ...) \
	do {if (g_fih_usb_debuglog)\
	{pr_debug("F@USB-2 fih_usb: " "%s():" x, __func__, ##__VA_ARGS__); } \
	} while (0)

struct usb_info{

	struct delayed_work       usb_work;
	struct workqueue_struct  *usb_workqueue;

};
static int g_fih_usb_debuglog;

//extern int fih_tool_register_func(int (*callback)(int, void*));
extern void fih_tool_unregister_func(void (*callback)(int, void*));
extern int android_usb_product_id(void);

struct switch_dev sw_pid;
struct switch_dev sw_adb_secure_dis;
struct switch_dev sw_root_detection;
static int scsi_set_adb_root;

static int mode_switch_reboot_flag;
static int reboot_flag;

int g_fih_ums = 0;

#ifdef CONFIG_SECURITY_SELINUX
#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
extern int selinux_enforcing;
#endif
extern void selnl_notify_setenforce(int val);
extern void selinux_status_update_setenforce(int enforcing);
#endif

extern char *saved_command_line;

#define SC_MODE_SWITCH          0xd6
#define SC_READ_NV              0xf0
#define SC_SWITCH_STATUS        0xf1
#define SC_SWITCH_PORT          0xf2
#define SC_MODEM_STATUS         0xf4
#define SC_SHOW_PORT            0xf5
#define SC_MODEM_DISCONNECT     0xf6
#define SC_MODEM_CONNECT        0xf7
#define SC_DIAG_RUT             0xf8
#define SC_READ_BATTERY         0xf9
#define SC_READ_IMAGE           0xfa
#define SC_ENABLE_ALL_PORT      0xfd
#define SC_MASS_STORGE          0xfe
#define SC_ENTER_DOWNLOADMODE   0xff
#define SC_ENTER_FTMMODE        0xe0
#define SC_SWITCH_ROOT          0xe1
#define SC_MODE_CHANGE          0xe2
#define SC_ROOT_STATUS          0xe3

#define PATH_SUT_INFO           "/proc/sutinfo"
#define BUILD_ID_0              "/proc/fver"
#define BUILD_ID_1              "/system/build_id"
#define BATTERY_CAPACITY_PATH   "/sys/class/power_supply/battery/capacity"

struct usb_pid_mapping {
	unsigned short pid_no_adb;
	unsigned short pid_adb;
};

struct usb_pid_mapping usb_fih_pid_mapping[] = {
	{0xC025,0xC026}, //MTP
	{0xC029,0xC02A}, //PTP
	{0xC004,0xC001}, //mass_storage
	{0xC021,0xC000}, //All port
	{0x0, 0x0}
};

struct usb_pid_mapping *p_usb_pid_mapping = NULL;

static int do_mode_switch(struct fih_usb_data *input)
{
	//struct fsg_lun  *curlun = common->curlun;
	u8 *buf = (u8 *) input->bh_buf;

	if (input->cmnd[10] == 0) {
		memset(buf, 0, 1);
		buf[0] = '0';
		mode_switch_reboot_flag = true;
	}
	else {
		//curlun->sense_data = SS_INVALID_COMMAND;
		return -EINVAL;
	}
	return 0;
}

/*
 * Example :
 *
 * PIDINFO10035P003000000FIHXGOXXXXX1APPXXXXX103GOX20400013033.24011
 *
 * PIDINFO1    | 8 bytes      | Signature as partition header
 * 0035        | 4 bytes      | Data size of following data
 * P           | 1 bytes      | Signature as SUT information
 * 0030        | 4 bytes      | Data size of the following data for SUT information
 * 0000...4011 | 0x0030 bytes | SUT information
 *
 */
static int do_read_nv(struct fih_usb_data *input)
{
	struct file *file_filp = NULL;
	u8 *buf = (u8 *) input->bh_buf;
	unsigned long size;
	char tbuf[128];
	char tmp[128];
	mm_segment_t oldfs = get_fs();

	memset(tbuf, 0, sizeof(tbuf));
	memset(tmp , 0, sizeof(tmp));

	set_fs(KERNEL_DS);
	file_filp = filp_open(PATH_SUT_INFO, O_RDONLY|O_NONBLOCK, 0);
	if (!IS_ERR(file_filp)) {
		file_filp->f_op->read(file_filp, tmp, 13, &file_filp->f_pos);
		file_filp->f_op->read(file_filp, tmp,  4, &file_filp->f_pos);
		size = simple_strtoul(tmp, NULL, 16);
		if (size<=0) size = sizeof(tmp);
		file_filp->f_op->read(file_filp, tmp, size, &file_filp->f_pos);
		filp_close(file_filp, NULL);
		memcpy(tbuf, tmp, sizeof(tmp));
	} else {
		sprintf(buf, "%s: error", PATH_SUT_INFO);
		set_fs(oldfs);
		return 128;
	}

	file_filp = filp_open(BUILD_ID_0, O_RDONLY, 0);
	if (!IS_ERR(file_filp)) {
		pr_info("%s : BUILD_ID_0 = %s\n", __func__, BUILD_ID_0);
		file_filp->f_op->read(file_filp, tmp, sizeof(tmp), &file_filp->f_pos);
		filp_close(file_filp, NULL);
		pr_info("%s : tmp = %s\n", __func__, tmp);

		memcpy(tbuf+1, &tmp[8], 4);
		sprintf(buf, tbuf);
		pr_info("%s : tbuf = %s\n", __func__, tbuf);
	}
	else {
		file_filp = filp_open(BUILD_ID_1, O_RDONLY, 0);
		if (!IS_ERR(file_filp)) {
			pr_info("%s : BUILD_ID_1 = %s\n", __func__, BUILD_ID_1);
			file_filp->f_op->read(file_filp, tmp, sizeof(tmp), &file_filp->f_pos);
			filp_close(file_filp, NULL);
			pr_info("%s : tmp = %s\n", __func__, tmp);

			memcpy(tbuf+1, &tmp[5], 1);
			memcpy(tbuf+2, &tmp[7], 3);
			sprintf(buf, tbuf);
			pr_info("%s : tbuf = %s\n", __func__, tbuf);
		}
		else {
			pr_info("%s : fail to open build_id files\n", __func__);
		}
	}
	set_fs(oldfs);
	return 128;
}

static int do_switch_status(struct fih_usb_data *input)
{
	char online[36] = "1";
	u8  *buf = (u8 *) input->bh_buf;

	memset(buf, 0, 36);
	sprintf(buf , online);

	return 36;
}

static int do_read_battery(struct fih_usb_data *input)
{
	int batt_capasity = 50;
	struct file *file_filp = NULL;
	u8 *buf = (u8 *) input->bh_buf;
	char tmp[4];
	mm_segment_t oldfs = get_fs();

	memset(tmp , 0, sizeof(tmp));
	memset(buf, 0, sizeof(tmp));

	set_fs(KERNEL_DS);

	file_filp = filp_open(BATTERY_CAPACITY_PATH, O_RDONLY, 0);
	if (!IS_ERR(file_filp)) {
		pr_info("%s : BATTERY_CAPACITY_PATH = %s\n", __func__, BATTERY_CAPACITY_PATH);
		file_filp->f_op->read(file_filp, tmp, sizeof(tmp), &file_filp->f_pos);
		filp_close(file_filp, NULL);
		pr_info("%s : tmp = %s\n", __func__, tmp);

		sscanf(tmp,"%d",&batt_capasity);
		pr_info("%s : tmp = %d\n", __func__, batt_capasity);
	}
	else {
		pr_info("%s : fail to open build_id files\n", __func__);
	}
	set_fs(oldfs);

	buf[0] = batt_capasity;
	return 36;
}

static int do_mode_change(struct fih_usb_data *input)
{
	u8 *buf = (u8 *) input->bh_buf;

	memset(buf, 0, 1);
	buf[0] = '1';
	reboot_flag = true;
	return 36;
}

static int do_scsi_root_status(struct fih_usb_data *input)
{
	mm_segment_t oldfs;
	struct file *fp = NULL;
	char efs_statusroot_buf[5];
	u8 *buf = (u8 *) input->bh_buf;

	oldfs = get_fs();

	set_fs(KERNEL_DS);
	fp = filp_open("/BBSYS/status.cfg", O_RDONLY|O_NONBLOCK, 0);
	if (!IS_ERR(fp)) {
		memset(efs_statusroot_buf, 0x00, sizeof(efs_statusroot_buf));
		fp->f_op->read(fp, (unsigned char __user *)efs_statusroot_buf, 4, &fp->f_pos);
		filp_close(fp,NULL);
	}

	set_fs(oldfs);

	pr_info("root:0x%x", efs_statusroot_buf[0]);
	buf[0] = efs_statusroot_buf[0];
	buf[1] = efs_statusroot_buf[1];
	buf[2] = efs_statusroot_buf[2];
	buf[3] = efs_statusroot_buf[3];

	return 36;
}

/*-------------------------------------------------------------------------*/
static void do_open_adb(void)
{
	int i = 0;
	unsigned short pid = android_usb_product_id();

	/*disable adb security*/
	switch_set_state(&sw_adb_secure_dis, 1);

	while( p_usb_pid_mapping[i].pid_no_adb != 0x0 ){
		if( pid == p_usb_pid_mapping[i].pid_no_adb ) {
			sw_pid.state = -1;
			switch_set_state(&sw_pid, p_usb_pid_mapping[i].pid_adb );
			pr_info("%s : PID %x change to %x \n", __func__,pid,p_usb_pid_mapping[i].pid_adb);
			return;
		}else if( pid == p_usb_pid_mapping[i].pid_adb ) {
			pr_info("%s : already change PID to %x\n", __func__,pid);
			return;
		}
		i++;
	}

	sw_pid.state = -1;
	switch_set_state(&sw_pid, p_usb_pid_mapping[0].pid_adb );
	pr_info("%s : Other PID %x change to %x\n", __func__,pid,p_usb_pid_mapping[0].pid_adb);
}

static void do_close_adb(void)
{
	int i = 0;
	unsigned short pid = android_usb_product_id();

	/*disable adb security*/
	switch_set_state(&sw_adb_secure_dis, 1);

	while( p_usb_pid_mapping[i].pid_adb != 0x0 ){
		if( pid == p_usb_pid_mapping[i].pid_adb ) {
			sw_pid.state = -1;
			switch_set_state(&sw_pid, p_usb_pid_mapping[i].pid_no_adb );
			pr_info("%s : PID %x change to %x \n", __func__,pid,p_usb_pid_mapping[i].pid_no_adb);
			return;
		}else if( pid == p_usb_pid_mapping[i].pid_no_adb ) {
			pr_info("%s : already change PID to %x\n", __func__,pid);
			return;
		}
		i++;
	}

	sw_pid.state = -1;
	switch_set_state(&sw_pid, p_usb_pid_mapping[0].pid_no_adb );
	pr_info("%s : Other PID %x change to %x\n", __func__,pid,p_usb_pid_mapping[0].pid_no_adb);
}

static void do_open_debug_port(void)
{
	unsigned short pid = android_usb_product_id();

	/*disable adb security*/
	switch_set_state(&sw_adb_secure_dis, 1);

	switch (pid) {
		case 0xC00E: //debug_port with MTP
		case 0xC00F: //debug_port with PTP
			pr_info("%s : already change PID to %x\n", __func__,pid);
			break;
		case 0xC029: //ptp,mass_storage
		case 0xC02A: //ptp,mass_storage,adb
		case 0xC02B: //ptp,mass_storage,serial_smd
		case 0xC00D: //ptp,mass_storage,serial_smd,adb
		case 0xC00C: //ptp,mass_storage,diag,serial_smd,adb
			sw_pid.state = -1;
			switch_set_state(&sw_pid, 0xC00F);
			pr_info("%s : change PID to 0xC00F\n", __func__);
			break;
		default:
			sw_pid.state = -1;
			switch_set_state(&sw_pid, 0xC00E);
			pr_info("%s : change PID to 0xC00E\n", __func__);
			break;
	}
}

static void fih_switch_all_port(void)
{
	/*disable adb security*/
	switch_set_state(&sw_adb_secure_dis, 1);

	if (android_usb_product_id() != 0xC000)
	{
		sw_pid.state = -1;
		switch_set_state(&sw_pid, 0xC000);
		pr_info("%s : change PID to 0xC000\n", __func__);
	}else {
		pr_info("%s : already change PID to 0xC000\n", __func__);
	}
}

int fih_usb_check_common_command(u8 *input)
{
	if (input[0] == SC_MODE_SWITCH ||
			input[0] == SC_READ_NV ||
			input[0] == SC_SWITCH_STATUS ||
			input[0] == SC_READ_BATTERY ||
			input[0] == SC_MODE_CHANGE ||
			input[0] == SC_ROOT_STATUS)
		return 1;
	else
		return 0;
}

int fih_usb_check_exe_common_command(u8 *input)
{
	switch (input[0]) {
	case SC_MODE_SWITCH:
	case SC_READ_NV:
	case SC_SWITCH_STATUS:
	case SC_SWITCH_PORT:
	case SC_DIAG_RUT:
	case SC_READ_BATTERY:
	case SC_ENABLE_ALL_PORT:
	case SC_ENTER_DOWNLOADMODE:
	case SC_ENTER_FTMMODE:
	case SC_SWITCH_ROOT:
	case SC_MODE_CHANGE:
	case SC_ROOT_STATUS:
		return 1;
		break;
	default:
		return 0;
	}
}

int fih_usb_common_command_run(struct fih_usb_data *input)
{
	int reply = -EINVAL;
	char cmd = 0;

	/*The first 3 bits of common->cmnd[1] are altered  to 0 in Linux. We use a mask to omit first 3 bits in order to ensure  common->cmnd[1] are the same in both windows & Linux */
	char cmnd_1 = input->cmnd[1]  & 0x1f;
	char fmask_1 = 'F' & 0x1f;

	switch (input->cmnd[0]) {
	case SC_MODE_SWITCH:
		pr_info("%s : SC_MODE_SWITCH \n", __func__);
		*(input->data_size_from_cmnd) = input->cmnd[4];
		reply = do_mode_switch(input);
		break;

	case SC_READ_NV:
		pr_info("%s : SC_READ_NV \n", __func__);
		*(input->data_size_from_cmnd) = input->cmnd[4];
		if ((cmnd_1==fmask_1) && (input->cmnd[2]=='I') && (input->cmnd[3]=='H')) {
			pr_info("%s : do_read_nv\n", __func__);
			reply = do_read_nv(input);
		}
		break;

	case SC_SWITCH_STATUS:
		pr_info("%s : SC_SWITCH_STATUS \n", __func__);
		*(input->data_size_from_cmnd) = input->cmnd[4];
		if ((cmnd_1==fmask_1) && (input->cmnd[2]=='I') && (input->cmnd[3]=='H')) {
			pr_info("%s : do_switch_status\n", __func__);
			reply = do_switch_status(input);
		}
		break;

	case SC_SWITCH_PORT:
		pr_info("%s : SC_SWITCH_PORT \n", __func__);

		if ((cmnd_1==fmask_1) && (input->cmnd[2]=='I'))
			cmd=input->cmnd[3];
		switch(cmd) {
			case '0':
				pr_info("%s : SC_SWITCH_PORT 0\n", __func__);
				do_close_adb();
				break;
			case '1':
				pr_info("%s : SC_SWITCH_PORT 1\n", __func__);
				do_open_adb();
				break;
			case '2':
				pr_info("%s : SC_SWITCH_DEBUG_PORT 1\n", __func__);
				do_open_debug_port();
				break;
			default:
				pr_info("%s : SC_SWITCH_PORT fail\n", __func__);
		}
		break;

	case SC_DIAG_RUT:
		pr_info("%s : SC_DIAG_RUT \n", __func__);
		if ((cmnd_1==fmask_1) && (input->cmnd[2]=='I') && (input->cmnd[3]=='H')) {
			pr_info("%s : reboot to recovery mode\n", __func__);
			kernel_restart("recovery");
		}
		break;

	case SC_READ_BATTERY:
		pr_info("%s : SC_READ_BATTERY \n", __func__);
		*(input->data_size_from_cmnd) = input->cmnd[4];
		if ((cmnd_1==fmask_1) && (input->cmnd[2]=='I') && (input->cmnd[3]=='H')) {
			pr_info("%s : do_read_battery\n", __func__);
			reply = do_read_battery(input);
		}
		break;

	case SC_ENABLE_ALL_PORT:
		pr_info("%s : SC_ENABLE_ALL_PORT \n", __func__);
		if ((cmnd_1==fmask_1) && (input->cmnd[2]=='I') && (input->cmnd[3]=='H')) {
			pr_info("%s : switch to all port\n", __func__);
			fih_switch_all_port();
		}
		break;

	case SC_ENTER_DOWNLOADMODE:
		pr_info("%s : SC_ENTER_DOWNLOADMODE \n", __func__);
		if ((cmnd_1==fmask_1) && (input->cmnd[2]=='I') && (input->cmnd[3]=='H')){
			pr_info("%s : reboot to download mode\n", __func__);
			kernel_restart("bootloader");
		}
		break;

	case SC_ENTER_FTMMODE:
		pr_info("%s : SC_ENTER_FTMMODE \n", __func__);
		kernel_restart("ftm");
		break;
#ifdef CONFIG_SECURITY_SELINUX
#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
	case SC_SWITCH_ROOT:
		pr_info("%s : SC_SWITCH_ROOT \n", __func__);
		*(input->data_size_from_cmnd) = input->cmnd[4];
		if ((cmnd_1==fmask_1) && (input->cmnd[2]=='I') && (input->cmnd[3]=='H')) {
			scsi_set_adb_root = true;
			pr_info("%s : set adb to root\n", __func__);
			if(selinux_enforcing != 0) {
				pr_info("%s : selinux_enforcing = 0\n", __func__);
				selinux_enforcing = 0;
				selnl_notify_setenforce(selinux_enforcing);
				selinux_status_update_setenforce(selinux_enforcing);
			}
		}
		break;
#endif
#endif
	case SC_ROOT_STATUS:
		pr_info("%s : SC_ROOT_STATUS \n", __func__);
		if ((cmnd_1==fmask_1) && (input->cmnd[2]=='I') && (input->cmnd[3]=='H')) {
			switch_set_state(&sw_root_detection, 1);
			msleep(500);
			reply = do_scsi_root_status(input);
		}
		break;

	case SC_MODE_CHANGE:
		pr_info("%s : SC_MODE_CHANGE \n", __func__);
		if ((input->cmnd[1]=='R') && (input->cmnd[2]=='S') && (input->cmnd[3]=='T')) {
			*(input->data_size_from_cmnd) = input->cmnd[4];
			pr_info("%s : do_mode_change\n", __func__);
			reply = do_mode_change(input);
		}
		break;
	default:
		return reply;
	}
	return reply;
}

int fih_usb_check_status(int input)
{
	int ret = 0;
	switch(input) {
	case CHECK_STATUS_MODE_SWITCH_REBOOT_FLAG:
		ret = mode_switch_reboot_flag;
		mode_switch_reboot_flag = false;;
		break;
	case CHECK_STATUS_REBOOT_FLAG:
		ret = reboot_flag;
		reboot_flag = false;
		break;
	default:
		break;
	}
	return ret;
}

int fih_usb_parser_command(int command, void *data)
{
	struct fih_usb_data *input;
	int *input_int;
	u8 *buf;
	int ret;
	switch(command) {
	case CHECK_COMMAND:
		input = (struct fih_usb_data*)data;
		return fih_usb_check_common_command(input->cmnd);
		break;
	case EXE_COMMAND_CHECK:
		input = (struct fih_usb_data*)data;
		return fih_usb_check_exe_common_command(input->cmnd);
		break;
	case EXE_COMMAND:
		input = (struct fih_usb_data*)data;
		buf = (u8 *) input->bh_buf;
		ret = fih_usb_common_command_run(input);
		fih_debug("%s: buf[0]: %d, ret: %d\n",__func__, buf[0], ret);
		return ret;
		break;
	case CHECK_STATUS:
		input_int = (int *)data;
		return fih_usb_check_status(*input_int);
		break;
	default:
		return 0;
		break;
	}
}

#ifdef FIH_PROC_FS_NEW
static int fih_root_enable_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", scsi_set_adb_root);
	return 0;
}

static int fih_root_enable_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_root_enable_proc_show, NULL);
};

static struct file_operations root_enable_ops = {
	.owner   = THIS_MODULE,
	.open    = fih_root_enable_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

void fih_usb_root_enable_create_proc(char *root_path)
{
	struct proc_dir_entry *pfile;
	char file_path[64];

	sprintf(file_path, "%s/fih_usb_root_enable", root_path);
	pfile = proc_create(file_path, 0444, NULL, &root_enable_ops);
	if (!pfile) {
		pr_err("Can't create /proc/%s\n", file_path);
		return ;
	}
}
void fih_usb_root_enable_remove_proc(char *root_path)
{
	char file_path[64];

	sprintf(file_path, "%s/fih_usb_root_enable", root_path);
	remove_proc_entry(file_path, NULL);
}

static int fih_usb_log_enable_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", g_fih_usb_debuglog);
	return 0;
}

static ssize_t fih_usb_log_enable_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	unsigned char tmp[16];
	unsigned int size;

	size = (count > sizeof(tmp))? sizeof(tmp):count;

	if ( copy_from_user(tmp, buffer, size) ) {
		pr_err("%s: copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	g_fih_usb_debuglog = simple_strtoul(tmp, NULL, size);

	return size;
}

static int fih_usb_log_enable_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_usb_log_enable_read, NULL);
};

static struct file_operations fih_usb_log_enable_file_ops = {
	.owner   = THIS_MODULE,
	.open    = fih_usb_log_enable_proc_open,
	.read    = seq_read,
	.write   = fih_usb_log_enable_write,
	.llseek  = seq_lseek,
	.release = single_release
};

void fih_usb_log_enable_create_proc(char *root_path)
{
	struct proc_dir_entry *pfile;
	char file_path[64];

	sprintf(file_path, "%s/fih_usb_log_enable", root_path);
	pfile = proc_create(file_path, 0666, NULL, &fih_usb_log_enable_file_ops);
	if (!pfile) {
		fih_debug("Can't create /proc/%s\n", file_path);
		return ;
	}
}

void fih_usb_log_enable_remove_proc(char *root_path)
{
	char file_path[64];

	sprintf(file_path, "%s/fih_usb_log_enable", root_path);
	remove_proc_entry(file_path, NULL);
}

void fih_usb_create_proc_new(void)
{
	static struct proc_dir_entry *fihdir;

	char root_path[64];
	char file_path[64];

	sprintf(root_path, "AllHWList");
	sprintf(file_path, "%s/fih_usb", root_path);

	fihdir = proc_mkdir(file_path, NULL);
	if (!fihdir) {
		fih_debug("Can't create /proc/%s\n", file_path);
		fihdir = proc_mkdir(root_path, NULL);
		if (!fihdir) {
			fih_debug("Can't create /proc/%s\n", root_path);
			return ;
		}
		fihdir = proc_mkdir(file_path, NULL);
		if (!fihdir) {
			fih_debug("Can't create /proc/%s\n", file_path);
			return ;
		}
	}
	fih_usb_log_enable_create_proc(file_path);
	fih_usb_root_enable_create_proc(file_path);
}
void fih_usb_remove_proc_new(void)
{
	char root_path[64];
	char file_path[64];

	sprintf(root_path, "AllHWList");
	sprintf(file_path, "%s/fih_usb", root_path);
	fih_usb_log_enable_remove_proc(file_path);
	fih_usb_root_enable_remove_proc(file_path);
}
#endif

#ifdef FIH_PROC_FS
static int fih_usb_root_enable_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
		int len = 0;
		char buf[255];
		sprintf(buf, "%d\n", scsi_set_adb_root);
		len = strlen(buf);
		if (off >= len)
			return 0;
		if (count > len - off)
			count = len - off;
		memcpy(page + off, buf + off, count);
		return off + count;
}

void fih_usb_root_enable_create_proc(char *root_path)
{
	struct proc_dir_entry *pfile;
	char file_path[64];

	sprintf(file_path, "%s/fih_usb_root_enable", root_path);
	pfile = create_proc_entry(file_path, 0666, NULL);
	if (!pfile) {
		fih_debug("Can't create /proc/%s\n", file_path);
		return ;
	}
	pfile->read_proc = fih_usb_root_enable_read;
}

static int fih_usb_log_enable_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
		int len = 0;
		char buf[255];
		sprintf(buf, "%d\n", g_fih_usb_debuglog);
		len = strlen(buf);
		if (off >= len)
			return 0;
		if (count > len - off)
			count = len - off;
		memcpy(page + off, buf + off, count);
		return off + count;
}
static int fih_usb_log_enable_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
		unsigned long count2 = count;
		char buf[255];
		if (count2 >= sizeof(buf))
			count2 = sizeof(buf) - 1;
		if (copy_from_user(buf, buffer, count2))
			return -EFAULT;
		buf[count2] = '\0';
		sscanf(buf, "%d", &g_fih_usb_debuglog);
		fih_debug("g_fih_usb_debuglog = %d\n",  g_fih_usb_debuglog);
		return count;
}
void fih_usb_log_enable_create_proc(char *root_path)
{
	struct proc_dir_entry *pfile;
	char file_path[64];

	sprintf(file_path, "%s/fih_usb_log_enable", root_path);
	pfile = create_proc_entry(file_path, 0666, NULL);
	if (!pfile) {
		fih_debug("Can't create /proc/%s\n", file_path);
		return ;
	}
	pfile->read_proc = fih_usb_log_enable_read;
	pfile->write_proc = fih_usb_log_enable_write;
}

void fih_usb_create_proc(void)
{
	static struct proc_dir_entry *fihdir;

	char root_path[64];
	char file_path[64];

	sprintf(root_path, "AllHWList");
	sprintf(file_path, "%s/fih_usb", root_path);

	fihdir = proc_mkdir(file_path, NULL);
	if (!fihdir) {
		fih_debug("Can't create /proc/%s\n", file_path);
		fihdir = proc_mkdir(root_path, NULL);
		if (!fihdir) {
			fih_debug("Can't create /proc/%s\n", root_path);
			return ;
		}
		fihdir = proc_mkdir(file_path, NULL);
		if (!fihdir) {
			fih_debug("Can't create /proc/%s\n", file_path);
			return ;
		}
	}
	fih_usb_log_enable_create_proc(file_path);
	fih_usb_root_enable_create_proc(file_path);
}
#endif
#ifdef FIH_SYS_FS
static ssize_t fih_usb_root_enable_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", scsi_set_adb_root);
}
static ssize_t fih_usb_root_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d", &scsi_set_adb_root);
	fih_debug("fih_usb_root_enable(%d)\n", scsi_set_adb_root);
	return count;
}
static DEVICE_ATTR(fih_usb_root_enable, 0644, fih_usb_root_enable_show, fih_usb_root_enable_store);

static ssize_t fih_usb_log_enable_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_fih_usb_debuglog);
}
static ssize_t fih_usb_log_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d", &g_fih_usb_debuglog);
	fih_debug("g_fih_usb_debuglog(%d)\n", g_fih_usb_debuglog);
	return count;
}
static DEVICE_ATTR(fih_usb_log_enable, 0644, fih_usb_log_enable_show, fih_usb_log_enable_store);
/*
static void fih_usb_dev_release(struct device *dev)
{
	pr_info("device: '%s': %s\n", dev_name(dev), __func__);
	kfree(dev);
}
*/
int fih_usb_create_sys_fs(void)
{
	struct device *fih_usb_dev;
	int rc;

	fih_usb_dev = kzalloc(sizeof(*fih_usb_dev), GFP_KERNEL);
	device_initialize(fih_usb_dev);
	fih_usb_dev->parent = NULL;
	//fih_usb_dev->release = fih_usb_dev_release;
	rc = kobject_set_name(&fih_usb_dev->kobj, "%s", "fih_usb_interface");
	if (rc)
		goto kobject_set_name_failed;
	rc = device_add(fih_usb_dev);
	if (rc)
		goto device_add_failed;

	rc = device_create_file(fih_usb_dev, &dev_attr_fih_usb_root_enable);
	if (rc) {
		pr_err("F@BAT-1 %s: dev_attr_fih_usb_root_enable failed\n", __func__);
		return rc;
	}
	rc = device_create_file(fih_usb_dev, &dev_attr_fih_usb_log_enable);
	if (rc) {
		pr_err("F@BAT-1 %s: dev_attr_fih_usb_log_enable failed\n", __func__);
		return rc;
	}
kobject_set_name_failed:
device_add_failed:
	put_device(fih_usb_dev);
	return rc;
}
#endif
void fih_usb_init_value(void)
{
	g_fih_usb_debuglog = 0;
	mode_switch_reboot_flag = false;
	reboot_flag = false;
	scsi_set_adb_root = false;
}

bool scsi_adb_root_flag(void)
{
	return scsi_set_adb_root;
}
EXPORT_SYMBOL(scsi_adb_root_flag);

static int scsi_adb_switch_init(void)
{
	int ret;

	sw_pid.name = "sw_pid";
	ret = switch_dev_register(&sw_pid);
	if (ret < 0) {
		pr_err("register sw_pid failed!\n");
	}
	sw_pid.state = -1;

	sw_adb_secure_dis.name = "sw_adb_secure_dis";
	ret = switch_dev_register(&sw_adb_secure_dis);
	if (ret < 0) {
		pr_err("register sw_adb_secure_dis failed!\n");
	}
	sw_adb_secure_dis.state = -1;

	sw_root_detection.name = "sw_root_detection";
	ret = switch_dev_register(&sw_root_detection);
	if (ret < 0) {
		pr_err("register sw_root_detection failed!\n");
	}
	sw_root_detection.state = -1;

	return 0;
}

static void usb_all_port_setting(struct work_struct *work)
{
       struct usb_info *info;
       struct delayed_work *usb_work = container_of(work, struct delayed_work, work);
       info = container_of(usb_work, struct usb_info, usb_work);

       if(selinux_enforcing != 0) {
           pr_info("%s: disable selinux \n", __func__);
           selinux_enforcing = 0;
           selnl_notify_setenforce(selinux_enforcing);
           selinux_status_update_setenforce(selinux_enforcing);
       }
       pr_info("%s: enable all ports \n", __func__);
       fih_switch_all_port();
       //queue_delayed_work(info->usb_workqueue, &info->usb_work, msecs_to_jiffies(3000));
}
static int __init fih_usb_init(void)
{
	int rc = 0;
	struct usb_info *info = NULL;
	pr_info("%s\n", __func__);

	if (strstr(saved_command_line,"androidboot.securityfused=true")) {
		pr_info("%s:fused status isn't false, disable SCSI command\n", __func__);
		return rc;
	}
      else{
		pr_err("%s: securityfused=false\n", __func__);
      }

      if (strstr(saved_command_line,"androidboot.allport=true")){
		pr_info("%s: enable allport and disable selinux\n", __func__);

		info = kzalloc(sizeof (struct usb_info), GFP_KERNEL);

		info->usb_workqueue = alloc_workqueue("usb-queue", WQ_UNBOUND|WQ_HIGHPRI|WQ_CPU_INTENSIVE, 1);
		if (!info)
		{
		    pr_err("%s: alloc workqueue fail \n", __func__);
		}
		else
		{
		    INIT_DELAYED_WORK(&info->usb_work, usb_all_port_setting);
		    pr_info("%s: Init workqueue  \n", __func__);
		    queue_delayed_work(info->usb_workqueue, &info->usb_work, msecs_to_jiffies(11000));
		}

      }
      else{
        pr_err("%s: allport disable \n", __func__);
      }


	fih_usb_init_value();
#ifdef FIH_PROC_FS
	fih_usb_create_proc();
#endif
#ifdef FIH_SYS_FS
	fih_usb_create_sys_fs();
#endif

#ifdef FIH_PROC_FS_NEW
	fih_usb_create_proc_new();
#endif

	scsi_adb_switch_init();

	p_usb_pid_mapping = usb_fih_pid_mapping;

	//fih_tool_register_func(&fih_usb_parser_command);
	return rc;
}
static void __exit fih_usb_exit(void)
{
   pr_info("%s\n", __func__);
#ifdef FIH_PROC_FS_NEW
	fih_usb_remove_proc_new();
#endif
}

//module_init(fih_usb_init);
late_initcall(fih_usb_init);
module_exit(fih_usb_exit);
MODULE_LICENSE("GPL v2");
