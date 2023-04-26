#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include "fih_ramtable.h"

#define FIH_E2P_PARTITION "/dev/block/bootdevice/by-name/deviceinfo"
#define E2P_OFFSET 498*1024
#define PRECISION 6
#define MAGIC_GYRO "5678"
#define GYRO_DEFAULT "0.0,0.0,0.0"

static char *my_proc_name = "cali_gyro";
static unsigned int my_proc_addr = FIH_SENSORDATA_BASE+2*1024;//496KB is light sensor, 498KB is gyro sensor
static unsigned int my_proc_size = 1024;
static unsigned int my_proc_len = 0;
static int fih_ever_write=0;

static struct delayed_work cali_gyro_work;
static struct workqueue_struct *cali_gyro_workq;
static struct file *pid_filp = NULL;

static atomic_t open_file = ATOMIC_INIT(0);

struct gyro_data {
	char magic[4];
	unsigned long data[6];//r1,r2,r3,s1,s2,s3
};

static int exponent(int a, int x)
{
	int result = 1;

	while (x > 0) {
		result *= a;
		x--;
	}

	return result;
}
void remove_zero(char *buf)
{
	int i=0,j=0,done=0;
	int len=strlen(buf);
	char *temp=kzalloc(len+1,GFP_KERNEL);

	if(temp==NULL)
		return;

	pr_debug("[remove_zero]buf=%s,len=%d\n",buf,len);
	strncpy(temp,buf,len);
	memset(buf,0,len);
	for(i=0;i<len;i++)
	{
		if((temp[i]>=0x31&&temp[i]<=0x39)||done==1)
		{
			buf[j++]=temp[i];
			done=1;
		}
	}
	kfree(temp);
	pr_debug("[remove_zero]%s\n",buf);
}
void float_to_int(char *buf,unsigned long *integer_d,unsigned long *float_d)
{
	unsigned long temp=0;
	char * token=NULL;
	char delim[] = ".";
	int i=0,ret=0,len=0;

	*integer_d=0;
	*float_d=0;

	for(token = strsep(&buf, delim); token != NULL; token = strsep(&buf, delim)) {
		if(i==0)//int
		{
			pr_debug("[float_to_int]int=%s\n",token);
			ret=kstrtoul(token,0,&temp);
			if (ret){
				pr_err("[float_to_int]kstrtoul int fail\n");
				goto END;
			}
			*integer_d=temp;
		}else if(i==1)//float
		{
			pr_debug("[float_to_int]float=%s\n",token);
			len=strlen(token);
			remove_zero(token);
			if(len>PRECISION)
				token[PRECISION]='\0';
			ret=kstrtoul(token,0,&temp);
			if (ret){
				pr_err("[float_to_int]kstrtoul float fail\n");
				goto END;
			}
			*float_d=temp*exponent(10,PRECISION-len);;
		}
		i++;
	}
END:
	pr_err("[float_to_int]value=%lu.%06lu\n",*integer_d,*float_d);
	return;
}
static void cali_gyro_work_func(struct work_struct *work)
{
	mm_segment_t oldfs;
	pr_err("[cali_gyro_work_func]+++++++++++++\n");
	oldfs = get_fs();
	set_fs(KERNEL_DS);

	pid_filp = filp_open(FIH_E2P_PARTITION, O_RDWR, 0);
	if (IS_ERR(pid_filp)||pid_filp==NULL){
		pr_err("[cali_gyro_work_func]Open deviceinfo partition fail!\n");
	}
	set_fs(oldfs);
	atomic_set(&open_file,1);
}

static int write_ef(void *buf,size_t count)
{
	int len = 0,retry=0,ret=0;
	//struct file *pid_filp = NULL;
	mm_segment_t oldfs;

	pid_filp=NULL;//init pid_filp
	queue_delayed_work(cali_gyro_workq, &cali_gyro_work,0);//msecs_to_jiffies(100)

	while((atomic_read(&open_file)!=1))
	{
		if(retry++>10)
		{
			pr_err("[cali_gyro]write_ef cali_gyro_workq time out\n");
			cancel_delayed_work(&cali_gyro_work);
			break;
		}
		mdelay(100);
		pr_err("[cali_gyro]write_ef cali_gyro_workq mdelay\n");
	}
	atomic_set(&open_file,0);

	oldfs=get_fs();
	set_fs(KERNEL_DS);

	if((!IS_ERR(pid_filp))&&pid_filp!=NULL)
	{
		pid_filp->f_op->llseek(pid_filp,E2P_OFFSET,0);
		len = pid_filp->f_op->write(pid_filp,buf,count,&pid_filp->f_pos);
		if (len != count)
		{
			pr_err("[cali_gyro]write_ef write fail %d\n",len);
			ret=-1;
		}
		filp_close(pid_filp, NULL);
	}
	else {
		pr_err("[cali_gyro] open file fail %s\n", FIH_E2P_PARTITION);
		ret=-1;
	}
	set_fs(oldfs);
	return ret;
}

static int read_ef(void *buf,size_t count)
{
	int len = 0,retry=0,ret=0;
	//struct file *pid_filp = NULL;
	mm_segment_t oldfs;

	pid_filp=NULL;//init pid_filp
	queue_delayed_work(cali_gyro_workq, &cali_gyro_work,0);

	while((atomic_read(&open_file)!=1))
	{
		if(retry++>10)
		{
			pr_err("[cali_gyro]read_ef cali_gyro_workq time out\n");
			cancel_delayed_work(&cali_gyro_work);
			break;
		}
		mdelay(100);
		pr_err("[cali_gyro]read_ef cali_gyro_workq mdelay\n");
	}
	atomic_set(&open_file,0);

	oldfs=get_fs();
	set_fs(KERNEL_DS);

	if((!IS_ERR(pid_filp))&&pid_filp!=NULL)
	{
		pid_filp->f_op->llseek(pid_filp,E2P_OFFSET,0);
		len = pid_filp->f_op->read(pid_filp,buf,count,&pid_filp->f_pos);
		if (len != count)
		{
			pr_err("[cali_gyro]read_ef read fail %d\n",len);
			ret=-1;
		}
		filp_close(pid_filp, NULL);
	}
	else {
		pr_err("[cali_gyro] open file fail %s\n", FIH_E2P_PARTITION);
		ret=-1;
	}
	set_fs(oldfs);
	return ret;
}

static int my_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
	char * temp=NULL;
	char * token=NULL;
	char delim[] = ",";
	struct gyro_data data;
	int i=0,ret=0;
	char *s = NULL;

	fih_ever_write=1;

	temp=kzalloc(count,GFP_KERNEL);
	if(temp==NULL){
		pr_err("kzalloc fail\n");
		goto FAIL1;
	}
	if(copy_from_user(temp, buf, count)){
		pr_err("copy_from_user fail\n");
		goto FAIL;
	}

	pr_err("[cali_gyro]input=%s,size=%d\n",temp,count);

	for(token = strsep(&temp, delim); token != NULL; token = strsep(&temp, delim)) {
		pr_debug("[cali_gyro]string[%d]=%s\n",i,token);
		s = kstrdup(token,GFP_KERNEL);
		if(s==NULL){
			pr_err("[cali_gyro]kstrdup fail\n");
			goto FAIL;
		}
		float_to_int(s,&data.data[i],&data.data[i+1]);
		kfree(s);
		i=i+2;
	}
	if(i<6){
		pr_err("[cali_gyro]input too few,%d\n",i);
		goto FAIL;
	}
	memcpy(data.magic,MAGIC_GYRO,sizeof(char)*4);
	ret=write_ef((void *)&data, sizeof(struct gyro_data));
	if (ret < 0) {
		pr_err("[cali_gyro]write_ef fail\n");
		goto FAIL;
	}

FAIL:
	kfree(temp);
FAIL1:
	return count;
}

/* This function is called at the beginning of a sequence.
 * ie, when:
 * - the /proc file is read (first time)
 * - after the function stop (end of sequence)
 */
static void *my_seq_start(struct seq_file *s, loff_t *pos)
{
	if (((*pos)*PAGE_SIZE) >= my_proc_len) return NULL;
	return (void *)((unsigned long) *pos+1);
}

/* This function is called after the beginning of a sequence.
 * It's called untill the return is NULL (this ends the sequence).
 */
static void *my_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	++*pos;
	return my_seq_start(s, pos);
}

/* This function is called at the end of a sequence
 */
static void my_seq_stop(struct seq_file *s, void *v)
{
	/* nothing to do, we use a static value in start() */
}

/* This function is called for each "step" of a sequence
 */
static int my_seq_show(struct seq_file *s, void *v)
{
	int ret=0;
	struct gyro_data *data=NULL;
	char *buf = NULL;
	if(!fih_ever_write)
	{
		buf = (char *)ioremap(my_proc_addr, my_proc_size);
		if (buf == NULL) {
			pr_err("[cali_gyro]ioremap fail\n");
			return 0;
		}
		data=(struct gyro_data*)buf;
		pr_err("read gyro data from memory\n");
	}else{
		data=kzalloc(sizeof(struct gyro_data),GFP_KERNEL);
		if (data == NULL) {
			pr_err("[cali_gyro]kzalloc fail\n");
			return 0;
		}
		ret=read_ef((void *)data,sizeof(struct gyro_data));
		if (ret < 0) {
			pr_err("[cali_gyro]read_ef fail\n");
			goto FAIL;
		}
		pr_err("read gyro data from mmmc\n");
	}
	if(memcmp(data->magic,MAGIC_GYRO,sizeof(char)*4)!=0)
	{
		pr_err("magic compare fail, return default value\n");
		seq_printf(s,GYRO_DEFAULT);//default value
		goto FAIL;
	}

	pr_err("[cali_gyro]read=%ld.%06ld,%ld.%06ld,%ld.%06ld",data->data[0],data->data[1],data->data[2],data->data[3],data->data[4],data->data[5]);
	seq_printf(s,"%ld.%06ld,%ld.%06ld,%ld.%06ld",data->data[0],data->data[1],data->data[2],data->data[3],data->data[4],data->data[5]);

FAIL:
	if(!fih_ever_write)
		iounmap(buf);
	else
		kfree(data);
	return 0;
}

/* This structure gather "function" to manage the sequence
 */
static struct seq_operations my_seq_ops = {
	.start = my_seq_start,
	.next  = my_seq_next,
	.stop  = my_seq_stop,
	.show  = my_seq_show
};

/* This function is called when the /proc file is open.
 */
static int my_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &my_seq_ops);
};

/* This structure gather "function" that manage the /proc file
 */
static struct file_operations my_file_ops = {
	.owner   = THIS_MODULE,
	.open    = my_open,
	.write   = my_write,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};

/* This function is called when the module is loaded
 */
static int __init my_module_init(void)
{
	struct proc_dir_entry *entry;

	my_proc_len = my_proc_size;

	entry = proc_create(my_proc_name, 0664, NULL, &my_file_ops);
	if (!entry) {
		pr_err("%s: fail create %s proc\n", my_proc_name, my_proc_name);
	}

	cali_gyro_workq = create_singlethread_workqueue("cali_gyro_work_Q");
	if (!cali_gyro_workq)
		return -ENOMEM;
	INIT_DELAYED_WORK(&cali_gyro_work, cali_gyro_work_func);

	return 0;
}

/* This function is called when the module is unloaded.
 */
static void __exit my_module_exit(void)
{
	cancel_delayed_work(&cali_gyro_work);
	destroy_workqueue(cali_gyro_workq);
	remove_proc_entry(my_proc_name, NULL);
}

module_init(my_module_init);
module_exit(my_module_exit);
