#include <linux/module.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <dt-bindings/gpio/gxtvbb.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <dt-bindings/dlp/dlp.h>

#include "drv8834.h"

#define HELLO_MAJOR 92
#define HELLO_MINOR 0
#define NUMBER_OF_DEVICES 2
#define IN_BUF_LEN 256


struct class *test_class;
static struct cdev cdev;
dev_t devtest;
static struct semaphore sem;

//static int dir=-1;

struct att_dev{
	struct platform_device *pdev;
	struct kobject *kobj;
	int x;
	int y;
	struct task_struct *run_thread;
	struct semaphore sem;
};
static struct att_dev *dev = NULL;

static ssize_t att_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,size_t count)
{
	int tmp,res,rval,gval,bval;
	res = sscanf(buf,"%d %d %d",&rval,&gval,&bval);
	printk("att_store rval=%d %x gval=%d %x bval=%d %x\n",rval,rval,gval,gval,bval,bval);
	//test_dlp_led(rval,gval,bval);

	tmp = simple_strtoul(buf,NULL,10);
	printk("simple_strtoul get_data tmp =%d, buf=%s\n",tmp,buf);

	printk("echo debug buf\n");
	return count;
}

static ssize_t att_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	printk("cat debug buf\n");
	return 0;
}

static ssize_t att_led_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,size_t count)
{
	int tmp,val,res,id;
	res = sscanf(buf,"%d %d",&val,&id);
	printk("att_store val=%d %x id=%d %x\n",val,val,id,id);
	//test_dlp(val,id);
#if 1
	tmp = simple_strtoul(buf,NULL,10);
	printk("simple_strtoul get_data tmp =%d, buf=%s\n",tmp,buf);
#endif

	printk("echo debug buf\n");
	return count;
}

static ssize_t att_led_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	printk("cat debug buf\n");
	return 0;
}

static DEVICE_ATTR(test_dlp,0777,att_show,att_store);
static DEVICE_ATTR(test_dlp_led,0777,att_led_show,att_led_store);


static struct attribute *attrs[] = {
	&dev_attr_test_dlp.attr,
	&dev_attr_test_dlp_led.attr,
	NULL,
};

static struct attribute_group test_dlp_group = {
	.attrs = attrs,
};

static int att_probe(struct platform_device *ppdev){
	int ret;

	dev->kobj = kobject_create_and_add("testkobj", NULL);
	if(dev->kobj == NULL){
		ret = -ENOMEM;
		goto kobj_err;
	}

	//ret = sysfs_create_file(&dev->pdev->dev.kobj,&dev_attr_test_dlp.attr);
	ret = sysfs_create_group(&dev->pdev->dev.kobj,&test_dlp_group);
	if(ret < 0 ){
		goto file_err;
	}
#if 0
	sema_init(&(dev->sem),0);

	dev->run_thread = kthread_run(vmouse_thread,dev,"vmouse_thread");
#endif
	return 0;

file_err:
	kobject_del(dev->kobj);
kobj_err:
	return ret;
}

static struct platform_driver att_driver = {
	.probe = att_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "att_test_dlp",
	},
};



static ssize_t hello_read(struct file *file, char __user *buf, size_t count,
		loff_t *ppos)
{
	char *str = "1";

	if(copy_to_user(buf,str,strlen(str))){
		printk("copy_to_user is error\n");
	}
	*(buf + strlen(str)) = '\n';
	return 1;
}

static int hello_close(struct inode *inode,struct file *file)
{
	printk("##########release########\n");
	return 0;
}

static int hello_open(struct inode *inode,struct file *file)
{
	return 0;
}
static ssize_t hello_write(struct file *filp,const char __user *buf,size_t count,loff_t *f_pos)
{
	//int tmp;
	//int val,res,id;
#if 1
	char *get_val;
	get_val = kzalloc(IN_BUF_LEN, GFP_KERNEL);
	//res = sscanf(buf,"%d %d",&val,&id);

	//printk("att_store val=%d id=%d\n",val,id);
	if(copy_from_user(get_val,buf,count)){
		printk("copy_from_user is error\n");
		return -EFAULT;
	}

#endif
	printk("#############write#########\n");
	printk("copy_from_user data=%x %x %x %x %x %x\n",get_val[0],get_val[1],get_val[2],get_val[3],get_val[4],get_val[5]);
	aml_dlp3438_led(0x54,get_val[1],get_val[0],get_val[3],get_val[2],get_val[5],get_val[4]);
#if 0
	tmp = simple_strtoul(buf,NULL,10);
	printk("simple_strtoul get_data tmp =%d\n",tmp);
	if(tmp > 123){
		printk("This goto motor enable\n");
	}
#endif
	return count;
}

static const struct file_operations hello_fops = {
	.open = hello_open,
	.read = hello_read,
	.owner = THIS_MODULE,
	.release = hello_close,
	.write = hello_write,
};

static int __init test_init(void)
{
	int ret;
	printk("%s is start\n",__func__);
	devtest = MKDEV(HELLO_MAJOR,HELLO_MINOR);
	/*jonah add attr*/
	dev = kzalloc(sizeof(struct att_dev),GFP_KERNEL);
	if(dev == NULL){
		printk("%s get dev memory error\n",__func__);
		return -ENOMEM;
	}
	dev->pdev = platform_device_register_simple("att_test_dlp", -1,NULL,0);
	if(IS_ERR(dev->pdev)){
		printk("%s pdev error\n",__func__);
		return -1;
	}
	ret = platform_driver_register(&att_driver);
	if(ret < 0){
		printk("%s register driver error\n",__func__);
		return ret;
	}

	if(HELLO_MAJOR){
		ret = register_chrdev_region(devtest,NUMBER_OF_DEVICES,"test_dlp");
	}else{
		ret = alloc_chrdev_region(&devtest, 0, NUMBER_OF_DEVICES, "test_dlp");
	}
	if(ret < 0){
		printk("%s register chrdev error\n",__func__);
		return ret;
	}

	test_class = class_create(THIS_MODULE,"hello_test_calss");
	if(IS_ERR(test_class)){
		printk("%s create class error\n",__func__);
		return -1;
	}

	device_create(test_class, NULL, devtest, NULL, "test_dlp");
	sema_init(&sem, 1);


	cdev_init(&cdev, &hello_fops);
	cdev.owner = THIS_MODULE;
	cdev_add(&cdev, devtest, NUMBER_OF_DEVICES);

	return 0;
}


static void __exit test_exit(void)
{
	printk("%s",__func__);
	cdev_del(&cdev);
	device_destroy(test_class,devtest);
	class_destroy(test_class);
	unregister_chrdev_region(devtest,NUMBER_OF_DEVICES);

}

module_init(test_init);
module_exit(test_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("yangheyao@cloudesteem.cn>");
