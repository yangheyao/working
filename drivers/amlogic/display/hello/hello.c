#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/device.h>

#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/fcntl.h>

#include <linux/poll.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/time.h>

/*jonah add for unifykey*/
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/reset.h>
#include <linux/amlogic/vout/lcd_unifykey.h>
/*end*/


#include "hello.h"

/*定义主设备和从设备号变量*/
static int hello_major = 0;
static int hello_minor = 0;

/*设备类别和设备变量*/
static struct class* hello_class = NULL;
static struct hello_test_dev* hello_dev = NULL;

/*传统的设备文件操作方法*/
static int hello_open(struct inode* inode, struct file* filp);
static int hello_release(struct inode* inode, struct file* filp);
static ssize_t hello_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos);
static ssize_t hello_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos);

/*设备文件操作方法表*/
static struct file_operations hello_fops = {
    .owner = THIS_MODULE,
    .open = hello_open,
    .release = hello_release,
    .read = hello_read,
    .write = hello_write,
};

/*访问设置属性方法*/
static ssize_t hello_val_show(struct device* dev, struct device_attribute* attr,  char* buf);
static ssize_t hello_val_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count);

/*定义设备属性*/
static DEVICE_ATTR(val, S_IRUGO | S_IWUSR, hello_val_show, hello_val_store);

/*打开设备方法*/
static int hello_open(struct inode* inode, struct file* filp) {
    struct hello_test_dev* dev;

    /*将自定义设备结构体保存在文件指针的私有数据域中，以便访问设备时拿来用*/
    dev = container_of(inode->i_cdev, struct hello_test_dev, dev);
    filp->private_data = dev;

    return 0;
}

/*设备文件释放时调用，空实现*/
static int hello_release(struct inode* inode, struct file* filp) {
    return 0;
}

extern void dlp_jni_read_led(uint8_t *led_buf,uint8_t *led_buf1);
extern void dlp_jni_read_filp(int *readfilp);
extern void dlp_jni_read_keystone(uint8_t *readkeyst,uint8_t *readkeyst1);

/*读取设备的寄存器val的值*/
static ssize_t hello_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos) {
    ssize_t err = 0;
	int ret;
    //struct hello_test_dev* dev = filp->private_data;
    //char test[4];
    char led_buf;
    char led_buf1;
    //int number = 123;
    char temp[9];
    uint16_t led_temp;
    uint16_t filp_temp;
    int i=0;

    char readkeyst;
    char readkeyst1;
    int readfilp;
#if 1
    char *key_name;
    unsigned int key_len;
    unsigned char buuf[600];
    key_name = "dlp_keystone";
    key_len = 2;
    ret = key_unify_read(key_name,buuf,key_len,&key_len);
#endif

    memset(temp,0,9);
    dlp_jni_read_keystone(&readkeyst,&readkeyst1);
    dlp_jni_read_filp(&readfilp);
    dlp_jni_read_led(&led_buf,&led_buf1);
    printk("hello.c one hello_read led_buf[0x%x] led_buf[0x%x] readkey[%d] readkey1[%d] readfilp[%d]\n",led_buf,led_buf1,readkeyst,readkeyst1,readfilp);
    /*This is need to len_buf && len_buff yunsuan*/

    filp_temp = simple_strtoul(buuf,NULL,10);
    led_temp = (led_buf1 <<8) | led_buf;
    printk("hello.c two hello_read led_buf[0x%x] led_buf[0x%x] led_temp[0x%x] readkeyst[0x%x]\n",led_buf,led_buf1,led_temp,filp_temp);
    sprintf(temp,"%d-%d-%d",led_temp,filp_temp,readfilp);
    while(temp[i] != '\0'){
	    i++;
    }
    printk("temp_data[%s]\n",temp);
#if 0
    sprintf(temp,"%d",number);
    strcpy(test,temp);
#endif


    printk("%s hello_read. datalen[%d]\n",__func__,i);
	/*
    if(down_interruptible(&(dev->sem))) {
		printk("hello_read down_interruptible is error\n");
        return -ERESTARTSYS;
    }
    */
    /*
    if(count < sizeof(dev->val)) {
		printk("hello_read count is error\n");
        goto out;
    }
    */
    //if(copy_to_user(buf, dev->val, sizeof(dev->val))) {
    if(copy_to_user(buf, temp, i+2)) {
		printk("hello_read copy_to_user is error\n");
        err = -EFAULT;
        //goto out;
    }

    //err = sizeof(dev->val);

    /*
out:
    up(&(dev->sem));
    */

    //return sizeof(int);
    return i+2;
}

/*写字符串*/
extern int dlp_unifykey_set(unsigned char *buf, int *len);
extern void dlp_jni_keystone(int);
extern void dlp_jni_led(int rval,int gval,int bval);
extern void dlp_jni_flip(int val);
extern void close_led(int data);
extern int key_unify_write_write(char *keyname,unsigned char *keydata,unsigned int datalen);
static ssize_t hello_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos) {
    struct hello_test_dev* dev = filp->private_data;
    ssize_t err = 0;
	//int res = 0;
	//char *after;
	//unsigned long res;
	//size_t haha;
	int tmp;
	int temp;
	int ret;
	char *dlp_keystone_set;
	char *dlp_led_set;
	char *dlp_filp_set;
	int dlp_led_set_len;
	int dlp_keystone_set_len;
	int dlp_filp_set_len;
	char *dlp_keystone_name;
	char *dlp_led_name;
	char *dlp_filp_name;
	char *test;

	dlp_keystone_name = "dlp_keystone";
	dlp_led_name = "dlp_led";
	dlp_filp_name = "dlp_filp";

	printk("hello_write buff is %s\n",buf);
	//tmp = buf[0];
	//printk("hello_write buff temp is %d\n",tmp);

	dlp_keystone_set = kzalloc(count,GFP_KERNEL);
	if(!dlp_keystone_set){
		printk("memory not enough,%s:%d\n",__func__,__LINE__);
		return -ENOMEM;
	}
	dlp_led_set = kzalloc(count,GFP_KERNEL);
	if(!dlp_led_set){
		printk("memory not enough,%s:%d\n",__func__,__LINE__);
		return -ENOMEM;
	}
	dlp_filp_set = kzalloc(count,GFP_KERNEL);
	if(!dlp_filp_set){
		printk("memory not enough,%s:%d\n",__func__,__LINE__);
		return -ENOMEM;
	}

	tmp = simple_strtoul(buf,NULL,10);
	//haha = after - buf;
	printk("hello_write buff  tmp=%d \n",tmp);

  printk("hello_write tmp %d\n",tmp);
  /*keystone is send -255---255*/
  if(tmp < 255){
    printk("This is dlp_jni_keystone is start tmp=%d\n",tmp);
    dlp_jni_keystone(tmp);
	if(copy_from_user(dlp_keystone_set,buf,count)){
	  ret = -EFAULT;
	  goto exit;
	}
    dlp_keystone_set_len = sizeof(dlp_keystone_set);
    mdelay(10);
    key_unify_write(dlp_keystone_name,dlp_keystone_set,dlp_keystone_set_len);
  }

  /*300--400 value*/
  if(tmp > 256){
    if(tmp < 420){
      printk("This is dlp_jni_led is start tmp=%d\n",tmp);
      dlp_jni_led(tmp,tmp,tmp);
	  if(copy_from_user(dlp_led_set,buf,count)){
		ret = -EFAULT;
	    goto exit;
	  }
      mdelay(10);
      dlp_led_set_len = sizeof(dlp_led_set);
      key_unify_write(dlp_led_name,dlp_led_set,dlp_led_set_len);
    }
  }

  /*1111/2222/3333/4444*/
  if(tmp > 450){
    printk("This is dlp_jni_flip is start tmp=%d\n",tmp);
	temp = tmp;
    dlp_jni_flip(tmp);
    if(temp == 1111){
	    printk("This is dlp_filp is 6\n");
	    test = "6";
    }
    if(temp == 2222){
	    printk("This is dlp_filp is 4\n");
	    test = "4";
    }
    if(temp == 3333){
	    printk("This is dlp_filp is 2\n");
	    test = "2";
    }
    if(temp == 4444){
	    printk("This is dlp_filp is 0\n");
	    test = "0";
    }
    /*
    if(copy_from_user(dlp_filp_set,buf,count)){
	    printk("This is error\n");
	ret = -EFAULT;
	goto exit;
     }
     */
    mdelay(10);
    //dlp_filp_set_len = sizeof(dlp_filp_set);
    //key_unify_write(dlp_filp_name,dlp_filp_set,dlp_filp_set_len);
    dlp_filp_set_len = sizeof(test);
    key_unify_write(dlp_filp_name,test,dlp_filp_set_len);
  }

    if(tmp ==1222){
       close_led(1);
    }
    if(tmp ==1333){
       close_led(0);
    }
    if(tmp == 1444){
      close_gsensor(1);
    }
    if(tmp == 1555){
      close_gsensor(0);
    }

    if(down_interruptible(&(dev->sem))) {
		printk("hello_write down_interruptible is error\n");
        return -ERESTARTSYS;
    }

    if(copy_from_user(dev->val, buf, 1)) {
		printk("hello_write copy_from_user is error\n");
        err = -EFAULT;
        goto out;
    }


out:
    up(&(dev->sem));

exit:
    kfree(dlp_keystone_set);
    return ret;

    return 0;
}


static ssize_t __hello_set_val(struct hello_test_dev* dev, const char* buf, size_t count) {

  /*同步访问*/
    if(down_interruptible(&(dev->sem))) {
        return -ERESTARTSYS;
    }
    printk(KERN_ALERT"__hello_set_val.buf:   count:\n");
    printk(KERN_ALERT"__hello_set_val.dev->val:  count:\n");
    strncpy(dev->val,buf, count);
    printk(KERN_ALERT"__hello_set_val.dev->val:   count:\n");
    up(&(dev->sem));

    return count;
}

/*读取设备属性val*/
static ssize_t hello_val_show(struct device* dev, struct device_attribute* attr, char* buf) {
    struct hello_test_dev* hdev = (struct hello_test_dev*)dev_get_drvdata(dev);
    printk(KERN_ALERT"hello_val_show.\n");
    printk(KERN_ALERT"%s\n",hdev->val);
    return 0;
}

/*写设备属性val*/
static ssize_t hello_val_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {
    struct hello_test_dev* hdev = (struct hello_test_dev*)dev_get_drvdata(dev);
    printk(KERN_ALERT"hello_val_store.buf:  count:\n");

    return __hello_set_val(hdev, buf, count);
}


/*初始化设备*/
static int  __hello_setup_dev(struct hello_test_dev* dev) {
    int err;
    dev_t devno = MKDEV(hello_major, hello_minor);

    memset(dev, 0, sizeof(struct hello_test_dev));

    cdev_init(&(dev->dev), &hello_fops);
    dev->dev.owner = THIS_MODULE;
    dev->dev.ops = &hello_fops;

    /*注册字符设备*/
    err = cdev_add(&(dev->dev),devno, 1);
    if(err) {
        return err;
    }

    /*初始化信号量和寄存器val的值*/
    //init_MUTEX(&(dev->sem));
    sema_init(&(dev->sem),1);
    dev->val = kmalloc(10,GFP_KERNEL);
    strncpy(dev->val,"hello",sizeof("hello"));
    return 0;
}

/*模块加载方法*/
static int __init hello_init(void){
    int err = -1;
    dev_t dev = 0;
    struct device* temp = NULL;

    printk(KERN_ALERT"hello_init.\n");

    /*动态分配主设备和从设备号*/
    err = alloc_chrdev_region(&dev, 0, 1, HELLO_DEVICE_NODE_NAME);
    if(err < 0) {
        printk(KERN_ALERT"Failed to alloc char dev region.\n");
        goto fail;
    }

    hello_major = MAJOR(dev);
    hello_minor = MINOR(dev);

    /*分配helo设备结构体变量*/
    hello_dev = kmalloc(sizeof(struct hello_test_dev), GFP_KERNEL);
    if(!hello_dev) {
        err = -ENOMEM;
        printk(KERN_ALERT"Failed to alloc hello_dev.\n");
        goto unregister;
    }

    /*初始化设备*/
    err = __hello_setup_dev(hello_dev);
    if(err) {
        printk(KERN_ALERT"Failed to setup dev: %d.\n", err);
        goto cleanup;
    }

    /*在/sys/class/目录下创建设备类别目录hello*/
    hello_class = class_create(THIS_MODULE, HELLO_DEVICE_CLASS_NAME);
    if(IS_ERR(hello_class)) {
        err = PTR_ERR(hello_class);
        printk(KERN_ALERT"Failed to create hello class.\n");
        goto destroy_cdev;
    }

    /*在/dev/目录和/sys/class/hello目录下分别创建设备文件hello*/
    temp = device_create(hello_class, NULL, dev, "%s", HELLO_DEVICE_FILE_NAME);
    if(IS_ERR(temp)) {
        err = PTR_ERR(temp);
        printk(KERN_ALERT"Failed to create hello device.");
        goto destroy_class;
    }

    /*在/sys/class/hello/hello目录下创建属性文件val*/
    err = device_create_file(temp, &dev_attr_val);
    if(err < 0) {
        printk(KERN_ALERT"Failed to create attribute val.");
        goto destroy_device;
    }

    dev_set_drvdata(temp, hello_dev);

    printk(KERN_ALERT"Succedded to initialize hello device.\n");
    return 0;

destroy_device:
    device_destroy(hello_class, dev);

destroy_class:
    class_destroy(hello_class);

destroy_cdev:
    cdev_del(&(hello_dev->dev));

cleanup:
    kfree(hello_dev);

unregister:
    unregister_chrdev_region(MKDEV(hello_major, hello_minor), 1);

fail:
    return err;
}

/*模块卸载方法*/
static void __exit hello_exit(void) {
    dev_t devno = MKDEV(hello_major, hello_minor);

    printk(KERN_ALERT"hello_exit\n");

    /*销毁设备类别和设备*/
    if(hello_class) {
        device_destroy(hello_class, MKDEV(hello_major, hello_minor));
        class_destroy(hello_class);
    }

    /*删除字符设备和释放设备内存*/
    if(hello_dev) {
        cdev_del(&(hello_dev->dev));
        kfree(hello_dev);
    }
    if(hello_dev->val != NULL){
	 kfree(hello_dev->val);
    }
    /*释放设备号*/
    unregister_chrdev_region(devno, 1);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Test Driver");

module_init(hello_init);
module_exit(hello_exit);




