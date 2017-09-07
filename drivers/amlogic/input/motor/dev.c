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

#include "drv8834.h"

#define HELLO_MAJOR 91
#define HELLO_MINOR 0  
#define NUMBER_OF_DEVICES 2  

/*jonah add*/
#define Bit_RESET 0
#define Bit_SET   1
//引脚宏定义  
#define GUA_DRV8834_SLEEP                                       "SLEEP_NAME"  
#define GUA_DRV8834_SLEEP_PIN                               ((GPIOAO_8)+117)  
#define GUA_DRV8834_DIR                                         "DIR_NAME"  
#define GUA_DRV8834_DIR_PIN                                 ((GPIOW_6)+131)  

#define GUA_DRV8834_STEP                                        "STEP_NAME"
#define GUA_DRV8834_STEP_PIN                                ((GPIOAO_11)+117) 
/*jonah end*/

struct class *hello_class;  
static struct cdev cdev;  
dev_t devno;  

static int dir=-1;

struct att_dev{
	struct platform_device *pdev;
	struct kobject *kobj;
};

static ssize_t att_store(struct device *dev, 
		struct device_attribute *attr,
		const char *buf,size_t count)
{
	int tmp;
#if 0
	char *test_get_val;
	test_get_val = kzalloc(count, GFP_KERNEL);
	if(copy_from_user(test_get_val,buf,count)){
		printk("copy_from_user is error\n");
	}
#endif
	tmp = simple_strtoul(buf,NULL,10);
	printk("simple_strtoul get_data tmp =%d, buf=%s\n",tmp,buf);
	//tmp = simple_strtoul(test_get_val,NULL,10);
	if(tmp == 1){
		printk("echo debug buf dir =%d\n",tmp);
		dir = 1;
	}
	if(tmp == 0){
		printk("echo debug buf dir =%d\n",tmp);
		dir = 0;
	}

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

static DEVICE_ATTR(test,0777,att_show,att_store);

static struct att_dev *dev = NULL;
static int att_probe(struct platform_device *ppdev){
	int ret;

	dev->kobj = kobject_create_and_add("attkobj", NULL);
	if(dev->kobj == NULL){
		ret = -ENOMEM;
		goto kobj_err;
	}

	ret = sysfs_create_file(&dev->pdev->dev.kobj,&dev_attr_test.attr);
	if(ret < 0 ){
		goto file_err;
	}
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
		.name = "att_test",
	},
};

//static int 

void DRV8834_IO_Init(void)
{
	int ret=0;
	
	ret = gpio_request(GUA_DRV8834_SLEEP_PIN,GUA_DRV8834_SLEEP);
	if(ret)
		printk("Request GUA_DRV8834_SLEEP_PIN is error\n");
	
	gpio_direction_output(GUA_DRV8834_SLEEP_PIN,1);
	
	ret = gpio_request(GUA_DRV8834_STEP_PIN,GUA_DRV8834_STEP);
	if(ret)
		printk("Request GUA_DRV8834_STEP_PIN is error\n");
	
	gpio_direction_output(GUA_DRV8834_STEP_PIN,1);
}
void DRV8834_Control(unsigned char nDRV8834_control,unsigned char nDRV8834_control_vaule )
{
	switch(nDRV8834_control)
	{
		case DRV8834_CONTROL_SLEEP:
	{
		      //睡眠  
      if(nDRV8834_control_vaule == DRV8834_CONTROL_SLEEP_ON)  
      {  
        gpio_set_value(GUA_DRV8834_SLEEP_PIN, Bit_RESET);     //SLEEP 0         

      }  
        
      //禁止睡眠  
      if(nDRV8834_control_vaule == DRV8834_CONTROL_SLEEP_OFF)  
      {  
        gpio_set_value(GUA_DRV8834_SLEEP_PIN, Bit_SET); 
	  }  
        
      break; 
	}
	
	    case DRV8834_CONTROL_DIR:   
    {                    
      //正向  
      if(nDRV8834_control_vaule == DRV8834_CONTROL_DIR_POSITIVE)  
      {  
		gpio_set_value(GUA_DRV8834_DIR_PIN, Bit_RESET);		//DIR 0                  
      }  
        
      //反向  
      if(nDRV8834_control_vaule == DRV8834_CONTROL_DIR_NEGATIVE)  
      {  
		gpio_set_value(GUA_DRV8834_DIR_PIN, Bit_SET);
		}  
        
      break;            
    }

    case DRV8834_CONTROL_STEP:   
    {       
      //初始化时，拉低电平做好准备  
      if(nDRV8834_control_vaule == DRV8834_CONTROL_STEP_INIT)  
      {  
		gpio_set_value(GUA_DRV8834_STEP_PIN, Bit_RESET);
	  }  
  
      //拉低STEP  
      if(nDRV8834_control_vaule == DRV8834_CONTROL_STEP_LOW)  
      {  
		gpio_set_value(GUA_DRV8834_STEP_PIN, Bit_RESET);
	  }  
  
      //拉高STEP  
      if(nDRV8834_control_vaule == DRV8834_CONTROL_STEP_HIGH)  
      {  
		gpio_set_value(GUA_DRV8834_STEP_PIN, Bit_SET);
	  }        
        
      break;            
    }

	//其他
	default:
	{
		break;
	}
  }
}

static void DRV8834_Set_Direction(int nDirection)
{
	//方向
  switch(nDirection)
  {
	case MOTOR_CONFIG_DIRECTION_NEGATIVE:
	{
	  printk("settings dir is 1\n");
		DRV8834_Control(DRV8834_CONTROL_DIR, DRV8834_CONTROL_DIR_NEGATIVE);
		break;
	}
	case MOTOR_CONFIG_DIRECTION_POSITIVE:
	{
	  printk("Settings dir is 0\n");
		DRV8834_Control(DRV8834_CONTROL_DIR, DRV8834_CONTROL_DIR_POSITIVE);
		break;
	}
	
	default:
	{
		break;
	}
  }
}

static int DRV8834_Step_Move(int setps)
{
	int nSteps = setps;
	//int nDirection = (nSteps >= 0)?1:-1;
	int nDirection = (dir == 1)?1:-1;
	int nPulse_duration = 12;
	int Status = MOTOR_CONFIG_STATUS_ACTIVE;
	int tmp;
	int StepsCount,DirectionPosition,DirectionSteps;

  DRV8834_Control(DRV8834_CONTROL_SLEEP, DRV8834_CONTROL_SLEEP_OFF);   
  //nSteps = nSteps * nDirection;  //获取步数  
  nSteps = nSteps;  //获取步数  
  DRV8834_Set_Direction(nDirection);  
  StepsCount = 0;
  while(nSteps && (Status == MOTOR_CONFIG_STATUS_ACTIVE))
  {  
    nSteps--;            //计算剩余步数  
    DirectionPosition += nDirection*1;
    StepsCount++;
        
    DRV8834_Control(DRV8834_CONTROL_STEP, DRV8834_CONTROL_STEP_LOW);  
      
    udelay(100);
      
    DRV8834_Control(DRV8834_CONTROL_STEP, DRV8834_CONTROL_STEP_HIGH);  
      
    udelay(100);
  }  
  DirectionSteps = 0;
  Status = MOTOR_CONFIG_STATUS_IDLE;   
    
  DRV8834_Control(DRV8834_CONTROL_SLEEP, DRV8834_CONTROL_SLEEP_ON);     
  DRV8834_Control(DRV8834_CONTROL_STEP,DRV8834_CONTROL_STEP_INIT);
  
  //计算实际位移  
  //pMotor_Config->DirectionPosition %= DRV8834_STEP_PER_CIRCLE;                 //每DRV8834_STEP_PER_CIRCLE次为1圈，重新计数
  DirectionPosition %= DRV8834_STEP_PER_CIRCLE;
    
  tmp = (StepsCount*2*nPulse_duration/10);
  printk("DRV8834_Step_Move return data=[%d]\n",tmp);
  return 0;
}

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

//static ssize_t hello_open(struct inode *inode,struct file *file)  
static int hello_open(struct inode *inode,struct file *file)  
{  
	return 0;   
}  
static ssize_t hello_write(struct file *filp,const char __user *buf,size_t count,loff_t *f_pos)
{
	int tmp;
#if 1
	char *get_val;
	get_val = kzalloc(count, GFP_KERNEL);
	if(copy_from_user(get_val,buf,count)){
		printk("copy_from_user is error\n");
	}
#endif
	printk("#############write#########\n");
	printk("copy_from_user data=%s\n",get_val);
	tmp = simple_strtoul(buf,NULL,10);
	printk("simple_strtoul get_data tmp =%d\n",tmp);
	if(tmp > 123){
		printk("This goto motor enable\n");
		DRV8834_Step_Move(tmp);
	}
	return count;
}

static const struct file_operations hello_fops = {  
	.open = hello_open,  
	.read = hello_read,  
	.owner = THIS_MODULE,  
	.release = hello_close,
	.write = hello_write,
};  

static int __init hello_init(void)  
{  
	int ret;  
	devno = MKDEV(HELLO_MAJOR,HELLO_MINOR);  
	/*jonah add attr*/
	dev = kzalloc(sizeof(struct att_dev),GFP_KERNEL);
	if(dev == NULL){
		printk("%s get dev memory error\n",__func__);
		return -ENOMEM;
	}
	dev->pdev = platform_device_register_simple("att_test", -1,NULL,0);
	if(IS_ERR(dev->pdev)){
		printk("%s pdev error\n",__func__);
		return -1;
	}
	ret = platform_driver_register(&att_driver);
	if(ret < 0){
		printk("%s register driver error\n",__func__);
		return ret;
	}

	DRV8834_IO_Init();
	DRV8834_Control(DRV8834_CONTROL_SLEEP, DRV8834_CONTROL_SLEEP_ON);
	
	//nENABL使能
	//DRV8834_Control(DRV8834_CONTROL_OUTPUTS,DRV8834_CONTROL_OUTPUTS_ENABLE);
	
	//STEP低
	DRV8834_Control(DRV8834_CONTROL_STEP,DRV8834_CONTROL_STEP_INIT);

	if(HELLO_MAJOR){  
		ret = register_chrdev_region(devno,NUMBER_OF_DEVICES,"chrdev");  
	}else{  
		ret = alloc_chrdev_region(&devno, 0, NUMBER_OF_DEVICES, "chrdev");  
	}  
	if(ret < 0){  
		printk("%s register chrdev error\n",__func__);  
		return ret;  
	}  

	hello_class = class_create(THIS_MODULE,"hello_char_calss");  
	if(IS_ERR(hello_class)){  
		printk("%s create class error\n",__func__);  
		return -1;  
	}  

	device_create(hello_class, NULL, devno, NULL, "chrdev");  


	cdev_init(&cdev, &hello_fops);  
	cdev.owner = THIS_MODULE;  
	cdev_add(&cdev, devno, NUMBER_OF_DEVICES);  

	return 0;  
}  


static void __exit hello_exit(void)  
{  
	printk("%s",__func__);  
	cdev_del(&cdev);  
	device_destroy(hello_class,devno);  
	class_destroy(hello_class);  
	unregister_chrdev_region(devno,NUMBER_OF_DEVICES);  

}  

module_init(hello_init);  
module_exit(hello_exit);  
MODULE_LICENSE("GPL");  
MODULE_AUTHOR("weed<weed_hz@126.com>");  
