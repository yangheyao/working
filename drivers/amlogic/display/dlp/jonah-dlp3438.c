#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/of_irq.h>
#include <dt-bindings/dlp/dlp.h>



#undef pr_fmt
#define pr_fmt(fmt) "DLP3438: " fmt

#define DLP3438_ADDR   0x1b
/*This is unless
#define DLP3438_ANALOG_ADDR  0x00
*/

//static int use_24m_clock;
struct i2c_client *g_aml_dlp3438_client = NULL;


#define CHECK_DRIVER()    \
{     \
  if(!g_aml_dlp3438_client) {         \
  printk("driver is not ready right now, wait.....\n");    \
  dump_stack();   \
  return -ENODEV;    \
  }  \
}
int aml_dlp3438_write_freeze(int32_t add, uint8_t val)
{
  int ret;
  uint8_t buf[2] = {};
  struct i2c_client *pdev;
  struct i2c_msg msg[] = {
    {
      .addr = DLP3438_ADDR,
      .flags = 0,
      .len = sizeof(buf),
      .buf = buf,
    }
  };

  CHECK_DRIVER();
  pdev = g_aml_dlp3438_client;

  buf[0] = add;
  buf[1] = val;

  printk("dlp write_test buf0[0x%x] buf1[0x%x]\n",buf[0],buf[1]);

  ret = i2c_transfer(pdev->adapter, msg,1);
  if(ret < 0){
    printk("%s: i2c transfer failed, reg:%d\n",__func__,ret);
    return ret;
  }

  return 0;
}

int aml_dlp3438_write_test(int32_t add, uint8_t val, uint8_t val1)
{
  int ret;
  //uint8_t buf[3] = {};
  uint8_t buf[3] = {};
  struct i2c_client *pdev;
  struct i2c_msg msg[] = {
    {
      .addr = DLP3438_ADDR,
      .flags = 0,
      .len = sizeof(buf),
      .buf = buf,
    }
  };

  CHECK_DRIVER();
  pdev = g_aml_dlp3438_client;

#if 0
  buf[0] = add & 0xff;
  buf[1] = (add >> 8) & 0xff;
  buf[2] = val & 0xff;
#endif
#if 0
  buf[0] = 0xbb;
  buf[1] = 0x00;
  buf[2] = 0x28;
#endif
  buf[0] = add;
  buf[1] = val;
  buf[2] = val1;

  printk("dlp write_test buf0[0x%x] buf1[0x%x] buf2[0x%x]\n",buf[0],buf[1],buf[2]);

  ret = i2c_transfer(pdev->adapter, msg,1);
  if(ret < 0){
    printk("%s: i2c transfer failed, reg:%d\n",__func__,ret);
    return ret;
  }

  return 0;
}

int aml_dlp3438_write(int32_t add, uint8_t val)
{
  int ret;
  //uint8_t buf[3] = {};
  //uint8_t buf[1] = {};
  uint8_t buf[2] = {};
  struct i2c_client *pdev;
  struct i2c_msg msg[] = {
    {
      .addr = DLP3438_ADDR,
      .flags = 0,
      .len = sizeof(buf),
      .buf = buf,
    }
  };

  CHECK_DRIVER();
  pdev = g_aml_dlp3438_client;

#if 0
  buf[0] = add & 0xff;
  buf[1] = (add >> 8) & 0xff;
  buf[2] = val & 0xff;
#endif
#if 0
  buf[0] = 0x35;
  buf[1] = 0x00;
#endif
  buf[0] = add;
  buf[1] = val;

  ret = i2c_transfer(pdev->adapter, msg,1);
  if(ret < 0){
    printk("%s: i2c transfer failed, reg:%d\n",__func__,ret);
    return ret;
  }

  return 0;
}


int aml_dlp3438_write16(int32_t add, uint16_t val)
{
  int ret;
  uint8_t buf[4] = {};
  struct i2c_client *pdev;
  struct i2c_msg msg[] = {
    {
      .addr = DLP3438_ADDR,
      .flags = 0,
      .len = sizeof(buf),
      .buf = buf,
    }
  };

  CHECK_DRIVER();
  pdev = g_aml_dlp3438_client;

  buf[0] = add & 0xff;
  buf[1] = (add >> 8) &0xff;
  buf[2] = val & 0xff;
  buf[3] = (val >> 8) & 0xff;

  ret = i2c_transfer(pdev->adapter, msg,1);
  if(ret <0){
    printk("%s: i2c transfer failed, ret:%d\n",__func__,ret);
    return ret;
  }
  return 0;
}

int aml_dlp3438_led(uint8_t add,uint8_t val1,uint8_t val2,uint8_t val3,uint8_t val4,uint8_t val5,uint8_t val6)
{
  int ret;
  //uint8_t buf[6] = {};
  uint8_t buf[7] = {};
  struct i2c_client *pdev;
  struct i2c_msg msg[] = {
    {
      .addr = DLP3438_ADDR,
      .flags = 0,
      .len = sizeof(buf),
      .buf = buf,
    }
  };

  CHECK_DRIVER();
  printk("This is dlp3438_keystone  i2c\n");

  pdev = g_aml_dlp3438_client;

  //buf[0] = 0x88;
  /*
  buf[1] = val & 0xff;
  buf[2] = (val >> 8) & 0xff;
  buf[3] = (val >> 16) & 0xff;
  buf[4] = (val >> 24) & 0xff;
  */
#if 0
  buf[1] = 0x01;
  buf[2] = 0x80;
  buf[3] = 0x01;
  buf[4] = 0x00;
  buf[5] = 0x00;
#endif
#if 0
  buf[4] = 0x00;
  buf[5] = 0xc8;
  buf[6] = 0x00;
#endif
  buf[0] = add;
  buf[1] = val1;
  buf[2] = val2;
  buf[3] = val3;
  buf[4] = val4;
  buf[5] = val5;
  buf[6] = val6;
  //printk("aml_dlp3438_keystone val1=0x%x val2=0x%x add=0x%x\n",val1,val2,add);
  printk("aml_dlp3438_led is start.\n");
  printk("buf0[0x%x],buf1[0x%x],buf2[0x%x]\n",buf[0],buf[1],buf[2]);
  printk("buf3[0x%x],buf4[0x%x],buf5[0x%x],buf6[0x%x]\n",buf[3],buf[4],buf[5],buf[6]);
  //printk("buf3[0x%x]\n",buf[3]);

  ret = i2c_transfer(pdev->adapter, msg,1);
  if(ret <0){
    printk("%s: i2c transfer failed, ret:%d\n",__func__,ret);
    return ret;
  }
  return 0;
}

//int aml_dlp3438_keystone(int8_t add, uint8_t val1,uint8_t val2)
//int aml_dlp3438_keystone(uint8_t add,uint32_t val1)
int aml_dlp3438_keystone(uint8_t add,uint8_t val1,uint8_t val2,uint8_t val3,uint8_t val4,uint8_t val5)
{
  int ret;
  //uint8_t buf[6] = {};
  uint8_t buf[6] = {};
  struct i2c_client *pdev;
  struct i2c_msg msg[] = {
    {
      .addr = DLP3438_ADDR,
      .flags = 0,
      .len = sizeof(buf),
      .buf = buf,
    }
  };

  CHECK_DRIVER();
  printk("This is dlp3438_keystone  i2c\n");

  pdev = g_aml_dlp3438_client;

  //buf[0] = 0x88;
  /*
  buf[1] = val & 0xff;
  buf[2] = (val >> 8) & 0xff;
  buf[3] = (val >> 16) & 0xff;
  buf[4] = (val >> 24) & 0xff;
  */
#if 0
  buf[1] = 0x01;
  buf[2] = 0x80;
  buf[3] = 0x01;
  buf[4] = 0x00;
  buf[5] = 0x00;
#endif
#if 0
  buf[4] = 0x00;
  buf[5] = 0xc8;
  buf[6] = 0x00;
#endif
  buf[0] = add;
  buf[1] = val1;
  buf[2] = val2;
  buf[3] = val3;
  buf[4] = val4;
  buf[5] = val5;
  //printk("aml_dlp3438_keystone val1=0x%x val2=0x%x add=0x%x\n",val1,val2,add);
  printk("aml_dlp3438_keystone val1=[0x%x]  add=[0x%x]\n",val1,add);
  printk("buf0[0x%x],buf1[0x%x],buf2[0x%x]\n",buf[0],buf[1],buf[2]);
  printk("buf3[0x%x],buf4[0x%x],buf5[0x%x]\n",buf[3],buf[4],buf[5]);
  //printk("buf3[0x%x]\n",buf[3]);

  ret = i2c_transfer(pdev->adapter, msg,1);
  if(ret <0){
    printk("%s: i2c transfer failed, ret:%d\n",__func__,ret);
    return ret;
  }
  return 0;
}

int aml_dlp3438_write32(int8_t add, uint32_t val)
{
  int ret;
  uint8_t buf[5] = {};
  struct i2c_client *pdev;
  struct i2c_msg msg[] = {
    {
      .addr = DLP3438_ADDR,
      .flags = 0,
      .len = sizeof(buf),
      .buf = buf,
    }
  };

  CHECK_DRIVER();
  printk("This is dlp3438_write32  32bit i2c\n");

  pdev = g_aml_dlp3438_client;

  buf[0] = add;
  buf[1] = val & 0xff;
  buf[2] = (val >> 8) & 0xff;
  buf[3] = (val >> 16) & 0xff;
  buf[4] = (val >> 24) & 0xff;
  printk("aml_dlp3438_write32 val=0x%x add=0x%x\n",val,add);
  printk("buf0[0x%x],buf1[0x%x],buf2[0x%x]\n",buf[0],buf[1],buf[2]);
  printk("buf3[0x%x],buf4[0x%x]\n",buf[3],buf[4]);

  ret = i2c_transfer(pdev->adapter, msg,1);
  if(ret <0){
    printk("%s: i2c transfer failed, ret:%d\n",__func__,ret);
    return ret;
  }
  return 0;
}

int aml_dlp3438_read(int add, uint8_t *val,uint8_t datalen)
//int aml_dlp3438_read(int add, char *val)
{
  int ret;
  uint8_t buf[2] = {};
  struct i2c_client *pdev;
  struct i2c_msg msg[] = {
    {
      .addr = DLP3438_ADDR,
      .flags = 0,
      .len = sizeof(buf),
      .buf = buf,
    }
    ,
    {
      .addr = DLP3438_ADDR,
      .flags = I2C_M_RD,
      .len = datalen,
      .buf = (uint8_t *)val,
    }
  };

  CHECK_DRIVER();
  pdev = g_aml_dlp3438_client;

  buf[0] = add& 0xff;
  buf[1] = (add >> 8) & 0xff;
  ret = i2c_transfer(pdev->adapter,msg,2);
  if(ret <0){
    printk("%s: i2c transfer failed, ret:%d\n",__func__,ret);
    return ret;
  }
  return 0;
}


int aml_dlp3438_read16(int add, uint16_t *val)
{
  int ret;
  uint8_t buf[2] = {};
  struct i2c_client *pdev;
  struct i2c_msg msg[] = {
    {
      .addr = DLP3438_ADDR,
      .flags = 0,
      .len = sizeof(buf),
      .buf = buf,
    }
    ,
    {
      .addr = DLP3438_ADDR,
      .flags = I2C_M_RD,
      .len = 2,
      .buf = (uint8_t *) val,
    }
  };

  CHECK_DRIVER();
  pdev = g_aml_dlp3438_client;

  buf[0] = add & 0xff;
  buf[1] = (add >> 8) &0xff;
  ret = i2c_transfer(pdev->adapter, msg,2);
  if(ret <0){
    printk("%s: i2c transfer failed, ret:%d\n",__func__,ret);
    return ret;
  }
  return 0;
}

#if 0
static int aml_dlp3438_reg_init(unsigned int reg_base,unsigned int *val, unsigned int reg_len)
{
  int ret = 0;
  unsigned int i = 0;

  for(i = 1;i <reg_len;i++){
    ret = aml_dlp3438_write(reg_base + i ,val[i]);
    if(ret <0)
      return ret;
  }
  return 0;

}
#endif

void dlp_jni_keystone(int temp)
{

  int tmp;
  int linjie = 40;

  aml_dlp3438_write_freeze(0x1a,0x01);
  mdelay(10);
  aml_dlp3438_keystone(0x88,0x01,0x4c,0x01,0x00,0x01);
  if(temp <= 40){
    //tmp = 0xff - temp;
  mdelay(100);
  aml_dlp3438_write_test(0xbb,0x00,temp);
  //printk("\033[;31mdlp_jni_test temp[0x%x] tmp[0x%x]\033[1m\n",temp,tmp);
  //aml_dlp3438_write_test(0xbb,0x00,0x16);
  mdelay(100);
  }else{
    temp = temp - linjie;
    tmp = 0xff -temp;
    mdelay(100);
    aml_dlp3438_write_test(0xbb,0x00,tmp);
    mdelay(100);
    printk("\033[;31mdlp_jni_test temp[0x%x] tmp[0x%x]\033[1m\n",temp,tmp);
    //printk("\033[;31mdlp_jni_test dir ==0 ,\033;\[1m\n",temp,tmp);
  }
  //aml_dlp3438_write_test(0xbb,0x0,0x1f);
  //mdelay(100);
  //aml_dlp3438_write_test(0xbb,0x00,0x20);
  aml_dlp3438_write_freeze(0x1a,0x00);
  //  printk("\033[;31mdlp_jni_test temp[0x%x] tmp[0x%x]\033[1m\n",temp,tmp);
  printk("\033[;\033[0m\n");

}
EXPORT_SYMBOL_GPL(dlp_jni_keystone);

void dlp_jni_led(int rval,int gval,int bval)
{
  int rhval,rlval,ghval,glval,bhval,blval;
  rhval = rval >> 8;
  rlval = rval & 0xff;

  ghval = gval >> 8;
  glval = gval & 0xff;

  bhval = bval >> 8;
  blval = bval & 0xff;

  aml_dlp3438_led(0x54,rlval,rhval,glval,ghval,blval,bhval);
  printk("This is dlp_jni_led rhval=%x,rlval=%x,ghval=%x,glval=%x,bhval=%x,blval=%x\n",rhval,rlval,ghval,glval,bhval,blval);

  mdelay(100);
  printk("jonah led set RGB is OK\n");

}
EXPORT_SYMBOL_GPL(dlp_jni_led);

void dlp_jni_read_filp(int *readfilp)
{
	uint8_t buff;
	aml_dlp3438_read(0x15,&buff,1);
	*readfilp = buff;
	printk("dlp3438.c dlp_jni_read_filp readfilp=[],buff[0x%x]\n",buff);
}
EXPORT_SYMBOL_GPL(dlp_jni_read_filp);

void dlp_jni_read_keystone(uint8_t *readkeyst,uint8_t *readkeyst1)
{
	uint8_t buff[2] = {};
	aml_dlp3438_read(0xbc,buff,2);
	*readkeyst = buff[0];
	*readkeyst1 = buff[1];
	printk("dlp3438.c dlp_jni_read_keystone buff0[0x%x],buff1[0x%x]\n",buff[0],buff[1]);

}
EXPORT_SYMBOL_GPL(dlp_jni_read_keystone);

void dlp_jni_read_led(uint8_t *buf,uint8_t *buf1)
{
  uint8_t buff[6] = {};
  aml_dlp3438_read(0x55,buff,6);
  printk("aml_dlp3438_read led 0x55 buf1[%x],buf[%x],buf[%x],buf[%x],buf[%x],buf[%x]\n",buff[0],buff[1],buff[2],buff[3],buff[4],buff[5]);
  *buf = buff[2];
  *buf1 = buff[3];
  printk("data=0x%x buff2[0x%x],buff3[0x%x]\n",buff[2]+buff[3],buff[2],buff[3]);
}
EXPORT_SYMBOL_GPL(dlp_jni_read_led);

void dlp_jni_flip(int val)
{
  /*This switch */
  switch(val){
  case 1111:
    aml_dlp3438_write(0x14,0x06);//This is normal
    mdelay(30);
    break;
  case 2222:
    aml_dlp3438_write(0x14,0x04);
    mdelay(30);
    break;
  case 3333:
    aml_dlp3438_write(0x14,0x02);
    mdelay(30);
    break;
  case 4444:
    aml_dlp3438_write(0x14,0x00);
    mdelay(30);
    break;
  default:
    aml_dlp3438_write(0x14,0x06);//This is normal
    break;
  }
  printk("This is dlp_jni_flip is OK\n");
}
EXPORT_SYMBOL_GPL(dlp_jni_flip);

void close_led(int data)
{
	if(data){
		aml_dlp3438_write(0x52,0x00);
	}else{
		aml_dlp3438_write(0x52,0x07);
	}
}
EXPORT_SYMBOL_GPL(close_led);

void dlp3438_phy_config(void)
{
  aml_dlp3438_led(0x54,0xef,0x01,0x26,0x02,0x26,0x02);
}

static int aml_dlp3438_power_init(void)
{
  //int ret = 0;
  //uint8_t ver = 0;


  dlp3438_phy_config();

  return 0;
}

static int aml_dlp3438_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
    printk("i2c check error, dev_id=%s--\n",id->name);
    return -ENODEV;
  }

  printk("This is aml_dlp3438_i2c_probe\n");
  i2c_set_clientdata(client,NULL);
  g_aml_dlp3438_client = client;

  /*aml_dlp3438 power init*/
  aml_dlp3438_power_init();

  return 0;
}

static int aml_dlp3438_i2c_remove(struct i2c_client *client)
{
  printk("enter %s\n",__func__);
  kfree(i2c_get_clientdata(client));

  return 0;
}

#define AML_I2C_BUS_AO	0
#define AML_I2C_BUS_A	1
#define AML_I2C_BUS_B	2
#define AML_I2C_BUS_C	3


static int aml_dlp3438_probe(struct platform_device *pdev)
{
  struct device_node *dlp3438_node = pdev->dev.of_node;
  struct device_node *child;
  struct i2c_board_info board_info;
  struct i2c_adapter *adapter;
  struct i2c_client *client;
  int err;
  int addr;
  int bus_type = -1;
  const char *str;
  printk("aml_dlp3438_probe is start\n");

  for_each_child_of_node(dlp3438_node, child){
    /*register exist dlp3438*/
    printk("%s, child name:%s\n",__func__,child->name);
    err = of_property_read_string(child, "i2c_bus", &str);
    if(err){
      printk("get 'i2c_bus' failed, ret:%d\n",err);
      continue;
    }

    if(!strncmp(str, "i2c_bus_ao",10))
      bus_type = AML_I2C_BUS_AO;
    else if(!strncmp(str, "i2c_bus_c", 9))
      bus_type = AML_I2C_BUS_C;
    else if(!strncmp(str, "i2c_bus_b", 9))
      bus_type = AML_I2C_BUS_B;
    else if(!strncmp(str, "i2c_bus_a", 9))
      bus_type = AML_I2C_BUS_A;
    else
      bus_type = AML_I2C_BUS_AO;
    
    err = of_property_read_string(child,"status", &str);
    if(err){
      printk("get 'status' failed, ret:%d\n",err);
      continue;
    }

    if(strcmp(str, "okay")&& strcmp(str, "ok")){
      /*status is not OK, do not probe it*/
      printk("device %s status is %s, stop probe it \n", child->name,str);
      continue;
    }

    err = of_property_read_u32(child, "reg", &addr);
    if(err){
      printk("get 'reg' failed, ret:%d\n",err);
      continue;
    }

    #if 0
    err = of_property_read_u32(child, "use_23m_clock", &use_24m_clock);
    if(err){
      pr_info("get 'use_24m_clock' failed, ret:%d\n",err);
      continue;
    }
    #endif

    memset(&board_info, 0, sizeof(board_info));
    adapter = i2c_get_adapter(bus_type);
    if(!adapter)
      printk("Wrong i2c adapter:%d\n",bus_type);

    err = of_property_read_string(child, "compatible", &str);
    if(err){
      printk("get 'compatibel' failed, ret:%d\n",err);
      continue;
    }

    strncpy(board_info.type, str, I2C_NAME_SIZE);
    board_info.addr = addr;
    board_info.of_node = child;/*for device driver*/
    board_info.irq=  irq_of_parse_and_map(child,0);
    client = i2c_new_device(adapter, &board_info);
    if(!client){
      printk("%s,allocate i2c_client failed\n",__func__);
      continue;
    }

    printk("%s: adapter:%d,addr:0x%x, node name:%s,type:%s\n","Allocate new i2c device", bus_type,addr,child->name,str);
  }
  return 0;
}

static int aml_dlp3438_remove(struct platform_device *pdev)
{
  /*nothing to do */
  return 0;
}

static const struct of_device_id aml_dlp3438_dt_match[] = {
  {
    .compatible = "amlogic, dlp3438_prober",
  },
  {}
};

static struct platform_driver aml_dlp3438_prober = {
  .probe  = aml_dlp3438_probe,
  .remove = aml_dlp3438_remove,
  .driver = {
    .name = "dlp3438_prober",
    .owner = THIS_MODULE,
    .of_match_table = aml_dlp3438_dt_match,
  },
};

#ifdef CONFIG_OF
static const struct of_device_id aml_dlp3438_match_id = {
  .compatible = "amlogic, dlp3438",
};
#endif

static const struct i2c_device_id aml_dlp3438_id_table[] = {
  {"aml_dlp3438",0},
  {}
};

static struct i2c_driver aml_dlp3438_i2c_driver = {
  .driver = {
    .name = "dlp3438",
    .owner = THIS_MODULE,
#ifdef CONFIG_OF
    .of_match_table = &aml_dlp3438_match_id,
#endif
  },
  .probe = aml_dlp3438_i2c_probe,
  .remove = aml_dlp3438_i2c_remove,
  //.resume = aml_dlp3438_i2c_resume,
  .id_table = aml_dlp3438_id_table,
};

static int __init aml_dlp3438_modinit(void)
{
  int ret;
  printk("This is aml_dlp3438_modinit\n");

  ret = platform_driver_register(&aml_dlp3438_prober);
  if(ret){
	  printk("platform driver register is error\n");
  }
  return i2c_add_driver(&aml_dlp3438_i2c_driver);
}


static void __exit aml_dlp3438_modexit(void)
{
  i2c_del_driver(&aml_dlp3438_i2c_driver);
  platform_driver_unregister(&aml_dlp3438_prober);
}

arch_initcall(aml_dlp3438_modinit);
module_exit(aml_dlp3438_modexit);

MODULE_DESCRIPTION("Cloudesteem Dlp3438 device driver");
MODULE_AUTHOR("Heyao Yang <yangheyao@cloudesteem.com>");
MODULE_LICENSE("GPL");
