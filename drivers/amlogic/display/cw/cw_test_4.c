#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>


#include <linux/of_gpio.h>
#include <dt-bindings/gpio/gxtvbb.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/amlogic/sd.h>


#define DC_DETECT_GPIO ((GPIOW_6)+131)

#define CWFG_ENABLE_LOG 1 //CHANGE   Customer need to change this for enable/disable log
#define CWFG_I2C_BUSNUM 2 //CHANGE   Customer need to change this number according to the principle of hardware
#define DOUBLE_SERIES_BATTERY 0
/*
#define USB_CHARGING_FILE "/sys/class/power_supply/usb/online" // Chaman
#define DC_CHARGING_FILE "/sys/class/power_supply/ac/online"
*/
#define queue_delayed_work_time  8000
#define CW_PROPERTIES "cw-bat"

#define CW2015_ADDR 0x62

#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_VTEMPL              0xC
#define REG_VTEMPH              0xD
#define REG_BATINFO             0x10

#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)        // ATHD = 0%

#define CW_I2C_SPEED		100000

#define BATTERY_UP_MAX_CHANGE   420*1000            // The time for add capacity when charging
#define BATTERY_DOWN_MAX_CHANGE 120*1000
#define BATTERY_JUMP_TO_ZERO    30*1000
#define BATTERY_CAPACITY_ERROR  40*1000
#define BATTERY_CHARGING_ZERO   1800*1000

#define CHARGING_ON 1
#define NO_CHARGING 0


#define cw_printk(fmt, arg...)        \
	({                                    \
		if(CWFG_ENABLE_LOG){              \
			printk("FG_CW2015 : %s : " fmt, __FUNCTION__ ,##arg);  \
		}else{}                           \
	})     //need check by Chaman


#define CWFG_NAME "cw2015"
#define SIZE_BATINFO    64

#if 1
static unsigned char config_info[SIZE_BATINFO] = {
    0x17, 0x67, 0x66, 0x6C, 0x6A, 0x69, 0x64, 0x5E,
    0x65, 0x6B, 0x4E, 0x52, 0x4F, 0x4F, 0x46, 0x3C,
    0x35, 0x2B, 0x24, 0x20, 0x21, 0x2F, 0x42, 0x4C,
    0x24, 0x4A, 0x0B, 0x85, 0x31, 0x51, 0x57, 0x6D,
    0x77, 0x6B, 0x6A, 0x6F, 0x40, 0x1C, 0x7C, 0x42,
    0x0F, 0x31, 0x1E, 0x50, 0x86, 0x95, 0x97, 0x27,
    0x57, 0x73, 0x95, 0xC3, 0x80, 0xD8, 0xFF, 0xCB,
    0x2F, 0x7D, 0x72, 0xA5, 0xB5, 0xC1, 0x73, 0x09,
};
#endif

#if 0
static unsigned char config_info[SIZE_BATINFO] = {
  0x15, 0x42, 0x60, 0x59, 0x52, 0x58, 0x4D, 0x48, 0x48, 0x44, 0x44, 0x46, 0x49,0x48,0x32,
  0x24, 0x20, 0x17, 0x13, 0x0F, 0x19, 0x3E, 0x51, 0x45, 0x08, 0x76, 0x0B, 0x85, 0x0E, 0x1C, 0x2E, 0x3E, 0x4D, 0x52, 0x52,
  0x57, 0x3D, 0x1B, 0x6A, 0x2D, 0x25, 0x43, 0x52, 0x87, 0x8F, 0x91, 0x94, 0x52, 0x82, 0x8C, 0x92, 0x96, 0xFF, 0x7B, 0xBB,
  0xCB, 0x2F, 0x7D, 0x72, 0xA5, 0xB5, 0xC1, 0x46, 0xAE,
};
#endif

//static struct power_supply *chrg_usb_psy;
static struct power_supply *chrg_ac_psy;

#if 0
#ifdef CONFIG_PM
static struct timespec suspend_time_before;
static struct timespec after;
static int suspend_resume_mark = 0;
#endif
#endif

struct cw_battery {
  struct i2c_client *client;

  struct workqueue_struct *cwfg_workqueue;
  struct delayed_work battery_delay_work;

  /*Jonah modify*/
  struct delayed_work dc_wakeup_work;
  //struct delayed_work

  struct power_supply cw_bat;

    int charger_mode;
    int capacity;
    int voltage;
    int status;
    int time_to_empty;
	int change;
    //int alt;
};

int g_cw2015_capacity = 0;
int g_cw2015_vol = 0;

static int i2c_master_reg8_send(const struct i2c_client *client,const char reg,const char *buf,int count,int scl_rate)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;
	char *tx_buf = (char *)kzalloc(count + 1,GFP_KERNEL);
	if(!tx_buf)
		return -ENOMEM;

	tx_buf[0] = reg;
	memcpy(tx_buf+1,buf,count);

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = count + 1;
	msg.buf = (char *)tx_buf;
	//msg.scl_rate = scl_rate;

	ret = i2c_transfer(adap, &msg,1);
	kfree(tx_buf);
	return (ret == 1) ? count : ret;
}

static int aml_cw2015_write(const struct i2c_client *client,const char reg,const char buf)
{
	int ret;
	char buff[2] = {};
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg[] = {
		{
			.addr = CW2015_ADDR,
			.flags = 0,
			.len = sizeof(buf),
			.buf = buff,
		}
	};

	printk("This is aml_cw2015_write buf: 0x%x\n",buf);
	buff[0] = reg;
	buff[1] = buf;

	ret = i2c_transfer(adap,msg,1);
	if(ret < 0){
		printk("%s: i2c transfer, reg:%d\n",__func__,ret);
		return ret;
	}
	return 0;

}

static int i2c_master_reg8_recv(const struct i2c_client *client,const char reg,char *buf,int count,int scl_rate)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msgs[2];
	int ret;
	char reg_buf = reg;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags;
	msgs[0].len = 1;
	msgs[0].buf = &reg_buf;
	//msgs[0].scl_rate = scl_rate;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].len = count;
	msgs[1].buf = (char *)buf;
	//msgs[1].scl_rate = scl_rate;

	ret = i2c_transfer(adap,msgs,2);

	return (ret== 2)? count : ret;
}

/*Define CW2015 iic read function*/
int cw_read(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret = 0;
	msleep(10);
	//ret = i2c_smbus_read_i2c_block_data( client, reg, 1, buf);
	ret = i2c_master_reg8_recv(client, reg, buf, 1, CW_I2C_SPEED);
	cw_printk("%2x = %2x\n", reg, buf[0]);
	return ret;
}
/*Define CW2015 iic write function*/
int cw_write(struct i2c_client *client, unsigned char reg, unsigned char const buf[])
{
	int ret = 0;
	msleep(10);
	//ret = i2c_smbus_write_i2c_block_data( client, reg, 1, &buf[0] );
	ret = i2c_master_reg8_send(client, reg, buf, 1, CW_I2C_SPEED);
	cw_printk("%2x = %2x\n", reg, buf[0]);
	return ret;
}

int cw_write_test(struct i2c_client *client, unsigned char reg,unsigned char const buf)
{
	int ret = 0;
	msleep(10);
	ret = aml_cw2015_write(client, reg,buf);
	printk("This is cw_write_test reg:0x%x, buf0x%x\n",reg,buf);
	//cw_printk("%2x = %2x\n",reg, buf[0]);
	return ret;
}
/*Define CW2015 iic read word function*/
int cw_read_word(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret = 0;
	msleep(10);
	ret = i2c_smbus_read_i2c_block_data( client, reg, 2, buf );
	cw_printk("%2x = %2x %2x\n", reg, buf[0], buf[1]);
	return ret;
}

/*CW2015 update profile function, Often called during initialization*/
int cw_update_config_info(struct cw_battery *cw_bat)
{
    int ret;
    unsigned char reg_val;
    int i;
    unsigned char reset_val;

    cw_printk("\n");
    cw_printk("[FGADC] test config_info = 0x%x\n",config_info[0]);


    // make sure no in sleep mode
    ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
    if(ret < 0) {
		printk("This is make sure no in sleep mode is reg_val:0x%x\n",reg_val);
        return ret;
    }

    reset_val = reg_val;
    if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        return -1;
    }

    // update new battery info
    for (i = 0; i < SIZE_BATINFO; i++) {
        ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info[i]);
        if(ret < 0)
			return ret;
    }

    reg_val |= CONFIG_UPDATE_FLG;   // set UPDATE_FLAG
    reg_val &= 0x07;                // clear ATHD
    reg_val |= ATHD;                // set ATHD
    ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
    if(ret < 0)
		return ret;
    // read back and check
    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if(ret < 0) {
        return ret;
    }

    if (!(reg_val & CONFIG_UPDATE_FLG)) {
		printk("Error: The new config set fail\n");
		//return -1;
    }

    if ((reg_val & 0xf8) != ATHD) {
		printk("Error: The new ATHD set fail\n");
		//return -1;
    }

    // reset
    reset_val &= ~(MODE_RESTART);
    reg_val = reset_val | MODE_RESTART;
    ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
    if(ret < 0) return ret;

    msleep(10);

    ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
    if(ret < 0) return ret;

	cw_printk("cw2015 update config success!\n");

    return 0;
}
/*CW2015 init function, Often called during initialization*/
static int cw_init(struct cw_battery *cw_bat)
{
    int ret;
    int i;
    unsigned char reg_val = MODE_SLEEP;

    printk("This is cw_init start\n");
    if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        reg_val = MODE_NORMAL;
	printk("This is cw_init write one reg_val:[0x%x]\n",reg_val);
        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        //ret = cw_write_test(cw_bat->client, REG_MODE, MODE_NORMAL);
        if (ret < 0){
		printk("This is cw_init cw_write is ret < 0 error\n");
		return ret;
		}
		ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
		printk("This is cw_init cw_read 1 reg_val=[0x%x]\n",reg_val);
		if(ret < 0){
			printk("This is cw_init cw_write for cw_read 1 is ret<0 error\n");
            return ret;
		}
    }

    printk("This is cw_init cw_read start one\n");
    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	printk("This is cw_init cw_read start reg_val:[0x%x]\n",reg_val);
    if (ret < 0){
	    printk("This is cw_init cw_read is ret < 0 error\n");
    	return ret;
	}

    if ((reg_val & 0xf8) != ATHD) {
        reg_val &= 0x07;    /* clear ATHD */
        reg_val |= ATHD;    /* set ATHD */
	printk("This is cw_init cw_write is two start\n");
        ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
        if (ret < 0){
		printk("This is cw_init cw_write is ret < 0 error\n");
            return ret;
		}
    }

    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0){
        return ret;
	}

    if (!(reg_val & CONFIG_UPDATE_FLG)) {
		cw_printk("update config flg is true, need update config\n");
        ret = cw_update_config_info(cw_bat);
        if (ret < 0) {
			printk("%s : update config fail\n", __func__);
            return ret;
        }
    } else {
    	for(i = 0; i < SIZE_BATINFO; i++) {
	        ret = cw_read(cw_bat->client, (REG_BATINFO + i), &reg_val);
	        if (ret < 0)
	        	return ret;

	        if (config_info[i] != reg_val)
	            break;
        }
        if (i != SIZE_BATINFO) {
			cw_printk("config didn't match, need update config\n");
        	ret = cw_update_config_info(cw_bat);
            if (ret < 0){
                return ret;
            }
        }
    }

    for (i = 0; i < 30; i++) {
        ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
        if (ret < 0)
            return ret;
        else if (reg_val <= 0x64)
            break;
        msleep(120);
    }

    if (i >= 30 ){
    	 reg_val = MODE_SLEEP;
         ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
         cw_printk("cw2015 input unvalid power error, cw2015 join sleep mode\n");
         return -1;
    }

	cw_printk("cw2015 init success!\n");
    return 0;
}

/*Functions:< check_chrg_usb_psy check_chrg_ac_psy get_chrg_psy get_charge_state > for Get Charger Status from outside*/
#if 0
static int check_chrg_usb_psy(struct device *dev, void *data)
{
        struct power_supply *psy = dev_get_drvdata(dev);

        if (psy->type == POWER_SUPPLY_TYPE_USB) {
                chrg_usb_psy = psy;
                return 1;
        }
        return 0;
}
#endif

static int check_chrg_ac_psy(struct device *dev, void *data)
{
        struct power_supply *psy = dev_get_drvdata(dev);

        if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
                chrg_ac_psy = psy;
                return 1;
        }
        return 0;
}

static void get_chrg_psy(void)
{
  #if 0
	if(!chrg_usb_psy)
		class_for_each_device(power_supply_class, NULL, NULL, check_chrg_usb_psy);
  #endif
	if(!chrg_ac_psy)
		class_for_each_device(power_supply_class, NULL, NULL, check_chrg_ac_psy);
}

static int get_charge_state(void)
{
        union power_supply_propval val;
        int ret = -ENODEV;
        //int usb_online = 0;
		int ac_online = 0;

#if 0
    if (!chrg_usb_psy || !chrg_ac_psy)
      get_chrg_psy();
#endif
    if(chrg_ac_psy){
      printk("This is at %s chrg_ac_psy is ok\n",__func__);
      get_chrg_psy();
    }

    #if 0
        if(chrg_usb_psy) {
            ret = chrg_usb_psy->get_property(chrg_usb_psy, POWER_SUPPLY_PROP_ONLINE, &val);
            if (!ret)
                usb_online = val.intval;
        }
        #endif
		if(chrg_ac_psy) {
            ret = chrg_ac_psy->get_property(chrg_ac_psy, POWER_SUPPLY_PROP_ONLINE, &val);
            if (!ret)
                ac_online = val.intval;
		}
    #if 0
		if(!chrg_usb_psy){
			cw_printk("Usb online didn't find\n");
		}
    #endif
		if(!chrg_ac_psy){
			cw_printk("Ac online didn't find\n");
		}
		//cw_printk("ac_online = %d    usb_online = %d\n", ac_online, usb_online);
		cw_printk("ac_online = %d  \n", ac_online);
		if(ac_online){
      printk("This at %s ac_online is ok return 1\n",__func__);
			return 1;
		}
        return 0;
}


static int cw_por(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reset_val;

	reset_val = MODE_SLEEP;
	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;
	reset_val = MODE_NORMAL;
	msleep(10);
	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;
	ret = cw_init(cw_bat);
	if (ret)
		return ret;
	return 0;
}

static int cw_get_capacity(struct cw_battery *cw_bat)
{
	int cw_capacity;
	int ret;
	unsigned char reg_val[2];

	static int reset_loop = 0;
	static int charging_loop = 0;
	static int discharging_loop = 0;
	static int jump_flag = 0;
	static int charging_5_loop = 0;
	//int sleep_cap = 0;

	ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
	if (ret < 0)
		return ret;

	cw_capacity = reg_val[0];

	if ((cw_capacity < 0) || (cw_capacity > 100)) {
		cw_printk("Error:  cw_capacity = %d\n", cw_capacity);
		reset_loop++;
		if (reset_loop > (BATTERY_CAPACITY_ERROR / queue_delayed_work_time)){
			cw_por(cw_bat);
			reset_loop =0;
		}

		return cw_bat->capacity; //cw_capacity Chaman change because I think customer didn't want to get error capacity.
	}else {
		reset_loop =0;
	}

	/* case 1 : aviod swing */
	if (((cw_bat->charger_mode > 0) && (cw_capacity <= (cw_bat->capacity - 1)) && (cw_capacity > (cw_bat->capacity - 9)))
					|| ((cw_bat->charger_mode == 0) && (cw_capacity == (cw_bat->capacity + 1)))) {
		if (!(cw_capacity == 0 && cw_bat->capacity <= 2)) {
			cw_capacity = cw_bat->capacity;
		}
	}

	/* case 2 : aviod no charge full */
	if ((cw_bat->charger_mode > 0) && (cw_capacity >= 95) && (cw_capacity <= cw_bat->capacity)) {
		cw_printk("Chaman join no charge full\n");
		charging_loop++;
		if (charging_loop > (BATTERY_UP_MAX_CHANGE / queue_delayed_work_time) ){
			cw_capacity = (cw_bat->capacity + 1) <= 100 ? (cw_bat->capacity + 1) : 100;
			charging_loop = 0;
			jump_flag = 1;
		}else{
			cw_capacity = cw_bat->capacity;
		}
	}

	/*case 3 : avoid battery level jump to CW_BAT */
	if ((cw_bat->charger_mode == 0) && (cw_capacity <= cw_bat->capacity ) && (cw_capacity >= 90) && (jump_flag == 1)) {
		cw_printk("Chaman join no charge full discharging\n");
#if 0
		#ifdef CONFIG_PM
		if(suspend_resume_mark == 1){
			suspend_resume_mark = 0;
			sleep_cap = (after.tv_sec + discharging_loop * (queue_delayed_work_time / 1000))/ (BATTERY_DOWN_MAX_CHANGE/1000) ;
			cw_printk("sleep_cap = %d\n", sleep_cap);

			if(cw_capacity >= cw_bat->capacity - sleep_cap) {
				return cw_capacity;
			}else{
				if(!sleep_cap)
					discharging_loop = discharging_loop + 1 + after.tv_sec / (queue_delayed_work_time/1000);
				else
					discharging_loop = 0;
				cw_printk("discharging_loop = %d\n", discharging_loop);
				return cw_bat->capacity - sleep_cap;
			}
		}
		#endif
#endif
		discharging_loop++;
		if (discharging_loop > (BATTERY_DOWN_MAX_CHANGE / queue_delayed_work_time) ){
			if (cw_capacity >= cw_bat->capacity - 1){
				jump_flag = 0;
			} else {
				cw_capacity = cw_bat->capacity - 1;
			}
			discharging_loop = 0;
		}else{
			cw_capacity = cw_bat->capacity;
		}
	}

	/*case 4 : avoid battery level is 0% when long time charging*/
	if((cw_bat->charger_mode > 0) &&(cw_capacity == 0))
	{
		charging_5_loop++;
		if (charging_5_loop > BATTERY_CHARGING_ZERO / queue_delayed_work_time) {
			cw_por(cw_bat);
			charging_5_loop = 0;
		}
	}else if(charging_5_loop != 0){
		charging_5_loop = 0;
	}
#if 0
	#ifdef CONFIG_PM
	if(suspend_resume_mark == 1)
		suspend_resume_mark = 0;
	#endif
#endif
	return cw_capacity;
}

/*This function called when get voltage from cw2015*/
static int cw_get_voltage(struct cw_battery *cw_bat)
{
    int ret;
    unsigned char reg_val[2];
    u16 value16, value16_1, value16_2, value16_3;
    int voltage;

    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if(ret < 0) {
        return ret;
    }
    value16 = (reg_val[0] << 8) + reg_val[1];

    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if(ret < 0) {
          return ret;
    }
    value16_1 = (reg_val[0] << 8) + reg_val[1];

    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if(ret < 0) {
        return ret;
    }
    value16_2 = (reg_val[0] << 8) + reg_val[1];

    if(value16 > value16_1) {
        value16_3 = value16;
        value16 = value16_1;
        value16_1 = value16_3;
    }

    if(value16_1 > value16_2) {
    value16_3 =value16_1;
    value16_1 =value16_2;
    value16_2 =value16_3;
    }

    if(value16 >value16_1) {
    value16_3 =value16;
    value16 =value16_1;
    value16_1 =value16_3;
    }

    voltage = value16_1 * 312 / 1024;

	if(DOUBLE_SERIES_BATTERY)
		voltage = voltage * 2;
    return voltage;
}

/*This function called when get RRT from cw2015*/
static int cw_get_time_to_empty(struct cw_battery *cw_bat)
{
    int ret;
    unsigned char reg_val;
    u16 value16;

    ret = cw_read(cw_bat->client, REG_RRT_ALERT, &reg_val);
    if (ret < 0)
            return ret;

    value16 = reg_val;

    ret = cw_read(cw_bat->client, REG_RRT_ALERT + 1, &reg_val);
    if (ret < 0)
            return ret;

    value16 = ((value16 << 8) + reg_val) & 0x1fff;
    return value16;
}
/*
int check_charging_state(const char *filename)
{
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	int read_size = 8;
	int state = 0;
	char buf[read_size];
	int ret;

	cw_printk("\n");
	fp = filp_open(filename, O_RDONLY, 0644);
	if (IS_ERR(fp))
		return -1;
	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	ret = vfs_read(fp, buf, read_size, &pos);
	if(ret < 0)
		return -1;

	filp_close(fp,NULL);
	set_fs(fs);

	state = buf[0] - '0';
	cw_printk(" filename = %s  state = %d \n", filename, state);
	return state;
}
*/ //Old function of get charger status

static void cw_update_charge_status(struct cw_battery *cw_bat)
{
/*
	int if_charging = 0;
	if(check_charging_state(USB_CHARGING_FILE) == 1
		|| check_charging_state(DC_CHARGING_FILE) == 1)
	{
		if_charging = CHARGING_ON;
	}else{
		if_charging = NO_CHARGING;
	}
	if(if_charging != cw_bat->charger_mode){
		cw_bat->charger_mode = if_charging;
	}
*/ //Old function of get charger status
	int cw_charger_mode;
	cw_charger_mode = get_charge_state();
	if(cw_bat->charger_mode != cw_charger_mode){
        cw_bat->charger_mode = cw_charger_mode;
		cw_bat->change = 1;
	}
}


static void cw_update_capacity(struct cw_battery *cw_bat)
{
    int cw_capacity;
    cw_capacity = cw_get_capacity(cw_bat);

    if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity)) {
        cw_bat->capacity = cw_capacity;
		cw_bat->change = 1;
    }
}



static void cw_update_vol(struct cw_battery *cw_bat)
{
    int ret;
    ret = cw_get_voltage(cw_bat);
    if ((ret >= 0) && (cw_bat->voltage != ret)) {
        cw_bat->voltage = ret;
		cw_bat->change = 1;
    }
}

static void cw_update_status(struct cw_battery *cw_bat)
{
    int status;

    if (cw_bat->charger_mode > 0) {
        if (cw_bat->capacity >= 100)
            status = POWER_SUPPLY_STATUS_FULL;
        else
            status = POWER_SUPPLY_STATUS_CHARGING;
    } else {
        status = POWER_SUPPLY_STATUS_DISCHARGING;
    }

    if (cw_bat->status != status) {
        cw_bat->status = status;
		cw_bat->change = 1;
    }
}

static void cw_update_time_to_empty(struct cw_battery *cw_bat)
{
    int ret;
    ret = cw_get_time_to_empty(cw_bat);
    if ((ret >= 0) && (cw_bat->time_to_empty != ret)) {
        cw_bat->time_to_empty = ret;
		cw_bat->change = 1;
    }
}


static void cw_bat_work(struct work_struct *work)
{
    struct delayed_work *delay_work;
    struct cw_battery *cw_bat;

    delay_work = container_of(work, struct delayed_work, work);
    cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);

	cw_update_capacity(cw_bat);
	cw_update_vol(cw_bat);
	cw_update_charge_status(cw_bat);
	cw_update_status(cw_bat);
	cw_update_time_to_empty(cw_bat);
	cw_printk("charger_mod = %d\n", cw_bat->charger_mode);
	cw_printk("status = %d\n", cw_bat->status);
	cw_printk("capacity = %d\n", cw_bat->capacity);
	cw_printk("voltage = %d\n", cw_bat->voltage);

#if 0
	#ifdef CONFIG_PM
	if(suspend_resume_mark == 1)
		suspend_resume_mark = 0;
	#endif
#endif

	#ifdef CW_PROPERTIES
	if (cw_bat->change == 1){
		power_supply_changed(&cw_bat->cw_bat);
		cw_bat->change = 0;
	}
	#endif
	g_cw2015_capacity = cw_bat->capacity;
    g_cw2015_vol = cw_bat->voltage;

	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(queue_delayed_work_time));
}

#ifdef CW_PROPERTIES
static int cw_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    int ret = 0;

    struct cw_battery *cw_bat;
    cw_bat = container_of(psy, struct cw_battery, cw_bat);

    switch (psp) {
    case POWER_SUPPLY_PROP_CAPACITY:
            val->intval = cw_bat->capacity;
            break;
	/*
    case POWER_SUPPLY_PROP_STATUS:   //Chaman charger ic will give a real value
            val->intval = cw_bat->status;
            break;
    */
    case POWER_SUPPLY_PROP_HEALTH:   //Chaman charger ic will give a real value
            val->intval= POWER_SUPPLY_HEALTH_GOOD;
            break;
    case POWER_SUPPLY_PROP_PRESENT:
            val->intval = cw_bat->voltage <= 0 ? 0 : 1;
            break;

    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            val->intval = cw_bat->voltage;
            break;

    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
            val->intval = cw_bat->time_to_empty;
            break;

    case POWER_SUPPLY_PROP_TECHNOLOGY:  //Chaman this value no need
            val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
            break;

    default:
            break;
    }
    return ret;
}

static enum power_supply_property cw_battery_properties[] = {
    POWER_SUPPLY_PROP_CAPACITY,
    //POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
    POWER_SUPPLY_PROP_TECHNOLOGY,
};
#endif

static int cw_bat_gpio_init(struct cw_battery *cw_bat)
{
  int ret;
 printk("This is cw_bat_gpio_init is start ok\n");
  if(gpio_is_valid(DC_DETECT_GPIO)){
  ret = gpio_request(DC_DETECT_GPIO,"cw_det_pin");
  if(ret){
    printk("This is failed to request dc_det_pin gpio\n");
    goto request_dc_det_pin_fail;
  }

  ret = gpio_direction_input(DC_DETECT_GPIO);
  if(ret){
    printk("This is failed to set dc_det_pin input\n");
    goto request_bat_low_pin_fail;
   }
  }
  return 0;

  //
 request_bat_low_pin_fail:
  if(gpio_is_valid(DC_DETECT_GPIO));
  gpio_free(DC_DETECT_GPIO);

 request_dc_det_pin_fail:
  return ret;

}
static void dc_detect_do_wakeup(struct work_struct *work)
{
  int ret;
  int irq;
  unsigned int type;

  struct delayed_work *delay_work;
  struct cw_battery *cw_bat;

  delay_work = container_of(work, struct delayed_work,work);
  cw_bat = container_of(delay_work,struct cw_battery, dc_wakeup_work);

  irq = gpio_to_irq(DC_DETECT_GPIO);
  type = gpio_get_value(DC_DETECT_GPIO) ? IRQ_TYPE_EDGE_FALLING : IRQ_TYPE_EDGE_RISING;
  ret = irq_set_irq_type(irq,type);
  if(ret < 0){
    printk("%s: irq_set_irq_type*%d, %d) failed\n",__func__,irq,type);

  }
  enable_irq(irq);
}

static irqreturn_t dc_detect_irq_handler(int irq, void *dev_id)
{
  struct cw_battery *cw_bat = dev_id;
  disable_irq_nosync(irq); //for irq debounce
  queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->dc_wakeup_work,msecs_to_jiffies(20));
  return IRQ_HANDLED;
}

static int cw2015_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    //int loop = 0;
    int irq;
    int irq_flags;
    int level = 0;
    unsigned int irqflags;
	struct cw_battery *cw_bat;
    //struct device *dev;
	cw_printk("\n");

    cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
    if (!cw_bat) {
		cw_printk("cw_bat create fail!\n");
        return -ENOMEM;
    }

    i2c_set_clientdata(client, cw_bat);

    cw_bat->client = client;
    cw_bat->capacity = 1;
    cw_bat->voltage = 0;
    cw_bat->status = 0;
	cw_bat->charger_mode = NO_CHARGING;
	cw_bat->change = 0;

  ret = cw_bat_gpio_init(cw_bat);
  printk("This is goto cw_bat_gpio_init is start over\n");
  if(ret){
    printk("cw_bat_gpio_init error\n");
    return ret;
  }

    ret = cw_init(cw_bat);
    //while ((loop++ < 200) && (ret != 0)) {
#if 0
    while ((loop++ < 4) && (ret != 0)) {
		msleep(200);
        ret = cw_init(cw_bat);
    }
#endif
    if (ret) {
		printk("%s : cw2015 init fail!\n", __func__);
        return ret;
    }

	#ifdef CW_PROPERTIES
    printk("This is cw_properites is ok\n");
	cw_bat->cw_bat.name = CW_PROPERTIES;
	cw_bat->cw_bat.type = POWER_SUPPLY_TYPE_BATTERY;
	cw_bat->cw_bat.properties = cw_battery_properties;
	cw_bat->cw_bat.num_properties = ARRAY_SIZE(cw_battery_properties);
	cw_bat->cw_bat.get_property = cw_battery_get_property;
	ret = power_supply_register(&client->dev, &cw_bat->cw_bat);
	if(ret < 0) {
	    power_supply_unregister(&cw_bat->cw_bat);
	    return ret;
	}
	#endif

	cw_bat->cwfg_workqueue = create_singlethread_workqueue("cwfg_gauge");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
  INIT_DELAYED_WORK(&cw_bat->dc_wakeup_work,dc_detect_do_wakeup);
	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work , msecs_to_jiffies(50));

  if(gpio_is_valid(DC_DETECT_GPIO)){
    ret = gpio_set_pullup(DC_DETECT_GPIO,1);
    if(ret < 0){
      printk("This is gpio_set_pullup is error\n");
      return ret;
    }
    irqflags = AML_GPIO_IRQ((DC_DETECT_GPIO - INT_GPIO_0),FILTER_NUM7,GPIO_IRQ_FALLING);
    irq = gpio_for_irq(DC_DETECT_GPIO,irqflags);
    //irq = gpio_to_irq(DC_DETECT_GPIO);
    printk("This is irq:%d\n",irq);
    level = gpio_get_value(DC_DETECT_GPIO);
    printk("This is level DC_DETECT_GPIO is %d\n",level);
    if(level == DC_DETECT_GPIO)
      {
        printk("This %s booting up with dc plug\n",__func__);
        cw_bat->status = POWER_SUPPLY_STATUS_CHARGING;
      }
    irq_flags = level ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
    //ret = request_irq(irq,dc_detect_irq_handler,irq_flags,"dc_detect",cw_bat);
    ret = request_irq(DC_DETECT_GPIO,dc_detect_irq_handler,IRQF_DISABLED,"dc_detect",cw_bat);
    if(ret < 0){
      printk("%s:request_irq(%d) failed\n",__func__,irq);
    }
    enable_irq_wake(irq);
  }

	cw_printk("cw2015 driver probe success!\n");
    return 0;
}

/*
static int cw2015_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	cw_printk("\n");
	strcpy(info->type, CWFG_NAME);
	return 0;
}
*/

#if 0
#ifdef CONFIG_PM
static int cw_bat_suspend(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct cw_battery *cw_bat = i2c_get_clientdata(client);
		read_persistent_clock(&suspend_time_before);
        cancel_delayed_work(&cw_bat->battery_delay_work);
        return 0;
}

static int cw_bat_resume(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct cw_battery *cw_bat = i2c_get_clientdata(client);
		suspend_resume_mark = 1;
		read_persistent_clock(&after);
		after = timespec_sub(after, suspend_time_before);
        queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(2));
        return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
        .suspend  = cw_bat_suspend,
        .resume   = cw_bat_resume,
};
#endif
#endif

static int cw2015_remove(struct i2c_client *client)
{
	cw_printk("\n");
	return 0;
}

static const struct i2c_device_id cw2015_id_table[] = {
	{CWFG_NAME, 0},
	{}
};

static struct i2c_driver cw2015_driver = {
	.driver 	  = {
		.name = CWFG_NAME,
#if 0
#ifdef CONFIG_PM
        .pm     = &cw_bat_pm_ops,
#endif
#endif
		.owner	= THIS_MODULE,
	},
	.probe		  = cw2015_probe,
	.remove 	  = cw2015_remove,
	//.detect 	  = cw2015_detect,
	.id_table = cw2015_id_table,
};


static struct i2c_board_info __initdata fgadc_dev = {
	I2C_BOARD_INFO(CWFG_NAME, 0x62)
};

static int __init cw215_init(void)
{
	struct i2c_client *client;
	struct i2c_adapter *i2c_adp;
	cw_printk("\n");

    //i2c_register_board_info(CWFG_I2C_BUSNUM, &fgadc_dev, 1);
	i2c_adp = i2c_get_adapter(CWFG_I2C_BUSNUM);
	client = i2c_new_device(i2c_adp, &fgadc_dev);

    i2c_add_driver(&cw2015_driver);
    return 0;
}


static void __exit cw215_exit(void)
{
    i2c_del_driver(&cw2015_driver);
}

module_init(cw215_init);
module_exit(cw215_exit);

MODULE_AUTHOR("Chaman Qi");
MODULE_DESCRIPTION("CW2015 FGADC Device Driver V3.0");
MODULE_LICENSE("GPL");
