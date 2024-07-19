#include "cw2015_battery.h"


#if defined(CONFIG_PRODUCT_X1000_Q1) || defined(CONFIG_PRODUCT_X1000_M2X) || defined(CONFIG_PRODUCT_X1000_M5S) //将电量计96%变成现有的100%进行处理
#define UI_FULL 96
#define DECIMAL_MAX 70
#define DECIMAL_MIN 30
#elif defined(CONFIG_PRODUCT_X1000_ECMINI)  //将电量计98%变成100%进行处理
#define UI_FULL 98
#define DECIMAL_MAX 70
#define DECIMAL_MIN 30
#endif


//****************添加电量显示*****************
static ssize_t
cw2015battery_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf,"%hu",cw2015_battery->capacity);
}

static ssize_t
cw2015battery_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t n)
{
        return n;
}

static ssize_t
cw2015_voltage_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf,"%hu",cw2015_battery->voltage);
}

static ssize_t
cw2015_voltage_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t n)
{
        return n;
}


static DEVICE_ATTR(cw2015_battery, 0644, cw2015battery_show, cw2015battery_store);
static DEVICE_ATTR(cw2015_voltage, 0644, cw2015_voltage_show, cw2015_voltage_store);

//*********************************************
static int cw_read(struct i2c_client *client, u8 reg)
{
	int ret = -1;
	unsigned char data[4]={0,0,0,0};
	int len = 1;
	
	ret = i2c_master_send(client, &reg, 1);
	if (ret < 1) {
		printk("%s 0x%02x err\n", __func__, reg);
		return -1;
	}
	ret = i2c_master_recv(client, data, len);
	if (ret < 0)	printk("%s 0x%02x err\n", __func__, reg);	
	return data[0];
}

static int cw_write(struct i2c_client *client, u8 reg, u8 buffer)
{
	unsigned char buf[5] = {0};
    int ret = -1;
	int len = 1;
    buf[0] = reg;
    buf[1] = buffer;
    ret = i2c_master_send(client, buf,2);	
	if (ret < len+1)	printk("%s 0x%02x err\n", __func__, reg);
	
    return ret < len+1 ? ret : 0;
}

#if defined(CONFIG_PRODUCT_X1000_Q1) || defined(CONFIG_PRODUCT_X1000_M2X) || defined(CONFIG_PRODUCT_X1000_M5S) || defined(CONFIG_PRODUCT_X1000_ECMINI)
static int cw_read_word(struct i2c_client *client, u8 reg, u8 buf[])
{
  	int ret = -1;
	int len = 2;
	
	ret = i2c_master_send(client, &reg, 1);
	if (ret < 1) {
		printk("%s 0x%02x err\n", __func__, reg);
		return -1;
	}
	ret = i2c_master_recv(client, buf, len);
	if (ret < 0)	printk("%s 0x%02x err\n", __func__, reg);	
	return ret;
}
#else
static int cw_read_word(struct i2c_client *client, u8 reg)
{
  	int ret = -1;
	unsigned char data[4]={0,0,0,0};
	int len = 2;
	
	ret = i2c_master_send(client, &reg, 1);
	if (ret < 1) {
		printk("%s 0x%02x err\n", __func__, reg);
		return -1;
	}
	ret = i2c_master_recv(client, data, len);
	if (ret < 0)	printk("%s 0x%02x err\n", __func__, reg);	
	return ((data[0]<<8)|data[1]);
}
#endif


static int cw_update_config_info(struct cw_battery *cw_bat)
{
    int ret;
    u8 reg_val;
    int i;
    u8 reset_val;

    /* make sure no in sleep mode */
    reg_val = cw_read(cw_bat->client, REG_MODE);
    if (reg_val < 0)	return -1;

    if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        dev_err(&cw_bat->client->dev, "Error, device in sleep mode, cannot update battery info\n");
        return -2;
    }

    /* update new battery info */
    for (i = 0; i < SIZE_BATINFO; i++) {
		reg_val = cw_bat_config_info[i];
        ret = cw_write(cw_bat->client, REG_BATINFO + i,reg_val);
        if (ret < 0)	return -1;
    }

    /* readback & check */
    for (i = 0; i < SIZE_BATINFO; i++) {
        reg_val = cw_read(cw_bat->client, REG_BATINFO + i);
        if (reg_val != cw_bat_config_info[i])
            return -3;
    }

    /* set cw2015/cw2013 to use new battery info */
    reg_val = cw_read(cw_bat->client, REG_CONFIG);
    if (reg_val < 0)	return -1;

    reg_val |= CONFIG_UPDATE_FLG;   /* set UPDATE_FLAG */
    reg_val &= 0x07;        /* clear ATHD */
    reg_val |= ATHD;        /* set ATHD */
    ret = cw_write(cw_bat->client, REG_CONFIG, reg_val);
    if (ret < 0)	return -1;

	//reset
	reset_val = MODE_NORMAL;
	reg_val = MODE_RESTART;
	ret = cw_write(cw_bat->client, REG_MODE, reg_val);
	if (ret < 0)	return -1;
    msleep(10);
    ret = cw_write(cw_bat->client, REG_MODE, reset_val);
    if (ret < 0)	return -1;
    return 0;
}




static int cw_init(struct cw_battery *cw_bat)
{
	int ret;
    int i;
	u8 reg_val = MODE_NORMAL;
	//wak up cw2015 from sleep mode
	ret = cw_write(cw_bat->client, REG_MODE, reg_val);
	if(ret < 0)		return -1;
	
	//check ATHD if not right	
	reg_val = cw_read(cw_bat->client,REG_CONFIG);
	if(reg_val < 0)		return -1;
	if ((reg_val & 0xf8) != ATHD)
	{
		reg_val &= 0x07;    /* clear ATHD */
		reg_val |= ATHD;    /* set ATHD */
		ret = cw_write(cw_bat->client, REG_CONFIG, reg_val);
		if(ret < 0)		return -1;
	}

	/* check config_update_flag if not right */
	reg_val = cw_read(cw_bat->client, REG_CONFIG);
	if(reg_val < 0)		return -1;
	if (!(reg_val & CONFIG_UPDATE_FLG))
	{
		ret = cw_update_config_info(cw_bat);;
		if(ret < 0)		return -2;
	}
	else
	{
		for(i = 0; i < SIZE_BATINFO; i++) 
		{
        	reg_val = cw_read(cw_bat->client, (REG_BATINFO + i));
            if(reg_val < 0)		return -1;

            if (cw_bat_config_info[i] != reg_val)
                break;
        }

        if (i != SIZE_BATINFO) 
		{
            ret = cw_update_config_info(cw_bat);
            if(ret < 0)		return -2;
        }
	}

	 for (i = 0; i < 30; i++) 
	 {
        reg_val = cw_read(cw_bat->client, REG_SOC);
       	if(reg_val < 0)		return -1;
        else if (reg_val <= 0x64)
            	break;

        msleep(100);
        if (i > 25)
          	dev_err(&cw_bat->client->dev, "cw2015/cw2013 input unvalid power error\n");

    }
    if (i >= 30){
		reg_val = MODE_SLEEP;
		ret = cw_write(cw_bat->client, REG_MODE, reg_val);
		if(ret < 0)		return -1;
		dev_info(&cw_bat->client->dev, "report battery capacity error");
		return -3;
    }
	return 0;
}

#if defined(CONFIG_PRODUCT_X1000_Q1) || defined(CONFIG_PRODUCT_X1000_M2X) || defined(CONFIG_PRODUCT_X1000_M5S) || defined(CONFIG_PRODUCT_X1000_ECMINI)
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
	int ui_100 = UI_FULL;
	int remainder = 0;
	int real_SOC = 0;
	int digit_SOC = 0;
	int UI_SOC = 0;
	//int cw_capacity;
	int ret;
	unsigned char reg_val[2];
	//unsigned char reg_0x4f;
	//unsigned char temp_value;
	static int reset_loop = 0;
	static int charging_zero_loop = 0;

	
	ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
	if (ret < 0)
		return ret;

	real_SOC = reg_val[0];
	digit_SOC = reg_val[1];
	//cyadd
	printk(KERN_INFO "CW2015[%d]: real_SOC = %d, digit_SOC = %d\n", __LINE__, real_SOC, digit_SOC);
	
	/*case 1 : avoid IC error, read SOC > 100*/
	if ((real_SOC < 0) || (real_SOC > 100)) {
		printk("Error:  real_SOC = %d\n", real_SOC);
		reset_loop++;			
		if (reset_loop > (BATTERY_CAPACITY_ERROR / 5)){ 
			cw_por(cw_bat);
			reset_loop =0;							 
		}
								 
		return cw_bat->capacity; //cw_capacity Chaman change because I think customer didn't want to get error capacity.
	}else {
		reset_loop =0;
	}

	/*case 2 : avoid IC error, battery SOC is 0% when long time charging*/
	if((cw_bat->usb_online > 0) &&(real_SOC == 0))
	{
		charging_zero_loop++;
		if (charging_zero_loop > BATTERY_CHARGING_ZERO / 5) {
			cw_por(cw_bat);
			charging_zero_loop = 0;
		}
	}else if(charging_zero_loop != 0){
		charging_zero_loop = 0;
	}

	
	UI_SOC = ((real_SOC * 256 + digit_SOC) * 100)/ (ui_100*256);
	remainder = (((real_SOC * 256 + digit_SOC) * 100 * 100) / (ui_100*256)) % 100;
	//cyadd
	printk(KERN_INFO "CW2015[%d]: ui_100 = %d, UI_SOC = %d, remainder = %d===online[%d]==voltage[%d]=====\n", __LINE__, ui_100, UI_SOC, remainder,cw_bat->usb_online,cw_bat->voltage);
	/*case 3 : aviod swing*/
	if(UI_SOC >= 100){
		//cyadd
		//printk(KERN_INFO "CW2015[%d]: UI_SOC = %d larger 100!!!!\n", __LINE__, UI_SOC);
		UI_SOC = 100;
	}else{
		if((remainder > 70 || remainder < 30) && UI_SOC >= cw_bat->capacity - 1 && UI_SOC <= cw_bat->capacity + 1)
		{
			UI_SOC = cw_bat->capacity;
			//cyadd
			//printk(KERN_INFO "CW2015[%d]: UI_SOC = %d, cw_bat->capacity = %d\n", __LINE__, UI_SOC, cw_bat->capacity);
		}
	}

	return UI_SOC;
}


static void jz_update_capacity(struct cw_battery *cw_bat)
{
	int cw_capacity = cw_get_capacity(cw_bat);
	if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity))
	{
		cw_bat->capacity = cw_capacity;
	}
	//printk("=====cw_capacity[%d]===usb[%d]===\n",cw_bat->capacity,cw_bat->usb_online);
}

unsigned int cw_get_vol(struct cw_battery *cw_bat)
{
	int ret = 0;
	unsigned char get_ad_times = 0;
	unsigned long ad_value = 0;
	unsigned char reg_val[2];
	unsigned int ad_buff = 0;
	unsigned int ad_value_min = 0;
	unsigned int ad_value_max = 0;
	for(get_ad_times = 0; get_ad_times < 3; get_ad_times++)
	{
		ret = cw_read_word(cw_bat->client,REG_VCELL, reg_val);
		if(ret < 0)		return 1;

		ad_buff = (reg_val[0] << 8) + reg_val[1];
		if(get_ad_times == 0)
		{
			ad_value_min = ad_buff;
			ad_value_max = ad_buff;
		}
		if(ad_buff < ad_value_min)
		{
			ad_value_min = ad_buff;
		}
		if(ad_buff > ad_value_max)
		{
			ad_value_max = ad_buff;
		}
		ad_value += ad_buff;
	}
	ad_value -= ad_value_min;
	ad_value -= ad_value_max;
	ad_value = ad_value  * 305 / 1000;
	return(ad_value);       //14λADC????
}
#else
int cw_get_capacity(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reset_val = 0;
	static int reset_loop = 0;
	static int allow_no_charger_full =0;
	static int no_charger_full_jump =0;
	static int allow_charger_always_zero =0;
	static int if_quickstart =0;
	u8 reg_val;
	int cw_capacity;
	int allow_capacity;
	
	 reg_val = cw_read(cw_bat->client, REG_SOC);
	 if(reg_val < 0)	return reg_val;

	 cw_capacity = reg_val;
	 if((cw_capacity < 0 ) || (cw_capacity > 100))
	 {
	 	reset_loop++;
		if(reset_loop > 5)
		{
			reset_val = MODE_SLEEP;
			ret = cw_write(cw_bat->client, REG_MODE, reset_val);
			if(ret < 0)	return ret;
			msleep(10);

			reset_val = MODE_NORMAL;
			ret = cw_write(cw_bat->client, REG_MODE,reset_val);
			if(ret < 0)	return ret;

		    ret = cw_update_config_info(cw_bat);
			if (ret < 0)	return ret; 
			reset_loop = 0;
		}
	 }
	 else
	 {
	 	reset_loop = 0;
	 }
	 //if usb link,capacity decrease or if usb not link capacity increase
	if(((cw_bat->usb_online == 1) && (cw_capacity == (cw_bat->capacity - 1))) || ((cw_bat->usb_online == 0) && (cw_capacity == (cw_bat->capacity + 1))))
	{
		if(!((cw_capacity == 0 && cw_bat->capacity <= 2)||(cw_capacity == 100 && cw_bat->capacity == 99)))
		{
			cw_capacity = cw_bat->capacity;
		}
	}

	//if usb link and capactiy >= 95 
	if((cw_bat->usb_online == 1) && (cw_capacity >= 95) && (cw_capacity <= cw_bat->capacity) )
	{     
		// avoid not charge full
		allow_no_charger_full++;
		if(allow_no_charger_full >= BATTERY_UP_MAX_CHANGE)
		{
			allow_capacity = cw_bat->capacity + 1; 
			cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
			no_charger_full_jump =1;
			allow_no_charger_full =0;
		}
		else if(cw_capacity <= cw_bat->capacity)
		{
			cw_capacity = cw_bat->capacity; 
		}
	}
	else if((cw_bat->usb_online == 0) && (cw_capacity <= cw_bat->capacity ) && (cw_capacity >= 90) && (no_charger_full_jump == 1))
	{
		// avoid battery level jump to CW_BAT
		if(cw_bat->usb_online == 0) 
		   allow_no_charger_full++;
		if(allow_no_charger_full >= BATTERY_DOWN_MIN_CHANGE)
		{
			allow_capacity = cw_bat->capacity - 1;
			allow_no_charger_full =0; 
			if (cw_capacity >= allow_capacity)
			{
				no_charger_full_jump =0;
			}
			else
			{
				cw_capacity = (allow_capacity > 0) ? allow_capacity : 0;
			}
		}
		else if(cw_capacity <= cw_bat->capacity)
		{
			cw_capacity = cw_bat->capacity;
		}
	}
	else
    {
  		allow_no_charger_full =0;
    }

	if((cw_bat->usb_online > 0) && (cw_capacity == 0))
	{		  
		allow_charger_always_zero++;
		if((allow_charger_always_zero >= BATTERY_DOWN_MIN_CHANGE_SLEEP) && (if_quickstart == 0))
		{
			reset_val = MODE_SLEEP;
			ret = cw_write(cw_bat->client, REG_MODE, reset_val);
			if(ret < 0)	return ret;
			msleep(10);

			reset_val = MODE_NORMAL;
			ret = cw_write(cw_bat->client, REG_MODE,reset_val);
			if(ret < 0)	return ret;

		    ret = cw_update_config_info(cw_bat);
			if (ret < 0)	return ret; 

			if_quickstart = 1;
			allow_charger_always_zero =0;
		}
	}
	else if((if_quickstart == 1)&&(cw_bat->usb_online == 0))
	{
		if_quickstart = 0;
	}
	 
	 return cw_capacity;
}


static void jz_update_capacity(struct cw_battery *cw_bat)
{
	int cw_capacity = cw_get_capacity(cw_bat);
	if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity))
	{
		cw_bat->capacity = cw_capacity;
	}
	//printk("=====cw_capacity[%d]===usb[%d]===\n",cw_bat->capacity,cw_bat->usb_online);
}



unsigned int cw_get_vol(struct cw_battery *cw_bat)
{
	int ret = 0;
	unsigned char get_ad_times = 0;
	unsigned long ad_value = 0;
	unsigned int ad_buff = 0;
	unsigned int ad_value_min = 0;
	unsigned int ad_value_max = 0;
	for(get_ad_times = 0; get_ad_times < 3; get_ad_times++)
	{
		ad_buff = cw_read_word(cw_bat->client,REG_VCELL);
		if(ad_buff < 0)		return 1;

		if(get_ad_times == 0)
		{
			ad_value_min = ad_buff;
			ad_value_max = ad_buff;
		}
		if(ad_buff < ad_value_min)
		{
			ad_value_min = ad_buff;
		}
		if(ad_buff > ad_value_max)
		{
			ad_value_max = ad_buff;
		}
		ad_value += ad_buff;
	}
	ad_value -= ad_value_min;
	ad_value -= ad_value_max;
	ad_value = ad_value  * 305 / 1000;
	return(ad_value);       //14λADC????
}
#endif

static void jz_update_vol(struct cw_battery *cw_bat)
{
	unsigned int cw_voltage;
	cw_voltage = cw_get_vol(cw_bat);
	if(cw_voltage == 1)
	{	//read voltage error
		cw_bat->voltage = cw_bat->voltage;
	}
	else if(cw_bat->voltage != cw_voltage)
	{
		cw_bat->voltage = cw_voltage;
	}
	//printk("======voltage[%d]===\n",cw_bat->voltage);
}

int cw_get_time_to_empty(struct cw_battery *cw_bat)
{
        signed char ret;
        unsigned char reg_val;
        unsigned int value16;

        reg_val = cw_read(cw_bat->client,REG_RRT_ALERT);
        if (reg_val < 0)	 return -1;
        value16 = (unsigned int)reg_val;

        reg_val = cw_read(cw_bat->client,REG_RRT_ALERT + 1);
        if (reg_val < 0)	 return -1;
        value16 = ((value16 << 8) + reg_val) & 0x1fff;
       
        return value16;
}



static void jz_update_time_to_empty(struct cw_battery *cw_bat)
{
	unsigned int rrt;
	rrt = (unsigned int)cw_get_time_to_empty(cw_bat);
	if((rrt >= 0) && (cw_bat->time_to_empty != rrt))
	{
		cw_bat->time_to_empty = rrt;
	}
}

#ifdef CONFIG_PRODUCT_X1000_ECMINI
extern int bq25890_get_chargestatus();
#endif

static void cw_bat_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
    struct cw_battery *cw_bat;
	delay_work = container_of(work, struct delayed_work, work);
    cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);
#ifdef CONFIG_PRODUCT_X1000_ECMINI
	if(bq25890_get_chargestatus() > 0)						cw_bat->usb_online = 1;
	else										cw_bat->usb_online = 0;
#else
	if(__gpio_get_value(GPIO_PA(17)) == 1)	cw_bat->usb_online = 1;
	else										cw_bat->usb_online = 0;
#endif	
	jz_update_capacity(cw_bat);
	jz_update_vol(cw_bat);
	//jz_update_time_to_empty(cw_bat);
	queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(5000));
}

#if defined(CONFIG_PRODUCT_X1000_Q1) || defined(CONFIG_PRODUCT_X1000_M2X) || defined(CONFIG_PRODUCT_X1000_M5S) || defined(CONFIG_PRODUCT_X1000_ECMINI)
static int cw_init_data(struct cw_battery *cw_bat)
{
	unsigned char reg_SOC[2];
	int real_SOC = 0;
	int digit_SOC = 0;
	int ret;
	int UI_SOC = 0;

	ret = cw_read_word(cw_bat->client, REG_SOC, reg_SOC);
    if (ret < 0)
    	return ret;

	real_SOC = reg_SOC[0];
	digit_SOC = reg_SOC[1];
	UI_SOC = ((real_SOC * 256 + digit_SOC) * 100)/ (UI_FULL*256);
	printk("[%d]: real_SOC = %d, digit_SOC = %d\n", __LINE__, real_SOC, digit_SOC);

	jz_update_vol(cw_bat);
	cw_bat->capacity = UI_SOC;
	return 0;
}
#endif


static int cw_bat_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cw_battery *cw_bat;
	int ret;
	int loop = 0;
	int max_value = 200;
#ifdef CONFIG_PRODUCT_X1000_ECMINI
	max_value = 5;
#endif
	cw_bat = kzalloc(sizeof(struct cw_battery), GFP_KERNEL);
	if (!cw_bat) {
		  printk(&cw_bat->client->dev, "fail to allocate memory\n");
		  return -ENOMEM;
	 }
	cw2015_battery = cw_bat;
	i2c_set_clientdata(client, cw_bat);

	cw_bat->client = client;
	
	ret = cw_init(cw_bat);
	 while ((loop++ < max_value) && (ret != 0)) {
      ret = cw_init(cw_bat);
    }
	if(ret)		return ret;


	cw_bat->usb_online = 0;
	cw_bat->time_to_empty = 0;
	#if defined(CONFIG_PRODUCT_X1000_Q1) || defined(CONFIG_PRODUCT_X1000_M2X) || defined(CONFIG_PRODUCT_X1000_M5S)

		cw_init_data(cw_bat);
	#else
		cw_bat->capacity =2 ;
		cw_bat->voltage = 0;	
	#endif

	cw_bat->battery_workqueue = create_singlethread_workqueue("cw2015_battery");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
	
	queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(1000));

	ret = device_create_file(&client->dev, &dev_attr_cw2015_battery);
	if(ret != 0)
	{
		dev_err(&client->dev,  "0Failed to create xxx sysfs files: %d\n",ret);
	}

	ret = device_create_file(&client->dev, &dev_attr_cw2015_voltage);
	if(ret != 0)
	{
		dev_err(&client->dev,  "1Failed to create xxx sysfs files: %d\n",ret);
	}
	return 0;
}

static int cw_bat_remove(struct i2c_client *client)
{
	return 0;
}


#ifdef CONFIG_PM
static int cw_bat_suspend(struct device *dev)
{

    return 0;
}

static int cw_bat_resume(struct device *dev)
{
    return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
    .suspend  = cw_bat_suspend,
    .resume   = cw_bat_resume,
};
#endif




static const struct i2c_device_id cw_id[] = {
	{ "cw2015", 0 },
};

static struct i2c_driver cw_bat_driver = {
    .driver     = {
        .name   = "cw2015",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
       .pm     = &cw_bat_pm_ops,
#endif
    },
   	.probe      = cw_bat_probe,
    .remove     = cw_bat_remove,
	.id_table	= cw_id,
};



static int __init cw_bat_init(void)
{
	int ret = i2c_add_driver(&cw_bat_driver);
	return ret;
}

static void __exit cw_bat_exit(void)
{
	i2c_del_driver(&cw_bat_driver);
}


fs_initcall(cw_bat_init);
module_exit(cw_bat_exit);
MODULE_LICENSE("GPL");

