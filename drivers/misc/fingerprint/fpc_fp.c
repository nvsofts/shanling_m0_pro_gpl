#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/gpio.h>

#include <linux/freezer.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>

#include <linux/spi/fpc_pdata.h>
#include "fpc_fp.h"


#define FPC_IOC_MAGIC                   'F'
#define READ_IMAGE                      _IOR(FPC_IOC_MAGIC, 0, int)
#define HW_RESET                        _IOW(FPC_IOC_MAGIC, 1, int)
#define ENCRYPT_IC_RST                  _IOW(FPC_IOC_MAGIC, 2, int)


struct finger_print_dev {
    dev_t idd;
    int major;
    int minor;
    struct cdev chd;
    struct class *cls;
    struct device *dev;
};

struct fpc_fp_data {
    int reset_pin;
    int int_pin;
    int cs_pin;
    int encrypt_ic_rst_pin;
    int irq;
    int is_irq;
    unsigned char*      image_buf;
    struct mutex        dev_lock;
    struct mutex        ioctl_lock;
    struct spi_message  msg;
    struct spi_transfer xfer;
    wait_queue_head_t   waitqueue;
    struct work_struct  work;
    struct wake_lock    process_wakelock;
    struct workqueue_struct *workqueue;
    struct spi_device       *spi;
    struct finger_print_dev fp_dev;
    struct fpc_platform_data* pdata;
};

#define SPI_SPEED                       (18*1000000)


#define SENSOR_IMGH                     (192)
#define SENSOR_IMGW                     (192)
#define SENSOR_IMAG_SIZE                (SENSOR_IMGH*SENSOR_IMGW)


  /**** Sensor 生成图像数据选择 ***/
#define  SWITCH_MODE_NORMAL             1
#define  SWITCH_MODE_TEST_0             2   //输出自测图，固定值
#define  SWITCH_MODE_TEST_1             3   //输出自测图，递增值


  /**** Sensor 输出图像数据方式 ***/
#define SWITCH_CAPTURE_MODE_CONTIN      0x01
#define SWITCH_CAPTURE_MODE_INT_ONE     0x02
#define SWITCH_CAPTURE_MODE_INT_TWO     0x03

  /**** 自测产生图像数据检查时，图像数据变化模式 ***/
#define TEST_IMAGE_FIX_VLAUE            1
#define TEST_IMAGE_INC_VLAUE            2


  /**** 读图方式 连续读图和按块读图 ***/
#define READ_IMAGE_MODE_BUNCK           0   //块方式读图
#define READ_IMAGE_MODE_CONTIN          1   //连续方式读图


  /**** 中断引脚输出功能  ***/
#define INT_OUT_BEZAL                   0x00
#define INT_OUT_VERT_CTL5               0x01
#define INT_OUT_VERT_CTL6               0x02
#define INT_OUT_VERT_CTL7               0x03
#define INT_OUT_ARRAY_CLK               0x04
#define INT_OUT_ADC_CLK                 0x05
#define INT_OUT_SAMPLE_CLK              0x06
#define INT_OUT_ARRAY_EN                0x07
#define INT_OUT_SAMPLE_EN               0x08
#define INT_OUT_rstn_soft               0x09
#define INT_OUT_rstn_spi_clk            0x0a
#define INT_OUT_CLK                     0x0b
#define INT_OUT_RD_PIX                  0x0c
#define INT_OUT_RD_PIX_STATUS           0x0d
#define INT_OUT_NULL1                   0x0e
#define INT_OUT_NULL2                   0x0f

#define RD_INT_ENABLE                   0x01
#define RD_INT_DISABLE                  0x00

#define SHAGC_BIT_SWITCH                0x01

#define RESETDELAYTIMES                 1000  //复位过程中延时时间 宏定义

#define START_POST_X_H_BIT              0
#define START_POST_Y_H_BIT              1
#define START_ADDR_X_L_REG              2
#define START_ADDR_Y_L_REG              3

#define  END_POST_X_H_BIT               4
#define  END_POST_Y_H_BIT               5

#define END_ADDR_X_L_REG                6
#define END_ADDR_Y_L_REG                7
#define START_X_INTERVAL_REG            8
#define START_Y_INTERVAL_REG            9

#define RD_PIX_DATA_DLY_BIT             10
#define RD_PIX_TEST_MODE_BIT            11
#define IS_TEST_RD_PIX_BIT              12
#define IS_BINARY_BIT                   13
#define DIG_TEST_IMGA_RANG_REG          14

#define P_CNT0_RD_CLK_GEN_EN_BIT        15
#define P_CNT0_RD_CLK_GEN_BIT           16
#define RD_SEL_XY_BIT                   17
#define IS_BLOCK_RD_MODE_BIT            18
#define RD_PIX_SPI_READ_START_BIT       19

#define INT_SEL_BIT                     20
#define RD_PIX_INT_CLR_BIT              21
#define RD_PIX_INT_FORCE_DATA_BIT       22
#define RD_PIX_INT_FORCE_EN_BIT         23
#define RD_PIX_INT_ENABLE_BIT           24

#define OFFSET_EN_ADC_BIT               25
#define SEL_CHN_BIT                     26
#define OUT_F_SEL_BIT                   27
#define CHANL_ONE_PGAC_AFE_BIT          28

#define PGAC_ADC_BIT                    29
#define SEL_TEST_BIT                    30
#define LBF_BIT                         31

#define SEL_CHARGE_PHASE_BIT            32
#define LBF_DAC_BIT                     33
#define SEL_BIAS1_BIT                   34

#define CLK_SKEW_BIT                    35
#define SEL_BIAS2_BIT                   36
#define SEL_BIAS3_BIT                   37

#define ISEL_ADC_BIT                    38
#define SEL_CHARGE_BIT                  39

#define ISEL_AFE_BIT                    40
#define SEL_PGA_DAC_BIT                 41

#define RANGE_ADC_BIT                   42
#define SEL_ADC_OFFSET                  43

#define SHAGC_L_BIT                     44
#define SEL_CDS_DAC_BIT                 45

#define BEZEL_FORCE_CFG_BIT             46
#define BEZEL_OEN_BIT                   47
#define SHAGC_H_BIT                     48
#define CDS_GAIN_BIT                    49

#define CHIP_ID_REG                     50
#define SEL_BIAS4_BIT                   51

#define RD_PIX_STATUS_OUT_INT_BIT       52
#define RD_PIX_STATUS_OUT_BIT           53
#define RD_PIX_INT_OUT_BIT              54

#define SOFT_RESET_BIT                  55
#define CHIP_IMAGE_DATA_REG             56



static unsigned char fpc_register_default[][2] = {
    { 0x13, 0x40},
    { 0x01, 0x00},
    { 0x02, 0x00},
    { 0x04, 0x00},
    { 0x05, 0xBf},
    { 0x06, 0xBf},
    { 0x08, 0x87},
    { 0x09, 0xc1},
    //  { 0x81, 0xd1},

    // DK7-50参数
    { 0x83, 0xe7},
    { 0x84, 0x02},
    { 0x85, 0x14},
    { 0x86, 0x52},
    { 0x87, 0x22},
    { 0x88, 0x1B},
    { 0x89, 0x63},
    { 0x8a, 0xB0},
    { 0x8b, 0x00},
    { 0x8d, 0x03},
};

const unsigned char fpc_register_map[] = {
    /**** REG, mask, shift  ****/
    0x00, 0x01, 0,  // 0 START_POST_X_H_BIT[0]
    0x00, 0x01, 4,  // 1 START_POST_Y_H_BIT[4]
    0x01, 0xFF, 0,  // 2 START_ADDR_X_L_REG[7:0]
    0x02, 0xFF, 0,  // 3 START_ADDR_Y_L_REG[7:0]

    0x04, 0x01, 0,  // 4 END_POST_X_H_BIT[0]
    0x04, 0x01, 4,  // 5 END_POST_Y_H_BIT[4]

    0x05, 0xFF, 0,  // 6 END_ADDR_X_L_REG[7:0]
    0x06, 0xFF, 0,  // 7 END_ADDR_Y_L_REG[7:0]
    0x03, 0xFF, 0,  // 8 START_X_INTERVAL_REG[7:0]
    0x07, 0xFF, 0,  // 9 START_Y_INTERVAL_REG[7:0]

    0x08, 0x0F, 0,  // 10 RD_PIX_DATA_DLY_BIT[3:0]
    0x08, 0x03, 4,  // 11 RD_PIX_TEST_MODE_BIT[5:4]
    0x08, 0x01, 6,  // 12 IS_TEST_RD_PIX_BIT[6]
    0x08, 0x01, 7,  // 13 IS_BINARY_BIT[7]
    0x09, 0xFF, 0,  // 14 DIG_TEST_IMGA_RANG_REG[7:0]

    0x0A, 0x01, 7,  // 15 P_CNT0_RD_CLK_GEN_EN_BIT[7]
    0x0A, 0x0F, 0,  // 16 P_CNT0_RD_CLK_GEN_BIT[3:0]
    0x13, 0x01, 7,  // 17 RD_SEL_XY_BIT[7]
    0x13, 0x01, 6,  // 18 IS_BLOCK_RD_MODE_BIT[6]
    0x80, 0x07, 0,  // 19 RD_PIX_SPI_READ_START_BIT[2:0]

    0x81, 0x0F, 4,  // 20 INT_SEL_BIT[7:4]
    0x81, 0x01, 3,  // 21 RD_PIX_INT_CLR_BIT[3]
    0x81, 0x01, 2,  // 22 RD_PIX_INT_FORCE_DATA_BIT[2]
    0x81, 0x01, 1,  // 23 RD_PIX_INT_FORCE_EN_BIT[1]
    0x81, 0x01, 0,  // 24 RD_PIX_INT_ENABLE_BIT[0]

    0x83, 0x01, 7,  // 25 OFFSET_EN_ADC_BIT[7]
    0x83, 0x01, 6,  // 26 SEL_CHN_BIT[6]
    0x83, 0x07, 3,  // 27 OUT_F_SEL_BIT[5:3]
    0x83, 0x07, 0,  // 28 CHANL_ONE_PGAC_AFE_BIT[2:0]

    0x84, 0x03, 6,  // 29 PGAC_ADC_BIT[7:6]
    0x84, 0x07, 3,  // 30 SEL_TEST_BIT[5:3]
    0x84, 0x07, 0,  // 31 LBF_BIT[2:0]

    0x85, 0x03, 6,  // 32 SEL_CHARGE_PHASE_BIT[7:6]
    0x85, 0x07, 3,  // 33 LBF_DAC_BIT[5:3]
    0x85, 0x07, 0,  // 34 SEL_BIAS1_BIT[2:0]

    0x86, 0x03, 6,  // 35 CLK_SKEW_BIT[7:6]
    0x86, 0X07, 3,  // 36 SEL_BIAS2_BIT[5:3]
    0x86, 0x07, 0,  // 37 SEL_BIAS3_BIT[2:0]

    0x87, 0x03, 6,  // 38 ISEL_ADC_BIT[7:6]
    0x87, 0x3F, 0,  // 39 SEL_CHARGE_BIT[5:0]

    0x88, 0x03, 6,  // 40 ISEL_AFE_BIT[7:6]
    0x88, 0x3F, 0,  // 41 SEL_PGA_DAC_BIT[5:0]

    0x89, 0x03, 6,  // 42 RANGE_ADC_BIT[7:6]
    0x89, 0x3F, 0,  // 43 SEL_ADC_OFFSET[5:0]

    0x8A, 0x03, 6,  // 44 SHAGC_L_BIT[7:6]
    0x8A, 0x3F, 0,  // 45 SEL_CDS_DAC_BIT[5:0]

    0x8B, 0x01, 5,  // 46 BEZEL_FORCE_CFG_BIT[5]
    0x8B, 0x01, 4,  // 47 BEZEL_OEN_BIT[4]
    0x8B, 0x01, 2,  // 48 SHAGC_H_BIT[3]
    0x8B, 0x03, 0,  // 49 CDS_GAIN_BIT[1:0]

    0x8C, 0xFF, 0,  // 50 CHIP_ID_REG[7:0]
    0x8D, 0x07, 0,  // 51 SEL_BIAS4_BIT[2:0]

    0xE0, 0x01, 3,  // 52 RD_PIX_STATUS_OUT_INT_BIT[3]
    0xE0, 0x02, 1,  // 53 RD_PIX_STATUS_OUT_BIT[2:1]
    0xE0, 0x01, 0,  // 54 RD_PIX_INT_OUT_BIT[0]

    0xED, 0x01, 0,  // 55 SOFT_RESET_BIT[0]
    0xEF, 0xFF, 0,  // 56 CHIP_IMAGE_DATA_REG[7:0]
};




static void fpc_set_cs(struct fpc_fp_data* fp_data, unsigned char sta)
{
    if (gpio_is_valid(fp_data->reset_pin)) {
        gpio_direction_output(fp_data->cs_pin, sta);
    }
}

static int fpc_fp_hw_reset(struct fpc_fp_data* fp_data)
{
    if (gpio_is_valid(fp_data->reset_pin)) {
        gpio_direction_output(fp_data->reset_pin, 0);
        msleep(1);
        gpio_direction_output(fp_data->reset_pin, 1);
        msleep(1);
    }

    return 0;
}

static void fpc_set_encrypt_rst(struct fpc_fp_data* fp_data, unsigned char val)
{
    if (gpio_is_valid(fp_data->encrypt_ic_rst_pin)) {
        gpio_direction_output(fp_data->encrypt_ic_rst_pin, val);
    }
}


/*
 * fpc write and read data
 * @tx_buf/rx_buf data to write/read
 * @len :data length
 * @retval：=0:   success
 *          =-1   failed
 */
static int fpc_spi_transfer(struct fpc_fp_data *fp_data,
            unsigned char *tx_buf, unsigned char *rx_buf, int len)
{
    int err = 0;
    struct spi_device *spi = fp_data->spi;

    mutex_lock(&fp_data->dev_lock);

    fp_data->xfer.tx_buf        = tx_buf;
    fp_data->xfer.rx_buf        = rx_buf;
    fp_data->xfer.len           = len;
    fp_data->xfer.bits_per_word = spi->bits_per_word;
    fp_data->xfer.speed_hz      = fp_data->spi->max_speed_hz;
    fp_data->xfer.delay_usecs   = 1;

    spi_message_init(&fp_data->msg);
    spi_message_add_tail(&fp_data->xfer, &fp_data->msg);

    err = spi_sync(fp_data->spi, &fp_data->msg);
    if (err < 0) {
        dev_err(&spi->dev,"spi transfer failed error [%d]!\n", err);
    }

    mutex_unlock(&fp_data->dev_lock);

    return err;
}

static int fpc_spi_write(struct fpc_fp_data *fp_data,
                        unsigned char reg_addr, unsigned char val)
{
    int err;
    struct spi_device *spi = fp_data->spi;
    unsigned char tx_buffer[3];

    tx_buffer[0] = reg_addr;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = val;

    fpc_set_cs(fp_data, 0);
    err = fpc_spi_transfer(fp_data, tx_buffer, NULL, 3);
    if (err != 0){
        dev_err(&spi->dev, "do init_connect failed");
    }
    fpc_set_cs(fp_data, 1);

    return err;
}

static int fpc_spi_read(struct fpc_fp_data *fp_data,
                        unsigned char reg_addr, unsigned char* val)
{
    int err;
    struct spi_device *spi = fp_data->spi;
    unsigned char tx_buffer[3];
    unsigned char rx_buffer[3];

    tx_buffer[0] = reg_addr;
    tx_buffer[1] = 0x80;
    tx_buffer[2] = 0XFE;

    fpc_set_cs(fp_data, 0);
    err = fpc_spi_transfer(fp_data, tx_buffer, rx_buffer, 3);
    if (err != 0){
        dev_err(&spi->dev, "do init_connect failed");
        return err;
    }
    fpc_set_cs(fp_data, 1);

    *val = rx_buffer[2];

    return 0;
}

static int fpc_spi_reg_write(struct fpc_fp_data *fp_data,
                    unsigned char register_name, unsigned char data)
{
    char regnum;
    char shift = 0;
    char mask  = 0;
    int index;
    unsigned char whole_data = 0x0;
    unsigned char data_s     = 0;

    index  = register_name*3;
    regnum = fpc_register_map[index];
    mask   = fpc_register_map[index+1];
    shift  = fpc_register_map[index+2];

    fpc_spi_read(fp_data, (unsigned char)regnum, &whole_data);

    data_s     = (unsigned char)(data << shift);
    whole_data = (unsigned char)(whole_data & ~(mask << shift));
    whole_data |= data_s;

    return fpc_spi_write(fp_data, regnum, whole_data);
}


static unsigned char fpc_soft_reset(struct fpc_fp_data *fp_data, unsigned short delay)
{
    unsigned char ret = 0;

    if(5 > delay)
        delay = RESETDELAYTIMES;
    //寄存器 和 IO 复位方式选择
    fpc_spi_reg_write(fp_data,SOFT_RESET_BIT, 0x01);  // 寄存器写值方式复位sensor.
    mdelay(delay);
    fpc_spi_reg_write(fp_data,SOFT_RESET_BIT, 0x00);

    return ret;
}

/********************************************************************************
*** fpc_repair_gnd_line
*** 函数功能：地线修复方法
*** 输入参数：
*** unsigned char *img:  图像首地址；
*** unsigned long rows:  图像行数；
*** unsigned long cols:  图像列数;
*** 输出参数：无
*** 返回数值：LIB_OK  配置成功后返回
***
*** 代码维护:
********************************************************************************/
static unsigned char fpc_repair_gnd_line(unsigned char *p_img_buf,
                                         unsigned short width, unsigned short height)
{
    unsigned char gnd_line[16];
    unsigned char *p_img;
    unsigned short line_idx;

    gnd_line[0] = 34;
    gnd_line[1] = 98;

    p_img = p_img_buf + (height-(SENSOR_IMGH))/2*width;
    for(line_idx=0; line_idx<(SENSOR_IMGH); line_idx++)
    {
        p_img += (width-(SENSOR_IMGW))/2;

        p_img[gnd_line[0]] = p_img[gnd_line[0]-1] * 7/10/*0.7*/ + p_img[gnd_line[0]+2] * 3/10/*0.3*/;
        p_img[gnd_line[0]+1] = p_img[gnd_line[0]-1] * 3/10/*0.3*/ + p_img[gnd_line[0]+2] * 7/10/*0.7*/;
        p_img[gnd_line[1]] = p_img[gnd_line[1]-1] * 5/10/*0.5*/ + p_img[gnd_line[1]+1] * 5/10/*0.5*/;

        p_img += width - (width-(SENSOR_IMGW))/2;
    }

    return 0;
}


/********************************************************************************
*** 函数名称：   fpc_start_capture
*** 函数功能：  对Sensor 发布开始采图指令，发送该指令后，可直接通过图像寄存器读图像数据
*** 输入参数：无
*** 输出参数：无
*** 返回数值：0: LIB_OK  配置成功后返回
***
*** 代码维护:
********************************************************************************/
static unsigned char fpc_start_capture(struct fpc_fp_data *fp_data)
{
    unsigned char ret = 0;

    fpc_spi_reg_write(fp_data, RD_PIX_SPI_READ_START_BIT, 0x00);
    mdelay(1);
    fpc_spi_reg_write(fp_data, RD_PIX_SPI_READ_START_BIT, 0x07);
    mdelay(1);

    return ret;
}


/********************************************************************************
*** 函数名称：   fpc_enable_int
*** 函数功能：  对Sensor 中断引脚功能开关
*** 输入参数：   unsigned char SetStatus： 1：打开中断； 0：关闭中断
*** 返回数值：   0: LIB_OK
*** 代码维护:
********************************************************************************/
static unsigned char fpc_enable_int(struct fpc_fp_data *fp_data,
                                    unsigned char enable)
{
    unsigned char ret = 0;

    if(enable)
        ret = fpc_spi_reg_write(fp_data, RD_PIX_INT_ENABLE_BIT, 0x01);
    else
        ret = fpc_spi_reg_write(fp_data, RD_PIX_INT_ENABLE_BIT, 0x00);

    return ret;
}


/********************************************************************************
*** 函数名称：   fpc_switch_capture_mode
*** 函数功能：  对Sensor 自产生图测试模式和正常采图模式切换
*** 输入参数：   unsigned char SelMode   模式选择：0、1、2;
#   SWITCH_MODE_NORMAL
#   SWITCH_MODE_TEST_0      //输出自测图，固定值
#   SWITCH_MODE_TEST_1      //输出自测图，递增值
*** 返回数值：   0: LIB_OK
*** 代码维护:
********************************************************************************/
static unsigned char fpc_switch_capture_mode(struct fpc_fp_data *fp_data,
                                            unsigned char mode)
{
    unsigned char ret = 0;

    switch(mode) {
      case SWITCH_MODE_NORMAL: //正常采图模式
        fpc_spi_reg_write(fp_data, IS_TEST_RD_PIX_BIT, 0x00);
        break;

      case SWITCH_MODE_TEST_0:  //读测试图模式 固定值
        fpc_spi_reg_write(fp_data, IS_TEST_RD_PIX_BIT, 0x01);
        fpc_spi_reg_write(fp_data, RD_PIX_TEST_MODE_BIT, 0x00);
        fpc_spi_reg_write(fp_data, DIG_TEST_IMGA_RANG_REG, 0xFF);

        break;
      case SWITCH_MODE_TEST_1:  //读测试图模式
        fpc_spi_reg_write(fp_data, IS_TEST_RD_PIX_BIT, 0x01);
        fpc_spi_reg_write(fp_data, RD_PIX_TEST_MODE_BIT, 0x01);
        fpc_spi_reg_write(fp_data, DIG_TEST_IMGA_RANG_REG, 0xC1); //测试生成最大值
        break;

      default:
        ret = 1;
        break;
    }

    return ret;
}
/********************************************************************************
*** 函数名称：   fpc_switch_read_image_mode
*** 函数功能：  对Sensor输出图像数据方式
*** 输入参数：   unsigned char SelMode   模式选择：0、1、2;
#   SWITCH_CAPTURE_MODE_CONTIN
#   SWITCH_CAPTURE_MODE_INT_ONE     //输出自测图，固定值
#   SWITCH_CAPTURE_MODE_INT_TWO     //输出自测图，递增值
*** 返回数值：   0: LIB_OK
*** 代码维护:
*   设置方式：暂时为该处设置功能寄存器。具体采图方式通过 Init_Sensor_Lib()功能换读图函数。
********************************************************************************/
static unsigned char fpc_switch_read_image_mode(struct fpc_fp_data *fp_data,
                                                unsigned char mode)
{
    unsigned char ret = 0;
    switch(mode) {
      case SWITCH_CAPTURE_MODE_CONTIN: //连续采图
        fpc_spi_reg_write(fp_data, IS_BLOCK_RD_MODE_BIT, 0x00);
        break;

      case SWITCH_CAPTURE_MODE_INT_ONE:  //中断方式1
        fpc_spi_reg_write(fp_data, IS_BLOCK_RD_MODE_BIT, 0x00);
        fpc_enable_int(fp_data, 1);
        break;

      case SWITCH_CAPTURE_MODE_INT_TWO:  //中断方式2
        break;

      default:
        ret = 1;
        break;
    }
    return ret;
}

/********************************************************************************
*** 函数名称：   fpc_clear_int
*** 函数功能：  对Sensor 清除读图中断（写0再写1）
*** 输入参数：   无：
*** 返回数值：   0: LIB_OK
*** 代码维护:
********************************************************************************/
static unsigned char fpc_clear_int(struct fpc_fp_data *fp_data)
{
    unsigned char ret = 0;

    ret = fpc_spi_reg_write(fp_data, RD_PIX_INT_CLR_BIT, 0);
    ret = fpc_spi_reg_write(fp_data, RD_PIX_INT_CLR_BIT, 1);

    return ret;
}


/********************************************************************************
*** 函数名称：   FPS_SelectINTPin
*** 函数功能：  对Sensor 中断引脚出输选择
*** 输入参数：   unsigned char SelIntOutMode     选择中断输出功能
# INT_OUT_BEZAL
# INT_OUT_VERT_CTL5
# INT_OUT_VERT_CTL6
# INT_OUT_VERT_CTL7
# INT_OUT_ARRAY_CLK
# INT_OUT_ADC_CLK
# INT_OUT_SAMPLE_CLK
# INT_OUT_ARRAY_EN
# INT_OUT_SAMPLE_EN
# INT_OUT_rstn_soft
# INT_OUT_rstn_spi_clk
# INT_OUT_CLK
# INT_OUT_RD_PIX
# INT_OUT_RD_PIX_STATUS
# INT_OUT_NULL1
# INT_OUT_NULL2

*** 返回数值：   0: LIB_OK
*** 代码维护:
********************************************************************************/
static unsigned char fpc_select_int_pin(struct fpc_fp_data *fp_data,
                                        unsigned char int_out_mode)
{
    unsigned char ret = 0;

    ret = fpc_spi_reg_write(fp_data, INT_SEL_BIT, int_out_mode);

    return ret;
}


static int fpc_enable_internal(struct fpc_fp_data* fp_data,
                                        unsigned char enable)
{
    unsigned short i = 0, len = 0;
    unsigned char id_data = 0;

    if(enable) {
        gpio_direction_output(fp_data->reset_pin, 0);
    } else {
        gpio_direction_output(fp_data->reset_pin, 1);
        mdelay(1);

        fpc_spi_read(fp_data, 0x8c, &id_data);
        if(0x2c != id_data)
            return  -1;

        len = sizeof(fpc_register_default)/2;
        for(i=0; i<len; i++) {
            fpc_spi_write(fp_data, fpc_register_default[i][0], fpc_register_default[i][1]);
        }

        // 寄存器写值方式复位sensor.
        fpc_spi_reg_write(fp_data, SOFT_RESET_BIT, 0x01);
        mdelay(1);
        fpc_spi_reg_write(fp_data, SOFT_RESET_BIT, 0x00);
    }

    return 0;
}

static unsigned char fpc_sensor_read_image(struct fpc_fp_data *fp_data,
                                    unsigned char *p_image_buf,
                                    unsigned short width,
                                    unsigned short height)
{
#define PER_READ_MAX   (width*1)
    int i;
    unsigned short read_len;
    unsigned char* p_img;
    unsigned short data_count = 0;
    unsigned short line_idx;
    unsigned char  tx_buffer[1];
    unsigned char  rx_buffer[PER_READ_MAX];

    fpc_enable_internal(fp_data, 0);

    fpc_spi_reg_write(fp_data, BEZEL_OEN_BIT, 1);
    fpc_spi_reg_write(fp_data, RD_PIX_SPI_READ_START_BIT, 0x00);
    mdelay(1);
    fpc_spi_reg_write(fp_data, RD_PIX_SPI_READ_START_BIT, 0x07);
    mdelay(1);

    fpc_set_cs(fp_data, 0);

    tx_buffer[0] = 0xEF;
    fpc_spi_transfer(fp_data, tx_buffer, NULL, 1);
    tx_buffer[0] = 0xC0;
    fpc_spi_transfer(fp_data, tx_buffer, NULL, 1);

    memset(p_image_buf, 0xFF, height * width);

    data_count = 0;
    read_len   = PER_READ_MAX;
    p_img      = p_image_buf;

    for(line_idx = 0; line_idx < (SENSOR_IMGH << 1); line_idx+=(read_len/width)) {
        read_len = (SENSOR_IMAG_SIZE-data_count) < PER_READ_MAX ?
                   (SENSOR_IMAG_SIZE-data_count) : PER_READ_MAX ;
        fpc_spi_transfer(fp_data, NULL, rx_buffer, read_len);
        for (i = 0 ;i < read_len; i++) {
            if (rx_buffer[i] != 0) {
                p_img[data_count] = rx_buffer[i];
                data_count++;
            }
        }
        if (data_count >= (height * width)) {
            data_count = 0;
            memset(rx_buffer, 0x00, PER_READ_MAX);
            break;
        }
    }
    fpc_set_cs(fp_data, 1);

    fpc_spi_reg_write(fp_data, BEZEL_OEN_BIT, 0);
    fpc_spi_reg_write(fp_data, RD_PIX_SPI_READ_START_BIT, 0x00);
    mdelay(1);
    fpc_spi_reg_write(fp_data, RD_PIX_SPI_READ_START_BIT, 0x07);
    mdelay(1);
    fpc_set_cs(fp_data, 0);

    tx_buffer[0] = 0xEF;
    fpc_spi_transfer(fp_data, tx_buffer, NULL, 1);
    tx_buffer[0] = 0xC0;
    fpc_spi_transfer(fp_data, tx_buffer, NULL, 1);

    data_count = 0;
    read_len   = PER_READ_MAX;
    for(line_idx = 0; line_idx < (SENSOR_IMGH << 1); line_idx+=(read_len/width)) {
        read_len = (SENSOR_IMAG_SIZE-data_count) < PER_READ_MAX ?
                   (SENSOR_IMAG_SIZE-data_count) : PER_READ_MAX ;
        fpc_spi_transfer(fp_data, NULL, rx_buffer, read_len);
        for (i = 0; i < read_len; i++) {
            if (rx_buffer[i] != 0) {
                if (p_img[data_count] > rx_buffer[i]) {
                    p_img[data_count] = 0xFF - (p_img[data_count] - rx_buffer[i]);
                } else {
                    p_img[data_count] = 0xff;
                }
                data_count++;
            }
        }

        if (data_count >= (height * width)) {
            data_count = 0;
            break;
        }
    }
    fpc_set_cs(fp_data, 1);

    fpc_enable_internal(fp_data, 1);
    fpc_repair_gnd_line(p_image_buf, width, height);

    return 100;
}

static irqreturn_t fpc_fp_interrupt(int irq, void *dev_id)
{
    struct fpc_fp_data *fp_data = dev_id;
    queue_work(fp_data->workqueue, &fp_data->work);

    return IRQ_HANDLED;
}

static void fpc_fp_work(struct work_struct *work)
{
    /*
    struct fpc_fp_data *fp_data = \
                         container_of(work, struct fpc_fp_data, work);
    */
}

static int fpc_dev_open(struct inode *inode, struct file *filp)
{
    struct cdev *cdev = inode->i_cdev;
    struct finger_print_dev *fp_dev = container_of(cdev, struct finger_print_dev, chd);
    struct fpc_fp_data *fp_data = container_of(fp_dev, struct fpc_fp_data, fp_dev);

    filp->private_data = fp_data;
    return 0;
}

static long fpc_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct fpc_fp_data *fp_data = (struct fpc_fp_data *)filp->private_data;

    switch (cmd) {
        case READ_IMAGE:
            fpc_sensor_read_image(fp_data, fp_data->image_buf, SENSOR_IMGH, SENSOR_IMGW);
            copy_to_user((unsigned char*)arg, fp_data->image_buf, SENSOR_IMAG_SIZE);
        break;

        case HW_RESET:
            fpc_fp_hw_reset(fp_data);
        break;

        case ENCRYPT_IC_RST:
            fpc_set_encrypt_rst(fp_data, (unsigned char)arg);
        default:
        break;
    }
    return 0;
}

static int fpc_dev_release(struct inode *inode, struct file *filp)
{
    /*
    struct fpc_fp_data *fp_data = (struct fpc_fp_data *)filp->private_data;
    */

    filp->private_data = NULL;

    return 0;
}

static const struct file_operations fpc_fp_fops = {
        .owner          =    THIS_MODULE,
        .open           = fpc_dev_open,
        .release        = fpc_dev_release,
        .unlocked_ioctl = fpc_dev_ioctl,
};

static int fpc_gpio_init(struct fpc_fp_data* fp_data)
{
    int err = 0;
    struct fpc_platform_data *pdata = fp_data->pdata;
    struct spi_device *spi = fp_data->spi;

    if (!pdata) {
        return 0;
    }

    fp_data->int_pin            = pdata->int_pin;
    fp_data->reset_pin          = pdata->reset_pin;
    fp_data->cs_pin             = pdata->cs_pin;
    fp_data->encrypt_ic_rst_pin = pdata->encrypt_ic_rst_pin;

    if (gpio_is_valid(fp_data->reset_pin)) {
        err = gpio_request(fp_data->reset_pin, "fp reset");
        if (err < 0) {
            dev_err(&spi->dev,"finger print: %s reset[%d] gpio_requests failed.\n",
                    __func__, fp_data->reset_pin);
            goto gpio_reset_failed;
        }
    }

    if (gpio_is_valid(fp_data->encrypt_ic_rst_pin)) {
        err = gpio_request(fp_data->encrypt_ic_rst_pin, "encrypt ic rst pin");
        if (err < 0) {
            dev_err(&spi->dev,"finger print: %s encrypt ic rst[%d] gpio_requests failed.\n",
                    __func__, fp_data->encrypt_ic_rst_pin);
            goto gpio_encrypt_rst_failed;
        }
    }

    if (gpio_is_valid(fp_data->int_pin)) {
        err = gpio_request(fp_data->int_pin, "fp interrupt");
        if (err < 0) {
            dev_err(&spi->dev,"finger print: %s interrupt[%d] gpio_requests failed.\n",
                    __func__, fp_data->int_pin);
            goto gpio_interrupt_failed;
        }

        gpio_direction_input(fp_data->int_pin);
    }

    fp_data->irq = gpio_to_irq(fp_data->int_pin);
    err = request_irq(fp_data->irq, fpc_fp_interrupt,
                IRQF_TRIGGER_FALLING, "fpc irq", fp_data);
    if (err < 0) {
        dev_err(&spi->dev,"request_irq failed err=%d.", err);
        goto gpio_request_irq_failed;
    }

    return 0;

gpio_request_irq_failed:
    gpio_free(fp_data->int_pin);
gpio_interrupt_failed:
    gpio_free(fp_data->encrypt_ic_rst_pin);
gpio_encrypt_rst_failed:
    gpio_free(fp_data->reset_pin);
gpio_reset_failed:
    return err;
}

static int fpc_gpio_free(struct fpc_fp_data* fp_data)
{
    if (fp_data->irq > 0)
        free_irq(fp_data->irq, fp_data);

    if (gpio_is_valid(fp_data->reset_pin))
        gpio_free(fp_data->reset_pin);

    if (gpio_is_valid(fp_data->encrypt_ic_rst_pin))
        gpio_free(fp_data->encrypt_ic_rst_pin);

    if (gpio_is_valid(fp_data->int_pin))
        gpio_free(fp_data->int_pin);

    return 0;
}



static int fpc_chip_init(struct fpc_fp_data* fp_data)
{
    return fpc_enable_internal(fp_data, 1);
}

static int fpc_fp_check_id(struct fpc_fp_data *fp_data)
{
    int ret;
    unsigned char id;

    ret = fpc_spi_read(fp_data, 0x8c, &id);
    if (ret < 0) {
        return -1;
    }
    return id;
}

static int fpc_fp_probe(struct spi_device* spi)
{
    struct fpc_fp_data *fp_data = NULL;
    int err = -1;

    dev_info(&spi->dev, "FPC finger print probe sucessfull.\n");

    fp_data = kzalloc(sizeof(struct fpc_fp_data), GFP_KERNEL);
    if (fp_data == NULL) {
        dev_err(&spi->dev, "micro array finger print kmalloc failed.\n");
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    dev_set_drvdata(&spi->dev, fp_data);

    fp_data->pdata  = spi->dev.platform_data;
    fp_data->spi    = spi;
    mutex_init(&fp_data->dev_lock);
    mutex_init(&fp_data->ioctl_lock);

    err = fpc_gpio_init(fp_data);
    if (err < 0) {
        dev_err(&spi->dev, "micro array gpio init failed.\n");
        goto exit_gpio_init_failed;
    }

    fp_data->fp_dev.minor = 0;
    err = alloc_chrdev_region(&fp_data->fp_dev.idd,
                               fp_data->fp_dev.minor,
                               1,
                               FPC_DRV_NAME);
    if (err < 0) {
        dev_err(&spi->dev, "alloc_chrdev_region failed!");
        goto exit_alloc_chr_dev_failed;
    }

    cdev_init(&fp_data->fp_dev.chd, &fpc_fp_fops);
    fp_data->fp_dev.chd.owner  = THIS_MODULE;
    fp_data->fp_dev.major = MAJOR(fp_data->fp_dev.idd);
    cdev_add(&fp_data->fp_dev.chd, fp_data->fp_dev.idd, 1);

    fp_data->fp_dev.cls = class_create(THIS_MODULE, FPC_DRV_NAME);
    if (IS_ERR(fp_data->fp_dev.cls)) {
        dev_err(&spi->dev, "class create failed!");
        err = -1;
        goto exit_cls_create_failed;
    }

    fp_data->fp_dev.dev = device_create(fp_data->fp_dev.cls,  NULL,
            fp_data->fp_dev.idd, NULL, FPC_DRV_NAME);
    err = IS_ERR(fp_data->fp_dev.dev) ? PTR_ERR(fp_data->fp_dev.dev) : 0;
    if (err) {
        dev_err(&spi->dev,"device_create failed. err = %d", err);
        goto exit_device_create_failed;
    }

    fp_data->spi->max_speed_hz = SPI_SPEED;
    err = spi_setup(spi);

    fpc_chip_init(fp_data);
    err = fpc_fp_check_id(fp_data);
    if (err != 0x2c || err < 0) {
        dev_err(&spi->dev,"check id failed. err = %d", err);
        err = -1;
        goto exit_check_id_failed;
    }

    //fpc_enable_int(fp_data,1);
    wake_lock_init(&fp_data->process_wakelock, WAKE_LOCK_SUSPEND,"fpc_process_wakelock");
    INIT_WORK(&fp_data->work, fpc_fp_work);
    fp_data->workqueue = create_singlethread_workqueue("mas_workqueue");
    if (!fp_data->workqueue) {
        dev_err(&spi->dev, "create_single_workqueue error!\n");
        err = -1;
        goto exit_create_workqueu_failed;
    }

    fp_data->image_buf = (unsigned char*)kzalloc(SENSOR_IMAG_SIZE, GFP_KERNEL);
    if (fp_data->image_buf == NULL) {
        dev_err(&spi->dev, "image buf kzalloc failed!\n");
        err = -ENOMEM;
        goto exit_alloc_buf_failed;
    }
    init_waitqueue_head(&fp_data->waitqueue);

    dev_err(&spi->dev, "FPC finger print register sucessfull.\n");
    return 0;
exit_alloc_buf_failed:
    destroy_workqueue(fp_data->workqueue);
exit_create_workqueu_failed:
    wake_lock_destroy(&fp_data->process_wakelock);
exit_check_id_failed:
    device_destroy(fp_data->fp_dev.cls, fp_data->fp_dev.idd);
exit_device_create_failed:
    class_destroy(fp_data->fp_dev.cls);
exit_cls_create_failed:
    cdev_del(&fp_data->fp_dev.chd);
    unregister_chrdev_region(fp_data->fp_dev.idd, 1);
exit_alloc_chr_dev_failed:
    fpc_gpio_free(fp_data);
exit_gpio_init_failed:
    kfree(fp_data);
    fp_data = NULL;
exit_alloc_data_failed:
    return err;
}

static int fpc_fp_remove(struct spi_device* spi)
{
    struct fpc_fp_data *fp_data = dev_get_drvdata(&spi->dev);

    fpc_gpio_free(fp_data);
    destroy_workqueue(fp_data->workqueue);
    wake_lock_destroy(&fp_data->process_wakelock);
    device_destroy(fp_data->fp_dev.cls, fp_data->fp_dev.idd);
    class_destroy(fp_data->fp_dev.cls);
    cdev_del(&fp_data->fp_dev.chd);
    unregister_chrdev_region(fp_data->fp_dev.idd, 1);

    kfree(fp_data->image_buf);
    kfree(fp_data);
    fp_data = NULL;

    return 0;
}

struct spi_driver fpc_fp_driver = {
        .probe = fpc_fp_probe,
        .remove = fpc_fp_remove,
        .driver = {
                .name = "fpc_fp",
                .owner = THIS_MODULE,
        },
};

static int fpc_init(void) {
    return spi_register_driver(&fpc_fp_driver);
}

static void fpc_exit(void) {
    spi_unregister_driver(&fpc_fp_driver);
}

module_init(fpc_init);
module_exit(fpc_exit);

MODULE_AUTHOR("Ingenic");
MODULE_DESCRIPTION("Driver for FPC fingerprint sensor");
MODULE_LICENSE("GPL");
