#ifndef __GW_FPGA_H
#define __GW_FPGA_H
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <soc/gpio.h>


#define MISC_NAME "shanling_fpga"


#define DOWNFPGA  1
#if defined(CONFIG_PRODUCT_X1000_M2X) || defined(CONFIG_PRODUCT_X1000_M5S) || defined(CONFIG_PRODUCT_X1000_H5)
#define FPGA_TCK GPIO_PB(14)
#define FPGA_TMS GPIO_PB(15)
#define FPGA_TDI GPIO_PB(13)
#define FPGA_TDO GPIO_PB(12)
#elif defined(CONFIG_PRODUCT_X1000_CD80) || defined(CONFIG_PRODUCT_X1000_CA80)
#define FPGA_TCK GPIO_PA(18)
#define FPGA_TMS GPIO_PA(19)
#define FPGA_TDI GPIO_PA(16)
#define FPGA_TDO GPIO_PA(17)
#elif defined(CONFIG_PRODUCT_X1000_ECMINI)
#define FPGA_TCK GPIO_PA(19)
#define FPGA_TMS GPIO_PA(17)
#define FPGA_TDI GPIO_PA(16)
#define FPGA_TDO GPIO_PA(18)
#else
#define FPGA_TCK GPIO_PC(4)
#define FPGA_TMS GPIO_PC(5)
#define FPGA_TDI GPIO_PC(3)
#define FPGA_TDO GPIO_PC(2)
#endif
#define DELAY_LEN 2
#define BUFFER_SIZE        0xFF           /*定义读写BUFFER大小*/
#define ID_GW1N4   0x100381B
#define ID_GW1N1   0x900281B


//******all fpga data load ram*********
static uint8_t _tmp_index;
uint32_t CYCLES = 0x00;
uint32_t gw_type_code = 0;
uint32_t _BYTE_count = 0x00;
uint8_t first_configgpio = 0;
#define FPGA_MAX_LEN (0x200000) // 2M
static char* fpga_buf = NULL;
static unsigned fpga_len = 0;
u16 SramTxBuffer[BUFFER_SIZE];
u16 SramRxBuffer[BUFFER_SIZE];

uint8_t saveBuffer[500000];

//*********************


int test_fpgaflag = 0;

typedef enum
{ TAP_RESET,
	
	TAP_IDLE,
	TAP_DRSELECT,
	TAP_DRCAPTURE,
	TAP_DRSHIFT,
	TAP_DREXIT1,
	TAP_DRPAUSE,
	TAP_DREXIT2,
	TAP_DRUPDATE,
	
	TAP_IRSELECT,
	TAP_IRCAPTURE,
	TAP_IRSHIFT,
	TAP_IREXIT1,
	TAP_IRPAUSE,
	TAP_IREXIT2,
	TAP_IRUPDATE,
	TAP_UNKNOWN
	
	
}TAP_TypeDef;

typedef enum
{ 
	ISC_NOOP          = 0x02,
	
	ISC_ERASE         = 0x05,
	
	ERASE_DONE        = 0x09,
	
	READ_ID_CODE      = 0x11,
	
	ISC_ENABLE        = 0x15,
	
	FAST_PROGRAM      = 0x17,
	
	STATUS_CODE       = 0x41,
	
	JTAG_EF_PROGRAM   = 0x71,
	
	JTAG_EF_READ      = 0x73,
	
	JTAG_EF_ERASE     = 0x75,
	
	ISC_DISABLE       = 0x3A,
	
	REPROGRAM         = 0x3C,
	
	Bypass            = 0xFF
	
	
}GWFPGA_Inst_Def;


void JTAG_TapMove_OneClock(uint8_t tms_value);
uint8_t JTAG_Write(uint8_t din, uint8_t dout, uint8_t tms, bool LSB);
void JTAG_MoveTap(TAP_TypeDef TAP_From, TAP_TypeDef TAP_To);
void JTAG_WriteInst(uint8_t inst);
uint32_t JTAG_ReadCode(uint8_t inst);	//read code

void JTAG_RunTest(int cycles);

bool JTGA_ef_erase_gw1n4();//erase flash

int format_fs_data();
bool JTAG_ef_prog_gw1n4(bool verify); //burn flash
void jtag_ef_prog_one_X_gw1n4(uint8_t data_array[256], uint32_t address_index);
void jtag_ef_prog_one_Y(uint8_t data_array[4]);
void jtag_ef_prog_write_one_X_address(uint32_t address_index);


#endif

