

#include "gw_fpgadown.h"

void JTAG_TapMove_OneClock(uint8_t tms_value)
{
	gpio_set_value(FPGA_TMS,tms_value);
	gpio_set_value(FPGA_TCK,0);
	for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
	gpio_set_value(FPGA_TCK,1);
	for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
}


void JTAG_MoveTap(TAP_TypeDef TAP_From, TAP_TypeDef TAP_To)
{
	 int i=0;
	 if ((TAP_From == TAP_UNKNOWN) &&  (TAP_To==TAP_IDLE) )
	 {
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(0);
			 JTAG_TapMove_OneClock(0);
	 }
	 
	 
	 else if ((TAP_From == TAP_IDLE) &&  (TAP_To==TAP_IDLE) )
	 {
				JTAG_TapMove_OneClock(0);
				JTAG_TapMove_OneClock(0);
				JTAG_TapMove_OneClock(0);
	 }
	 
	 
	 else if ((TAP_From == TAP_IDLE) &&  (TAP_To==TAP_IRSHIFT) )
	 {
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(0);
			 JTAG_TapMove_OneClock(0);
 
	 }
	 
	 else if ((TAP_From == TAP_IDLE) &&  (TAP_To==TAP_DRSHIFT) )
	 {
	 
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(0);
			 JTAG_TapMove_OneClock(0);
		 
	 }
	 
	 
	 else if ((TAP_From == TAP_IREXIT1) &&  (TAP_To==TAP_IDLE) )
	 {
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(0);
			 JTAG_TapMove_OneClock(0);
	 }
	 
	 
	 else if ((TAP_From == TAP_DREXIT1) &&  (TAP_To==TAP_IDLE) )
	 {
	 
			 JTAG_TapMove_OneClock(1);
			 JTAG_TapMove_OneClock(0);
			 JTAG_TapMove_OneClock(0);
	 }
   	 else	
	 {
		 printk("error tap walking.");
	 }		 
	 

}


uint8_t JTAG_Write(uint8_t din, uint8_t dout, uint8_t tms, bool LSB)
{
	
  /* 
	@param :
		tms =0/1
	*/
	
	int i=0;
	int tmp=0;
	uint8_t dout_new = 0;
	
	if (LSB==false)
	{
		 uint8_t _tmp_din=0;
		 uint8_t sign = 1;
		 for (i=0;i<=7;i++)
		 {
			_tmp_din |=((din&(sign<<i))>>i)<<(7-i);
		 }
		
		 
		din = _tmp_din;
	}
	
	gpio_set_value(FPGA_TMS,0);
	for (i=0; i<8; i++)  // LSB
	{
			if (i == 7)
			{
				if((tms & 1) == 1)
					gpio_set_value(FPGA_TMS,1);	
			}
			
			tmp = din>>i ;	
			if((1&tmp) == 0)
				gpio_set_value(FPGA_TDI,0);
			else
				gpio_set_value(FPGA_TDI,1);
		
			gpio_set_value(FPGA_TCK,0);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
			gpio_set_value(FPGA_TCK,1);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);

		  	dout_new <<= 1;
		  	dout_new |= (gpio_get_value(FPGA_TDO) & 1);
	
	}
	return dout_new;
}



void JTAG_WriteInst(uint8_t inst)
{
	
	JTAG_MoveTap(TAP_IDLE, TAP_IRSHIFT);
	JTAG_Write(inst, 0x0, 0x1,true);
	JTAG_MoveTap(TAP_IREXIT1, TAP_IDLE);
	 
	JTAG_MoveTap(TAP_IDLE, TAP_IDLE);
	JTAG_MoveTap(TAP_IDLE, TAP_IDLE);
	JTAG_MoveTap(TAP_IDLE, TAP_IDLE);
}

void JTAG_RunTest(int cycles)
{
	int i = 0;
	for (i=0; i<cycles; i++)
	{
		JTAG_TapMove_OneClock(0);
	}

}



uint32_t JTAG_ReadCode(uint8_t inst)
{
	 uint8_t i=0;
	 uint32_t out=0;
	 JTAG_WriteInst(inst);
	 JTAG_MoveTap(TAP_IDLE, TAP_DRSHIFT);
	
		gpio_set_value(FPGA_TMS,0);
		for (i=0; i<32; i++)
		{
			if (i == 31) gpio_set_value(FPGA_TMS,1);	

			gpio_set_value(FPGA_TCK,0);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
			gpio_set_value(FPGA_TCK,1);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);

			
			out |= (gpio_get_value(FPGA_TDO)) ? (1 << i) : 0;
		};
		JTAG_MoveTap(TAP_DREXIT1,  TAP_IDLE);
		return out;
			
}


//erase Internal flash
bool JTGA_ef_erase_gw1n1()
{
	int i=0;
	uint32_t status_code = 0x00;
	JTAG_WriteInst(READ_ID_CODE);
	status_code =  JTAG_ReadCode(STATUS_CODE);
	//printk("end1111 to ef erase with status_code: %04X\n", status_code);
	if (status_code == 0x19020	 || status_code == 0x15421	|| status_code == 0x11424)
	{
		return true;
	}
	else
	{
		JTAG_WriteInst(ISC_ENABLE);
		JTAG_WriteInst(ISC_ERASE);
		JTAG_WriteInst(ISC_NOOP);
		CYCLES = 4000;
		while(CYCLES--)
		{
		gpio_set_value(FPGA_TCK,0);
		for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
		gpio_set_value(FPGA_TCK,1);
		for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
		}
		JTAG_WriteInst(ERASE_DONE);
		JTAG_WriteInst(ISC_NOOP);
		JTAG_WriteInst(ISC_DISABLE);
		JTAG_WriteInst(ISC_NOOP);
		JTAG_RunTest(500);
	}

	status_code =  JTAG_ReadCode(0x41);
	//printk("end2222 to ef erase with status_code: %04X\n", status_code);
	if (status_code == 0x19020	 || status_code == 0x15421	|| status_code == 0x11424)
	{
	}
	else
	{
		JTAG_WriteInst(ISC_ENABLE);
		JTAG_WriteInst(ISC_NOOP);
		JTAG_WriteInst(ISC_ERASE);
		JTAG_WriteInst(ISC_NOOP);
		CYCLES = 4000;
		while(CYCLES--)
		{
			gpio_set_value(FPGA_TCK,0);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
			gpio_set_value(FPGA_TCK,1);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);	
		}	
		JTAG_WriteInst(ERASE_DONE);
		JTAG_WriteInst(ISC_NOOP);
		JTAG_WriteInst(ISC_DISABLE);
		JTAG_WriteInst(ISC_NOOP);
		JTAG_RunTest(500);
	}
	status_code =  JTAG_ReadCode(0x41);
	//printk("end33333 to ef erase with status_code: %04X\n", status_code);
	if (status_code == 0x19020	 || status_code == 0x15421	|| status_code == 0x11424)
	{
	}
	else
	{
		return false;
	}
	
	JTAG_WriteInst(ISC_ENABLE);
	JTAG_WriteInst(JTAG_EF_ERASE);

	for (i=0; i<65; i++)
	{			
		JTAG_MoveTap(TAP_IDLE, TAP_DRSHIFT);
		 
		JTAG_Write(0x0, 0x0,false,true);
		JTAG_Write(0x0, 0x0,false,true);
		JTAG_Write(0x0, 0x0,false,true);
		JTAG_Write(0x0, 0x0,true,true);
				
		JTAG_MoveTap(TAP_DREXIT1, TAP_IDLE);
	}

	CYCLES = 24500;  //��С21ms����󲻳���96ms�� ������Լ���ʱ�����ڼ���    Ƶ�� >=1.3Mhz 
	while(CYCLES--)
	{
		gpio_set_value(FPGA_TCK,0);
		for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
		gpio_set_value(FPGA_TCK,1);
		for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);					
	}
	JTAG_WriteInst(ISC_DISABLE);
	JTAG_WriteInst(READ_ID_CODE);
	CYCLES=0xffffff;
	while(CYCLES--);
	
	status_code =  JTAG_ReadCode(0x41);
	//printk("end555555 to ef erase with status_code: %04X\n", status_code);
	if (status_code == 0x19020)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}


//erase Internal flash
bool JTGA_ef_erase_gw1n4()
{
	uint32_t status_code = 0x00;
	JTAG_WriteInst(READ_ID_CODE);
	status_code =  JTAG_ReadCode(STATUS_CODE);
	//printk("end1111 to ef erase with status_code: %04X\n", status_code);
	if (status_code == 0x19020   || status_code == 0x15421  || status_code == 0x11424)
	{
	}
	else
	{
		JTAG_WriteInst(ISC_ENABLE);
		JTAG_WriteInst(ISC_ERASE);
		JTAG_WriteInst(ISC_NOOP);
		CYCLES = 4000;
		while(CYCLES--)
		{
		gpio_set_value(FPGA_TCK,0);
		for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
		gpio_set_value(FPGA_TCK,1);
		for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
		}	
		JTAG_WriteInst(ERASE_DONE);
		JTAG_WriteInst(ISC_NOOP);
		JTAG_WriteInst(ISC_DISABLE);
		JTAG_WriteInst(ISC_NOOP);
		JTAG_RunTest(500);
	}
	
	status_code =  JTAG_ReadCode(0x41);
	//printk("end2222 to ef erase with status_code: %04X\n", status_code);
	if (status_code == 0x19020	 || status_code == 0x15421	|| status_code == 0x11424)
	{
	}
	else
	{
		JTAG_WriteInst(ISC_ENABLE);
		JTAG_WriteInst(ISC_ERASE);
		JTAG_WriteInst(ISC_NOOP);
		CYCLES = 4000;
		while(CYCLES--)
		{
			gpio_set_value(FPGA_TCK,0);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
			gpio_set_value(FPGA_TCK,1);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);	
		}	
		JTAG_WriteInst(ERASE_DONE);
		JTAG_WriteInst(ISC_NOOP);
		JTAG_WriteInst(ISC_DISABLE);
		JTAG_WriteInst(ISC_NOOP);
		JTAG_RunTest(500);
	}
	status_code =  JTAG_ReadCode(0x41);
	//printk("ready to ef erase  with status_code: %04X\n", status_code);
	if (status_code == 0x19020	 || status_code == 0x15421	|| status_code == 0x11424)
	{
	}
	else
	{
		return false;
	}
	JTAG_WriteInst(ISC_ENABLE);
	JTAG_WriteInst(JTAG_EF_ERASE);
	 
	 
	JTAG_MoveTap(TAP_IDLE, TAP_DRSHIFT);
	 
			
	 
			
	JTAG_TapMove_OneClock(0);
	JTAG_TapMove_OneClock(0);
	JTAG_TapMove_OneClock(0);
	JTAG_TapMove_OneClock(0);
	JTAG_TapMove_OneClock(1);
	JTAG_TapMove_OneClock(0);
			
	JTAG_TapMove_OneClock(1);
	JTAG_TapMove_OneClock(0);
	JTAG_TapMove_OneClock(1);
	JTAG_TapMove_OneClock(1);
	JTAG_TapMove_OneClock(0);
	JTAG_TapMove_OneClock(0);
	JTAG_TapMove_OneClock(0);
	JTAG_TapMove_OneClock(0);
	CYCLES = 290000;	
	while(CYCLES--)
	{
			gpio_set_value(FPGA_TCK,0);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
			gpio_set_value(FPGA_TCK,1);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
	}
	JTAG_WriteInst(ISC_DISABLE);
	JTAG_WriteInst(ISC_NOOP);
			
	CYCLES=0xffffff;
	while(CYCLES--);	
	JTAG_WriteInst(READ_ID_CODE);
	JTAG_WriteInst(REPROGRAM);
	JTAG_WriteInst(ISC_NOOP);
			 
				
	JTAG_WriteInst(READ_ID_CODE);
				
	CYCLES=0xffffff;
	while(CYCLES--);
	JTAG_RunTest(500000); 
	status_code =  JTAG_ReadCode(0x41);
	//printk("end to ef erase with status_code: %04X\n", status_code);
	if (status_code == 0x19020)
	{
		return true;
	}
	else
	{
		return false;
	}
}



void SRAM_WriteBuffer(uint16_t* pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite)
{
  for (; NumHalfwordToWrite != 0; NumHalfwordToWrite--) /* while there is data to write */
  {
    /* Transfer data to the memory */
    *(uint16_t *)(WriteAddr) = *pBuffer++;

    /* Increment the address*/
    WriteAddr += 2;
  }
}

void SRAM_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr, uint32_t NumHalfwordToRead)
{
  for (; NumHalfwordToRead != 0; NumHalfwordToRead--) /* while there is data to read */
  {
    /* Read a half-word from the memory */
    *pBuffer++ = *(uint16_t*)(ReadAddr);

    /* Increment the address*/
    ReadAddr += 2;
  }
}



int format_fs_data()
{
	
	int i=0,offset=0,k=0;
	int pos = 0;
	int res;
	int diff_index = 0;
	int diff_res = 0;
	
	uint16_t _BYTE = 0x0000;
	uint8_t _BYTE_index = 0x00;
	
	uint32_t _buffer_index = 0x00;
	uint32_t _page_index = 0x00;
	
	uint8_t buffer[512];
	uint32_t _tmp_index = 0x00;
	uint32_t br;

	_BYTE_count = 0;

	for(_tmp_index=0;_tmp_index<10;_tmp_index++) 
	{
		SramTxBuffer[_buffer_index++] = 0xFFFF ;
	}

	br = 512; 
	offset = 0;
	while(1)
	{
		for(k=0;k<br;k++)
		{
			buffer[k] = fpga_buf[offset+k];
		}

		for(i=0;i<br;i++)
		{
			if ((_BYTE_index % 16 == 0) && (_BYTE_index > 0) )
			{
				if ( (_buffer_index % BUFFER_SIZE == 0)  &&  (_buffer_index > 0) )
				{	
					  SRAM_WriteBuffer(SramTxBuffer,saveBuffer+(BUFFER_SIZE*2*_page_index),BUFFER_SIZE);
					  SRAM_ReadBuffer(SramRxBuffer, saveBuffer+(BUFFER_SIZE*2*_page_index),BUFFER_SIZE);
					  for (diff_index=0; diff_index<BUFFER_SIZE; diff_index++)
						{
							 if (SramTxBuffer[diff_index] != SramRxBuffer[diff_index])
							 {
								 //printk("%04X:%04X ",SramTxBuffer[diff_index], SramRxBuffer[diff_index]);
								 diff_res = 1;
							 }
						 }
					  _buffer_index = 0;
					  _page_index += 1;
				}
				SramTxBuffer[_buffer_index++] = _BYTE ;				
				
				_BYTE_count+=2;

				
				_BYTE = 0;
				_BYTE_index =0;
			}
			if (buffer[i] == '1'  || buffer[i] == '0' )
			{
				_BYTE <<= 1; 
				_BYTE_index++;
			}
			
			
			if (buffer[i] == '1' )
			{
				   _BYTE += 1;
			}

		}


		offset += br;
		if(offset + 512 < fpga_len)
		{
			br = 512;
		}
		else
			br = fpga_len-offset-1;
		if(br <= 0)	break;
	}

	for(;_buffer_index<BUFFER_SIZE; _buffer_index++)
	{
		SramTxBuffer[_buffer_index] = 0;
	}

	SRAM_WriteBuffer(SramTxBuffer,saveBuffer+(BUFFER_SIZE*2*_page_index),BUFFER_SIZE);
	SRAM_ReadBuffer(SramRxBuffer, saveBuffer+(BUFFER_SIZE*2*_page_index),BUFFER_SIZE);
	for (diff_index=0; diff_index<BUFFER_SIZE; diff_index++)
	{
		if (SramTxBuffer[diff_index] != SramRxBuffer[diff_index])
		{
			//printk("%04X:%04X ",SramTxBuffer[diff_index], SramRxBuffer[diff_index]);
			diff_res = 1;
		}
	}

	//printk("======diff_res[%d]====\n", diff_res);
	return 1;
	
}


void jtag_ef_prog_write_one_X_address(uint32_t address_index)
{
	 int i=0;
	 int tmp=0;
	 
	int tmpaddr = (address_index << 6)&0xFFFFFFC0;

	 JTAG_MoveTap(TAP_IDLE, TAP_DRSHIFT);
	/*
     for (i=0; i<6; i++) 
		{	
			gpio_set_value(FPGA_TDI,0);
#ifdef STM32_DEBUG
			gpio_set_value(FPGA_TCK,0);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
			gpio_set_value(FPGA_TCK,1);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
#else
			gpio_set_value(FPGA_TCK,0);
			ndelay(1);
			gpio_set_value(FPGA_TCK,1);
#endif
		}
		*/
		for (i=0; i<32; i++)  
		{
			if (i == 31) 	gpio_set_value(FPGA_TMS,1);	
			tmp = tmpaddr >>i;
			if((1 & tmp) == 0)
				gpio_set_value(FPGA_TDI,0);
			else
				gpio_set_value(FPGA_TDI,1);	

			gpio_set_value(FPGA_TCK,0);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
			gpio_set_value(FPGA_TCK,1);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);			
 			
		}
		JTAG_MoveTap(TAP_DREXIT1, TAP_IDLE);
		
		/* 延时20us */
		CYCLES = 45;   
		while(CYCLES--)
		{
			gpio_set_value(FPGA_TCK,0);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
			gpio_set_value(FPGA_TCK,1);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);					
		}	
							
}

void jtag_ef_prog_one_Y(uint8_t data_array[4])
{
	int i=3;
	uint8_t tms=0;	
	//printk("%02X%02X%02X%02X   ", data_array[3], data_array[2],data_array[1],data_array[0]);
	JTAG_MoveTap(TAP_IDLE, TAP_DRSHIFT);
	for (i=3; i>=0; i--)
	{ 
			tms = (i==0) ? 1:0; 
		 	JTAG_Write(data_array[i], 0x0, tms, true);
	}
	
	JTAG_MoveTap(TAP_DREXIT1, TAP_IDLE);
	JTAG_MoveTap(TAP_IDLE, TAP_IDLE);
	JTAG_MoveTap(TAP_IDLE, TAP_IDLE);
	JTAG_MoveTap(TAP_IDLE, TAP_IDLE);
}



void jtag_ef_prog_one_X_gw1n4(uint8_t data_array[256], uint32_t address_index)
{
	    int i=0;
	  	uint8_t y_page[4] = {0};
		
		JTAG_WriteInst(JTAG_EF_PROGRAM);
	   	if (address_index > 0)
		{	
				CYCLES = 30;
				while(CYCLES--)
				{
					gpio_set_value(FPGA_TCK,0);
					for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
					gpio_set_value(FPGA_TCK,1);
					for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);	
				}			
		}
		jtag_ef_prog_write_one_X_address(address_index);

		for (i=0; i<256; i+=4)
		{

			y_page[0] = data_array[i];
			y_page[1] = data_array[i+1];
			y_page[2] = data_array[i+2];
			y_page[3] = data_array[i+3];
		
			jtag_ef_prog_one_Y(y_page);	
 		
			//发送一个Ypage的数据后，需要至少等待12us，最大不超过16us
		    //JTAG_RunTest(30); // 15000/500 = 30
			//CYCLES = 30;
			
			CYCLES = 30;
			while(CYCLES--)
			{
				gpio_set_value(FPGA_TCK,0);
				for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
				gpio_set_value(FPGA_TCK,1);
				for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);								
			}
			
			
			//发送最后一个Ypage的数据后，要至少5us等待数据处理完成。
			if (i==252)
			{
  
					CYCLES = 12;// 6000/500 = 12
					while(CYCLES--)
					{
							gpio_set_value(FPGA_TCK,0);
							for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
							gpio_set_value(FPGA_TCK,1);
							for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);						
					}	
			}
		}
}


//burn internal flash
bool JTAG_ef_prog_gw1n4(bool verify)
{
	  int i =0;
	  int j=0;
	  int page_index = 0;
	  int page_count = 0;

	  uint8_t _tmpbyte_H = 0x00;
	  uint8_t _tmpbyte_L = 0x00;
	
	  uint32_t status_code = 0x00;
	  uint32_t save_array = 0x00;
	  uint8_t data_array[256]={0};
	  u16 SramBuffer[128];

	  page_count = _BYTE_count/256 +1  ;

	  // 发送 配置使能 指令
	  JTAG_WriteInst(ISC_ENABLE);
	  
	  //编程 内部flash 第 1 个以后的所有pages
	  for(page_index=0; page_index<page_count; page_index++)
	  {
	  		_tmp_index = 0;
			SRAM_ReadBuffer(SramBuffer, saveBuffer + 256*(page_index), 128); 
			for(i=0; i < 128; i++)
			{
				_tmpbyte_L = 0xFF & SramBuffer[i];
				_tmpbyte_H = 0xFF &  (SramBuffer[i]>>8);

				data_array[_tmp_index++] = _tmpbyte_H;
				data_array[_tmp_index++] = _tmpbyte_L;
			}
			
			if(page_index == 0)
			{
				//此段为 autoboot 代码，需要最先加入
				data_array[0] = 0x47;
				data_array[1] = 0x57;
				data_array[2] = 0x31;
				data_array[3] = 0x4E;
			}
			jtag_ef_prog_one_X_gw1n4(data_array, page_index);
	  }
		
	   // 退出 配置模式，关闭配置使能 
		JTAG_WriteInst(ISC_DISABLE);
		JTAG_WriteInst(ISC_NOOP);
		JTAG_RunTest(1000);
		 
		JTAG_WriteInst(READ_ID_CODE);
	
	  	// 发送 重配置指令， 确保cfg-mode[2:0]=000 ，	指令完成后设备将读取内部flash进行重加载
		JTAG_WriteInst(REPROGRAM);
		JTAG_WriteInst(ISC_NOOP);
		
		CYCLES=0xffffff;
		while(CYCLES--);


		CYCLES = 20000;
		while(CYCLES--)
		{
			gpio_set_value(FPGA_TCK,0);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
			gpio_set_value(FPGA_TCK,1);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);		
		}					
		
		mdelay(500);
		JTAG_WriteInst(READ_ID_CODE);	
	    JTAG_WriteInst(READ_ID_CODE);	
		status_code =  JTAG_ReadCode(0x41);
		status_code =  JTAG_ReadCode(0x41);

		//printk("====status_code===%x===\n",status_code);
		if (status_code == 0x1F020   ||  status_code == 0x1B020  )
		{
			return true;
		}
		else
		{
			return false;
		}
}



//************************gw1n1****************
void jtag_ef_prog_one_X_gw1n1(uint8_t data_array[256], uint32_t address_index)
{
	    int i=0;
	  	uint8_t y_page[4] = {0};

		JTAG_WriteInst(ISC_ENABLE); 
		JTAG_WriteInst(JTAG_EF_PROGRAM);
		
	   	if (address_index > 0)
		{	
				CYCLES = 50;
				while(CYCLES--)
				{
					gpio_set_value(FPGA_TCK,0);
					for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
					gpio_set_value(FPGA_TCK,1);
					for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);	
				}			
		}
		jtag_ef_prog_write_one_X_address(address_index);

		for (i=0; i<256; i+=4)
		{

			y_page[0] = data_array[i];
			y_page[1] = data_array[i+1];
			y_page[2] = data_array[i+2];
			y_page[3] = data_array[i+3];
		
			jtag_ef_prog_one_Y(y_page);	
		}
			
		CYCLES = 15500;
		while(CYCLES--)
		{
			gpio_set_value(FPGA_TCK,0);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
			gpio_set_value(FPGA_TCK,1);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);								
		}
}


bool JTAG_ef_prog_gw1n1(bool verify)
{
	  int i =0;
	  int j=0;
	  int page_index = 0;
	  int page_count = 0;

	  uint8_t _tmpbyte_H = 0x00;
	  uint8_t _tmpbyte_L = 0x00;

	  uint8_t  all00[256] = {0};
	  uint32_t status_code = 0x00;
	  uint32_t save_array = 0x00;
	  uint8_t data_array[256]={0};
	  u16 SramBuffer[128];

	  page_count = _BYTE_count/256 +1  ;

	  // 发送 配置使能 指令
	  JTAG_WriteInst(READ_ID_CODE);
	  JTAG_WriteInst(ISC_ENABLE);
	  
	  //编程 内部flash 第 1 个以后的所有pages
	  for(page_index=0; page_index<page_count; page_index++)
	  {
	  		_tmp_index = 0;
			SRAM_ReadBuffer(SramBuffer, saveBuffer + 256*(page_index), 128); 
			for(i=0; i < 128; i++)
			{
				_tmpbyte_L = 0xFF & SramBuffer[i];
				_tmpbyte_H = 0xFF &  (SramBuffer[i]>>8);

				data_array[_tmp_index++] = _tmpbyte_H;
				data_array[_tmp_index++] = _tmpbyte_L;
			}
			
			if(page_index == 0)
			{
				//此段为 autoboot 代码，需要最先加入
				data_array[0] = 0x47;
				data_array[1] = 0x57;
				data_array[2] = 0x31;
				data_array[3] = 0x4E;
			}
			jtag_ef_prog_one_X_gw1n1(data_array, page_index);
	  }

	  for (page_index=page_count; page_index < 384 ; page_index++)
	  {
	  		jtag_ef_prog_one_X_gw1n1(all00, page_index);
	  }
	  
	  JTAG_WriteInst(ISC_DISABLE);
	  JTAG_WriteInst(ISC_NOOP);
	  JTAG_RunTest(20000);

	  JTAG_WriteInst(READ_ID_CODE);
	  JTAG_WriteInst(REPROGRAM);
	  JTAG_WriteInst(ISC_NOOP);
	  JTAG_RunTest(10000);

	  CYCLES=0xffffff;
	  while(CYCLES--);

	  CYCLES = 20000;
	  while(CYCLES--)
	  {
			gpio_set_value(FPGA_TCK,0);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);
			gpio_set_value(FPGA_TCK,1);
			for(_tmp_index=0;_tmp_index < DELAY_LEN;_tmp_index++);								
	  }
	  
	  mdelay(500);
	  JTAG_WriteInst(READ_ID_CODE);   
	  JTAG_WriteInst(READ_ID_CODE);   
	  status_code =  JTAG_ReadCode(0x41);
	  status_code =  JTAG_ReadCode(0x41);
	  
	  //printk("====status_code===%x===\n",status_code);
	  if (status_code == 0x1F020   ||  status_code == 0x1B020  )
	  {
		  return true;
	  }
	  else
	  {
		  return false;
	  }

	   
}


//***********д��ram***************
void JTAG_Reset(void)
{
	JTAG_MoveTap(TAP_UNKNOWN, TAP_IDLE);
}

bool JTGA_Prog_sram(void)
{
		int i=0;
		int page_index =0;
		int page_count = 0;
		uint8_t _tmpbyte_H = 0x00;
	  	uint8_t _tmpbyte_L = 0x00;
		uint32_t status_code = 0x00;	
		u16 SramBuffer[128];
		page_count = _BYTE_count/256 +1;
		
		JTAG_Reset();
	 	//send config Enable inst
		JTAG_WriteInst(ISC_ENABLE);
		//send Write SRAM inst
		JTAG_WriteInst(FAST_PROGRAM);
		JTAG_MoveTap(TAP_IDLE, TAP_DRSHIFT);

		for (page_index=0; page_index<page_count; page_index++)
		{
			SRAM_ReadBuffer(SramBuffer, saveBuffer + 256*(page_index), 128);
			for(i=0;i<128;i++)
			{
				//write to fpga
				_tmpbyte_L = 0xFF &  SramBuffer[i];
				_tmpbyte_H = 0xFF &  (SramBuffer[i]>>8);

				JTAG_Write( _tmpbyte_H, 0, 0, false);
				JTAG_Write( _tmpbyte_L, 0, 0, false);
			}
		}
		 //write 0xff and move TAP to DR-EXIT1
		 JTAG_Write(0xFF, 0x0, 0x1, false);
		 //TAP move from TAP_DREXIT1 to TAP_IDLE 
		JTAG_MoveTap(TAP_DREXIT1,  TAP_IDLE);
		//send config disable inst
		JTAG_WriteInst(ISC_DISABLE);
		 //send Noop AND play over.		
		JTAG_WriteInst(ISC_NOOP);
		status_code =  JTAG_ReadCode(0x41);
		printk("=====status_code[%X]===\n",status_code);
		if(status_code ==  0x1F020   ||  status_code == 0x1B020 || status_code == 0x1E020 )		
				return true;
		else
				return false;
 	
}


//*********************************************


int fpga_open(struct inode *inode, struct file *file)
{
	if(first_configgpio == 0)
	{
		 gpio_request(FPGA_TCK, "jtag_tck");
		 gpio_direction_output(FPGA_TCK, 1);

		 gpio_request(FPGA_TMS, "jtag_tms");
		 gpio_direction_output(FPGA_TMS, 1);

		 gpio_request(FPGA_TDI, "jtag_tdi");
		 gpio_direction_output(FPGA_TDI, 1);

		 gpio_request(FPGA_TDO, "jtag_tdo");
		 gpio_direction_input(FPGA_TDO);
		 first_configgpio = 1;
	}

	if (!fpga_buf) 	fpga_buf = vmalloc(FPGA_MAX_LEN);	
	if (!fpga_buf)	return -ENOMEM;

	fpga_len = 0;
	return 0;
}



int fpga_release(struct inode *inode, struct file *file)
{
	if (fpga_buf) vfree(fpga_buf);
	fpga_buf = NULL;
	fpga_len = 0;
	
	return 0;
}


ssize_t fpga_read (struct file *file, char __user *data, size_t len, loff_t * ppos)
{
	return 0;	
}

ssize_t fpga_write (struct file *file, char __user *data, size_t len, loff_t * ppos)
{
	if (!fpga_buf) return -ENOSPC;	
	if ((fpga_len + len ) >  FPGA_MAX_LEN ) return ENOSPC;
	if (copy_from_user(fpga_buf+fpga_len, data, len))
	{
		return -EFAULT;
	}
	fpga_len += len;
	return len;
}

long fpga_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	int i=0;
	bool res = false;
	switch(cmd)
	{
		case DOWNFPGA:
		{
			JTAG_MoveTap(TAP_UNKNOWN, TAP_IDLE );

			for (i=0; i<4; i++)
			{
				uint32_t _code = JTAG_ReadCode(READ_ID_CODE);
				if(_code == ID_GW1N4 || _code == ID_GW1N1)
				{
						gw_type_code = _code;
						break;
				}
				mdelay(200);
			}
			
			if(gw_type_code == ID_GW1N4 && fpga_len > 0)
			{
				for(i=0;i<10;i++)
				{
					res = JTGA_ef_erase_gw1n4();
					if(res == true)	break;
				}
				if(res)
				{
					format_fs_data();
			
					local_irq_disable();
					if (JTAG_ef_prog_gw1n4( false ))
					{
						ret = 1;
						//printk("===success====\n");
					}
					else
					{
						ret = 0; //write error
						printk("==write==failed====\n");
					}
					local_irq_enable();
				}
				else
				{
					printk("====erase error====\n");
					ret = 0; //erase error
				}
			}
			else if(gw_type_code == ID_GW1N1 && fpga_len > 0)
			{
				#if 0 //�ڲ�flash
				for(i=0;i<10;i++)
				{
					res = JTGA_ef_erase_gw1n1();
					if(res == true)	break;
				}
				if(res)
				{
					format_fs_data();
					local_irq_disable();
					if (JTAG_ef_prog_gw1n1( false ))
					{
						ret = 1;
						//printk("===success====\n");
					}
					else
					{
						ret = 0; //write error
						printk("==write==failed====\n");
					}
					local_irq_enable();
				}
				#else //�����ڲ�flash������Ѳ�����ֱ���˳� 
				for(i=0;i<5;i++)
				{
					res = JTGA_ef_erase_gw1n1();
					if(res == true)	break;
				}

				format_fs_data();
				for(i=0;i< 5;i++)
				{
					res = JTGA_Prog_sram();
					if(res == true)		break;
					else
					{
		 					//����
		 					//i2c
		            		jz_gpio_set_func(GPIO_PC(26),GPIO_OUTPUT0);
		            		jz_gpio_set_func(GPIO_PC(27),GPIO_OUTPUT0);
							//i2s
		            		jz_gpio_set_func(GPIO_PB(1),GPIO_OUTPUT0);
		            		jz_gpio_set_func(GPIO_PB(2),GPIO_OUTPUT0);
		            		jz_gpio_set_func(GPIO_PB(4),GPIO_OUTPUT0);
							//pcm
		            		jz_gpio_set_func(GPIO_PC(6),GPIO_OUTPUT0);
		            		jz_gpio_set_func(GPIO_PC(7),GPIO_OUTPUT0);
		            		jz_gpio_set_func(GPIO_PC(9),GPIO_OUTPUT0);

							gpio_direction_output(GPIO_PB(05),0);
		 					msleep(200);
		 					//�ϵ�
		 					//i2c
		 					jz_gpio_set_func(GPIO_PC(26),GPIO_FUNC_0);
		 			 		jz_gpio_set_func(GPIO_PC(27),GPIO_FUNC_0);
		 			 		//i2s
		 			 		jz_gpio_set_func(GPIO_PB(1),GPIO_FUNC_1);
		 			 		jz_gpio_set_func(GPIO_PB(2),GPIO_FUNC_1);
		 			 		jz_gpio_set_func(GPIO_PB(4),GPIO_FUNC_1);
		 			 		//pcm
		 			 		jz_gpio_set_func(GPIO_PC(6),GPIO_FUNC_0);
		 			 		jz_gpio_set_func(GPIO_PC(7),GPIO_FUNC_0);
		 			 		jz_gpio_set_func(GPIO_PC(9),GPIO_FUNC_0);
		 					gpio_direction_output(GPIO_PB(05),1);
		 					msleep(20);
					}
				}
				if(res == true)		ret = 1;
				else				ret = 0;
				
				#if 0
				if(JTGA_Prog_sram())
				{
					ret = 1;
				}
				else
				{
					if(JTGA_Prog_sram())
					{
						ret = 1;
					}
					else
					{
						ret = 0;
					}
				}
				#endif
				printk("======fpga=ret[%d]===\n",ret);
				#endif
			}
			else
			{
				printk("===read type error===\n");
				ret = 0;  //fpga type error 
			}
		}
		break;
		default:
		ret = 0;
		printk("=====error cmd===\n");
		break;
	}
	return ret;
}




static const struct file_operations miscfpga_fops =
{
    .owner   =   THIS_MODULE,
    .open    =   fpga_open,
    .read     =  fpga_read,
    .write    =  fpga_write,
    .release  = fpga_release,
    .unlocked_ioctl = fpga_ioctl,
};



static struct miscdevice miscfpga_dev =
{
 .minor = MISC_DYNAMIC_MINOR,
 .name = MISC_NAME,
 .fops = &miscfpga_fops,
};


static int __init fpgadown_init(void)
{
	 int ret;
	
	 ret = misc_register(&miscfpga_dev);
	 if (ret)
	 {
 	 	printk("*****fpgadown init error*****\n");
  		return ret;
	 }
 	return 0;
}

static void __exit fpgadown_exit(void)
{
	misc_deregister(&miscfpga_dev);
}



module_init(fpgadown_init);
module_exit(fpgadown_exit);
MODULE_LICENSE("GPL");

