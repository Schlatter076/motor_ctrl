
/******************************************/
/*           广州旭东阪田电子有限公司     */        
/*Project:      FCT测试主板               */
/*Guest:                                  */
/*Name:             delay.c               */
/*Mcu chip:         Atmega64              */
/*Main clock:       内部晶体8MHz          */
/*Rtc clock:                              */
/*Author:           Jack.Fu               */
/*Create date:      2008.11.20            */
/*Design date:                            */
/*Complete date:                          */
/******************************************/

/*-----------------------------------------------------------------------
延时函数
系统时钟：8M内部，外部11.0592M
-----------------------------------------------------------------------*/
#define xtal 11

void delay_1us(void)                 //1us延时函数
{
	asm("nop");
}

void delay_nus(unsigned int n)       //N us延时函数
{
	unsigned int i=0;
	for (i=0;i<n;i++)
	delay_1us();
}
  
void delay_1ms(void) 				 //1ms延时函数
{
	unsigned int i;
	for(i=0;i<(unsigned int)(xtal*143-2);i++)
	{
		;
	}
}

void delay_nms(unsigned int n)		 //N ms延时函数				 
{
	unsigned int i;
	for(i=0;i<n;i++)
	{
		delay_1ms();
	}
}