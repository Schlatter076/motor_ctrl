
/******************************************/
/*           广州旭东阪田电子有限公司     */
/*Project:      FCT测试主板               */
/*Guest:                                  */
/*Name:             port.h                */
/*Mcu chip:         Atmega64              */
/*Main clock:       外部晶体11.0592MHz    */
/*Rtc clock:                              */
/*Author:           Jack.Fu               */
/*Create date:      2008.11.20            */
/*Design date:                            */
/*Complete date:                          */
/******************************************/

#ifndef _PORT_H
#define _PORT_H


/*================================================*/
/************定义MCU I/O口 start*******************/
//PORTA
#define LCD_data PORTA
#define PA0 0
#define PA1 1
#define PA2 2
#define PA3 3
#define PA4 4
#define PA5 5
#define PA6 6
#define PA7 7

//PORTB 
#define PB0 0
#define PB1 1
#define PB2 2
#define PB3 3
#define PB4 4
#define PB5 5
#define PB6 6
#define PB7 7

//PORTC
#define PC0 0
#define PC1 1
#define PC2 2
#define PC3 3
#define PC4 4
#define PC5 5
#define PC6 6
#define PC7 7 

//PORTD
#define PD0  0
#define PD1  1
#define RXD1     2  //RXD输入，且上拉有效
#define TXD1     3  //TXD输出
#define key1 4  //自动测试开始的按键,输入
#define key2 5  //急停按键，输入
#define bell 6  //蜂鸣器控制，pnp管，低电平有效。  输出
/////////////
#define PD2 2
#define PD3 3
#define PD4 4
#define PD5 5
#define PD6 6
//////////////
#define PD7  7
 
//PORTE
#define RXD0     0  //RXD输入，且上拉有效    
#define TXD0     1  //TXD输出
#define LCD_RS   2
#define LCD_EN   3  //LCD的控制口，输出
/////////////
#define PE2 2
#define PE3 3
//////////////
#define PE4  4
#define PE5  5
#define PE6  6
#define PE7  7
//PORTF
#define PF0 0
#define PF1 1
#define PF2 2
#define PF3 3
#define PF4 4
#define PF5 5
#define PF6 6
#define PF7 7

//PORTG
#define PG0 0
#define PG1 1
#define PG2 2
#define PG3 3
#define PG4 4

/************定义MCU I/O口 end*********************/
/************定义MCU Flag标志位 start**************/
#pragma global_register flag1:20 flag2:21 flag3:22 flagerr:23
unsigned char flag1;
#define keyprq_flag1     0	//有键按下标志位
#define keyeff_flag1     1	//按键有效标志位
#define Usart0_RECVFlag1 2 //=1表示串口0接收到了一个完整的数据包
#define Usart1_RECVFlag1 3 //=1表示串口1接收到了一个完整的数据包
#define error_flag1      4   //通信错误标志位
#define AD0_RECVFlag1    5 //=1表示ADC0接收到了一个完整的数据包
#define time_goFlag1     6
#define recv_okFlag1     7

unsigned char flag2;
#define Master_RECVFlag2 0 //SPI主机接收完整数据包
#define Slave_RECVFlag2 1 //SPI从机接收完整数据包

unsigned char flag3;
unsigned char flagerr;


/************定义MCU Flag标志位 end****************/

#endif

