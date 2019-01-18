
/******************************************/
/*           广州旭东阪田电子有限公司     */        
/*Project:      FCT测试主板               */
/*Guest:                                  */
/*Name:             delay.h               */
/*Mcu chip:         Atmega64              */
/*Main clock:       内部晶体8MHz          */
/*Rtc clock:                              */
/*Author:           Jack.Fu               */
/*Create date:      2008.11.20            */
/*Design date:                            */
/*Complete date:                          */
/******************************************/

#ifndef _DELAY_H
#define _DELAY_H

void delay_1us(void);
void delay_1ms(void);    
void delay_nus(unsigned int n);
void delay_nms(unsigned int n);

#endif