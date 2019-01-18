/******************************************/
/*           广州旭东阪田电子有限公司     */
/*Project:     精准定位控制程序           */
/*Guest:                                  */
/*Name:             main.c                */
/*Mcu chip:         Atmega64              */
/*Main clock:       外部晶体11.0592MHz    */
/*Rtc clock:                              */
/*Author:           Loyer                 */
/*Create date:      2019.01.16            */
/*Design date:                            */
/*Complete date:                          */
/******************************************/
#include <iom64v.h>
#include <stdio.h>
#include <macros.h>
#include <port.h>
#include <default.h>
#include <delay.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

//==按键变量定义=============================
uchar key_now = 0;
uchar key_old = 0;
uchar key_code = 0;
uchar key_cnt = 0;
#define KEY_COUNTER 10
//==命令格式及脉冲个数变量定义===============
/**
 * 其中command[9]为命令类型字节
 * command[2]-command[5]返回脉冲个数，仅当command[9]为0x03时
 */
uchar command[] = {0xf3, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0a};
uchar debugCommand[] = {0xf3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0a};
INT32 stroke_Val = 0;  //行程脉冲个数
//把脉冲个数拆分成4个字节，赋值给command
void fill_paulse(INT32 val) 
{
    *(command+2) = (val >> 24) & 0xff; //取24-31位值
    *(command+3) = (val >> 16) & 0xff; //取16-23位值
    *(command+4) = (val >> 8) & 0xff;  //取8-15位值
    *(command+5) = val & 0xff;         //取0-7位值
}
//发送调试好的位置值到上位机
void fill_position(INT32 Y_val, INT32 Z_val)
{
    *(debugCommand+1) = (Y_val >> 24) & 0xff; //取24-31位值
		*(debugCommand+2) = (Y_val >> 16) & 0xff; //取16-23位值
    *(debugCommand+3) = (Y_val >> 8) & 0xff; //取8-15位值
    *(debugCommand+4) = Y_val & 0xff;  //取0-7位值
		*(debugCommand+5) = (Z_val >> 24) & 0xff; //取24-31位值
		*(debugCommand+6) = (Z_val >> 16) & 0xff; //取16-23位值
    *(debugCommand+7) = (Z_val >> 8) & 0xff; //取8-15位值
    *(debugCommand+8) = Z_val & 0xff;  //取0-7位值
}
//定义结构体存放位置坐标
typedef struct motorPosition
{
    INT32 Y;
		INT32 Z;
}Position;

Position last_Pos = {0, 0};
Position now_Pos = {0, 0};
INT32 Y_paulse = 0, Y_runPaulse = 0;
INT32 Z_paulse = 0, Z_runPaulse = 0;
char* Y_allowRun = "false";
char* Z_allowRun = "false";
char* IO_status = "common";
uchar isPullAlarm = FALSE; //拉力报警标志位
uchar isConductive = FALSE;
uchar isDebug = FALSE;
uchar MOTOR_SPEED = 1;
uchar T_count = 0;

//电机运行函数
void motor_run(char* Y_Direction, INT32 Y_PaulseNum, char* Z_Direction, INT32 Z_PaulseNum, uchar speed)
{
		    if(Y_PaulseNum > 0) 
			{
			    if(Y_Direction == "Y_forward") 
				{
				    Y_forward();
					if(Y_arrivalsLimit()) 
					{
					    delay_nms(5);
						if(Y_arrivalsLimit())
						{
						    Y_runPaulse = 0;
					        Y_allowRun = "false";
							Y_disable();
						}
					}
					else
					{
					    Y_runPaulse = Y_PaulseNum;
						Y_allowRun = "true";
						Y_enable();
					}
				}
				else
				{
				    Y_backward();
					if(Y_arrivalsOrigin())
					{
					    delay_nms(5);
						if(Y_arrivalsOrigin())
						{
						    Y_runPaulse = 0;
							Y_allowRun = "false";
							Y_disable();
						}
					}
					else
					{
					    Y_runPaulse = Y_PaulseNum;
						Y_allowRun = "true";
						Y_enable();
					}
				}
			}
			else
			{
			    Y_allowRun = "false";
			}
			//////////////////////////////////////////////////////////////////
			if(Z_PaulseNum > 0) 
			{
			    if(Z_Direction == "Z_forward") 
				{
				    Z_forward();
					if(Z_arrivalsLimit()) //拉力报警
					{
					    isPullAlarm = TRUE;
					}
					Z_runPaulse = Y_PaulseNum;
				    Z_allowRun = "true";
				    Z_enable();
				}
				else
				{
				    Z_backward();
					if(Z_arrivalsOrigin())
					{
					    delay_nms(5);
						if(Z_arrivalsOrigin())
						{
						    Z_runPaulse = 0;
							Z_allowRun = "false";
						}
					}
					else
					{
					    Z_runPaulse = Z_PaulseNum;
						Z_allowRun = "true";
						Z_enable();
					}
				}
			}
			else
			{
			    Z_allowRun = "false";
			}
			///////////////////////////////////////////////////////
			if(Y_PaulseNum > 0 || Z_PaulseNum > 0)
			{
			    MOTOR_SPEED = speed;
				Timer1_enable();
			}
			else 
			{
			    Timer1_disable();
			}
}
/*============================================================*/	
/***********USART0接收中断服务函数 start**********************/ 
//USART接收缓冲区
#define RX_BUFFER_SIZE 11                  //接收缓冲区大小，可根据需要修改。
unsigned char rx_buffer[RX_BUFFER_SIZE];   //定义接收缓冲区
unsigned char rx_counter=0;                //定义rx_counter为存放在队列中的已接收到字符个数。

//定义一个标志位Usart0_RECVFlag1:=1表示串口0接收到了一个完整的数据包
//在port.h中定义

#pragma interrupt_handler usart0_rxc_isr:19  //接收中断服务程序
void usart0_rxc_isr(void)
{
   uchar status,data;
   status=UCSR0A;
   data=UDR0;
	if((flag1&(1<<Usart0_RECVFlag1))==0)   //判断是否允许接收一个新的数据包
	{
		if ((status & (USART0_FRAMING_ERROR | USART0_PARITY_ERROR | USART0_DATA_OVERRUN))==0)
        {
            rx_buffer[rx_counter] = data;
            rx_counter++;
			    switch (rx_counter)
            {
                case 1: {     // 检验起始字符
							if (data != FIRST_TEXT) rx_counter = 0;
						}break;
				    case 2: {    
							if (data != SECOND_TEXT) rx_counter = 0;
						}break;
				    case 10: {    
							if(data == 0x02) IO_status = "conductive";
							else if(data == 0x20) IO_status = "debug";
							else if(data == 0x22) IO_status = "IO_CTRL";
							else if(data == 0x0d) IO_status = "IO_READ";
							else IO_status = "common";
						}break;
                case 11: {    // 检验结束字符
							rx_counter = 0;
							if (data == END_TEXT) set_bit(flag1,Usart0_RECVFlag1);// Usart0_RecvFlag=1，表示正确接收到一个数据包
						}break;   
				    default:break;
            }
        }
	}
}
/***************USART0接收中断服务函数 end**********************/ 
/*============================================================*/
/*============================================================*/	
/***************USART0发送中断服务函数 start********************/ 
#define TX_BUFFER_SIZE 11    
unsigned char tx_buffer[TX_BUFFER_SIZE];  
unsigned char tx_wr_index=0,tx_rd_index=0,tx_counter=0;

#pragma interrupt_handler usart0_txc_isr:21  //发送中断服务程序
void usart0_txc_isr(void)
{
    if (tx_counter)//队列不为空
    {
        --tx_counter;//出队列
        UDR0=tx_buffer[tx_rd_index];
        if (++tx_rd_index == TX_BUFFER_SIZE) tx_rd_index=0;
    }
}
/***********USART0发送中断服务函数 end**********************/ 

/*============================================================*/
/***********USART0发送一个字符函数 start**********************/ 
void USART0_putchar(unsigned char c)
{
    while (tx_counter == TX_BUFFER_SIZE);
    CLI();//#asm("cli")关闭全局中断允许
    if (tx_counter || ((UCSR0A & USART0_DATA_REGISTER_EMPTY)==0))//发送缓冲器不为空
    {
        tx_buffer[tx_wr_index]=c;//数据进入队列
        if (++tx_wr_index == TX_BUFFER_SIZE) tx_wr_index=0;//队列已满
        ++tx_counter;
    }
    else
        UDR0=c;
    SEI(); //#asm("sei")打开全局中断允许
}
/***********USART0发送服务函数 end**********************/ 
//发送行程值到主机
void sendStrokeToHost(void)
{
    uchar cnt = 0;
		fill_paulse(stroke_Val); //把行程值打包发回
		stroke_Val = 0; //清零行程值
		for(cnt = 0; cnt < TX_BUFFER_SIZE; cnt++)
		{
		    USART0_putchar(command[cnt]);
		}
}
//发送调试好的坐标值给上位机
void sendDebugPosition(void)
{
    uchar cnt = 0;
		fill_position(Y_paulse, Z_paulse);
		for(cnt = 0; cnt < TX_BUFFER_SIZE; cnt++)
		{
		    USART0_putchar(debugCommand[cnt]);
		}
}
uchar iostream[] = {0xf3, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x0a};
//将端口状态返回给上位机
void sendIOstatu(void)
{
    uchar cnt = 0;
		*(iostream+2) = PINA;
		*(iostream+3) = PINB;
		*(iostream+4) = PINC;
		*(iostream+5) = PIND;
		*(iostream+6) = PINE;
		*(iostream+7) = PINF;
		*(iostream+8) = PING;
		for(cnt = 0; cnt < TX_BUFFER_SIZE; cnt++)
		{
		    USART0_putchar(iostream[cnt]);
		}
}
//端口控制
void IO_contrl(void)
{
    uchar PA_var = rx_buffer[2];
		uchar PB_var = rx_buffer[3];
		uchar PC_var = rx_buffer[4];
		uchar PD_var = rx_buffer[5];
		uchar PE_var = rx_buffer[6];
		uchar PF_var = rx_buffer[7];
		uchar PG_var = rx_buffer[8];
		uchar bit_count = 0;
		//输出控制开始
		for(bit_count = 0; bit_count < 8; bit_count++)
		{
			if((PA_var & (1 << bit_count)) != 0)
			{
			    set_bit(PORTA, bit_count);
			}
			else
			{
			    clr_bit(PORTA, bit_count);
			}
			if((PB_var & (1 << bit_count)) != 0)
			{
			    if(bit_count != 5 && bit_count != 6)
				    set_bit(PORTB, bit_count);
			}
			else
			{
			    if(bit_count != 5 && bit_count != 6)
				    clr_bit(PORTB, bit_count);
			}
			if((PC_var & (1 << bit_count)) != 0)
			{
			    if(bit_count > 3)
				    set_bit(PORTC, bit_count);
			}
			else
			{
			    if(bit_count > 3)
				    clr_bit(PORTC, bit_count);
			}
			if((PE_var & (1 << bit_count)) != 0)
			{
			    if(bit_count != 0 && bit_count != 1 && bit_count != 4 && bit_count != 5)
				{
				    set_bit(PORTE, bit_count);
				}
			}
			else
			{
			    if(bit_count != 0 && bit_count != 1 && bit_count != 4 && bit_count != 5)
				{
				    clr_bit(PORTE, bit_count);
				}
			}//*/
			if((PF_var & (1 << bit_count)) != 0)
			{
			    set_bit(PORTF, bit_count);
			}
			else
			{
			    clr_bit(PORTF, bit_count);
			}
			if((PG_var & (1 << bit_count)) != 0 && bit_count < 5)
			{
			    set_bit(PORTG, bit_count);
			}
			else
			{
			    if(bit_count < 5)
				{
				    clr_bit(PORTG, bit_count);
				}
			}
		}//输出控制结束
}
//定时器0初始化
void init_TIMER0_OVF(void) 
{
   TCCR0 = 0x04; //64分频
	TCNT0 = 256 - CRYSTAL/64/2*0.05;  //0.05s定时
	TIMSK |= (1<<TOIE0);  //定时器0中断使能
	SREG = 0x80;
}
char* sendPos = "false";
#pragma interrupt_handler timer0_isr:17
void timer0_isr(void)
{
	TCNT0 = 256 - CRYSTAL/64/2*0.0002;  //重装0.0002s定时
	if((flag1 & (1 << Usart0_RECVFlag1)) != 0) //收到主机发来的数据
	{
		   clr_bit(flag1, Usart0_RECVFlag1);
		   if(IO_status == "conductive") 
		   {
		       isDebug = FALSE;
			   isConductive = TRUE;
		   }
		   else if(IO_status == "debug")
		   {
		       Timer1_disable();
			   isDebug = TRUE;
			   rx_buffer[2] == 0x01?Y_forward():Y_backward();
			   rx_buffer[3] == 0x01?Z_forward():Z_backward();
			   if(rx_buffer[4] == 0x01) {Y_allowRun = "true"; Y_enable();}
			   else {Y_allowRun = "false"; Y_disable();}
			   if(rx_buffer[5] == 0x01) {Z_allowRun = "true"; Z_enable();}
			   else {Z_allowRun = "false"; Z_disable();}
			   rx_buffer[6] == 0x01?(sendPos = "true"):(sendPos = "false");
		   }
		   else if(IO_status == "IO_CTRL")
		   {
		       isDebug = FALSE;
			   IO_contrl();
		   }
		   else if(IO_status == "IO_READ")
		   {
		       isDebug = FALSE;
			   TIMSK&=~(1<<TOIE0);
		   	   sendIOstatu();
		   	   TIMSK|=(1<<TOIE0);
		   }
		   else 
		   {
		       isDebug = FALSE;
		   }
	}
	if(isDebug == TRUE)
	{
	    if(Y_allowRun == "true")
		   {
			   Y_RUN();
			   Y_paulse++;
		   }
		   if(Z_allowRun == "true")
		   {
			   Z_RUN();
			   LED_BLINK();
			   Z_paulse++;
		   }
		   else
		   {
		       set_bit(PORTD, PD6); //防止指示灯一直闪
		   }
	}
	if(sendPos == "true")
	{
	    sendPos = "false";
		   TIMSK&=~(1<<TOIE0);
		   sendDebugPosition();
		   if(Y_allowRun != "true") Y_paulse = 0;
		   if(Z_allowRun != "true") Z_paulse = 0;
		   TIMSK|=(1<<TOIE0);
	}
}
//定时中断函数(自动检测用) 
#pragma interrupt_handler timer1_count_isr:15  
void timer1_count_isr(void) 
{ 
	uchar timer_statu = 0;
	TCNT1=65536-CRYSTAL/8/2*0.0002;  //重装0.0002s定时
	if(++T_count != MOTOR_SPEED) return;
	T_count = 0;
	if(Z_allowRun == "true" && Y_allowRun == "false")
	{
	    Z_RUN();
		   Z_runPaulse--;
		   LED_BLINK();
		   if(Z_arrivalsLimit()) //到达拉力报警位置
	    {
	        isPullAlarm = TRUE;
		   }
		   if(isPullAlarm == TRUE)
		   {
	        stroke_Val++; //从拉力报警处计数脉冲个数
			   MOTOR_SPEED = 2;
		   }
		   if(isConductive == TRUE) //按键已导通
		   {
		       isPullAlarm = FALSE;
			   Z_allowRun = "false";
			   timer_statu = TIMSK;
			   Timer1_disable();
			   sendStrokeToHost();
			   TIMSK = timer_statu;
			   isConductive = FALSE;
		   }
	} else set_bit(PORTD, PD6); //防止指示灯一直闪
	if(Y_allowRun == "true")
	{
	    if(Y_runPaulse > 0 && !Y_arrivalsLimit() && !Y_arrivalsOrigin())
		   {
		       Y_RUN();
			   Y_runPaulse--;
		   }
		   else
		   {
		       Y_allowRun = "false";
		   }
	}
	////////////////////////////////////////////////////////////
}
//定时器1初始化
void init_TIMER1_OVF(void)
{
	TCCR1B=0x02;  //8分频
	TCNT1=65536-CRYSTAL/8/2*0.05;//0.05s定时
	//TIMSK|=(1<<TOIE1);  //定时器使能
	Timer1_disable();
	//SREG=0x80;      //全局使能中断
}
/***************USART01初始化函数 start*************************/ 
void init_usart0(void)
{
    UCSR0B = 0x00; 
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);        //异步，8位数据，无奇偶校验，一个停止位，无倍速 
    UBRR0L = BAUD_L;                        //设定波特率 
    UBRR0H = BAUD_H; 
    UCSR0A = 0x00; 
    UCSR0B = (1<<RXCIE0)|(1<<TXCIE0)|(1<<RXEN0)|(1<<TXEN0); //0XD8  接收、发送使能， 开中断使能
}
/***************USART0初始化函数 end***************************/
/***************系统初始化函数 start ***********/
void init_cpu(void)
{
    EIMSK = 0x00; //屏蔽INT0~INT1的所有外部中断
    clr_bit(SFIOR,PUD); //设置输入上拉电阻有效
		
    DDRA=0xff;//1是输出，0是输入
    PORTA=0x00; //控制PINC与PINF通路切换
	
    DDRB=0xff;  //1是输出，0是输入
    PORTB =0x60; //PB5/6脉冲脚上拉
	
    DDRC=0xff; 
    PORTC =0x00; 

    DDRD  = 0x4c; //PIND6为输出，其余输入
		PORTD = 0xff;  //外接上拉，按键带上拉
		DDRE =0xce;  //RXD0输入，且上拉有效,PE4/5输入
		PORTE =0x31;  //TXD0输出
	
		DDRF=0xff;  //PINF全是输出
		PORTF =0x00;     
	
		DDRG =0xff;  
		PORTG =0x00;  //PING口全输出
	
		init_usart0();
		init_TIMER0_OVF();
		init_TIMER1_OVF();
		SEI();
		
    flag1=0;
    flag2=0;
    flag3=0;
    flagerr=0;
}
/***************系统初始化函数 end ***********/
//按键处理函数===============================================
void key_scan(void)
{
    if((flag1 && (1 << keyprq_flag1)) == 0)  //如果没有按键按下
	{
	    if((PIND & (1<<key1)) == 0)  //启动测试按键
		{
		    key_now = 1;  
		}
		else if((PIND & (1<<key2)) == 0)  //急停按键
		{
		    key_now = 2;  
		}
		else
		{
		    key_now = 0;
			key_old = 0;
			key_code = 0;
		}
		if(key_now != 0)
		{
		    if(key_now != key_code)
			{
			    key_code = key_now;
				key_cnt = 0;
			}
			else
			{
			    key_cnt++;
				if(key_cnt >= KEY_COUNTER)
				{
				    set_bit(flag1, keyprq_flag1);
				}
			}
		}
	}
}
//按键处理函数===============================================
void key_process(void)
{
    if((flag1 & (1 << keyprq_flag1)) != 0)
	{
	    clr_bit(flag1, keyprq_flag1);
		if(key_code == key_old)
		{
		    ; //do nothing~
		}
		else
		{
		    key_old = key_code;
			set_bit(flag1, keyeff_flag1);  //按键有效
		}
		if((flag1 & (1 << keyeff_flag1)) != 0)
		{
		    clr_bit(flag1, keyeff_flag1);
			switch(key_old)
			{
			    case 1:  //启动测试按键按下
				{
				    motor_run("Y_forward", 68745, "Z_forward", 68745965, 1);
				}break;
				case 2:  
				{
				    
				}break;
				default: break;
			}
		}
	}
}
/***************主函数 start *******************/
void main(void)
{
   init_cpu();    //初始化CPU
	while(1)
    {
		key_scan();
		key_process();
		delay_nms(10);
	}
}


