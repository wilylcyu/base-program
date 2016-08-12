#include "stm32f10x.h"
#include "usart.h"
#include <stdio.h>
#include "encoder.h"
#include "contact.h"
#include "stdbool.h"
#include "UltrasonicWave.h"
#include "gpio_conf.h"
#include "tim2_5_6.h"
#include "delay.h"
#include "nvic_conf.h"
#include "rcc_conf.h"
#include "stm32f10x_it.h"
#include "odometry.h"
#include "string.h"
#include "math.h"


#define COMMAND_WRONG 0x00;
#define COMMAND_CORRECT 0X01;

char odometry_data[13]={0};   //发送的里程计数据数组

extern u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
extern u16 USART_RX_STA;       //接收状态标记	

union recieveData  
{
	float d;
	unsigned char data[4];
}leftdata,rightdata;       //接收到的数据

float odometry_right=0,odometry_left=0;

union odometry
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data;     //要发布的里程计数据

extern float position_x,position_y,oriention;         //计算得到的里程计数值

extern float Milemeter_L_Motor,Milemeter_R_Motor;     //左右轮的dt时间内的里程计数据

u8 reset_odom=0;   //复位里程计

u8 main_sta=0; //用作处理主函数各种if，去掉多余的flag

void GPIO_Configuration(void);
void TIM2_PWM_Init(void);
void NVIC_Configuration(void);
void TIM5_Configuration(void);
void TIM6_Configuration(void);
void TIM1_Configuration(void);
	
int main(void)
{		
	u8 t=0;
	u8 i=0,j=0,k=0,m=0;
	
	TIM2_PWM_Init();	        //控制电机
	ENC_Init2();              //设置电机A TIM3编码器模式PB6 PB7
	ENC_Init1();              //设置电机B TIM3编码器模式PA6 PA7 左电机
	TIM5_Configuration();     //TIM5的中断对陀螺仪有影响！！！
	TIM1_Configuration();     //TIM5的中断对陀螺仪有影响！！！
	TIM6_Configuration();     //超声的定时
	RCC_Configuration();      //系统时钟配置		
	NVIC_Configuration();     //中断优先级配置
	GPIO_Configuration(); 
	UltrasonicWave_Configuration();	
	USART1_Config();	
	USART2_Config();
 // EXTIX_Init();             //听声辩位，声音接收外部中断初始化
 // IWDG_Init(4,625);         //独立看门狗初始化
	Systick_Init();           //系统滴答定时器初始化
        

	while (1)
	{
		//Set_CarStraight(1200,628);		
		if(main_sta&0x01)   //发送里程计数据
		{
			
			x_data.odoemtry_float=position_x;
			y_data.odoemtry_float=position_y;
			theta_data.odoemtry_float=oriention;
			for(j=0;j<4;j++)
			{
				odometry_data[j]=x_data.odometry_char[j];
				odometry_data[j+4]=y_data.odometry_char[j];
				odometry_data[j+8]=theta_data.odometry_char[j];
			}
			odometry_data[12]='\n';
			
			for(i=0;i<13;i++)
			{
				USART_ClearFlag(USART1,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题				
				USART_SendData(USART1,odometry_data[i]);	
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);				
			}
			main_sta&=0xFE;
		}
		if(main_sta&0x02)      //调用计算里程计数据函数
		{
			odometry(Milemeter_L_Motor,Milemeter_R_Motor);
			main_sta&=0xFD;
		} 
		if(main_sta&0x04)     //当发送指令正确接收
		{
				USART_ClearFlag(USART1,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题
				for(k=0;k<3;k++)
				{
					USART_SendData(USART1,0x01);	
					while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
				}					
				USART_SendData(USART1,'\n');	
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);		
				main_sta&=0xFB;
		}
		if(main_sta&0x08)      //当发送指令没有正确接收时
		{
			  USART_ClearFlag(USART1,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题
				for(m=0;m<3;m++)
				{
					USART_SendData(USART1,0x00);	
					while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
				}		
				USART_SendData(USART1,'\n');	
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	
				main_sta&=0xF7;
		}
		if(USART_RX_STA&0x8000)
		{					   
				for(t=0;t<4;t++)
				{
					rightdata.data[t]=USART_RX_BUF[t];
					leftdata.data[t]=USART_RX_BUF[t+4];
				}
				odometry_right=rightdata.d;
				odometry_left=leftdata.d;

			USART_RX_STA=0;
		}
	car_control(rightdata.d,leftdata.d);
		
	}//end_while
}//end main
/*********************************************END OF FILE**********************/
