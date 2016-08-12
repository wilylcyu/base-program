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

char odometry_data[13]={0};   //���͵���̼���������

extern u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
extern u16 USART_RX_STA;       //����״̬���	

union recieveData  
{
	float d;
	unsigned char data[4];
}leftdata,rightdata;       //���յ�������

float odometry_right=0,odometry_left=0;

union odometry
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data;     //Ҫ��������̼�����

extern float position_x,position_y,oriention;         //����õ�����̼���ֵ

extern float Milemeter_L_Motor,Milemeter_R_Motor;     //�����ֵ�dtʱ���ڵ���̼�����

u8 reset_odom=0;   //��λ��̼�

u8 main_sta=0; //������������������if��ȥ�������flag

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
	
	TIM2_PWM_Init();	        //���Ƶ��
	ENC_Init2();              //���õ��A TIM3������ģʽPB6 PB7
	ENC_Init1();              //���õ��B TIM3������ģʽPA6 PA7 ����
	TIM5_Configuration();     //TIM5���ж϶���������Ӱ�죡����
	TIM1_Configuration();     //TIM5���ж϶���������Ӱ�죡����
	TIM6_Configuration();     //�����Ķ�ʱ
	RCC_Configuration();      //ϵͳʱ������		
	NVIC_Configuration();     //�ж����ȼ�����
	GPIO_Configuration(); 
	UltrasonicWave_Configuration();	
	USART1_Config();	
	USART2_Config();
 // EXTIX_Init();             //������λ�����������ⲿ�жϳ�ʼ��
 // IWDG_Init(4,625);         //�������Ź���ʼ��
	Systick_Init();           //ϵͳ�δ�ʱ����ʼ��
        

	while (1)
	{
		//Set_CarStraight(1200,628);		
		if(main_sta&0x01)   //������̼�����
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
				USART_ClearFlag(USART1,USART_FLAG_TC);  //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����				
				USART_SendData(USART1,odometry_data[i]);	
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);				
			}
			main_sta&=0xFE;
		}
		if(main_sta&0x02)      //���ü�����̼����ݺ���
		{
			odometry(Milemeter_L_Motor,Milemeter_R_Motor);
			main_sta&=0xFD;
		} 
		if(main_sta&0x04)     //������ָ����ȷ����
		{
				USART_ClearFlag(USART1,USART_FLAG_TC);  //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����
				for(k=0;k<3;k++)
				{
					USART_SendData(USART1,0x01);	
					while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
				}					
				USART_SendData(USART1,'\n');	
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);		
				main_sta&=0xFB;
		}
		if(main_sta&0x08)      //������ָ��û����ȷ����ʱ
		{
			  USART_ClearFlag(USART1,USART_FLAG_TC);  //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����
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
