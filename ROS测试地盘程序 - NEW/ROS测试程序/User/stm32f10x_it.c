#include "stm32f10x_it.h"
#include <stdio.h>
#include "encoder.h"
#include "contact.h"
#include "string.h"
#include "delay.h"

int Flag_IMU,Flag_Motion,Flag_milemeter;

u8 Flag_Utra=0;
u8 ch;
u8 Rcindex=0,FC_REC=0,Flagcom=0,recDataState=0,checkSum=0;

extern u8 bSpeed_Buffer_Index;
extern s32 ENC_Calc_Rot_Speed2(void);
extern s32 ENC_Calc_Rot_Speed1(void);
extern s32 hSpeed_Buffer1[],hSpeed_Buffer2[];

extern float Milemeter_L_Motor,Milemeter_R_Motor;      //累计电机一次运行的里程 cm		

u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART_RX_STA=0;       //接收状态标记	
u8 serial_rec=0x31;   //接收串口数据变量


u8 command_correct=0;        //当指令接收正确时返回给上位机
u8 command_wrong=0;          //当指令错误时返回给上位机，并重新发送

extern u8 main_sta;

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //是否接受到数据
  {
		serial_rec =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
					
		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
				if(USART_RX_STA&0x4000)//接收到了0x0d
				{
					if(serial_rec==0x0a)
					{
						if((USART_RX_STA&0x3f)==8)
						{							
							USART_RX_STA|=0x8000;	//接收完成了 
							main_sta|=0x04;
							main_sta&=0xF7;
						}
						else
						{
							main_sta|=0x08;
							main_sta&=0xFB;
							USART_RX_STA=0;//接收错误,重新开始
						}
					}
					else 
					{
						main_sta|=0x08;
						USART_RX_STA=0;//接收错误,重新开始
					}
				}
				else //还没收到0X0D
				{	
					if(serial_rec==0x0d)USART_RX_STA|=0x4000;
					else
					{
						USART_RX_BUF[USART_RX_STA&0X3FFF]=serial_rec ;
						USART_RX_STA++;
						if(USART_RX_STA>(USART_REC_LEN-1))
						{
							main_sta|=0x08;
							USART_RX_STA=0;//接收数据错误,重新开始接收
						}							
					}		 
				}
			}   		 
  }
}
	 

void USART2_IRQHandler(void)
{
}

void TIM5_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM5 , TIM_IT_Update) != RESET ) 
	{	
				if(Utra_sample!=0)
				{
					Utra_sample--;
				}
			else
					{
						Utra_sample=UTRA_SAMPLING_TIME;
						Flag_Utra=1;
					}
			
			if(Spin_sample!=0)
				{
					Spin_sample--;
				//	printf("%d \r\n",Utra_sample);
				}
			else
					{
						Spin_sample=Spin_SAMPLING_TIME;
						Flag_IMU=1;
					}						
			
			if (hSpeedMeas_Timebase_500us !=0)
				{
					hSpeedMeas_Timebase_500us--;	
				}
			else
					{
						s32 wtemp2,wtemp1;
					//	s32 wtemp2_buff,wtemp1_buff;
						hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME; //SPEED_SAMPLING_TIME;    
						
						wtemp2 = ENC_Calc_Rot_Speed2(); //A 获取的脉冲数
						wtemp1 = ENC_Calc_Rot_Speed1(); //B
						
						
						Flag_milemeter=1;
						main_sta|=0x02;
						Milemeter_L_Motor= (float)wtemp1; //累计脉冲数
						Milemeter_R_Motor= (float)wtemp2;
	
						hSpeed_Buffer2[bSpeed_Buffer_Index] = wtemp2; //缓存脉冲数
						hSpeed_Buffer1[bSpeed_Buffer_Index] = wtemp1;
						bSpeed_Buffer_Index++;
						
						if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)
							{
								bSpeed_Buffer_Index=0;
							}
							ENC_Calc_Average_Speed();  //计算转速
							Gain2(); //电机A转速PID调节控制 右
							Gain1(); //电机B转速PID调节控制 左
					}
		TIM_ClearITPendingBit(TIM5 , TIM_FLAG_Update);  		 
	}		 
}


void TIM1_UP_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM1 , TIM_IT_Update) != RESET ) 
	{	
		main_sta|=0x01;
		
		TIM_ClearITPendingBit(TIM1 , TIM_FLAG_Update);  		 
	}		 
}

