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

extern float Milemeter_L_Motor,Milemeter_R_Motor;      //�ۼƵ��һ�����е���� cm		

u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 USART_RX_STA=0;       //����״̬���	
u8 serial_rec=0x31;   //���մ������ݱ���


u8 command_correct=0;        //��ָ�������ȷʱ���ظ���λ��
u8 command_wrong=0;          //��ָ�����ʱ���ظ���λ���������·���

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
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //�Ƿ���ܵ�����
  {
		serial_rec =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
					
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
				if(USART_RX_STA&0x4000)//���յ���0x0d
				{
					if(serial_rec==0x0a)
					{
						if((USART_RX_STA&0x3f)==8)
						{							
							USART_RX_STA|=0x8000;	//��������� 
							main_sta|=0x04;
							main_sta&=0xF7;
						}
						else
						{
							main_sta|=0x08;
							main_sta&=0xFB;
							USART_RX_STA=0;//���մ���,���¿�ʼ
						}
					}
					else 
					{
						main_sta|=0x08;
						USART_RX_STA=0;//���մ���,���¿�ʼ
					}
				}
				else //��û�յ�0X0D
				{	
					if(serial_rec==0x0d)USART_RX_STA|=0x4000;
					else
					{
						USART_RX_BUF[USART_RX_STA&0X3FFF]=serial_rec ;
						USART_RX_STA++;
						if(USART_RX_STA>(USART_REC_LEN-1))
						{
							main_sta|=0x08;
							USART_RX_STA=0;//�������ݴ���,���¿�ʼ����
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
						
						wtemp2 = ENC_Calc_Rot_Speed2(); //A ��ȡ��������
						wtemp1 = ENC_Calc_Rot_Speed1(); //B
						
						
						Flag_milemeter=1;
						main_sta|=0x02;
						Milemeter_L_Motor= (float)wtemp1; //�ۼ�������
						Milemeter_R_Motor= (float)wtemp2;
	
						hSpeed_Buffer2[bSpeed_Buffer_Index] = wtemp2; //����������
						hSpeed_Buffer1[bSpeed_Buffer_Index] = wtemp1;
						bSpeed_Buffer_Index++;
						
						if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)
							{
								bSpeed_Buffer_Index=0;
							}
							ENC_Calc_Average_Speed();  //����ת��
							Gain2(); //���Aת��PID���ڿ��� ��
							Gain1(); //���Bת��PID���ڿ��� ��
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

