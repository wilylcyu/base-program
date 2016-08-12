#include "encoder.h"

s16 ENC_Calc_Rot_Speed2(void);
s16 ENC_Calc_Rot_Speed1(void);

static unsigned int hRot_Speed2;
static unsigned int hRot_Speed1;
unsigned int Speed2=0; //A
unsigned int Speed1=0; //B
float A_REMP_PLUS;
int span;

struct PID Control_left  ={0.01,0.1,0.75,0,0,0,0,0,0}; //适于新电机4096
struct PID Control_right ={0.01,0.1,0.75,0,0,0,0,0,0};

s32 hPrevious_angle2, hPrevious_angle1;
s32 hSpeed_Buffer2[SPEED_BUFFER_SIZE]={0}, hSpeed_Buffer1[SPEED_BUFFER_SIZE]={0};

u8 bSpeed_Buffer_Index = 0;

static volatile u16 hEncoder_Timer_Overflow1; 
static volatile u16 hEncoder_Timer_Overflow2;
static bool bIs_First_Measurement2 = true;
static bool bIs_First_Measurement1 = true;
unsigned char a[2];

//电机A码盘采集定时器，TIM4编码器模式
void ENC_Init2(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;    
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  TIM_DeInit(ENCODER2_TIMER);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0;  
  TIM_TimeBaseStructure.TIM_Period = (4*ENCODER2_PPR)-1;
	
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(ENCODER2_TIMER, &TIM_TimeBaseStructure);
 
  TIM_EncoderInterfaceConfig(ENCODER2_TIMER, TIM_EncoderMode_TI12, 
                             TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = ICx_FILTER;
  TIM_ICInit(ENCODER2_TIMER, &TIM_ICInitStructure);
  
  TIM_ClearFlag(ENCODER2_TIMER, TIM_FLAG_Update);
  TIM_ITConfig(ENCODER2_TIMER, TIM_IT_Update, ENABLE);

	ENC_Clear_Speed_Buffer();

  TIM_Cmd(ENCODER2_TIMER, ENABLE); 
}

//电机B码盘采集定时器，TIM3编码器模式
void ENC_Init1(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
   
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	
  TIM_DeInit(ENCODER1_TIMER);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = (4*ENCODER1_PPR)-1;  
	
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(ENCODER1_TIMER, &TIM_TimeBaseStructure);
 
  TIM_EncoderInterfaceConfig(ENCODER1_TIMER, TIM_EncoderMode_TI12, 
                             TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = ICx_FILTER;
  TIM_ICInit(ENCODER1_TIMER, &TIM_ICInitStructure);
  
  TIM_ClearFlag(ENCODER1_TIMER, TIM_FLAG_Update);
  TIM_ITConfig(ENCODER1_TIMER, TIM_IT_Update, ENABLE);

  TIM_Cmd(ENCODER1_TIMER, ENABLE); 
}


//存储器清零
void ENC_Clear_Speed_Buffer(void)
{   
  u32 i;
	
  for (i=0;i<SPEED_BUFFER_SIZE;i++)
  {
		hSpeed_Buffer2[i] = 0;
		hSpeed_Buffer1[i] = 0;
  }
	bIs_First_Measurement2 = true;
	bIs_First_Measurement1 = true;
}


//计算电机A的旋转速度
s16 ENC_Calc_Rot_Speed2(void)
{   
  s32 wDelta_angle;
  u16 hEnc_Timer_Overflow_sample_one;
  u16 hCurrent_angle_sample_one;
	s32 temp;
  s16 haux;
  
  if (!bIs_First_Measurement2)
  {  
    hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow2; 	
    hCurrent_angle_sample_one = ENCODER2_TIMER->CNT;
    hEncoder_Timer_Overflow2 = 0;
    haux = ENCODER2_TIMER->CNT;   
     
    if ( (ENCODER2_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
    {
			// encoder timer down-counting 反转的速度计算     
			wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER2_PPR) -(hCurrent_angle_sample_one - hPrevious_angle2));
    }
    else  
    {
			//encoder timer up-counting 正转的速度计算
		 wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle2 + (hEnc_Timer_Overflow_sample_one) * (4*ENCODER2_PPR));
    }		
		temp=wDelta_angle;
  } 
  else
  {
    bIs_First_Measurement2 = false;
    temp = 0;
    hEncoder_Timer_Overflow2 = 0;
    haux = ENCODER2_TIMER->CNT;       
  }
  hPrevious_angle2 = haux;  
  return((s16) temp);
}


//计算电机B的旋转速度
s16 ENC_Calc_Rot_Speed1(void)
{   
  s32 wDelta_angle;
  u16 hEnc_Timer_Overflow_sample_one;
  u16 hCurrent_angle_sample_one;
	s32 temp;
  s16 haux;
  
  if (!bIs_First_Measurement1)
  {   
    hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow1; 		
    hCurrent_angle_sample_one = ENCODER1_TIMER->CNT;
    hEncoder_Timer_Overflow1 = 0;
    haux = ENCODER1_TIMER->CNT;   
     
    if ( (ENCODER1_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
    {
			// encoder timer down-counting 反转的速度计算
			wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR) -(hCurrent_angle_sample_one - hPrevious_angle1));	
    }
    else  
    {
			//encoder timer up-counting 正转的速度计算
			wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle1 + (hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR));
    }
		temp=wDelta_angle;
  } 
  else
  {
    bIs_First_Measurement1 = false;
    temp = 0;
    hEncoder_Timer_Overflow1 = 0;
    haux = ENCODER1_TIMER->CNT;       
  }
  hPrevious_angle1 = haux;  
  return((s16) temp);
}


//计算三个电机的平均旋转速度
void ENC_Calc_Average_Speed(void)
{   
 
  u32 i;
	signed long long wtemp3=0;
	signed long long wtemp4=0;

	for (i=0;i<SPEED_BUFFER_SIZE;i++)
	{
		wtemp4 += hSpeed_Buffer2[i];
		wtemp3 += hSpeed_Buffer1[i];
	}		
	wtemp3 /= (SPEED_BUFFER_SIZE);
	wtemp4 /= (SPEED_BUFFER_SIZE); //平均脉冲数 个/s	
	wtemp3 = (wtemp3 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER1_PPR);
	wtemp4 = (wtemp4 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER2_PPR); //平均转速 r/min
		
	hRot_Speed2= ((s16)(wtemp4));
	hRot_Speed1= ((s16)(wtemp3));
	Speed2=hRot_Speed2;
	Speed1=hRot_Speed1;
}

//执行TIM4(电机A编码器采集)计数中断
void TIM4_IRQHandler (void)
{   
  TIM_ClearFlag(ENCODER2_TIMER, TIM_FLAG_Update);
  if (hEncoder_Timer_Overflow2 != U16_MAX)  
  {
   hEncoder_Timer_Overflow2++; 
  }
}
//执行TIM3(电机B编码器采集)计数中断
void TIM3_IRQHandler (void)
{  
  TIM_ClearFlag(ENCODER1_TIMER, TIM_FLAG_Update);
  if (hEncoder_Timer_Overflow1 != U16_MAX)  
  {
    hEncoder_Timer_Overflow1++;	 
  }
}

//设置电机A PID调节 PA2
void Gain2(void)
{
	static float pulse = 0;
	span=1*(Speed1-Speed2);
	pulse= pulse + PID_calculate(&Control_right,hRot_Speed2);
	if(pulse > 3600) pulse = 3600;
	if(pulse < 0) pulse = 0;
	A_REMP_PLUS=pulse;
}

//设置电机B PID调节 PA1
void Gain1(void)
{
	static float pulse1 = 0;
	span=1*(Speed2-Speed1);
	pulse1= pulse1 + PID_calculate(&Control_left,hRot_Speed1);
	if(pulse1 > 3600) pulse1 = 3600;
	if(pulse1 < 0) pulse1 = 0;
	
	TIM2->CCR2 = pulse1;
	TIM2->CCR3 = A_REMP_PLUS;
}

