#ifndef __STM32F10x_ENCODER_H
#define __STM32F10x_ENCODER_H

#define U16_MAX    ((u16)65535u)
#define U32_MAX    ((u32)4294967295uL)
#include "stm32f10x.h"
#include "stdio.h"
#include "stdbool.h"
#include "PID.h"
#include "delay.h"


typedef enum {DISPLAY_TIMCNT = 0,DISPLAY_THETA,DISPLAY_W} DisplayType;

/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define SPEED_SAMPLING_TIME  2    //  (2+1)*1/9    ,300hz
#define SPEED_SAMPLING_TIME  9    // (9+1)*500usec = 5MS   ,200hz
#define UTRA_SAMPLING_TIME 299
#define Spin_SAMPLING_TIME 299
#define SPEED_BUFFER_SIZE 3

#define ENCODER1_TIMER TIM3   // 电机B码盘采集定时器 TIM3
#define ENCODER2_TIMER TIM4   // 电机A码盘采集定时器 TIM4
#define ENCODER2_PPR           (u16)(4096)  // 电机2码盘线数
#define ENCODER1_PPR           (u16)(4096)  // 电机2码盘线数

#define COUNTER_RESET   (u16)0
#define ICx_FILTER      (u8) 6 // 6<-> 670nsec

#define TIMx_PRE_EMPTION_PRIORITY 1
#define TIMx_SUB_PRIORITY 0

#define SPEED_SAMPLING_FREQ (u16)(2000/(SPEED_SAMPLING_TIME+1))  //200hz
//#define SPEED_SAMPLING_FREQ (u16)(10000/(SPEED_SAMPLING_TIME+1)) //1000hz

void ENC_Init(void);
void ENC_Init1(void);
void ENC_Init2(void);

signed short int ENC_Get_Electrical_Angle(void);
void ENC_Clear_Speed_Buffer(void);
void ENC_Calc_Average_Speed(void);

static unsigned short int hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;
static unsigned short int Utra_sample=UTRA_SAMPLING_TIME;
static unsigned short int Spin_sample=Spin_SAMPLING_TIME;
void Gain(void);
void Gain1(void);
void Gain2(void);

#endif 
