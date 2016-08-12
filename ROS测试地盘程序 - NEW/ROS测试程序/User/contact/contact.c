#include "stm32f10x.h"
#include "contact.h"
#include "PID.h"
#include "stm32f10x_it.h"
#include "math.h"
#include <stdio.h>
#include "cstring"


extern struct PID Control_left ;
extern struct PID Control_right ;
extern struct PID Control_front ;
extern unsigned int bb[3];
extern unsigned int Speed1;
extern unsigned int Speed2;
extern unsigned int Speed3;


//自定义回调函数:写函数
float  Milemeter_L_Motor=0,Milemeter_R_Motor=0,Sum_L_Motor=0,Sum_R_Motor=0;//累计电机一次运行的里程 cm
extern int Flag_milemeter,flag_FSS;
extern int flag_car,flag_carturn,flag_back_car;

extern int state,stateTemp,orderIndex,recDataState;//caculate_direction,
//extern int sequence[];
//extern int number;
extern char recData[],recDatabuff[];

int flag_car=1;

int wtemp_given; //给定距离换算的脉冲数

/*左车轮 B*/	
void LeftMovingSpeedW(unsigned int val)
{     
   if(val>10000)
	 {  
			GPIO_SetBits(GPIOC, GPIO_Pin_6);	
			GPIO_ResetBits(GPIOC, GPIO_Pin_7);				
		  Control_left.OwenValue=(val-10000);			
	 }
	 if(val<10000)
	 {  
			GPIO_SetBits(GPIOC, GPIO_Pin_7);	
			GPIO_ResetBits(GPIOC, GPIO_Pin_6);		
			Control_left.OwenValue=(10000-val);	 
	  }	
    if(val==10000)
		{
			 GPIO_SetBits(GPIOC, GPIO_Pin_6);	
			 GPIO_SetBits(GPIOC, GPIO_Pin_7);			
			 Control_left.OwenValue=0;
		}					
}
	/*右车轮 A*/
void RightMovingSpeedW(unsigned int val2)
{    
	if(val2>10000)
	{  
	  /* motor A 正转*/
		GPIO_SetBits(GPIOC, GPIO_Pin_10);	
		GPIO_ResetBits(GPIOC, GPIO_Pin_11); 
		Control_right.OwenValue=(val2-10000);
	}
	if(val2<10000)
	{  
		/* motor A 反转*/
		GPIO_SetBits(GPIOC, GPIO_Pin_11);	
	  GPIO_ResetBits(GPIOC, GPIO_Pin_10);		
	  Control_right.OwenValue=(10000-val2);	 
	}	
  if(val2==10000)
  {
	  GPIO_SetBits(GPIOC, GPIO_Pin_10);	
		GPIO_SetBits(GPIOC, GPIO_Pin_11);
		Control_right.OwenValue=0;
	}												
}

//距离转脉冲数函数
int dis_to_pulse(int dis)  //给定的dis单位是mm
{
	int multiplier=4;           //倍频数
	int deceleration_ratio=90;  //减速比
	int wheel_diameter=100;     //轮子直径，单位mm
	float pi=3.14;              //π
	int line_number=4096;       //码盘线数
	int given_pulse=(int)((multiplier*deceleration_ratio*line_number*dis)/(pi*wheel_diameter));
	//4*4096*90/314
	return given_pulse;         //返回给定的距离转换后的脉冲数
}

//角度转换脉冲数的函数
int angle_to_pulse(int angle)
{
	float wheel_interval=272.5,pi=3.14;  //两轮之间间距单位为mm
	int given_pulse=dis_to_pulse((int)(angle*pi*wheel_interval/360));
	return given_pulse;                  //返回给定的角度转换后的脉冲数
}

void Contact_Init(void)
{
	LeftMovingSpeedW(12000); //电机B
	RightMovingSpeedW(12000);//电机A	
}

void Set_carStop(void)
{
	LeftMovingSpeedW(10000); //电机B
	RightMovingSpeedW(10000);//电机A	
	TIM2->CCR2 = 100;
	TIM2->CCR3 = 100;
}
void Set_carFoward(int speed)
{
	LeftMovingSpeedW(10000+speed); //电机B
	RightMovingSpeedW(10000+speed);//电机A
}
void Set_carLeft(int speed)
{
  LeftMovingSpeedW(10000-speed); //电机B
	RightMovingSpeedW(10000+speed);//电机A
}
void Set_carRight(int speed)
{
  LeftMovingSpeedW(10000+speed); //电机B
	RightMovingSpeedW(10000-speed);//电机A
}
void Set_carBack(int speed)
{
	LeftMovingSpeedW(10000-speed); //电机B
	RightMovingSpeedW(10000-speed);//电机A	
}


void car_control(float rightspeed,float leftspeed)
{
	float k2=17.179;         //速度转换比例,转/分钟
	
	int right_speed=(int)k2*rightspeed;
	int left_speed=(int)k2*leftspeed;
	

		RightMovingSpeedW(right_speed+10000);

		LeftMovingSpeedW(left_speed+10000);

}
void Set_CarStraight(int speed,float S_mile)//S_mile单位是mm
{
	 Set_carFoward(10000); 
	 LeftMovingSpeedW(10000+speed); //电机B
	 RightMovingSpeedW(10000+speed);//电机A	
	
   wtemp_given=dis_to_pulse(S_mile);
	
	 if(Flag_milemeter==1)
	 {
		  Flag_milemeter=0;
		
		  Sum_L_Motor+=Milemeter_L_Motor;
		  Sum_R_Motor+=Milemeter_R_Motor;		
		 //printf("%f/r/n",Sum_L_Motor);
   }
	// printf("%f %d\r\n",(Sum_L_Motor+Sum_R_Motor)/2,wtemp_given);
	 
	 if((fabs(wtemp_given-(Sum_L_Motor+Sum_R_Motor)/2)<=500)||(wtemp_given<=((Sum_L_Motor+Sum_R_Motor)/2)))
	 {
		  Sum_L_Motor=0;
		  Sum_R_Motor=0;
		  flag_car=0;


		 recDataState=0;
	 }
}
