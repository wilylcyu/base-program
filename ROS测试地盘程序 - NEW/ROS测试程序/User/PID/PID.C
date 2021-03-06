#include "PID.H"
#include "stdio.h"

extern unsigned int Speed2,Speed1;
extern int span;
float MaxValue=9;//输出最大限幅
float MinValue=-9;//输出最小限幅
float CurrentValue_left;//采样值
float CurrentValue_right;

float OutputValue;
char temp1[32];
   
float PID_calculate(struct PID *Control,float CurrentValue_left )//位置PID计算B
{
	
	float Value_Kp;//比例分量
	float Value_Ki;//积分分量
	float Value_Kd;//微分分量
	
	Control->error_0     = Control->OwenValue-CurrentValue_left+0*span;//基波分量
	Value_Kp                  = Control->Kp*Control->error_0 ;
	Control->Sum_error  += Control->error_0;     
	
		/***********************积分饱和抑制********************************************/
	OutputValue=Control->OutputValue;
	if(OutputValue>5||OutputValue<-5)	
	{
	Control->Ki=0; 
	}
	/*******************************************************************/
	
	Value_Ki                  = Control->Ki*Control->Sum_error;
	Value_Kd                  = Control->Kd*( Control->error_0-Control->error_1);
	Control->error_1     = Control->error_0;//保存一次谐波
	Control->OutputValue = Value_Kp  + Value_Ki + Value_Kd;//输出值计算，注意加减
	
	if( Control->OutputValue>MaxValue)//限幅
		Control->OutputValue=MaxValue;
	if (Control->OutputValue<MinValue)
		Control->OutputValue= MinValue;
	return (Control->OutputValue) ;
}
