#include "odometry.h"
#include <math.h>
#include <stdlib.h>

float position_x=0,position_y=0,oriention=0,oriention_1=0;
float wheel_interval= 268.0859f;//    272.0f;        //  1.0146
//float wheel_interval=276.089f;    //���У��ֵ=ԭ���/0.987

extern float odometry_right,odometry_left;

float multiplier=4.0f;           //��Ƶ��
float deceleration_ratio=90.0f;  //���ٱ�
float wheel_diameter=100.0f;     //����ֱ������λmm
float pi_1_2=1.570796f;					//��/2
float pi=3.141593f;              //��
float pi_3_2=4.712389f;					//��*3/2
float pi_2_1=6.283186f;					//��*2
float dt=0.005f;                 //����ʱ����5ms

float line_number=4096.0f;       //��������
float oriention_interval=0;  //dtʱ���ڷ���仯ֵ

float velocity_linear=0,velocity_angular=0;
 
//float sign_x=0;     //������,ȷ��x,y�������޵�����
//float sign_y=0;

float sin_=0;        //�Ƕȼ���ֵ
float cos_=0;

//float oriention_=0;  //�����Ƕ������ж���

//float result1_0=0,result1_2pi=0,result2=0,result3=0,result4=0;

float delta_distance=0,delta_oriention=0;   //����ʱ�������˶��ľ���

float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0;

u8 once=1;

//float angle_equ(float angle_value,float angle)
//{
//	float result=angle-angle_value;
//	return result;
//}

//�Ƕ�˳ʱ��Ϊ������ʱ��Ϊ��

//void quadrant_sign(float oriention_input,float theta_)   //���޷����ж�
//{
//	if(oriention<0)
//	{
//		oriention_=oriention+pi_2_1;
//	}
//	else
//	{
//		oriention_=oriention;
//	}
//	
//	result1_0=angle_equ(0,oriention_);
//	result1_2pi=angle_equ(pi_2_1,oriention_);
//	result2=angle_equ(pi_1_2,oriention_);
//	result3=angle_equ(pi,oriention_);
//	result4=angle_equ(pi_3_2,oriention_);
//	
//	if(((result1_0<0.0001)&&(result1_2pi>-0.0001))||((oriention_<pi_1_2)&&(oriention_>0)))
//	{
//		sign_x=1;
//		sign_y=1;
//	}
//	else if(((result2>-0.0001)&&(result2<-0.0001))||((oriention_<pi)&&(oriention_>pi_1_2)))
//	{
//		sign_x=-1;
//		sign_y=1;
//	}
//	else if(((result3>-0.0001)&&(result3<-0.0001))||((oriention_<pi_3_2)&&(oriention_<pi)))
//	{
//		sign_x=-1;
//		sign_y=-1;
//	}
//	else if(((result4>-0.0001)&&(result4<-0.0001))||((oriention_<pi_2_1)&&(oriention_<pi_3_2)))
//	{
//		sign_x=1;
//		sign_y=-1;
//	}
//	else
//	{
//		sign_x=1;
//		sign_y=1;		
//	}
//}
void odometry(float right,float left)
{	
	if(once)  //����������һ��
	{
		const_frame=wheel_diameter*pi/(line_number*multiplier*deceleration_ratio);
		const_angle=const_frame/wheel_interval;
		once=0;
	}
	distance_sum=0.5f*(right+left);
	distance_diff=right-left;
	
//	if(oriention>pi_2_1)     //����Ƕȳ���һȦʱ�����
//	{
//		oriention=oriention-pi_2_1;
//	}
//	if(oriention<-pi_2_1)
//	{
//		oriention=oriention+pi_2_1;
//	}
	
	if((odometry_right>0)&&(odometry_left>0))    //���Ҿ���
	{
		delta_distance=distance_sum;
		delta_oriention=distance_diff;
	}
	else if((odometry_right<0)&&(odometry_left<0))   //���Ҿ���
	{
		delta_distance=-distance_sum;
		delta_oriention=-distance_diff;
	}
	else if((odometry_right<0)&&(odometry_left>0))     //�����Ҹ�
	{
		delta_distance=-distance_diff;
		delta_oriention=-2.0f*distance_sum;		
	}
	else if((odometry_right>0)&&(odometry_left<0))     //������
	{
		delta_distance=distance_diff;
		delta_oriention=2.0f*distance_sum;
	}
	else
	{
		delta_distance=0;
		delta_oriention=0;
	}
	
	sin_=sin(oriention_1);
	cos_=cos(oriention_1);
	
	oriention_interval=delta_oriention*const_angle;
	position_x=position_x+delta_distance*cos_*const_frame;
	position_y=position_y+delta_distance*sin_*const_frame;
	oriention=oriention+oriention_interval;
	oriention_1=oriention+0.5f*oriention_interval;
	
	velocity_linear=delta_distance*const_frame/dt;
	velocity_angular=oriention_interval/dt;
	
	if(oriention>pi)
	{
		oriention-=pi_2_1;
	}
	else
	{
		if(oriention<-pi)
		{
			oriention+=pi_2_1;
		}
	}
}
