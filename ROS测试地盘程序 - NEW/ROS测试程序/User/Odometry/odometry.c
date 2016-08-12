#include "odometry.h"
#include <math.h>
#include <stdlib.h>

float position_x=0,position_y=0,oriention=0;
float wheel_interval=272.5;

extern float odometry_right,odometry_left;

int multiplier=4;           //倍频数
int deceleration_ratio=90;  //减速比
int wheel_diameter=100;     //轮子直径，单位mm
float pi=3.14;              //π
int line_number=4096;       //码盘线数
float oriention_interval=0;  //dt时间内方向变化值
//角度顺时针为正，逆时针为负
void odometry(float right,float left)
{	
	float const_frame=wheel_diameter*pi/(line_number*multiplier*deceleration_ratio);
	float const_angle=const_frame/wheel_interval;
	float distance_sum=(right+left)/2;
	float distance_diff=left-right;
	int odom_right=(int)odometry_right;
	int odom_left=(int)odometry_left;
	int abs_right=abs(odometry_right);
	int abs_left=abs(odometry_left);
	
	if((odom_right>0)&&(odom_left>0))
	{
		oriention_interval=distance_diff*const_angle;
		position_y=position_y+distance_sum*sin(oriention_interval)*const_frame;
		if(odom_right>odom_left)
		{
			position_x=position_x-distance_sum*cos(oriention_interval)*const_frame;
			oriention=oriention+oriention_interval;
		}
		else
		{
			position_x=position_x+distance_sum*cos(oriention_interval)*const_frame;
			oriention=oriention-oriention_interval;
		}		
	}
	else if((odom_right<0)&&(odom_left<0))
	{
		oriention_interval=distance_diff*const_angle;
		position_y=position_y-distance_sum*sin(oriention_interval)*const_frame;
		if(abs_right>abs_left)
		{
			position_x=position_x-distance_sum*cos(oriention_interval)*const_frame;
			oriention=oriention-oriention_interval;
		}
		else
		{
			position_x=position_x+distance_sum*cos(oriention_interval)*const_frame;
			oriention=oriention+oriention_interval;
		}	
	}
	else if((odom_right<0)&&(odom_left>0))
	{	
		oriention_interval=distance_diff*const_angle;		
		if(abs_right<abs_left)
		{
			position_x=position_x+distance_sum*cos(oriention_interval)*const_frame;
			position_y=position_y+distance_sum*sin(oriention_interval)*const_frame;
		}
		else
		{
			position_x=position_x-distance_sum*cos(oriention_interval)*const_frame;
			position_y=position_y-distance_sum*sin(oriention_interval)*const_frame;
		}	
		oriention=oriention-oriention_interval;
		
	}
	else if((odom_right>0)&&(odom_left<0))
	{
		oriention_interval=distance_diff*const_angle;
		
		if(abs_right>abs_left)
		{
			position_x=position_x-distance_sum*cos(oriention_interval)*const_frame;
			position_y=position_y+distance_sum*sin(oriention_interval)*const_frame;

		}
		else
		{
			position_x=position_x+distance_sum*cos(oriention_interval)*const_frame;
			position_y=position_y-distance_sum*sin(oriention_interval)*const_frame;
		}	
		oriention=oriention+oriention_interval;
	}
	else
	{
		oriention=oriention;
		position_x=position_x;
		position_y=position_y;	
	}
}
