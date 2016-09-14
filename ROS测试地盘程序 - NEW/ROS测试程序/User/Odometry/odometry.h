#ifndef __ODOMETRY_H
#define __ODOMETRY_H
#include "stm32f10x.h"
#define ODOMETRY_LENGTH 12;

void odometry(float,float);
void quadrant_sign(float,float);
float angle_equ(float,float);


#endif
