#ifndef __CONTACT_H
#define __CONTACT_H
void Contact_Init(void);

int dis_to_pulse(int dis);
int angle_to_pulse(int angle);

void Set_carFoward(int speed);
void Set_carBack(int speed);
void Set_carRight(int speed);
void Set_carLeft(int speed);
void Set_carStop(void);
void set_stop(void);

void Set_Carback_Straight(int speed,float S_mile);
void Set_CarStraight(int speed,float S_mile);
void Set_CarTurn(int speed_L,int speed_R,int alpha);

void Car_turn(int speedw,int radius,int direction);
void Set_Caromega(int speedomega,int speed,int direction);

void car_move(int speedw,int speedv,int direction);

void car_control(float rightspeed,float leftspeed);

#endif  
