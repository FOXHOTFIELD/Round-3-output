#ifndef __PID_H
#define __PID_H

void PID_Init(void);
void Motor_DateClear(void);

//extern volatile float Target, Actual, Out;			//目标值，实际值，输出值
//extern volatile float Error0, Error1, Error2;		//本次误差，上次误差，上上次误差

struct MOTOR
{
    int16_t Actual; /*在Serial内接收后赋值*/
    float Target;			//目标值
    float Out;			//输出值
    float Error0, Error1, Error2;		//本次误差，上次误差，上上次误差
    float Kp;
    float Ki;
    float Kd;
};

extern struct MOTOR Motor1_Data, Motor2_Data;


extern int flag;
extern volatile int BaseSpeed;
extern volatile int CurveSpeed;
#endif
