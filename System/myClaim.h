#include <stdint.h>
#ifndef __MYCLAIM_H
#define __MYCLAIM_H

extern volatile int16_t Encoder_Count;					//全局变量（volatile），用于计数旋转编码器的增量值


enum Motor_Mode{
    Motor_Mode_break,
    Motor_Mode_frd_rotation,
    Motor_Mode_rvs_rotation,
    Motor_Mode_stop
};


extern int8_t TargetSpeed;

extern enum Motor_Mode Motor1_Mode;
extern enum Motor_Mode Motor2_Mode;

extern volatile int16_t Motor1_Speed;
extern volatile int16_t Motor2_Speed;


extern volatile float Target, Actual, Out;			//目标值，实际值，输出值
extern float Kp, Ki, Kd;					//比例项，积分项，微分项的权重
extern volatile float Error0, Error1, Error2;		//本次误差，上次误差，上上次误差


#endif
