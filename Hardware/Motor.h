#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdint.h"

//void ButtonInit(void);
void Motor1_SetPWM(int8_t PWM);
int16_t Motor1_getSpeed(void);
void PWM_Init(void);
void Motor_SetMode(int numMotor, enum Motor_Mode Mode);
void Motor1_SetPrescaler(uint16_t Prescaler);
void Motor1_SetCompare(uint16_t Compare);
void TIM1_Init(void);
void STBY_cmd(FunctionalState state);
void Motor2_SetCompare(uint16_t Compare);

void Motor2_SetPWM(int8_t PWM);
#endif
