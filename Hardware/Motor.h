#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdint.h"

//void ButtonInit(void);
void Motor_SetPWM(uint8_t index, int8_t PWM);
void PWM_Init(void);
void Motor_SetMode(int numMotor, enum Motor_Mode Mode);
void Motor1_SetPrescaler(uint16_t Prescaler);
void Motor1_SetCompare(uint16_t Compare);
void TIM1_Init(void);
void Motor2_SetCompare(uint16_t Compare);
//void Motor2_SetPWM(int8_t PWM);

#define STBY_ENABLE()   GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define STBY_DISABLE()  GPIO_ResetBits(GPIOB, GPIO_Pin_0)

/*电机开关控制*/
#define STBY_cmd(state)            \
	do {                          \
		if ((state) == ENABLE) {  \
			STBY_ENABLE();        \
		} else {                  \
			STBY_DISABLE();       \
		}                         \
	} while (0)                         
#endif
