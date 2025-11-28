#include <stdint.h>
#ifndef __MYCLAIM_H
#define __MYCLAIM_H
extern volatile int16_t Encoder_Count;					//全局变量（volatile），用于计数旋转编码器的增量值

extern volatile BaseType_t oled_update_blocked;

enum Motor_Mode{
    Motor_Mode_break,
    Motor_Mode_frd_rotation,
    Motor_Mode_rvs_rotation,
    Motor_Mode_stop
};


extern enum Motor_Mode Motor1_Mode;
extern enum Motor_Mode Motor2_Mode;

extern volatile int16_t Motor1_Speed;
extern volatile int16_t Motor2_Speed;

extern volatile float offset;
extern uint8_t status;


extern enum MENU_STATE menu_state;
#endif
