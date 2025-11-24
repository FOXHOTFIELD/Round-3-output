#include "myHeader.h"
enum Motor_Mode Motor1_Mode;
enum Motor_Mode Motor2_Mode;
MenuState curState = {
    .mode = Wait,
    .psost = start
};
volatile int BaseSpeed = 50;
int flag = 0;

struct MOTOR Motor1_Data = {
	0,
	0.0f,
	0.0f,
	0.0f,
	0.0f,
	0.0f,
	0.3f,
	0.4f,
	0.0f
};

struct MOTOR Motor2_Data = {
	0,
	0.0f,
	0.0f,
	0.0f,
	0.0f,
	0.0f,
	0.3f,
	0.0f,
	0.0f
};

int main(void)
{
	/* 系统初始化（startup/系统初始化通常已在启动代码中完成） */
	SystemInit();
    OLED_Init();
    Menu_Init();

	Serial_Init();
    Serial2_Init();
    PWM_Init();    //Motor_SetMode(1, Motor_Mode_frd_rotation);
    PID_Init();

	/* 启动调度 */
	vTaskStartScheduler();

	///* 若调度器返回，进入死循环 */
    
	while (1)
	{}
}
