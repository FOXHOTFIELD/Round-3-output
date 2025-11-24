#include "myHeader.h"00
enum Motor_Mode Motor2_Mode;
MenuState curState = {
    .mode = Wait,
    .psost = start
};
volatile int BaseSpeed = 50;
int flag = 0;

extern struct MOTOR Motor1_Data = {0}, Motor2_Data = {0};

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
