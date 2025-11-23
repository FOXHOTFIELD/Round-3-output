#include "myHeader.h"

float Kp = 1, Ki = 0, Kd = 0;					//比例项，积分项，微分项的权重
enum Motor_Mode Motor1_Mode;
enum Motor_Mode Motor2_Mode;
MenuState curState = {
    .mode = Wait,
    .psost = start
};
int TargetSpeed = 20;
int flag = 0;


int main(void)
{
	/* 系统初始化（startup/系统初始化通常已在启动代码中完成） */
	//SystemInit();
    OLED_Init();
    Menu_Init();

	Serial_Init();
    PWM_Init();    //Motor_SetMode(1, Motor_Mode_frd_rotation);


	/* 启动调度 */
	//vTaskStartScheduler();

	///* 若调度器返回，进入死循环 */
    
	while (1)
	{    Motor2_SetPWM(-50);
	}
}
