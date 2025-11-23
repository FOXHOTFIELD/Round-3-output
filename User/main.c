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
	SystemInit();
    OLED_Init();
    Menu_Init();
    //I2C2_Init();
	///* 初始化串口与 LED */
	Serial_Init();
	//LED_Init();

	///* 创建任务：优先级 Blink>Print */
	//xTaskCreate(vTaskBlink, "Blink", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
	//xTaskCreate(vTaskPrint, "Print", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

	/* 启动调度 */
	vTaskStartScheduler();

	///* 若调度器返回，进入死循环 */
    
	while (1)
	{
        if (Serial_RxFlag == 1)		//如果接收到数据包
		{
            OLED_ShowString(1,1, Serial_RxPacket, OLED_6X8);
	        OLED_Update();
            Serial_SendString(Serial_RxPacket);
			
			Serial_RxFlag = 0;			//处理完成后，需要将接收数据包标志位清零，否则将无法接收后续数据包
		}
        Delay_ms(30);
	}
}
