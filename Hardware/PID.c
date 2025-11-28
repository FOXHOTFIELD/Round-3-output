#include "myHeader.h"
#include <math.h>
#include <string.h>

// 在文件开头，定义积分分离阈值（根据实际系统调整此值）
#define Outer_INTEGRAL_SEPARATION_THRESHOLD 200
#define INTEGRAL_SEPARATION_THRESHOLD 200

static void vPIDTask(void *pvParameters);
static void PIDControl(uint8_t index, struct MOTOR *motor);

volatile float offset = 0;

/*
 * 初始化vPIDTask的Freertos控制
 */
void PID_Init(void)
{
    xTaskCreate(vPIDTask, "vPidTask", 128, NULL, tskIDLE_PRIORITY+3, NULL);
}

/* Freertos控制下的任务函数*/
static void vPIDTask(void *pvParameters)
{
    (void) pvParameters;

    for(;;){
        if(xSemaphoreTake(xSerialSemphr, portMAX_DELAY) == pdTRUE){
            if(curState.mode == Run) {
                STBY_cmd(ENABLE);
                PIDControl(1, &Motor1_Data);
                PIDControl(2, &Motor2_Data);
            }else {
                STBY_cmd(DISABLE);
            }
        }
    }
}

/*Motor数据在开始Run时清零 避免起步突冲*/
void Motor_DateClear(void)
{
    int aera = sizeof(int16_t) + 3 * sizeof(float);
    memset(&Motor1_Data, 0, aera);
    memset(&Motor2_Data, 0, aera);
}

/*具体的PID控制操作*/
static void PIDControl(uint8_t index, struct MOTOR *motor)
{
    /*
     *   if offset > 0, turn right
     *   motor2 in right 
     *   motor2.speed - offset (sub)
     *   motor1.speed + offset (add)
     ????*/
    
    if(status != 1 && status != 5) {
        motor->Target = (float)BaseSpeed + (index == 1 ? (+offset) : (-offset));
    } else if(status == 1) {
        // 保持上一次输出状态，不更新Target和Out
        // 直接返回，不做后续PID计算和PWM输出
        motor->Target = (index == 2 ? BaseSpeed+10  : 15);
    }else if(status == 5){
        motor->Target = (index == 1 ? BaseSpeed+10 : 15);

    }
    //motor->Target = (float)BaseSpeed;
    OLED_ShowSignedNum(90, 56, motor->Target, 3, OLED_6X8);
        
    /*获取本次误差、上次误差和上上次误差*/
    motor ->Error2 = motor ->Error1;            //获取上上次误差
    motor ->Error1 = motor ->Error0;            //获取上次误差
    motor ->Error0 = motor ->Target - motor ->Actual;    //获取本次误差


    if((fabs(motor ->Error0)) > 1)
    {
    
        /*积分分离PID计算*/
        float deltaP = motor->Kp * ((motor ->Error0) - (motor ->Error1));  //比例项
        float deltaD = motor->Kd * ((motor ->Error0) - 2 * (motor ->Error1) + (motor ->Error2)); //微分项
        float deltaI = 0;  //积分项，默认设为0（分离状态）
    
    // 积分分离判断：只有当误差在阈值内时才加入积分作用[1,2](@ref)
        if(fabs(motor ->Error0) <= INTEGRAL_SEPARATION_THRESHOLD)
        {
            deltaI = motor->Ki * (motor ->Error0);  //引入积分控制
        }
    
        //计算总输出增量
        (motor ->Out) += deltaP + deltaI + deltaD;
            
        /*输出限幅*/
        if (motor ->Out > 100) {motor ->Out = 100;}     //限制输出值最大为100
        if (motor ->Out < -100) {motor ->Out = -100;}   //限制输出值最小为-100
        
        /*目标和实际速度都为0时，停止输出*/
        if(fabs(motor ->Target) < 0.1f && motor ->Actual == 0) motor ->Out = 0;
    } 

	/*执行控制 - 类型转换为int8_t*/
    Motor_SetPWM(index, (int8_t)(motor ->Out));

    OLED_ShowSignedNum(56, 56, Motor1_Data.Out, 3, OLED_6X8);
    OLED_UpdateArea(1, 56, 24, 8);
}
