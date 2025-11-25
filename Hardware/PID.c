#include "myHeader.h"
#include <math.h>
#include <string.h>

// 在文件开头，定义积分分离阈值（根据实际系统调整此值）
#define Outer_INTEGRAL_SEPARATION_THRESHOLD 200
#define INTEGRAL_SEPARATION_THRESHOLD 200

static void vPIDTask(void *pvParameters);
static void PIDControl(uint8_t index, struct MOTOR *motor);


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
    ////////////////////////////////////////////////////////*获取实际速度值*/
    ///////////////////////////////////////////////////////Actual = Motor1_Speed;
    /* 将菜单设定的 BaseSpeed 作为目标速度（setpoint） */
    motor->Target = (float)BaseSpeed;
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
            
        //motor -> Out = 0;
        /*输出限幅*/
        //if (motor ->Out > 100) {motor ->Out = 100;}     //限制输出值最大为100
        if (motor ->Out > 100) {motor ->Out = 100;}     //限制输出值最大为100
        if (motor ->Out < -100) {motor ->Out = -100;}   //限制输出值最小为-100
        //if (motor ->Out < -10) {motor ->Out = -10;}   //限制输出值最小为-100
        if(motor ->Target == 0 && motor ->Actual == 0) motor ->Out = 0;

        //if(Out == 0) Motor_SetMode(1, Motor_Mode_break);
        //else if(Out > 0) Motor_SetMode(1, Motor_Mode_frd_rotation);
        //else if (Out < 0) Motor_SetMode(1, Motor_Mode_rvs_rotation)
    } 

	/*执行控制*/
    Motor_SetPWM(index, (motor ->Out));

    OLED_ShowSignedNum(56, 56, Motor1_Data.Out, 3, OLED_6X8);
    OLED_UpdateArea(1, 56, 24, 8);
}
