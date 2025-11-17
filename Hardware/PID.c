#include "myHeader.h"

// 在文件开头，定义积分分离阈值（根据实际系统调整此值）
#define Outer_INTEGRAL_SEPARATION_THRESHOLD 200
#define INTEGRAL_SEPARATION_THRESHOLD 200

void PIDControl(void)
{
    ///*获取实际速度值*/
    //Actual = Motor1_Speed;
        
    ///*获取本次误差、上次误差和上上次误差*/
    //Error2 = Error1;            //获取上上次误差
    //Error1 = Error0;            //获取上次误差
    //Error0 = Target - Actual;    //获取本次误差


    //if((Error0 > 0 ? Error0 : -Error0) > 1)
    //{
    
        ///*积分分离PID计算*/
        //float deltaP = Kp * (Error0 - Error1);  //比例项
        //float deltaD = Kd * (Error0 - 2 * Error1 + Error2); //微分项
        //float deltaI = 0;  //积分项，默认设为0（分离状态）
    
    //// 积分分离判断：只有当误差在阈值内时才加入积分作用[1,2](@ref)
        //if((Error0 >= 0 ? Error0 : -Error0) <= INTEGRAL_SEPARATION_THRESHOLD)
        //{
            //deltaI = Ki * Error0;  //引入积分控制
        //}
    
        ////计算总输出增量
        //Out += deltaP + deltaI + deltaD;
            
        ///*输出限幅*/
        //if (Out > 100) {Out = 100;}     //限制输出值最大为100
        //if (Out < -100) {Out = -100;}   //限制输出值最小为-100
        //if(Target == 0 && Actual == 0) Out = 0;

        ////if(Out == 0) Motor_SetMode(1, Motor_Mode_break);
        ////else if(Out > 0) Motor_SetMode(1, Motor_Mode_frd_rotation);
        ////else if (Out < 0) Motor_SetMode(1, Motor_Mode_rvs_rotation)
    //} 
	/*执行控制*/
	//Motor1_SetPWM(Out);
}
