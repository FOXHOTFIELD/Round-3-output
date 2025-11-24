#include "MyHeader.h"

void PWM_Init(void)
{
    /*PWM输出*/
        /*PWMA -> PA15 TIM2_CH1*/

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //重映射PA0

    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
    // 常用配置：禁用JTAG，但保留SWD（可用于ST-LINK调试）
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitTypeDef GPIO_IS;
    GPIO_IS.GPIO_Mode = GPIO_Mode_AF_PP;            //TIM2_CH1_ETR
    GPIO_IS.GPIO_Pin = GPIO_Pin_15;                  //PWMA TIM2
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_IS);

    TIM_TimeBaseInitTypeDef TIM_TBIS;
    TIM_TBIS.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TBIS.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TBIS.TIM_Period = 100 - 1;                // ARR = 100 |Out| < 100
    TIM_TBIS.TIM_Prescaler = 72 - 1;              //10k Hz
    TIM_TBIS.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TBIS); 

    TIM_OCInitTypeDef TIM_OCIS;
    TIM_OCStructInit(&TIM_OCIS);

    TIM_OCIS.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCIS.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCIS.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCIS.TIM_Pulse = 0;
    TIM_OC1Init(TIM2, &TIM_OCIS);               //在这里启动TIM2_CH1的OC

    TIM_Cmd(TIM2, ENABLE);


        /*PWMB -> PA6 TIM3_CH1*/

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);


    //GPIO_InitTypeDef GPIO_IS;
    GPIO_IS.GPIO_Mode = GPIO_Mode_AF_PP;            //TIM3_CH1 使用复用推挽输出
    GPIO_IS.GPIO_Pin = GPIO_Pin_6;                  //PWMB TIM3
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_IS);

    //TIM_TimeBaseInitTypeDef TIM_TBIS;
    TIM_TBIS.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TBIS.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TBIS.TIM_Period = 100 - 1;                // ARR = 100 |Out| < 100
    TIM_TBIS.TIM_Prescaler = 72 - 1;              //10k Hz
    TIM_TBIS.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TBIS); 

    //TIM_OCInitTypeDef TIM_OCIS;
    TIM_OCStructInit(&TIM_OCIS);

    TIM_OCIS.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCIS.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCIS.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCIS.TIM_Pulse = 0;
    TIM_OC1Init(TIM3, &TIM_OCIS);               //在这里启动TIM3_CH1的OC

    TIM_Cmd(TIM3, ENABLE);


    /*电机1模式输出*/
        /*AIN1 -> PA11 AIN2 -> PA12*/
    GPIO_IS.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_IS.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;     //AIN1 AIN2
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_IS);

    /*电机2模式输出*/
        /*BIN1 -> PA4 BIN2 -> PA5*/
    GPIO_IS.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_IS.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;     //BIN1 BIN2
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_IS);
    
    Motor_SetMode(1, Motor_Mode_stop);                 //初始化均为高电平 制动模式
    Motor_SetMode(2, Motor_Mode_stop);

    /*STBY控制*/
        /*STBY -> B0*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_IS.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_IS.GPIO_Pin = GPIO_Pin_0;
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_IS);

    STBY_cmd(ENABLE);


}

void STBY_cmd(FunctionalState state)
{
    if(state == ENABLE){
        GPIO_SetBits(GPIOB, GPIO_Pin_0);
    }else if(state == DISABLE){
        GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    }
}

void TIM1_Init(void)                //定时中断 10ms
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 1. 使能TIM1和GPIO时钟（若使用复用功能引脚）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // 2. 配置定时器基本参数
    // 目标：10ms定时。假设系统时钟72MHz，TIM1挂载在APB2（通常与系统时钟同频）
    // 设置预分频器为7199，则计数器时钟频率 = 72MHz / (7199 + 1) = 10kHz（周期0.1ms）
    // 设置自动重载值ARR为99，则定时周期 = (99 + 1) * 0.1ms = 10ms
    TIM_TimeBaseInitStruct.TIM_Prescaler = 7199;          // 预分频值
    TIM_TimeBaseInitStruct.TIM_Period = 99;               // 自动重载值ARR，改为99以实现10ms
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;     // 时钟分频
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;     // 重复计数器（高级定时器特有）
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

    // 3. 使能更新中断
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);            // 使能更新中断

    // 4. 配置NVIC（嵌套向量中断控制器）
    NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_IRQn;       // TIM1更新中断通道
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;       // 子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // 5. 启动定时器
    TIM_Cmd(TIM1, ENABLE);                                 // 启动TIM1
}

// 6. 编写中断服务函数
void TIM1_UP_IRQHandler(void)                             // TIM1更新中断服务函数[4,5](@ref)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)    // 检查更新中断标志
    {

        //PIDControl();
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);       // 清除中断标志位[4,5](@ref)
    }
}


/**
  * 函    数：PWM1设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~100
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void Motor1_SetCompare(uint16_t Compare)
{
	TIM_SetCompare1(TIM2, Compare);		//设置CCR1的值
}

/**
  * 函    数：PWM2设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~100
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void Motor2_SetCompare(uint16_t Compare)
{
	TIM_SetCompare1(TIM3, Compare);		//设置CCR2的值
}

void Motor_SetMode(int numMotor, enum Motor_Mode Mode)
{
    if(numMotor == 1){
        switch (Mode)
        {
        case Motor_Mode_break:
            GPIO_SetBits(GPIOA, GPIO_Pin_11);
            GPIO_SetBits(GPIOA, GPIO_Pin_12);
            break;
        case Motor_Mode_stop:
            GPIO_ResetBits(GPIOA, GPIO_Pin_11);
            GPIO_ResetBits(GPIOA, GPIO_Pin_12);
            break;
        case Motor_Mode_frd_rotation:
            GPIO_SetBits(GPIOA, GPIO_Pin_11);
            GPIO_ResetBits(GPIOA, GPIO_Pin_12);
            break;
        case Motor_Mode_rvs_rotation:
            GPIO_ResetBits(GPIOA, GPIO_Pin_11);
            GPIO_SetBits(GPIOA, GPIO_Pin_12);
            break;
        default:
            break;
        }
        Motor1_Mode = Mode;
    }

    else if(numMotor == 2){
        switch (Mode)
                {
        case Motor_Mode_break:
            GPIO_SetBits(GPIOA, GPIO_Pin_4);
            GPIO_SetBits(GPIOA, GPIO_Pin_5);
            break;
        case Motor_Mode_stop:
            GPIO_ResetBits(GPIOA, GPIO_Pin_4);
            GPIO_ResetBits(GPIOA, GPIO_Pin_5);
            break;
        case Motor_Mode_frd_rotation:
            GPIO_SetBits(GPIOA, GPIO_Pin_4);
            GPIO_ResetBits(GPIOA, GPIO_Pin_5);
            break;
        case Motor_Mode_rvs_rotation:
            GPIO_ResetBits(GPIOA, GPIO_Pin_4);
            GPIO_SetBits(GPIOA, GPIO_Pin_5);
            break;
        default:
            break;
        }
        Motor2_Mode = Mode;
    }
}
/**
  * 函    数：直流电机设置PWM
  * 参    数：index 选择设置哪个电机 1或2
  * 参    数：PWM 要设置的PWM值，范围：-100~100（负数为反转）
  * 返 回 值：无
  */
void Motor_SetPWM(uint8_t index, int8_t PWM)
{
    switch (index)
    {
    case 1:
        /* code */
        if (PWM >= 0)							//如果设置正转的PWM
        {
            //GPIO_ResetBits(GPIOB, GPIO_Pin_12);	//PB12置低电平
            //GPIO_SetBits(GPIOB, GPIO_Pin_13);	//PB13置高电平
            Motor_SetMode(1, Motor_Mode_frd_rotation);
            Motor1_SetCompare(PWM);				//设置PWM占空比
        }
        else									//否则，即设置反转的速度值
        {
            //GPIO_SetBits(GPIOB, GPIO_Pin_12);	//PB12置高电平
            //GPIO_ResetBits(GPIOB, GPIO_Pin_13);	//PB13置低电平
            Motor_SetMode(1, Motor_Mode_rvs_rotation);
            Motor1_SetCompare(-PWM);				//设置PWM占空比
        }
        break;
    case 2:
        if (PWM >= 0)							//如果设置正转的PWM
        {
            //GPIO_ResetBits(GPIOB, GPIO_Pin_12);	//PB12置低电平
            //GPIO_SetBits(GPIOB, GPIO_Pin_13);	//PB13置高电平
            Motor_SetMode(2, Motor_Mode_frd_rotation);
            Motor2_SetCompare(PWM);				//设置PWM占空比
        }
        else									//否则，即设置反转的速度值
        {
            //GPIO_SetBits(GPIOB, GPIO_Pin_12);	//PB12置高电平
            //GPIO_ResetBits(GPIOB, GPIO_Pin_13);	//PB13置低电平
            Motor_SetMode(2, Motor_Mode_rvs_rotation);
            Motor2_SetCompare(-PWM);				//设置PWM占空比
        }
    default:
        break;
    }
}

/**
  * 函    数：直流电机2设置PWM
  * 参    数：PWM 要设置的PWM值，范围：-100~100（负数为反转）
  * 返 回 值：无
  */
void Motor2_SetPWM(int8_t PWM)
{
}

/**
  * 函    数：PWM设置PSC
  * 参    数：Prescaler 要写入的PSC的值，范围：0~65535
  * 返 回 值：无
  * 注意事项：PSC和ARR共同决定频率，此函数仅设置PSC的值，并不直接是频率
  *           频率Freq = CK_PSC / (PSC + 1) / (ARR + 1)
  */
void Motor1_SetPrescaler(uint16_t Prescaler)
{
	TIM_PrescalerConfig(TIM2, Prescaler, TIM_PSCReloadMode_Immediate);		//设置PSC的值
}

