#include "MyHeader.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define SIZE_OF_RXPACKET 64
char Serial_RxPacket[SIZE_OF_RXPACKET];				//定义接收数据包数组，数据包格式"@MSG\r\n"
uint8_t Serial_RxData;		//(uint8_t)定义串口接收的数据变量
uint8_t Serial_RxFlag;		//(uint8_t)定义串口接收的标志位变量			//定义接收数据包标志位
#define JUSTFLOAT_TAIL   {0x00, 0x00, 0x80, 0x7f} // 帧尾[1]

/*Freertos*/
QueueHandle_t xRxQueue = NULL;          //(QueueHandle_t)Freertos下队列 传输input数据
SemaphoreHandle_t xSerialSemphr = NULL; //(SemaphoreHandle_t)Freertos下的信号量 标记来自input数据收到

/* =========================  第二串口 USART2 相关变量  ========================= */
#define SIZE_OF_RX2PACKET 64                 // USART2 接收包大小（与1号串口一致）
char Serial2_RxPacket[SIZE_OF_RX2PACKET];    // USART2 接收数据包缓存
static uint8_t Serial2_RxData;               // USART2 最近接收的字节
uint8_t Serial2_RxFlag;                      // USART2 接收完成标志
QueueHandle_t xRx2Queue = NULL;              // USART2 接收队列

/* =========================  第二串口接收处理任务声明  ========================= */
void Serial2_rxTask(void *pvParameters);

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//开启USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);    //开启GPIOB的时钟（改用PB6/PB7）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);     //开启AFIO时钟以支持引脚重映射
	/*重映射USART1到PB6/PB7*/
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	/* USART1 remapped pins: PB6 -> TX (AF_PP), PB7 -> RX (IPU) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; /* PB6 TX */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);                    //将PB6引脚初始化为复用推挽输出（USART1 TX）

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; /* PB7 RX */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);                    //将PB7引脚初始化为上拉输入（USART1 RX）

	USART_InitTypeDef USART_InitStructure;                    //定义结构体变量
	USART_InitStructure.USART_BaudRate = 115200;				//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式和接收模式均选择
	USART_InitStructure.USART_Parity = USART_Parity_No;		//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位
	USART_Init(USART1, &USART_InitStructure);				//将结构体变量交给USART_Init，配置USART1

	/*中断输出配置*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
	
	/*NVIC中断分组：使用分组4，全部为抢占优先级，便于与FreeRTOS对齐*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//选择配置NVIC的USART1线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY; // 数值大=优先级低，满足可调用FromISR
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*USART使能*/
	USART_Cmd(USART1, ENABLE);								//使能USART1，串口开始运行

	/* 先创建串口接收队列与任务，避免中断早到导致空指针 */
	xRxQueue = xQueueCreate(5, sizeof(Serial_RxPacket));
	if(xRxQueue == NULL) return;
	BaseType_t xReturn = pdFAIL;
	xReturn = xTaskCreate(vSerial_rxTask, "vSerial_rxTask", 512, NULL, tskIDLE_PRIORITY+3, NULL);
	if(xReturn == pdFAIL) return;

    xSerialSemphr = xSemaphoreCreateBinary();
    //xSemaphoreGive(xSerialSemphr);

    //OLED_ShowString(56, 1, "SerialOK", OLED_6X8);
}

/*处理Serial*/
void vSerial_rxTask(void *pvParameters)
{
    (void)pvParameters;
    char Rx_buf[SIZE_OF_RXPACKET];
	for(;;){
		if (xQueueReceive(xRxQueue, Rx_buf, portMAX_DELAY) == pdPASS)		//如果接收到数据包
		{
            //sscanf(Rx_buf, "%4d%4d", &(Motor1_Data.Actual), &(Motor2_Data.Actual));
            //OLED_ShowSignedNum(1, 56, Motor1_Data.Actual, 4, OLED_6X8);
            //OLED_UpdateArea(1, 47, );
			/* 解析通过 Serial_mySend 发送的小端 int16_t 数据：s1L,s1H,s2L,s2H */
			int16_t s1 = (int16_t)(
				((uint16_t)(uint8_t)Rx_buf[0]) |
				((uint16_t)(uint8_t)Rx_buf[1] << 8)
			);
			int16_t s2 = (int16_t)(
				((uint16_t)(uint8_t)Rx_buf[2]) |
				((uint16_t)(uint8_t)Rx_buf[3] << 8)
			);

			/* 直接赋值 int16 到 Actual */
			Motor1_Data.Actual = s1;
			Motor2_Data.Actual = s2;

            OLED_ShowSignedNum(1, 56, Motor1_Data.Actual, 4, OLED_6X8);
            OLED_ShowSignedNum(31, 56, Motor2_Data.Actual, 4, OLED_6X8);
            
            if(oled_update_blocked != pdTRUE) OLED_Update();

            xSemaphoreGive(xSerialSemphr);

			Serial_RxFlag = 0;			//处理完成后，需要将接收数据包标志位清零，否则将无法接收后续数据包
		}
    }
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
	}
	return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}

/**
  * 函    数：自己封装的prinf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial_SendString(String);		//串口发送字符数组（字符串）
}

/**
  * 函    数：获取串口接收标志位
  * 参    数：无
  * 返 回 值：串口接收标志位，范围：0~1，接收到数据后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)			//如果标志位为1
	{
		Serial_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}

/**
  * 函    数：获取串口接收的数据
  * 参    数：无
  * 返 回 值：接收的数据，范围：0~255
  */
uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;			//返回接收的数据变量
}

/**
  * 函    数：串口发送VOFA+ JustFloat协议格式数据
  * 参    数：data 浮点型数组的首地址
  * 参    数：num 浮点型数组的长度（通道数量）
  * 返 回 值：无
  */
void Serial_SendJustFloat(float *data, uint16_t num)
{
    uint8_t i;
    uint8_t *dataBytes;
    //uint8_t header[] = JUSTFLOAT_HEADER;
    uint8_t tail[] = JUSTFLOAT_TAIL;
    
    /*发送帧头*/
    //Serial_SendArray(header, sizeof(header));
    
    /*发送浮点数据（小端模式）*/
    for (i = 0; i < num; i++) {
        dataBytes = (uint8_t *)&data[i];
        Serial_SendArray(dataBytes, 4); // 每个float占4字节
    }
    
    /*发送帧尾*/
    Serial_SendArray(tail, sizeof(tail));
}

/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;		//定义表示当前状态机状态的静态变量
	static uint8_t pRxPacket = 0;	//定义表示当前接收数据位置的静态变量
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)	//判断是否是USART1的接收事件触发的中断
	{
		uint8_t RxData = USART_ReceiveData(USART1);			//读取数据寄存器，存放在接收的数据变量
		
		/*使用状态机的思路，依次处理数据包的不同部分*/
		
		/*当前状态为0，接收数据包包头*/
		if (RxState == 0)
		{
			if (RxData == '@' && Serial_RxFlag == 0)		//如果数据确实是包头，并且上一个数据包已处理完毕
			{
				RxState = 1;			//置下一个状态
				pRxPacket = 0;			//数据包的位置归零
			}
		}
		/*当前状态为1，接收数据包数据，同时判断是否接收到了第一个包尾*/
		else if (RxState == 1)
		{
			if (RxData == '\r')			//如果收到第一个包尾
			{
				RxState = 2;			//置下一个状态
			}
			else					//接收到了正常的数据
			{
				if (pRxPacket < SIZE_OF_RXPACKET - 1)
				{
					Serial_RxPacket[pRxPacket] = RxData;		//将数据存入数据包数组的指定位置
					pRxPacket ++;			//数据包的位置自增
				}
			}
		}
		/*当前状态为2，接收数据包第二个包尾*/
		else if (RxState == 2)
		{
			if (RxData == '\n')			//如果收到第二个包尾
			{

				RxState = 0;			//状态归0
				Serial_RxPacket[pRxPacket] = '\0';			//将收到的字符数据包添加一个字符串结束标志
				Serial_RxFlag = 1;		//接收数据包标志位置1，成功接收一个数据包

				if (xRxQueue != NULL) {
					BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					xQueueSendFromISR(xRxQueue, Serial_RxPacket, &xHigherPriorityTaskWoken);
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				}
			}
		}
		
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);		//清除标志位
	}
}

/* ============================================================================
 * 函数：Serial2_Init
 * 作用：初始化 USART2，实现与现有 USART1 基本一致的功能，包含：
 *       - PA2 (TX) 推挽复用输出，PA3 (RX) 上拉输入
 *       - 9600 波特率，8-N-1 格式
 *       - 开启接收中断，建立接收队列与任务
 * 注意：须确保 FreeRTOS 已正常启动环境；若初始化失败则直接返回。
 * ============================================================================ */
void Serial2_Init(void)
{
	/* 开启外设与 GPIO 时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // USART2 位于 APB1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // PA2 / PA3 引脚

	/* 配置 GPIOA PA2 -> TX 复用推挽 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 配置 GPIOA PA3 -> RX 上拉输入 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 配置 USART2 参数 */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;                                   // 波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;              // 允许收发
	USART_InitStructure.USART_Parity = USART_Parity_No;                          // 无奇偶校验
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                       // 1 位停止位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                  // 8 位数据位
	USART_Init(USART2, &USART_InitStructure);

	/* 创建接收队列和任务（先于中断使能） */
	xRx2Queue = xQueueCreate(5, sizeof(Serial2_RxPacket));
	if (xRx2Queue == NULL) return;
	BaseType_t xReturn = xTaskCreate(Serial2_rxTask, "Serial2_rxTask", 512, NULL, tskIDLE_PRIORITY+2, NULL);
	if (xReturn == pdFAIL) return;
	xReturn = xTaskCreate(vHostTask, "vHostTask", 128, NULL, tskIDLE_PRIORITY+1, NULL);
	if (xReturn == pdFAIL) return;

	/* 开启接收中断 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/* NVIC 配置（优先级分组已在 Serial_Init 中设为 Group4，可重复调用） */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY; // 可安全使用 FromISR
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	/* 使能 USART2 */
	USART_Cmd(USART2, ENABLE);
	//OLED_ShowString(56, 2, "Serial2OK", OLED_6X8);
}

//向主机上传数据
//在Serial2_Init中创建
void vHostTask(void *pvParameters)
{
    float arr[3];
    for(;;)
    {
//        arr[0] = Motor1_Data.Actual;
        //arr[1] = Motor1_Data.Out;
        //arr[2] = Motor1_Data.Target;
        //OLED_ShowSignedNum(15, 1, Motor1_Data.Target, 2, OLED_6X8);
        arr[0] = Motor2_Data.Actual;
        arr[1] = Motor2_Data.Out;
        arr[2] = Motor2_Data.Target;
        Serial2_SendJustFloat(arr, 3);
    }
}

/* ============================================================================
 * 函数：Serial2_rxTask
 * 作用：USART2 接收数据包处理任务，与 Serial_rxTask 类似。
 * 说明：当前示例仅演示解析与显示，可按需扩展。
 * ============================================================================ */
void Serial2_rxTask(void *pvParameters)
{
	(void)pvParameters;
	char Rx_buf[SIZE_OF_RX2PACKET];
	for(;;){
		if (xQueueReceive(xRx2Queue, Rx_buf, portMAX_DELAY) == pdPASS)
		{
            	float data = 0;
				char Cmd;
            	sscanf(Rx_buf, "%c%f", &Cmd, &data);
				//OLED_ShowString(2, 5, Cmd);
                //OLED_ShowString(10, 1, Rx_buf, OLED_6X8);
				if(Cmd == 'S') {
                    //BaseSpeed = data;
                    BaseSpeed = data;
                    OLED_ClearArea(100, 17, 8, 16);
                    OLED_ShowSignedNum(100, 17, BaseSpeed, 2, OLED_8X16);
                }
				//else if(Cmd == 'i') Motor1_Data.Ki = data;
				//else if(Cmd == 'p') Motor1_Data.Kp = data;
				//else if(Cmd == 'd') Motor1_Data.Kd = data;
				//OLED_ShowFloatNum(15, 1, Motor1_Data.Kp,1, 2, OLED_6X8);
				//OLED_ShowFloatNum(50, 1, Motor1_Data.Ki,1, 2, OLED_6X8);
				//OLED_ShowFloatNum(85, 1, Motor1_Data.Kd,1, 2, OLED_6X8);
				else if(Cmd == 'i') Motor2_Data.Ki = data;
				else if(Cmd == 'p') Motor2_Data.Kp = data;
				else if(Cmd == 'd') Motor2_Data.Kd = data;


				OLED_ShowFloatNum(15, 1, Motor2_Data.Kp,1, 2, OLED_6X8);
				OLED_ShowFloatNum(50, 1, Motor2_Data.Ki,1, 2, OLED_6X8);
                OLED_ShowFloatNum(85, 1, Motor2_Data.Kd, 1, 2, OLED_6X8);
                if(oled_update_blocked != pdTRUE) OLED_Update();
                Serial2_RxFlag = 0;
		}
	}
}

/*
 * 函数：Serial2_SendByte
 * 作用：USART2 发送单个字节
 */
void Serial2_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/* ============================================================================
 * 函数：Serial2_SendArray
 * ============================================================================ */
void Serial2_SendArray(uint8_t *Array, uint16_t Length)
{
	for(uint16_t i=0; i<Length; i++){
		Serial2_SendByte(Array[i]);
	}
}

/* ============================================================================
 * 函数：Serial2_SendString
 * ============================================================================ */
void Serial2_SendString(char *String)
{
	for(uint16_t i=0; String[i] != '\0'; i++){
		Serial2_SendByte(String[i]);
	}
}

/* ============================================================================
 * 函数：Serial2_SendNumber
 * 说明：定长数字输出，不足位数前面补 0
 * ============================================================================ */
void Serial2_SendNumber(uint32_t Number, uint8_t Length)
{
	for(uint8_t i=0; i<Length; i++){
		Serial2_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/* ============================================================================
 * 函数：Serial2_Printf
 * ============================================================================ */
void Serial2_Printf(char *format, ...)
{
	char buf[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(buf, format, arg);
	va_end(arg);
	Serial2_SendString(buf);
}

/* ============================================================================
 * 函数：Serial2_GetRxFlag / Serial2_GetRxData
 * ============================================================================ */
uint8_t Serial2_GetRxFlag(void)
{
	if(Serial2_RxFlag){
		Serial2_RxFlag = 0;
		return 1;
	}
	return 0;
}

uint8_t Serial2_GetRxData(void)
{
	return Serial2_RxData;
}

/* ============================================================================
 * 函数：Serial2_SendJustFloat
 * 作用：VOFA+ JustFloat 协议简单帧（与 Serial_SendJustFloat 相同）
 * ============================================================================ */
void Serial2_SendJustFloat(float *data, uint16_t num)
{
	uint8_t tail[] = JUSTFLOAT_TAIL;
	for(uint16_t i=0; i<num; i++){
		uint8_t *p = (uint8_t *)&data[i];
		Serial2_SendArray(p, 4);
	}
	Serial2_SendArray(tail, sizeof(tail));
}

/* ============================================================================
 * 函数：USART2_IRQHandler
 * 作用：USART2 中断服务函数，实现与 USART1 一致的包协议：@ 开始，\r\n 结束
 * ============================================================================ */
void USART2_IRQHandler(void)
{
	static uint8_t RxState = 0;      // 状态机当前状态
	static uint8_t pRxPacket = 0;    // 当前填充位置索引
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART2); // 读取数据
		Serial2_RxData = RxData;                    // 保存最后一个字节

		if (RxState == 0)
		{
			if (RxData == '@' && Serial2_RxFlag == 0)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			if (RxData == '\r')
			{
				RxState = 2;
			}
			else
			{
				if (pRxPacket < SIZE_OF_RX2PACKET - 1)
				{
					Serial2_RxPacket[pRxPacket++] = RxData;
				}
			}
		}
		else if (RxState == 2)
		{
			if (RxData == '\n')
			{
				RxState = 0;
				Serial2_RxPacket[pRxPacket] = '\0';
				Serial2_RxFlag = 1;
				if (xRx2Queue != NULL) {
					BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					xQueueSendFromISR(xRx2Queue, Serial2_RxPacket, &xHigherPriorityTaskWoken);
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				}
			}
		}
		USART_ClearITPendingBit(USART2, USART_IT_RXNE); // 清除中断标志
	}
}


