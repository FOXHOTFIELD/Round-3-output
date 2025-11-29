#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>
#include <stdio.h>

/*
 * 串口引脚切换宏：
 *  - 默认使用 USART1 的 PA9(TX) / PA10(RX)
 *  - 若希望使用 PA2(TX) / PA3(RX)（即 USART2），请取消下面的宏注释：
 */
// #define SERIAL_USE_PA23    // 注释掉表示使用默认 PA9/PA10；取消注释则使用 PA2/PA3

/* 根据是否定义 SERIAL_USE_PA23，选择具体的 TX/RX 引脚宏，便于在实现中使用 */
#ifdef SERIAL_USE_PA23
#define SERIAL_TX_PIN GPIO_Pin_2   // PA2 作为 TX
#define SERIAL_RX_PIN GPIO_Pin_3   // PA3 作为 RX
#else
#define SERIAL_TX_PIN GPIO_Pin_9   // PA9 作为 TX
#define SERIAL_RX_PIN GPIO_Pin_10  // PA10 作为 RX
#endif

extern char Serial_RxPacket[];
extern QueueHandle_t Serial_RxSemaphore;

/* 新增：串口接收信号量外部声明 */
extern SemaphoreHandle_t xSerialSemphr;

/* 新增：USART2 的接收缓冲与标志变量外部声明 */
extern char Serial2_RxPacket[];
extern uint8_t Serial2_RxFlag;

void Serial_Init(void);
void Serial_DeInit(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);
void Serial_SendJustFloat(float *data, uint16_t num);
void Serial_mySendString(char *cmd);

/* 以下两个接口用于获取接收标志与接收数据 */
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

/* ================= 新增 USART2 相关接口 ================= */
void Serial2_Init(void);              // USART2 初始化（PA2/PA3）
void Serial2_SendByte(uint8_t Byte);  // USART2 发送单字节
void Serial2_SendArray(uint8_t *Array, uint16_t Length);   // USART2 发送数组
void Serial2_SendString(char *String);                     // USART2 发送字符串
void Serial2_SendNumber(uint32_t Number, uint8_t Length);  // USART2 发送数字（定长）
void Serial2_Printf(char *format, ...);                    // USART2 printf 封装
uint8_t Serial2_GetRxFlag(void);      // USART2 获取接收完成标志
uint8_t Serial2_GetRxData(void);      // USART2 获取最后接收字节
void Serial2_SendJustFloat(float *data, uint16_t num);     // USART2 发送 VOFA+ JustFloat 数据
void vHostTask(void *pvParameters);

#endif
