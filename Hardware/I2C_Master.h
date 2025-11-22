/* I2C_Master.h
 * 简单的 I2C 主机读取帮助器与 FreeRTOS 任务声明
 * 从从机（7 位地址 0x30）周期性读取 8 字节并解析为 th1/th2/th3/enc
 */
#ifndef __I2C_MASTER_H
#define __I2C_MASTER_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * 阻塞式读取函数：从 7 位从机地址 `slaveAddr7` 读取 `len` 字节到 `buf`。
 * 函数会阻塞直到读取完成或超时（由 `xTimeout` 指定，单位为 RTOS tick）。
 * 返回值：`pdPASS` 表示成功，`pdFAIL` 表示出错或超时。
 */
BaseType_t I2C_Master_ReadBytes(I2C_TypeDef *I2Cx, uint8_t slaveAddr7, uint8_t *buf, uint16_t len, TickType_t xTimeout);
void I2C2_Init(void);

/*
 * FreeRTOS 任务：示例任务，每隔固定周期从从机 0x30 读取 8 字节并解析。
 * 解析规则（小端）：
 *   th1 = buf[0] | (buf[1] << 8)  // uint16_t
 *   th2 = buf[2] | (buf[3] << 8)  // uint16_t
 *   th3 = buf[4] | (buf[5] << 8)  // uint16_t
 *   enc = buf[6] | (buf[7] << 8)  // int16_t
 * 使用示例：
 *   xTaskCreate(vThrd_I2C_MasterTask, "I2C_MST", 256, NULL, tskIDLE_PRIORITY+2, NULL);
 */
void vThrd_I2C_MasterTask(void *pvParameters);

#endif /* __I2C_MASTER_H */
