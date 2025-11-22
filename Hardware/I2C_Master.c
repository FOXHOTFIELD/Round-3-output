/* I2C_Master.c
 * 使用软件（Bit-bang）I2C 的主机侧读取实现：用于从一个发送 8 字节数据的从机读取数据。
 * 保持原有引脚不变：PB10 = SCL, PB11 = SDA（与原硬件实现引脚一致）
 * 数据格式（小端）：uint16_t th1, uint16_t th2, uint16_t th3, int16_t enc
 * 注意：函数签名保持与原来一致，以便项目其它部分无需改动。
 */

#include "myHeader.h"

/* 可配置的软件 I2C 延时计数：用于调整 SCL 频率（假设 CPU=72MHz）
    估算：SOFT_I2C_DELAY_COUNT = 90 可使 SCL 接近 100 kHz（上板实测并微调） */
#ifndef SOFT_I2C_DELAY_COUNT
#define SOFT_I2C_DELAY_COUNT 90
#endif

/* 软件 I2C 使用的引脚定义：保持原引脚不变（PB10 SCL, PB11 SDA） */
#define SOFT_I2C_GPIO       GPIOB
#define SOFT_I2C_SCL_PIN    GPIO_Pin_10
#define SOFT_I2C_SDA_PIN    GPIO_Pin_11

/* 延时函数：用于控制时序，阻塞短延时即可，必要时可根据主频调整 */
static void SoftI2C_Delay(void)
{
    volatile int i = SOFT_I2C_DELAY_COUNT; /* 可通过宏调整以改变 SCL 频率 */
    while (i--) __asm volatile ("nop");
}

/* 将 SCL/SDA 设置为高（释放线）或低；线为开漏输出，写 1 = 释放 */
static inline void SOFT_SCL_HIGH(void)  { GPIO_SetBits(SOFT_I2C_GPIO, SOFT_I2C_SCL_PIN); }
static inline void SOFT_SCL_LOW(void)   { GPIO_ResetBits(SOFT_I2C_GPIO, SOFT_I2C_SCL_PIN); }
static inline void SOFT_SDA_HIGH(void)  { GPIO_SetBits(SOFT_I2C_GPIO, SOFT_I2C_SDA_PIN); }
static inline void SOFT_SDA_LOW(void)   { GPIO_ResetBits(SOFT_I2C_GPIO, SOFT_I2C_SDA_PIN); }
static inline uint8_t SOFT_SDA_READ(void){ return (uint8_t)GPIO_ReadInputDataBit(SOFT_I2C_GPIO, SOFT_I2C_SDA_PIN); }

/* 初始化：只启用 GPIOB 时钟并配置 PB10/PB11 为开漏输出，初始为高（释放） */
void I2C2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 只启用 GPIOB 时钟（保持引脚不变），不再启用硬件 I2C 外设 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* 配置 PB10 / PB11 为通用开漏输出，50MHz */
    GPIO_InitStructure.GPIO_Pin = SOFT_I2C_SCL_PIN | SOFT_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(SOFT_I2C_GPIO, &GPIO_InitStructure);

    /* 释放总线（拉高） */
    SOFT_SCL_HIGH();
    SOFT_SDA_HIGH();

    /* 创建 FreeRTOS 任务：与原来一致，任务内部会调用软件 I2C 读函数 */
    xTaskCreate(vThrd_I2C_MasterTask, "iicTask", 256, NULL, tskIDLE_PRIORITY+1, NULL);
}

/* ---------- 软件 I2C 辅助函数（标准 C 实现） ---------- */
static void SoftI2C_Start(void)
{
    SOFT_SDA_HIGH();
    SOFT_SCL_HIGH();
    SoftI2C_Delay();
    SOFT_SDA_LOW();
    SoftI2C_Delay();
    SOFT_SCL_LOW();
    SoftI2C_Delay();
}

static void SoftI2C_Stop(void)
{
    SOFT_SDA_LOW();
    SoftI2C_Delay();
    SOFT_SCL_HIGH();
    SoftI2C_Delay();
    SOFT_SDA_HIGH();
    SoftI2C_Delay();
}

static uint8_t SoftI2C_WriteByte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) SOFT_SDA_HIGH(); else SOFT_SDA_LOW();
        SoftI2C_Delay();
        SOFT_SCL_HIGH();
        SoftI2C_Delay();
        SOFT_SCL_LOW();
        SoftI2C_Delay();
        data <<= 1;
    }
    /* 读取 ACK：释放 SDA，拉高 SCL，再读取 SDA */
    SOFT_SDA_HIGH();
    SoftI2C_Delay();
    SOFT_SCL_HIGH();
    SoftI2C_Delay();
    uint8_t ack = (SOFT_SDA_READ() == 0) ? 1 : 0;
    SOFT_SCL_LOW();
    SoftI2C_Delay();
    return ack;
}

static uint8_t SoftI2C_ReadByte(uint8_t ack)
{
    uint8_t val = 0;
    SOFT_SDA_HIGH(); /* 释放 SDA，外部或上拉提供高电平 */
    for (int i = 0; i < 8; i++) {
        val <<= 1;
        SoftI2C_Delay();
        SOFT_SCL_HIGH();
        SoftI2C_Delay();
        if (SOFT_SDA_READ()) val |= 0x01;
        SOFT_SCL_LOW();
        SoftI2C_Delay();
    }
    /* 发送 ACK（低）或 NACK（高） */
    if (ack) SOFT_SDA_LOW(); else SOFT_SDA_HIGH();
    SoftI2C_Delay();
    SOFT_SCL_HIGH();
    SoftI2C_Delay();
    SOFT_SCL_LOW();
    SoftI2C_Delay();
    SOFT_SDA_HIGH();
    return val;
}

/* 阻塞式读取帮助函数：使用软件 I2C，从从机 `slaveAddr7` 读取 `len` 字节到 `buf`。
 * - 使用软件 I2C（bit-bang），保持 PB10/PB11 引脚不变
 * - 超时由 xTimeout 控制（以 Tick 为单位），超时返回 pdFAIL
 */
BaseType_t I2C_Master_ReadBytes(I2C_TypeDef *I2Cx, uint8_t slaveAddr7, uint8_t *buf, uint16_t len, TickType_t xTimeout)
{
    TickType_t start = xTaskGetTickCount();

    /* 超时检测宏 */
    #define CHECK_TIMEOUT() do { if ((xTaskGetTickCount() - start) > xTimeout) return pdFAIL; } while (0)

    if (len == 0) return pdPASS;

    /* 产生 START 并发送地址（读位=1） */
    SoftI2C_Start();
    CHECK_TIMEOUT();
    uint8_t addr_rw = (uint8_t)((slaveAddr7 << 1) | 0x01);
    if (!SoftI2C_WriteByte(addr_rw)) {
        SoftI2C_Stop();
        return pdFAIL + 7;
    }

    for (uint16_t i = 0; i < len; i++) {
        CHECK_TIMEOUT();
        uint8_t need_ack = (i < (len - 1)) ? 1 : 0; /* 最后一个字节发送 NACK */
        buf[i] = SoftI2C_ReadByte(need_ack);
    }

    SoftI2C_Stop();
    return pdPASS;
}


/*
 * FreeRTOS 示例任务：每 200 ms 从从机 0x30 读取 8 字节并按小端解析
 * 解析后的数据可在此处发送到队列、打印或用于控制逻辑。
 */
void vThrd_I2C_MasterTask(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t xDelay = pdMS_TO_TICKS(200);
    uint8_t rxbuf[8] = {0};
    static uint16_t i2c_err_cnt = 0;

    for (;;) {
        /* Read 8 bytes, timeout 50ms */
        int temp = 3;
        if ((temp = I2C_Master_ReadBytes(I2C2, 0x48, rxbuf, sizeof(rxbuf), pdMS_TO_TICKS(50)) )== pdPASS) {
            uint16_t th1 = (uint16_t)rxbuf[0] /*| ((uint16_t)rxbuf[1] << 8)*/;
            uint16_t th2 = (uint16_t)rxbuf[2] | ((uint16_t)rxbuf[3] << 8);
            uint16_t th3 = (uint16_t)rxbuf[4] | ((uint16_t)rxbuf[5] << 8);
            int16_t enc = (int16_t)((uint16_t)rxbuf[6] | ((uint16_t)rxbuf[7] << 8));
            //OLED_ShowNum(50, 50, temp, 1, OLED_6X8);
            /* TODO: 在此处处理解析结果，例如：发送到队列/打印/更新全局状态 */
            OLED_ShowNum(1, 56, th1, 4, OLED_6X8);
            OLED_ShowNum(56, 56, th2, 4, OLED_6X8);
            //OLED_ShowNum(26, 56, ++i2c_err_cnt, 3, OLED_6X8);
            OLED_ShowNum(26, 56, enc, 4, OLED_6X8);

            (void)th1; (void)th2; (void)th3; (void)enc;
        } else {
            /* 读取失败或超时：记录错误并在 OLED/串口上提示 */
            
            OLED_ShowString_simplified(4, "err");
            
            //Serial_Printf("I2C read failed, count=%u\r\n", (unsigned int)i2c_err_cnt);
            //OLED_Printf(64, 0, OLED_6X8, "I2C ERR:%u", (unsigned int)i2c_err_cnt);
            /* 若需要：当 i2c_err_cnt 超过阈值时可尝试重置 I2C 外设或通知其他任务 */
        }
        vTaskDelay(xDelay);
    }
}
