#include "myHeader.h"
#define num_of_option (3)

// 按键事件队列句柄（模块内部使用）
static QueueHandle_t xKeyQueue = NULL;
// 菜单显示任务句柄，用于接收通知唤醒刷新
static TaskHandle_t xMenuShowHandle = NULL;
// 当某些操作（如准备运行）需要临时阻止自动刷新时置位
static volatile BaseType_t oled_update_blocked = pdFALSE;

char menu[num_of_option][16] = {
    {"  Start"},
    {"  BaseSpeed"},
    {"  Thrd Correct"}
};

// 按键轮询任务：轮询 GPIO，消抖后在检测到按下时发送事件到队列
static void Menu_KeyTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        // 上键（PA8）按下检测（下拉为 0，因为配置为上拉输入）
        if (GPIO_ReadInputDataBit(GPIOA, KEY_UP) == Bit_RESET) {
            // 简单消抖：延时后再次确认
            vTaskDelay(pdMS_TO_TICKS(20));
            if (GPIO_ReadInputDataBit(GPIOA, KEY_UP) == Bit_RESET) {
                KeyEvent_t ev = KEY_EVENT_UP;
                Menu_CMDProcess(ev);
                //xQueueSend(xKeyQueue, &ev, 0);
                // 等待按键释放，避免连发
                while (GPIO_ReadInputDataBit(GPIOA, KEY_UP) == Bit_RESET) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }

        // 其余按键（PB15/PB14/PB13）按下检测
        if (GPIO_ReadInputDataBit(GPIOB, KEY_DOWN) == Bit_RESET) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if (GPIO_ReadInputDataBit(GPIOB, KEY_DOWN) == Bit_RESET) {
                KeyEvent_t ev = KEY_EVENT_DOWN;
                Menu_CMDProcess(ev);
                //xQueueSend(xKeyQueue, &ev, 0);
                while (GPIO_ReadInputDataBit(GPIOB, KEY_DOWN) == Bit_RESET) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }

        if (GPIO_ReadInputDataBit(GPIOB, KEY_SURE) == Bit_RESET) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if (GPIO_ReadInputDataBit(GPIOB, KEY_SURE) == Bit_RESET) {
                KeyEvent_t ev = KEY_EVENT_SURE;
                Menu_CMDProcess(ev);
                //xQueueSend(xKeyQueue, &ev, 0);
                while (GPIO_ReadInputDataBit(GPIOB, KEY_SURE) == Bit_RESET) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }

        if (GPIO_ReadInputDataBit(GPIOB, KEY_BACK) == Bit_RESET) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if (GPIO_ReadInputDataBit(GPIOB, KEY_BACK) == Bit_RESET) {
                KeyEvent_t ev = KEY_EVENT_BACK;
                Menu_CMDProcess(ev);
                //xQueueSend(xKeyQueue, &ev, 0);
                while (GPIO_ReadInputDataBit(GPIOB, KEY_BACK) == Bit_RESET) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }

        // 轮询间隔，避免占用过多 CPU
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

 /*Menu显示任务
  * 不再周期刷新，而是等待来自 `Menu_CMDProcess` 的通知唤醒刷新。
  * 在某些操作（例如 Menu_prepareRun）期间可阻止刷新以避免与倒计时冲突。
  */
 static void Menu_ShowTask(void *pvParameters)
 {
    (void)pvParameters;

    for(;;){
        // 等待通知（从 Menu_CMDProcess 或初始化发送），被唤醒后执行一次刷新
        // 清除通知计数（pdTRUE）并阻塞直到下次通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 如果刷新被临时阻止则跳过（例如 prepareRun 期间）
        if (oled_update_blocked == pdTRUE) {
            continue;
        }

        OLED_ShowNum(120, 1, (int)curState.psost, 1, OLED_6X8);
        OLED_ShowNum(120, 9, (int)curState.mode, 1, OLED_6X8);
        //OLED_ShowNum(56, 56, flag, 3, OLED_6X8);
        /* 定期刷新 OLED 缓冲到显示，否则仅在其它地方调用 OLED_Update 时才会刷新
           之前的行为是：只有在按键处理（Menu_CMDProcess）调用了 OLED_Update，
           屏幕才会更新，所以看起来“第一次按键后才开始显示”。
           在这里增加刷新可以保证显示任务独立地更新屏幕。*/
        OLED_Update();
    }
 }
// 初始化菜单显示并创建按键队列与任务
void Menu_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_IS;
    GPIO_IS.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_IS.GPIO_Pin = KEY_UP;
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_IS);

    GPIO_IS.GPIO_Pin = KEY_DOWN | KEY_BACK | KEY_SURE;
    GPIO_Init(GPIOB, &GPIO_IS);

    for(int i = 0; i < num_of_option; i++){
        OLED_ShowString_simplified(i+1, menu[i]);
    }
    //OLED_ShowString_simplified(1, menu[0]);
    //OLED_ShowString_simplified(2, menu[1]);
    OLED_ShowString_simplified(1, "->");
    OLED_ShowSignedNum(100, 17, BaseSpeed, 2, OLED_8X16);

    // 创建按键事件队列（长度 10），每个元素为 KeyEvent_t
    if (xKeyQueue == NULL) {
        xKeyQueue = xQueueCreate(10, sizeof(KeyEvent_t));
    }

    // 创建轮询任务，优先级可根据程序其他任务调整
    if (xKeyQueue != NULL) {
        xTaskCreate(Menu_KeyTask, "MenuKey", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);
    }

    // 创建显示任务并保存句柄，以便其他地方通过任务通知唤醒
    xTaskCreate(Menu_ShowTask, "MenuShow", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, &xMenuShowHandle);

    // 给一次初始通知，让显示任务把初始值绘制到屏幕
    if (xMenuShowHandle != NULL) {
        xTaskNotifyGive(xMenuShowHandle);
    }
}

// 从按键队列取一条事件，超时返回 KEY_EVENT_NONE
KeyEvent_t Menu_GetKeyEvent(TickType_t xTicksToWait)
{
    KeyEvent_t ev = KEY_EVENT_NONE;
    if (xKeyQueue == NULL) return KEY_EVENT_NONE;
    if (xQueueReceive(xKeyQueue, &ev, xTicksToWait) == pdPASS) {
        return ev;
    }
    return KEY_EVENT_NONE;
}

/*
 *在Menu_KeyTask内部直接接受ev
 *转到具体功能实现
*/
void Menu_CMDProcess(KeyEvent_t ev)
{
    switch (curState.mode)
    {
    case Wait:
        if (ev == KEY_EVENT_UP || ev == KEY_EVENT_DOWN)
        {
            Menu_MovePoint(ev);
        }else if (ev == KEY_EVENT_SURE)
        {
            if(curState.psost == speed) Menu_turnEdit();
            if(curState.psost == start) Menu_prepareRun(ev);
        }else if (ev == KEY_EVENT_BACK)
        {
            /* code */
        }
        
        
        break;
    case Edit:
        if(ev == KEY_EVENT_SURE){
            Menu_turnEdit();
        }else if (ev == KEY_EVENT_UP || ev == KEY_EVENT_DOWN)
        {
            Menu_changeSpeed(ev);
        }
        
    default:
        break;
    }

    // 每次命令处理后通知显示任务刷新一次（如果可用）
    if (xMenuShowHandle != NULL) {
        xTaskNotifyGive(xMenuShowHandle);
    }

}
/*
 *移动光标
*/
void Menu_MovePoint(KeyEvent_t ev)
{
    OLED_ClearArea(1, 1, 16, 48);
    switch (ev)
    {
    case KEY_EVENT_UP:
        curState.psost = (MenuPost)(((int8_t)curState.psost - 1 + num_of_option ) % num_of_option);
        break;
    case KEY_EVENT_DOWN:
        curState.psost = (MenuPost)(((int8_t)curState.psost + 1 + num_of_option ) % num_of_option);
        break;

    default:
        break;
    }
    OLED_ShowString_simplified((int8_t)curState.psost+1, "->");
}

static void Menu_turnEdit(void)
{
    if(curState.mode == Edit){
        curState.mode = Wait;
        OLED_ClearArea(115, 40, 8, 16);
    }else if (curState.mode == Wait)
    {
        curState.mode = Edit;
        /* code */
        OLED_ShowString(115, 40, "E", OLED_8X16);
    }

}

static void Menu_changeSpeed(KeyEvent_t ev)
{
    OLED_ClearArea(100, 17, 8, 16);
    if(curState.mode == Edit && curState.psost == speed){
        BaseSpeed += (ev == KEY_EVENT_UP ? 5 : -5);
        //if (BaseSpeed > 99) BaseSpeed = 99;  // 与显示宽度/占空限制对齐
        //if (BaseSpeed < 0)  BaseSpeed = 0;
    }
    OLED_ShowSignedNum(100, 17, BaseSpeed, 2, OLED_8X16);
}

static void Menu_prepareRun(KeyEvent_t ev){
    // 在倒计时期间阻止显示任务的并发刷新，避免与倒计时画面冲突
    oled_update_blocked = pdTRUE;
    OLED_BufBackUp();
    OLED_Clear();
    for(int i = 3; i > 0; i--){
        OLED_Clear();
        OLED_ShowNum(60, 24, i, 1, OLED_8X16);
        OLED_DrawCircle(63, 32, 9, OLED_UNFILLED);
        OLED_Reverse();
        OLED_Update();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    OLED_BufRestore();
    // 结束倒计时后立刻更新一次物理屏幕
    OLED_Update();
    oled_update_blocked = pdFALSE;
    // 通知显示任务在恢复后做一次同步刷新（如果需要）
    if (xMenuShowHandle != NULL) {
        xTaskNotifyGive(xMenuShowHandle);
    }
}
