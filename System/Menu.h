#ifndef __MENU_H
#define __MENU_H
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define KEY_UP (GPIO_Pin_8)        //PA8
#define KEY_DOWN (GPIO_Pin_15)     //PB15
#define KEY_SURE (GPIO_Pin_14)     //PB14
#define KEY_BACK (GPIO_Pin_13)     //PB13

// 按键事件枚举：按下时会把对应事件放入队列
typedef enum {
	KEY_EVENT_NONE = 0,
	KEY_EVENT_UP,
	KEY_EVENT_DOWN,
	KEY_EVENT_SURE,
	KEY_EVENT_BACK
} KeyEvent_t;
typedef enum {
    Edit, //编辑参数
    Wait, //等待发车
    Run   //行驶
} MenuMode;

typedef enum {
    start,
    speed,
    correct
} MenuPost;

typedef struct MENUSTATE
{
    MenuMode mode;
    MenuPost psost;
} MenuState;

extern MenuState curState;

// 初始化菜单显示及按键轮询任务（调用后会创建任务和队列）
void Menu_Init(void);
void Menu_MovePoint(KeyEvent_t ev);
void Menu_CMDProcess(KeyEvent_t ev);
// 从按键事件队列获取一个事件，参数为等待的最大 tick 数，
// 若超时返回 KEY_EVENT_NONE
KeyEvent_t Menu_GetKeyEvent(TickType_t xTicksToWait);
static void Menu_turnEdit(void);
static void Menu_changeSpeed(KeyEvent_t ev);



#endif
