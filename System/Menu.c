#include "myHeader.h"

char menu[][16] = {
    {"    START"},
    {"    Speed"}
};

void Menu_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_IS;
    GPIO_IS.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_IS.GPIO_Pin = KEY_UP;
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_IS);

    GPIO_IS.GPIO_Pin = KEY_DOWN | KEY_BACK | KEY_SURE;
    GPIO_Init(GPIOB, &GPIO_IS);

    OLED_ShowString_simplified(1, menu[0]);
    OLED_ShowString_simplified(2, menu[1]);
    OLED_Update();
}
