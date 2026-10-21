#include "LED.h"
void LED_Change(uint8_t state)
{
    if (state)
        HAL_GPIO_WritePin(UserLED_GPIO_Port, UserLED_Pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(UserLED_GPIO_Port, UserLED_Pin, GPIO_PIN_SET);
}
