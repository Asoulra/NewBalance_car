#include "Key.h"

volatile uint8_t KEY_Flag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_press_tick = 0;
    static uint8_t has_last_press = 0;
    uint32_t now;

    if (GPIO_Pin == UsesKey_Pin &&
        HAL_GPIO_ReadPin(UsesKey_GPIO_Port, UsesKey_Pin) == GPIO_PIN_RESET)
    {
        now = HAL_GetTick();
        if (!has_last_press || now - last_press_tick >= 20U)
        {
            KEY_Flag = !KEY_Flag;
            LED_Change(KEY_Flag);
            last_press_tick = now;
            has_last_press = 1;
        }
    }
}