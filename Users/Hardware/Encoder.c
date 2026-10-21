#ifndef __Encoder_H
#define __Encoder_H

#include <Encoder.h>

/**
 * @brief 初始化编码器，计数清零
 *
 */
void Encoder_Clear()
{
    __HAL_TIM_SetCounter(&htim2, 0);
    __HAL_TIM_SetCounter(&htim4, 0);
}

/**
 * @brief 启动Encoder编码器
 *
 */
void Encoder_Start()
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
}

/**
 * @brief 用于获取编码器的值
 *
 * @param n 电机编号，2为右电机J2，1为左电机J1
 * @return int16_t
 */
int16_t Encoder_GetCount(uint8_t n)
{
    int temp;
    if (n == 2)
    {
        temp = (short)__HAL_TIM_GetCounter(&htim4);
        __HAL_TIM_SetCounter(&htim4, 0);
        return temp;
    }
    else
    {
        temp = (short)__HAL_TIM_GetCounter(&htim2);
        __HAL_TIM_SetCounter(&htim2, 0);
        return temp;
    }
}
#endif // !__Encoder_H
