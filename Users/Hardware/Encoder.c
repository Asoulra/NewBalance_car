#ifndef __Encoder_H
#define __Encoder_H

#include <Encoder.h>

/**
 * @brief 编码器计数清零
 *
 */
void Encoder_Clear(void)
{
    __HAL_TIM_SetCounter(&htim3, 0);
    __HAL_TIM_SetCounter(&htim4, 0);
}

/**
 * @brief 启动 TIM3(左轮) 和 TIM4(右轮) 编码器接口
 *
 */
void Encoder_Start(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

/**
 * @brief 读取编码器绝对计数，不清零
 *
 * @param n 1为左轮(TIM3/J1/PA6-PA7)，2为右轮(TIM4/J2/PB6-PB7)
 * @return int16_t
 */
int16_t Encoder_GetRawCount(uint8_t n)
{
    if (n == 1)
        return (int16_t)__HAL_TIM_GetCounter(&htim3);

    return (int16_t)__HAL_TIM_GetCounter(&htim4);
}

/**
 * @brief 读取自上电调用以来的计数增量并清零
 *
 * @param n 1为左轮(TIM3/J1/PA6-PA7)，2为右轮(TIM4/J2/PB6-PB7)
 * @return int16_t
 */
int16_t Encoder_GetCount(uint8_t n)
{
    int16_t count = Encoder_GetRawCount(n);

    if (n == 1)
        __HAL_TIM_SetCounter(&htim3, 0);
    else
        __HAL_TIM_SetCounter(&htim4, 0);

    return count;
}
#endif // !__Encoder_H