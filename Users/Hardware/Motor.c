#include "Motor.h"

#define MOTOR_PWM_MAX 2700     /* TIM1 ARR=3599，限制为约 75% 占空比 */
#define MOTOR_PWM_DEADZONE 160 /* 落地实测启动阈值约149~155，留少量余量 */
#define MOTOR_PWM_ZERO_BAND 5  /* 抑制零点附近微小噪声导致的方向反复切换 */
#define MOTOR_FALL_ANGLE_DEG 50.0f

void Motor_Clear(void)
{
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin | AIN2_Pin | BIN1_Pin | BIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
}

/**
 * @brief 车体倾倒保护。超过平衡参考角允许范围时立即关闭两路电机。
 * @return 1: 已触发保护；0: 姿态处于允许范围内。
 */
uint8_t Motor_FallProtection(float angle, float reference_angle)
{
    if (angle < (reference_angle - MOTOR_FALL_ANGLE_DEG) ||
        angle > (reference_angle + MOTOR_FALL_ANGLE_DEG))
    {
        Motor_Clear();
        KEY_Flag = 0;
        LED_Change(0);
        return 1;
    }

    return 0;
}

void Motor_Limit(int16_t *pwm, int16_t Targe)
{
    if (*pwm > Targe)
    {
        *pwm = Targe;
    }
    else if (*pwm < -Targe)
    {
        *pwm = -Targe;
    }
}

/**
 * @brief 填补电机无法克服静摩擦的PWM区间，不改变中高PWM输出。
 */
// static void Motor_DeadzoneCompensate(int16_t *pwm)
// {
//     if (*pwm >= -MOTOR_PWM_ZERO_BAND && *pwm <= MOTOR_PWM_ZERO_BAND)
//     {
//         *pwm = 0;
//     }
//     else if (*pwm > MOTOR_PWM_ZERO_BAND && *pwm < MOTOR_PWM_DEADZONE)
//     {
//         *pwm = MOTOR_PWM_DEADZONE;
//     }
//     else if (*pwm < -MOTOR_PWM_ZERO_BAND && *pwm > -MOTOR_PWM_DEADZONE)
//     {
//         *pwm = -MOTOR_PWM_DEADZONE;
//     }
// }

/**
 * @brief 电机速度设置,如果直行两个电机转速不一样排查这里
 *
 * @param n 电机编号，1为左电机AIN(TIM1_CH4/PA11)，2为右电机BIN(TIM1_CH1/PA8)
 * @param pwm 指定电机的PWM速度
 */
void Motor_SetSpeed(uint8_t n, int16_t pwm)
{
    if (n != 1 && n != 2)
        return;

    Motor_Limit(&pwm, MOTOR_PWM_MAX);
    // Motor_DeadzoneCompensate(&pwm);

    if (n == 1) // 指定左电机
    {
        if (pwm == 0)
        {
            HAL_GPIO_WritePin(GPIOB, AIN1_Pin | AIN2_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
            return;
        }
        if (pwm >= 0) // 如果设置正转的PWM
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // PB12置高电平
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // PB13置低电平
        }
        else // 否则，即设置反转的PWM
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // PB12置低电平
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // PB13置高电平
        }
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, AbsPwm(pwm)); // 设置占空比
    }
    else if (n == 2) // 指定右电机
    {
        if (pwm == 0)
        {
            HAL_GPIO_WritePin(GPIOB, BIN1_Pin | BIN2_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
            return;
        }
        if (pwm >= 0) // 如果设置正转的PWM
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // PB14置低电平
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // PB15置高电平
        }
        else // 否则，即设置反转的PWM
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // PB14置高电平
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // PB15置低电平
        }
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, AbsPwm(pwm)); // 设置占空比
    }
}

/**
 * @brief PWM专用取绝对值函数
 *
 * @param pwm pwm值
 * @return uint16_t
 */
static uint16_t AbsPwm(int16_t pwm)
{
    return (pwm < 0) ? (uint16_t)(-(int32_t)pwm) : (uint16_t)pwm;
}
