#include "AllHead.h"

void ALL_Init(void)
{
    Serial_Init();   // 串口状态初始化
    OLED_Init();     // OLED初始化
    MPU6050_Init();  // MPU6050初始化
    Encoder_Clear(); // 编码器计数清零
    Motor_Clear();   // 电机方向脚置低、占空比清零
    PID_Init();      // PID参数与状态初始化
}

void ALL_Start(void)
{
    PWM_Start();                   // PWM通道开启
    Encoder_Start();               // 编码器接口开启
    HAL_TIM_Base_Start_IT(&htim2); // TIM2 1kHz 控制中断开启
    Serial_ReciveData();           // 启动USART3的空闲中断接收
}
