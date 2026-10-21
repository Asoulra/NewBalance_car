#ifndef __ALLHEAD_H
#define __ALLHEAD_H

#include "stdio.h"
#include "string.h"
#include "math.h"
/***** HAL库驱动 *****/
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "usart.h"
#include "tim.h"
#include "i2c.h"
#include "main.h"

/*自定义库驱动*/
// 硬件驱动
#include "BlueSerial.h"
#include "Encoder.h"
#include "Key.h"
#include "LED.h"
#include "Motor.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "OLED.h"

// 软件驱动
#include "PWM.h"
#include "Serial.h"
#include "SoftIIC.h"
#include "PID.h"

void ALL_Init(void);
void ALL_Start(void);

#endif /* __ALLHEAD_H */
