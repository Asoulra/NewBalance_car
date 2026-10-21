#ifndef __MOTOR_H
#define __MOTOR_H

#include "AllHead.h"

/* 接口声明 */
void Motor_Clear(void);
uint8_t Motor_FallProtection(float angle, float reference_angle);
void Motor_SetSpeed(uint8_t n, int16_t pwm);
void Motor_Limit(int16_t *pwm, int16_t Targe);
static uint16_t AbsPwm(int16_t pwm);

#endif /* __MOTOR_H */
