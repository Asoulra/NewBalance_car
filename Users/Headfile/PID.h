#ifndef __PID_H
#define __PID_H

#include "AllHead.h"

#define Med_Angle 0.5f
#define Open_Velo 1     // 0表示关闭速度环，1表示开启速度环
#define Open_Turnaway 1 // 0表示关闭速度环，1表示开启速度环

// pid结构体定义
typedef struct PID
{
    float Kp;
    float Ki;
    float Kd;
    float Err;
    float Last_Err;
    float Proport;
    float Integral;
    float Integral_max;
    float Diff;

    float Output;
} pid_t;

// 对外参量声明
extern pid_t Vert; // 直立环pid

extern pid_t Velo; // 速度环pid
extern pid_t Turn; // 转向环pid

// 接口声明
void PID_Init(void);
void IntegraLimit(pid_t *temp);
void ClearPID(pid_t *temp);

// 三环
float Vertical(float Med, float Angle, float gyo_x);
float Velocity(float target, float speed_l, float speed_r);
float Turnaway(float gyro_Z, int16_t Target_Turn, int16_t Actual_Turn);

#endif
