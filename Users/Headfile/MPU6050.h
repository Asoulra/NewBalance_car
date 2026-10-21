#ifndef __MPU6050_H
#define __MPU6050_H

#include "AllHead.h"

// 变量声明
extern int16_t Ax, Ay, Az, Gx, Gy, Gz;
extern float Angle, AngleGyro, AngleAcc;
extern float Alpha;
/* 接口声明 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_CalcuAngle(int16_t AY, int16_t AZ, int16_t GX);

#endif /* __MPU6050_H */
