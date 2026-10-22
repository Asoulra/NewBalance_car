#include "MPU6050.h"

#define MPU6050_ADDRESS 0xD0 // MPU6050的I2C从机地址：7位地址0x68左移1位
#define MPU6050_TIMEOUT 10   // HAL库I2C读写超时时间，单位：ms（一次14字节读取约1.6ms）

int16_t Ax, Ay, Az, Gx, Gy, Gz;
float AngleAcc;  // 由加速度计得到的角度值
float AngleGyro; // 由陀螺仪得到的角度值，执行互补滤波后，此值基本与Angle相等
float Angle;
float Alpha = 0.03f;
/**
 * 函    数：MPU6050写寄存器
 * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
 * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
 * 返 回 值：无
 * 说    明：使用硬件I2C1（MX_I2C1_Init配置），HAL库会自动完成起始、地址、寄存器地址、数据和停止
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, MPU6050_TIMEOUT);
}

/**
 * 函    数：MPU6050读寄存器
 * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
 * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
 * 说    明：HAL库内部会自动置读写位，所以从机地址统一用0xD0，不需要再或上0x01
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data = 0x00;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, MPU6050_TIMEOUT);

    return Data;
}

/**
 * 函    数：MPU6050连续读多个寄存器
 * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
 * 参    数：DataArray 输出参数，用于存储多个寄存器值的数组
 * 参    数：Count 指定读取寄存器的数量
 * 返 回 值：无
 * 说    明：HAL库的存储器读接口自带"写寄存器地址+重复起始+读数据+非应答+停止"的完整时序，
 *           MPU6050收到寄存器地址后内部指针会自动自增，因此可以一次读出连续的多个寄存器
 */
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, DataArray, Count, MPU6050_TIMEOUT);
}

/**
 * 函    数：MPU6050初始化
 * 参    数：无
 * 返 回 值：无
 * 说    明：底层硬件I2C1由CubeMX生成的MX_I2C1_Init()在main()中初始化，此处只需配置MPU6050的寄存器
 */
void MPU6050_Init(void)
{
    /*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);   // 电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);   // 电源管理寄存器2，保持默认值0，所有轴均不待机
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x00);   // DLPF开启时：1kHz/(1+0)=1kHz，STM32仍每5ms读取一次
    MPU6050_WriteReg(MPU6050_CONFIG, 0x00);       //
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);  // 陀螺仪配置寄存器，选择满量程为±2000°/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); // 加速度计配置寄存器，选择满量程为±16g
}

/**
 * 函    数：MPU6050获取ID号
 * 参    数：无
 * 返 回 值：MPU6050的ID号
 */
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I); // 返回WHO_AM_I寄存器的值
}

/**
 * 函    数：MPU6050获取数据
 * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
 * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
 * 返 回 值：无
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[14];

    /*通过MPU6050_ReadRegs连续读取多个数据，效率更高*/
    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14); // 从MPU6050_ACCEL_XOUT_H开始，连续读取14个字节，存入Data数组里

    /*数据拼接，通过输出参数返回*/
    *AccX = (int16_t)((Data[0] << 8) | Data[1]); // Data[0]和Data[1]为加速度计X轴数据
    *AccY = (int16_t)((Data[2] << 8) | Data[3]); // Data[2]和Data[3]为加速度计Y轴数据
    *AccZ = (int16_t)((Data[4] << 8) | Data[5]); // Data[4]和Data[5]为加速度计Z轴数据

    // Data[6]和Data[7]为温度数据，此处暂时不用

    *GyroX = (int16_t)((Data[8] << 8) | Data[9]);   // Data[8]和Data[9]为陀螺仪X轴数据
    *GyroY = (int16_t)((Data[10] << 8) | Data[11]); // Data[10]和Data[11]为陀螺仪Y轴数据
    *GyroZ = (int16_t)((Data[12] << 8) | Data[13]); // Data[12]和Data[13]为陀螺仪Z轴数据
}

/**
 * @brief 计算角度值
 *
 * @param AY 加速度计y轴分量
 * @param AZ 加速度计z轴方向分量
 * @param GX 陀螺仪x轴分量
 */
void MPU6050_CalcuAngle(int16_t AY, int16_t AZ, int16_t GX)
{

    /*由加速度计计算得到角度值*/
    /*atan2计算反正切，得到角度（弧度制）， / 3.14159 * 180可将弧度制转为角度值*/
    // 红色
    AngleAcc = -atan2(AY, AZ) / 3.14159 * 180;

    /*由陀螺仪积分得到角度值*/
    /*互补滤波下，角度积分要在上次滤波后的Angle上进行*/
    /*陀螺仪满量程为±2000°/s，姿态更新周期为5ms*/
    AngleGyro = Angle + GX / 32768.0 * 2000 * 0.005f;

    /*执行互补滤波*/
    // 互补滤波系数，值越大，越偏向于加速度计，值越小，越偏向于陀螺仪
    // 绿色
    Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro; // 互补滤波计算，得到稳定的角度值
}
