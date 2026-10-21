#include "PID.h"

pid_t Vert; // 直立环pid
pid_t Velo; // 速度环pid
pid_t Turn; // 转向环pid

/**
 * @brief PID初始化，默认只打开直立环,三项pid参数后期可以选择屏蔽,通过蓝牙调整
 *
 */
void PID_Init()
{
    // 直立环参数
    Vert.Kp = 0;
    Vert.Ki = 0;
    Vert.Kd = 0;

    // 速度环参数参数
    Velo.Kp = 0;
    Velo.Ki = 0;
    Velo.Kd = 0;

    // 转向环参数
    Turn.Kp = 0;
    Turn.Ki = 0;
    Turn.Kd = 0;

    ClearPID(&Vert);
    ClearPID(&Velo);
    ClearPID(&Turn);
}
/**
 * @brief 清理目标pid环的缓存
 *
 * @param temp 目标pid环
 */
void ClearPID(pid_t *temp)
{
    temp->Err = 0;
    temp->Integral = 0;
    temp->Last_Err = 0;
    temp->Proport = 0;
    temp->Diff = 0;
    temp->Eerr = 0;
}

void IntegraLimit(pid_t *temp)
{
    if (temp->Integral_max <= 0.0f)
        return;

    if (temp->Integral > temp->Integral_max)
        temp->Integral = temp->Integral_max;
    else if (temp->Integral < -temp->Integral_max)
        temp->Integral = -temp->Integral_max;
}

/**
 * @brief 直立环(内环)pid函数
 *
 * @param Med 输入目标角度
 * @param Angle 输入现角度
 * @param gyo_x 输入x轴角速度
 * @return float
 */
float Vertical(float Med, float Angle, float gyo_x)
{
    Vert.Last_Err = Vert.Err;

    Vert.Err = Angle - Med;
    Vert.Proport = Vert.Kp * Vert.Err;

    Vert.Eerr = Vert.Err - Vert.Last_Err;
    Vert.Integral = Vert.Ki * (Vert.Integral + Vert.Eerr);
    IntegraLimit(&Vert);

    Vert.Diff = Vert.Kd * gyo_x;

    Vert.Output = Vert.Proport + Vert.Integral + Vert.Diff;
    return Vert.Output;
}

/**
 * @brief 速度环控制
 *
 * @param Target 目标速度
 * @param encoder_L 左电机现速度
 * @param encoder_R 右电机现速度
 * @return float
 */
float Velocity(int16_t Target, int16_t encoder_L, int16_t encoder_R)
{
    Velo.Err = (float)((encoder_L + encoder_R) / 2.0 - Target);
    // 滤波
    static const float a = 0.7f;
    static uint8_t stop = 0;

    if (stop == 1)
    {
        Velo.Err = 0.0f;
        Velo.Last_Err = 0.0f;
        Velo.Proport = 0;
        Velo.Integral = 0;
        stop = 0;
        return 0.0f;
    }

    Velo.Err = (1.0f - a) * Velo.Err + a * Velo.Last_Err;
    Velo.Last_Err = Velo.Err;
    // 积分
    Velo.Integral += Velo.Err;
    // 积分限幅
    IntegraLimit(&Velo);
    // 速度环计算
    Velo.Output = Velo.Kp * Velo.Proport + Velo.Ki * Velo.Integral;
    // 目标角度的10°限幅
    if (Velo.Output > 10.0f)
        Velo.Output = 10.0f;
    if (Velo.Output < -10.0f)
        Velo.Output = -10.0f;

    return Velo.Output;
}

/**
 * @brief 转向环控制
 *
 * @param gyro_Z z轴角加速度
 * @param Target_turn左转右转
 * @return float
 */
float Turnaway(float gyro_Z, int16_t Target_turn)
{
    int temp;
    temp = Turn.Kp * Target_turn + Turn.Kd * gyro_Z;
    return temp;
}
/**
 * @brief 开启电机控制，在定时器中断中调用
 *
 */
void Control(void)
{
    // 考虑是否移动到全局
    static float Vert_Out = 0, Velo_Out = 0, Turn_Out = 0;
    int16_t Targe_Speed = 0, Target_Turn = 0, Encoder_L, Encoder_R;
    static int16_t AllEncoder_L = 0, AllEncoder_R = 0;
    static uint8_t Counts = 0;

    /* 车体已倒下时关闭电机；PID 缓存仍由控制层复位。 */
    if (Motor_FallProtection(Angle, Med_Angle))
    {
        ClearPID(&Vert);
        ClearPID(&Velo);
        ClearPID(&Turn);
        AllEncoder_L = AllEncoder_R = 0;
        Counts = 0;
        return;
    }

    // 获取电机编码器值
    Encoder_L = Encoder_GetCount(1);
    Encoder_R = Encoder_GetCount(2);

    /*****************************速度环控制***************************/
    if (Open_Velo)
    {
        Counts++;
        if (Counts >= 5)
        {
            AllEncoder_L = Encoder_L / 44.0 / 0.05 / 9.27666;
            AllEncoder_R = Encoder_R / 44.0 / 0.05 / 9.27666;
            Velo_Out = Velocity(Targe_Speed, AllEncoder_L, AllEncoder_R);
            AllEncoder_L = AllEncoder_R = 0;
            Counts = 0;
        }
    }
    else
    {
        Velo_Out = 0;
        AllEncoder_L = AllEncoder_R = 0;
        Counts = 0;
    }
    /*---------------------------------------------------------------*/

    /*****************************直立环控制***************************/
    /* Gx 是 MPU6050 原始计数；转换为 °/s 后再参与 D 项，
       否则 Kd=1 时就可能产生数千 PWM 的错误输出。 */
    Vert_Out = Vertical(Med_Angle, Angle + Velo_Out,
                        (float)Gx * 2000.0f / 32768.0f);
    /*---------------------------------------------------------------*/

    /*****************************转向环控制***************************/
    if (Open_Turnaway)
        Turn_Out = Turnaway(Gz, Target_Turn);
    else
        Turn_Out = 0;
    /*---------------------------------------------------------------*/

    int16_t PwmOut_L = (int16_t)(Vert_Out - Turn_Out);
    int16_t PwmOut_R = (int16_t)(Vert_Out + Turn_Out);

    Motor_SetSpeed(1, -PwmOut_L);
    Motor_SetSpeed(2, -PwmOut_R);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM3)
    {
        static uint8_t control_tick_count = 0;
        static uint8_t imu_tick_count = 0;

        if (++imu_tick_count >= 5)
        {
            imu_tick_count = 0;
            /* 姿态读取、解算，保持严格的5ms周期 */
            MPU6050_GetData(&Ax, &Ay, &Az, &Gx, &Gy, &Gz);
            Gy += 60;
            Gx += 76;
            /* 与 AngleAcc 的增减方向保持一致：前倾为负，后倾为正。 */
            Gx = -Gx;
            MPU6050_CalcuAngle(Ay, Az, Gx);
        }
        if (++control_tick_count >= 10)
        {
            control_tick_count = 0;

            if (KEY_Flag)
            {
                Control();
            }
            else
            {
                Motor_Clear();
                Encoder_Clear();
            }
        }
    }
}
