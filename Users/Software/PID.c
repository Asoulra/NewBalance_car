#include "PID.h"

#define VERTICAL_CONTROL_DT 0.01f           // 直立环控制周期
#define VERTICAL_INTEGRAL_ACTIVE_ANGLE 5.0f // 直立环积分控制的角度范围
#define VELOCITY_CONTROL_DT 0.05f           // 速度环控制周期
#define ENCODER_LINES_PER_MOTOR_REV 44.0f   // mpu极性的转换(可能写错了)
#define MOTOR_GEAR_RATIO 9.27666f           // mpu取模数值(可能写错了)

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
    Vert.Kp = 460.0f;
    Vert.Ki = 0.0f;
    Vert.Kd = 16.4f;
    /* Integral 保存的是 I 项对 PWM 的直接贡献。 */
    Vert.Integral_max = 200.0f;

    // 速度环参数参数
    Velo.Kp = 0.0f;
    Velo.Ki = 0.0f;
    Velo.Kd = 0.0f;
    /* 速度环输出是目标倾角，最大限制为 ±10°。 */
    Velo.Integral_max = 10.0f;

    // 转向环参数
    Turn.Kp = 0.0f;
    Turn.Ki = 0.0f;
    Turn.Kd = 0.0f;
    Turn.Integral_max = 0.0f;

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
    temp->Output = 0;
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
    Vert.Err = Angle - Med;
    Vert.Proport = Vert.Kp * Vert.Err;

    /* 只在平衡点附近积分，避免倾倒或输出饱和时积分饱和。 */
    if (Vert.Err > -VERTICAL_INTEGRAL_ACTIVE_ANGLE &&
        Vert.Err < VERTICAL_INTEGRAL_ACTIVE_ANGLE)
    {
        Vert.Integral += Vert.Ki * Vert.Err * VERTICAL_CONTROL_DT;
    }
    else
    {
        Vert.Integral = 0.0f;
    }
    IntegraLimit(&Vert);

    Vert.Diff = Vert.Kd * gyo_x;

    Vert.Output = Vert.Proport + Vert.Integral + Vert.Diff;
    return Vert.Output;
}

/**
 * @brief 速度环控制
 *
 * @param target 目标速度
 * @param speed_l 左轮实际速度
 * @param speed_r 右轮实际速度
 * @return float
 */
float Velocity(float target, float speed_l, float speed_r)
{
    const float alpha = 0.7f;

    float raw_err = (speed_l + speed_r) * 0.5f - target;

    Velo.Err = alpha * Velo.Last_Err + (1.0f - alpha) * raw_err;
    Velo.Last_Err = Velo.Err;

    Velo.Proport = Velo.Kp * Velo.Err;
    /* Integral 与直立环一样，直接保存 I 项的输出贡献。 */
    Velo.Integral += Velo.Ki * Velo.Err * VELOCITY_CONTROL_DT;
    IntegraLimit(&Velo);

    Velo.Output = Velo.Proport + Velo.Integral;

    if (Velo.Output > 10.0f)
        Velo.Output = 10.0f;
    else if (Velo.Output < -10.0f)
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
 * @brief 浮点控制量在转换为 int16_t 前限幅，避免越界转换后突然反号。
 */
static int16_t PwmCommandToInt16(float command)
{
    if (command > 32767.0f)
        return 32767;
    if (command < -32768.0f)
        return -32768;
    return (int16_t)command;
}

/**
 * @brief 开启电机控制，在定时器中断中调用
 *
 */
void Control(void)
{
    // 考虑是否移动到全局
    static float Vert_Out = 0, Velo_Out = 0, Turn_Out = 0;
    const float Target_Speed = 0.0f;
    int16_t Target_Turn = 0, Encoder_L, Encoder_R;
    static int32_t Encoder_Sum_L = 0, Encoder_Sum_R = 0;
    static uint8_t Counts = 0;

    /* 车体已倒下时关闭电机；PID 缓存仍由控制层复位。 */
    if (Motor_FallProtection(Angle, Med_Angle))
    {
        ClearPID(&Vert);
        ClearPID(&Velo);
        ClearPID(&Turn);
        Encoder_Sum_L = Encoder_Sum_R = 0;
        Counts = 0;
        return;
    }

    // 获取电机编码器值
    Encoder_L = Encoder_GetCount(1);
    Encoder_R = Encoder_GetCount(2);

    /*****************************速度环控制***************************/
    if (Open_Velo)
    {
        Encoder_Sum_L += Encoder_L;
        Encoder_Sum_R += Encoder_R;
        Counts++;
        if (Counts >= 5)
        {
            const float speed_l = (float)Encoder_Sum_L /
                                  (ENCODER_LINES_PER_MOTOR_REV * MOTOR_GEAR_RATIO *
                                   VELOCITY_CONTROL_DT);
            const float speed_r = (float)Encoder_Sum_R /
                                  (ENCODER_LINES_PER_MOTOR_REV * MOTOR_GEAR_RATIO *
                                   VELOCITY_CONTROL_DT);

            Velo_Out = Velocity(Target_Speed, speed_l, speed_r);
            Encoder_Sum_L = Encoder_Sum_R = 0;
            Counts = 0;
        }
    }
    else
    {
        Velo_Out = 0;
        Encoder_Sum_L = Encoder_Sum_R = 0;
        Counts = 0;
        ClearPID(&Velo);
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

    int16_t PwmOut_L = PwmCommandToInt16(Vert_Out - Turn_Out);
    int16_t PwmOut_R = PwmCommandToInt16(Vert_Out + Turn_Out);

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
                ClearPID(&Vert);
                ClearPID(&Velo);
                ClearPID(&Turn);
            }
        }
    }
}
