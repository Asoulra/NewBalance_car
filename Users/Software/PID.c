#include "PID.h"

#define VERTICAL_CONTROL_DT 0.01f           // 直立环控制周期
#define VERTICAL_INTEGRAL_ACTIVE_ANGLE 5.0f // 直立环积分控制的角度范围
#define CONTROL_DT 0.05f                    // 速度环控制周期
#define TurnAway_OUTPUT_MAX 1800.0f         // 转向环最大输出
#define VELOCITY_ACTIVE_ANGLE 50.0f         // 倾角超过该范围时速度环退出，交由直立环恢复

pid_t Vert; // 直立环pid
pid_t Velo; // 速度环pid
pid_t Turn; // 转向环pid

/* Control() 的跨周期状态。停止或跌倒时必须统一复位。 */
float Vert_Out = 0.0f;
float Velo_Out = 0.0f;
float Turn_Out = 0.0f;
int32_t Encoder_Sum_L = 0;
int32_t Encoder_Sum_R = 0;
static uint8_t Counts = 0;

static void ResetControlState(void)
{
    Vert_Out = 0.0f;
    Velo_Out = 0.0f;
    Turn_Out = 0.0f;
    Encoder_Sum_L = 0;
    Encoder_Sum_R = 0;
    Counts = 0;
}

/**
 * @brief PID初始化，默认只打开直立环,三项pid参数后期可以选择屏蔽,通过蓝牙调整
 *
 */
void PID_Init(void)
{
    // 直立环参数
    Vert.Kp = 460.0f;
    Vert.Ki = 0.1f;
    Vert.Kd = 16.4f;
    /* Integral 保存的是 I 项对 PWM 的直接贡献。 */
    Vert.Integral_max = 200.0f;

    // 速度环参数
    Velo.Kp = 1.83f;
    Velo.Ki = 0.52f;
    Velo.Kd = 0.0f; /* Velocity() 当前没有使用 D 项，保持为 0。 */
    /* 速度环输出是目标倾角，最大限制为 ±20°。 */
    Velo.Integral_max = 20.0f;

    // 转向环参数
    Turn.Kp = 2.6f;
    Turn.Ki = 0.36f;
    Turn.Kd = 0.34f;
    Turn.Integral_max = 800.0f;

    ResetControlState();
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

    /* Integral 与直立环一样，直接保存 I 项的输出贡献。
     * 输出已经饱和且误差仍推向同一方向时，停止累积积分。 */
    const float integral_next =
        Velo.Integral + Velo.Ki * Velo.Err * CONTROL_DT;
    const float output_next = Velo.Proport + integral_next;

    // 通过判断输出是否超过了最大限幅，来决定是否保留Integral
    if (!((output_next > 20.0f && Velo.Err > 0.0f) ||
          (output_next < -20.0f && Velo.Err < 0.0f)))
    {
        Velo.Integral = integral_next;
    }
    IntegraLimit(&Velo);

    Velo.Output = Velo.Proport + Velo.Integral;

    // 输出限幅
    if (Velo.Output > 20.0f)
        Velo.Output = 20.0f;
    else if (Velo.Output < -20.0f)
        Velo.Output = -20.0f;

    return Velo.Output;
}

/**
 * @brief 转向环控制
 *
 * @param gyro_Z Z轴角速度，用于微分项
 * @param Target_Turn 目标转向角速度
 * @param Actual_Turn 实际转向角速度
 * @return float
 */
float Turnaway(float gyro_Z, int16_t Target_Turn, int16_t Actual_Turn)
{
    Turn.Err = Target_Turn - Actual_Turn;
    Turn.Proport = Turn.Kp * Turn.Err;

    const float next_Integral = Turn.Integral + Turn.Ki * Turn.Err * CONTROL_DT;
    /* 微分项不累积，这里用当前 Gz 作为下一步的微分项预测。 */
    const float next_Diff = Turn.Kd * gyro_Z;
    const float output_next = Turn.Proport + next_Integral + next_Diff;

    /* 输出饱和且误差仍向同一方向增长时，不更新积分项。 */
    if (!((output_next > TurnAway_OUTPUT_MAX && Turn.Err > 0.0f) ||
          (output_next < -TurnAway_OUTPUT_MAX && Turn.Err < 0.0f)))
    {
        Turn.Integral = next_Integral;
    }
    IntegraLimit(&Turn);

    Turn.Diff = next_Diff;
    Turn.Output = Turn.Proport + Turn.Integral + Turn.Diff;

    if (Turn.Output > TurnAway_OUTPUT_MAX)
        Turn.Output = TurnAway_OUTPUT_MAX;
    else if (Turn.Output < -TurnAway_OUTPUT_MAX)
        Turn.Output = -TurnAway_OUTPUT_MAX;

    return Turn.Output;
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
    const float Target_Speed = BlueSerial_Target_Speed;
    int16_t Target_Turn = BlueSerial_Target_Turn;
    int16_t Encoder_L, Encoder_R;

    /* 车体已倒下时关闭电机；PID 缓存仍由控制层复位。 */
    if (Motor_FallProtection(Angle, Med_Angle))
    {
        ClearPID(&Vert);
        ClearPID(&Velo);
        ClearPID(&Turn);
        ResetControlState();
        return;
    }

    /* Control() 每 10 ms 调用一次，累加 5 次后执行一次速度环，即 50 ms。 */
    if (++Counts >= 5U)
    {
        Counts = 0;
        // 编码器测得的反馈速度，作用于速度环与转向环
        //  左轮：TIM3(J1/PA6-PA7)；右轮：TIM4(J2/PB6-PB7)。
        //  两路编码器相序均与角度正方向相反，取反后前进时两轮速度均为正。
        Encoder_L = (int16_t)(-(Encoder_GetCount(1)));
        Encoder_R = (int16_t)(-(Encoder_GetCount(2)));
        Encoder_Sum_L = Encoder_L;
        Encoder_Sum_R = Encoder_R;

        const float speed_l = (float)Encoder_Sum_L /
                              (44.0f * 9.27666f * CONTROL_DT);
        const float speed_r = (float)Encoder_Sum_R /
                              (44.0f * 9.27666f * CONTROL_DT);

        // 差分速度
        const float Diff_Speed = speed_l - speed_r;
        /*****************************速度环控制***************************/
        if (Open_Velo && fabsf(Angle - Med_Angle) <= VELOCITY_ACTIVE_ANGLE)
        {

            Velo_Out = Velocity(Target_Speed, speed_l, speed_r);
        }
        else
        {
            /* 关闭速度环或倾角过大时，速度环不能继续修改直立环目标角。 */
            Velo_Out = 0;
            Encoder_Sum_L = Encoder_Sum_R = 0;
            Encoder_Clear();
            ClearPID(&Velo);
        }
        /*---------------------------------------------------------------*/

        /*****************************转向环控制***************************/
        if (Open_Turnaway)
        {
            /* 当前没有独立航向反馈，先用 Gz 作为实际转向角速度。 */
            Turn_Out = Turnaway(Gz, Target_Turn, Diff_Speed);
        }
        else
        {
            Turn_Out = 0.0f;
            ClearPID(&Turn);
        }
        /*---------------------------------------------------------------*/
        Encoder_Sum_L = Encoder_Sum_R = 0;
    }

    /*****************************直立环控制***************************/
    /* Gx 是 MPU6050 原始计数；转换为 °/s 后再参与 D 项，
       否则 Kd=1 时就可能产生数千 PWM 的错误输出。 */
    Vert_Out = Vertical(Med_Angle - Velo_Out, Angle,
                        (float)Gx * 2000.0f / 32768.0f);
    /*---------------------------------------------------------------*/

    int16_t PwmOut_L = PwmCommandToInt16(Vert_Out - Turn_Out);
    int16_t PwmOut_R = PwmCommandToInt16(Vert_Out + Turn_Out);

    Motor_SetSpeed(1, -PwmOut_L);
    Motor_SetSpeed(2, -PwmOut_R);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM2)
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
            /* 与 AngleAcc 的增减方向保持一致；当前实测前倾为正、后倾为负。 */
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
                ResetControlState();
            }
        }
    }
}