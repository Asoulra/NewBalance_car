#include "SoftIIC.h"

/*引脚定义*********************/
/*本驱动不配置引脚，引脚由CubeMX生成的MX_GPIO_Init()配置，要求：
  PB4(SDA) 和 PB5(SCL) 都必须是"开漏输出(GPIO_MODE_OUTPUT_OD) + 上拉(GPIO_PULLUP) + 初始电平高(GPIO_PIN_SET)"，
  开漏是为了让从机可以拉低SDA发出应答，若配成推挽输出，应答阶段主从会同时驱动SDA，出现电平冲突*/

#define SOFTIIC_GPIO_PORT   GPIOB
#define SOFTIIC_SCL_PIN     GPIO_PIN_5      /*B5 -> SCL*/
#define SOFTIIC_SDA_PIN     GPIO_PIN_4      /*B4 -> SDA*/

/*SCL半周期延时循环次数，数值越大SCL频率越低（通信不稳定时可适当增大）*/
#define SOFTIIC_DELAY_LOOP  40

/*写电平：1为高，0为低*/
#define SOFTIIC_SCL_W(v)    HAL_GPIO_WritePin(SOFTIIC_GPIO_PORT, SOFTIIC_SCL_PIN, (v) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define SOFTIIC_SDA_W(v)    HAL_GPIO_WritePin(SOFTIIC_GPIO_PORT, SOFTIIC_SDA_PIN, (v) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/*读SDA电平：1为高，0为低*/
#define SOFTIIC_SDA_R()     HAL_GPIO_ReadPin(SOFTIIC_GPIO_PORT, SOFTIIC_SDA_PIN)

/*私有函数声明*********************/

static void MyI2C_Delay(void);

/*函数定义*********************/

/**
  * 函    数：MyI2C_Delay
  * 描    述：软件I2C位延时，约4us@72MHz（SCL约100kHz）
  */
static void MyI2C_Delay(void)
{
    volatile uint32_t i = SOFTIIC_DELAY_LOOP;
    while (i--);
}

/**
  * 函    数：MyI2C_Start
  * 描    述：发送起始条件（SCL高电平期间，SDA由高变低）
  */
void MyI2C_Start(void)
{
    SOFTIIC_SDA_W(1);
    SOFTIIC_SCL_W(1);
    MyI2C_Delay();
    SOFTIIC_SDA_W(0);
    MyI2C_Delay();
    SOFTIIC_SCL_W(0);
    MyI2C_Delay();
}

/**
  * 函    数：MyI2C_Stop
  * 描    述：发送停止条件（SCL高电平期间，SDA由低变高）
  */
void MyI2C_Stop(void)
{
    SOFTIIC_SDA_W(0);
    SOFTIIC_SCL_W(1);
    MyI2C_Delay();
    SOFTIIC_SDA_W(1);
    MyI2C_Delay();
}

/**
  * 函    数：MyI2C_SendByte
  * 描    述：发送一个字节，高位在前
  * 参    数：Byte 要发送的字节
  */
void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        SOFTIIC_SDA_W(Byte & (0x80 >> i));
        MyI2C_Delay();
        SOFTIIC_SCL_W(1);
        MyI2C_Delay();
        SOFTIIC_SCL_W(0);
        MyI2C_Delay();
    }
}

/**
  * 函    数：MyI2C_ReceiveByte
  * 描    述：接收一个字节，高位在前
  * 返 回 值：接收到的字节
  */
uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i, Byte = 0x00;

    SOFTIIC_SDA_W(1);       /*释放SDA，交由从机驱动*/
    for (i = 0; i < 8; i++)
    {
        MyI2C_Delay();
        SOFTIIC_SCL_W(1);
        MyI2C_Delay();
        if (SOFTIIC_SDA_R() == GPIO_PIN_SET)
        {
            Byte |= (0x80 >> i);
        }
        SOFTIIC_SCL_W(0);
        MyI2C_Delay();
    }
    return Byte;
}

/**
  * 函    数：MyI2C_SendAck
  * 描    述：主机发送应答
  * 参    数：Ack 0为应答，1为非应答
  */
void MyI2C_SendAck(uint8_t Ack)
{
    SOFTIIC_SDA_W(Ack ? 1 : 0);
    MyI2C_Delay();
    SOFTIIC_SCL_W(1);
    MyI2C_Delay();
    SOFTIIC_SCL_W(0);
    MyI2C_Delay();
}

/**
  * 函    数：MyI2C_ReceiveAck
  * 描    述：主机接收从机应答
  * 返 回 值：0为从机应答，1为从机非应答
  */
uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t Ack;

    SOFTIIC_SDA_W(1);       /*释放SDA，交由从机驱动*/
    MyI2C_Delay();
    SOFTIIC_SCL_W(1);
    MyI2C_Delay();
    Ack = SOFTIIC_SDA_R();
    SOFTIIC_SCL_W(0);
    MyI2C_Delay();
    return Ack;
}