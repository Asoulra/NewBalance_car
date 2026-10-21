#include "OLED.h"
#include <stdarg.h>
#include <string.h>
#include <math.h>

/*从机地址*********************/

#define OLED_ADDRESS 0x78 // OLED的I2C从机地址：7位地址0x3C左移1位

/*显存*********************/

/*显存共8页，每页128列，每个字节对应一列的8个像素点，位0在最上方，位7在最下方*/
uint8_t OLED_DisplayBuf[8][128];

/*私有函数声明*********************/

static void OLED_WriteCommand(uint8_t Command);
static void OLED_WriteData(uint8_t *Data, uint16_t Count);
static void OLED_ClipRange(int16_t X, int16_t Y, uint8_t A, uint8_t B,
                           int16_t *I0, int16_t *I1, int16_t *J0, int16_t *J1);
static uint8_t OLED_EllipseInside(int16_t I, int16_t J, uint8_t A, uint8_t B);
static uint8_t OLED_EllipseBoundary(int16_t I, int16_t J, uint8_t A, uint8_t B);
static uint8_t OLED_AngleInRange(int16_t I, int16_t J, int16_t Start, int16_t End);
static int16_t OLED_LineIntersectX(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1,
                                   int16_t Y, uint8_t Last, uint8_t *Valid);

/*函数定义*********************/

/**
 * 函    数：OLED写命令
 * 参    数：Command 要写入的命令，范围：0x00~0xFF
 * 返 回 值：无
 * 说    明：使用软件I2C（MyI2C系列函数）作为通信协议，控制字节0x00表示后面跟随的是命令
 */
static void OLED_WriteCommand(uint8_t Command)
{
    MyI2C_Start();                // I2C起始
    MyI2C_SendByte(OLED_ADDRESS); // 发送从机地址，读写位为0，表示即将写入
    MyI2C_ReceiveAck();           // 接收应答
    MyI2C_SendByte(0x00);         // 发送控制字节，0x00表示即将发送命令
    MyI2C_ReceiveAck();           // 接收应答
    MyI2C_SendByte(Command);      // 发送命令
    MyI2C_ReceiveAck();           // 接收应答
    MyI2C_Stop();                 // I2C终止
}

/**
 * 函    数：OLED写数据
 * 参    数：Data 要写入数据的起始地址
 * 参    数：Count 要写入数据的数量，范围：0~65535
 * 返 回 值：无
 * 说    明：控制字节0x40表示后面跟随的是数据，数据在一个I2C帧内连续发送，效率更高
 */
static void OLED_WriteData(uint8_t *Data, uint16_t Count)
{
    uint16_t i;

    MyI2C_Start();                // I2C起始
    MyI2C_SendByte(OLED_ADDRESS); // 发送从机地址，读写位为0，表示即将写入
    MyI2C_ReceiveAck();           // 接收应答
    MyI2C_SendByte(0x40);         // 发送控制字节，0x40表示即将发送数据
    MyI2C_ReceiveAck();           // 接收应答
    for (i = 0; i < Count; i++)   // 循环Count次，依次发送每一个数据
    {
        MyI2C_SendByte(Data[i]); // 发送数据
        MyI2C_ReceiveAck();      // 接收应答
    }
    MyI2C_Stop(); // I2C终止
}

/**
 * 函    数：OLED初始化
 * 参    数：无
 * 返 回 值：无
 * 说    明：OLED的通信引脚由CubeMX生成的MX_GPIO_Init()配置（PB4为SDA、PB5为SCL，均为开漏输出+上拉）
 */
void OLED_Init(void)
{
    HAL_Delay(20); // 等待OLED上电稳定，可以使用HAL库的延时函数

    OLED_WriteCommand(0xAE); // 关闭显示
    OLED_WriteCommand(0xD5); // 设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80); // 0x80：分频比为1，振荡器频率为默认值
    OLED_WriteCommand(0xA8); // 设置多路复用率
    OLED_WriteCommand(0x3F); // 0x3F：多路复用率为64
    OLED_WriteCommand(0xD3); // 设置显示偏移
    OLED_WriteCommand(0x00); // 0x00：偏移为0
    OLED_WriteCommand(0x40); // 设置显示开始行，0x40：从第0行开始
    OLED_WriteCommand(0x8D); // 电荷泵设置
    OLED_WriteCommand(0x14); // 0x14：开启电荷泵
    OLED_WriteCommand(0x20); // 设置内存寻址模式
    OLED_WriteCommand(0x02); // 0x02：页寻址模式
    OLED_WriteCommand(0xA1); // 设置左右方向，0xA1：左右正常显示
    OLED_WriteCommand(0xC8); // 设置上下方向，0xC8：上下正常显示
    OLED_WriteCommand(0xDA); // 设置COM硬件引脚配置
    OLED_WriteCommand(0x12); // 0x12：交替COM引脚配置
    OLED_WriteCommand(0x81); // 设置对比度
    OLED_WriteCommand(0xCF); // 0xCF：对比度值
    OLED_WriteCommand(0xD9); // 设置预充电周期
    OLED_WriteCommand(0xF1); // 0xF1：预充电周期值
    OLED_WriteCommand(0xDB); // 设置VCOMH取消选择级别
    OLED_WriteCommand(0x30); // 0x30：VCOMH取消选择级别值
    OLED_WriteCommand(0xA4); // 设置整个显示打开/关闭，0xA4：正常显示（跟随显存）
    OLED_WriteCommand(0xA6); // 设置正常/反色显示，0xA6：正常显示
    OLED_WriteCommand(0xAF); // 开启显示

    OLED_Clear();  // 清除显存
    OLED_Update(); // 将显存更新到屏幕
}

/**
 * 函    数：OLED更新显存到屏幕
 * 参    数：无
 * 返 回 值：无
 */
void OLED_Update(void)
{
    OLED_UpdateArea(0, 0, 128, 64); // 更新整个屏幕
}

/**
 * 函    数：OLED更新指定区域
 * 参    数：X 指定区域左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定区域左上角的纵坐标，范围：-32768~32767
 * 参    数：Width 指定区域宽度，范围：0~65535
 * 参    数：Height 指定区域高度，范围：0~65535
 * 返 回 值：无
 * 说    明：指定区域超出屏幕范围的部分会被裁剪，只更新屏幕内的部分
 */
void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t Page, Page1, j, i;
    int16_t X0 = X, Y0 = Y, X1, Y1;
    uint8_t Buffer[128];

    X1 = (int16_t)(X + Width - 1);  // 计算区域右下角的横坐标
    Y1 = (int16_t)(Y + Height - 1); // 计算区域右下角的纵坐标

    /*裁剪到显存范围内*/
    if (X0 < 0)
        X0 = 0;
    if (Y0 < 0)
        Y0 = 0;
    if (X1 > 127)
        X1 = 127;
    if (Y1 > 63)
        Y1 = 63;
    if (X0 > X1 || Y0 > Y1)
        return; // 区域完全在屏幕外，无需更新

    Page = (int16_t)(Y0 / 8);  // 区域起始页
    Page1 = (int16_t)(Y1 / 8); // 区域结束页

    for (j = Page; j <= Page1; j++) // 逐页发送，每页发送区域内的所有列
    {
        OLED_WriteCommand((uint8_t)(0xB0 | j));                  // 设置页地址
        OLED_WriteCommand((uint8_t)(0x10 | ((X0 >> 4) & 0x0F))); // 设置列地址高4位
        OLED_WriteCommand((uint8_t)(0x00 | (X0 & 0x0F)));        // 设置列地址低4位

        for (i = X0; i <= X1; i++) // 取出该页中指定列的数据
        {
            Buffer[i - X0] = OLED_DisplayBuf[j][i];
        }
        OLED_WriteData(Buffer, (uint16_t)(X1 - X0 + 1)); // 发送该页的数据
    }
}

/**
 * 函    数：OLED清除全屏
 * 参    数：无
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_Clear(void)
{
    uint8_t i, j;

    for (j = 0; j < 8; j++)
    {
        for (i = 0; i < 128; i++)
        {
            OLED_DisplayBuf[j][i] = 0x00;
        }
    }
}

/**
 * 函    数：OLED清除指定区域
 * 参    数：X 指定区域左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定区域左上角的纵坐标，范围：-32768~32767
 * 参    数：Width 指定区域宽度，范围：0~65535
 * 参    数：Height 指定区域高度，范围：0~65535
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t i, j;

    for (j = Y; j < Y + Height; j++)
    {
        for (i = X; i < X + Width; i++)
        {
            if (i >= 0 && i <= 127 && j >= 0 && j <= 63)
            {
                OLED_DisplayBuf[j / 8][i] &= (uint8_t)~(0x01 << (j % 8));
            }
        }
    }
}

/**
 * 函    数：OLED反转全屏
 * 参    数：无
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_Reverse(void)
{
    uint8_t i, j;

    for (j = 0; j < 8; j++)
    {
        for (i = 0; i < 128; i++)
        {
            OLED_DisplayBuf[j][i] ^= 0xFF;
        }
    }
}

/**
 * 函    数：OLED反转指定区域
 * 参    数：X 指定区域左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定区域左上角的纵坐标，范围：-32768~32767
 * 参    数：Width 指定区域宽度，范围：0~65535
 * 参    数：Height 指定区域高度，范围：0~65535
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t i, j;

    for (j = Y; j < Y + Height; j++)
    {
        for (i = X; i < X + Width; i++)
        {
            if (i >= 0 && i <= 127 && j >= 0 && j <= 63)
            {
                OLED_DisplayBuf[j / 8][i] ^= (uint8_t)(0x01 << (j % 8));
            }
        }
    }
}

/**
 * 函    数：OLED设置像素点
 * 参    数：X 指定像素点的横坐标，范围：0~127
 * 参    数：Y 指定像素点的纵坐标，范围：0~63
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_DrawPoint(int16_t X, int16_t Y)
{
    if (X >= 0 && X <= 127 && Y >= 0 && Y <= 63)
    {
        OLED_DisplayBuf[Y / 8][X] |= (uint8_t)(0x01 << (Y % 8));
    }
}

/**
 * 函    数：OLED获取像素点
 * 参    数：X 指定像素点的横坐标，范围：0~127
 * 参    数：Y 指定像素点的纵坐标，范围：0~63
 * 返 回 值：指定像素点的状态，1表示亮，0表示灭
 */
uint8_t OLED_GetPoint(int16_t X, int16_t Y)
{
    if (X >= 0 && X <= 127 && Y >= 0 && Y <= 63)
    {
        if (OLED_DisplayBuf[Y / 8][X] & (0x01 << (Y % 8)))
        {
            return 1;
        }
    }
    return 0;
}

/**
 * 函    数：OLED显示一个字符
 * 参    数：X 指定字符左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定字符左上角的纵坐标，范围：-32768~32767
 * 参    数：Char 指定要显示的字符，范围：ASCII码可见字符
 * 参    数：FontSize 指定字体大小，范围：OLED_8X16、OLED_6X8
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
    uint8_t i, j, k;

    /*字模数据只包含可见ASCII字符，超出范围的字符统一用问号代替*/
    if (Char < ' ' || Char > '~')
    {
        Char = '?';
    }

    if (FontSize == OLED_8X16)
    {
        /*宽8像素，高16像素，字模数据共16字节：前8字节为上半页的8列，后8字节为下半页的8列*/
        for (j = 0; j < 2; j++)
        {
            for (i = 0; i < 8; i++)
            {
                for (k = 0; k < 8; k++)
                {
                    if (OLED_F8x16[Char - ' '][j * 8 + i] & (0x01 << k))
                    {
                        OLED_DrawPoint(X + i, Y + j * 8 + k);
                    }
                }
            }
        }
    }
    else
    {
        /*宽6像素，高8像素，字模数据共6字节，每个字节对应一列*/
        for (i = 0; i < 6; i++)
        {
            for (k = 0; k < 8; k++)
            {
                if (OLED_F6x8[Char - ' '][i] & (0x01 << k))
                {
                    OLED_DrawPoint(X + i, Y + k);
                }
            }
        }
    }
}

/**
 * 函    数：OLED显示字符串（超出右边界时自动换行）
 * 参    数：X 指定字符串左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定字符串左上角的纵坐标，范围：-32768~32767
 * 参    数：String 指定要显示的字符串，范围：ASCII码可见字符组成的字符串
 * 参    数：FontSize 指定字体大小，范围：OLED_8X16、OLED_6X8,6*8最大21字符
 * 返 回 值：无
 * 说    明：换行时行间距为字符高度，即OLED_8X16为16、OLED_6X8为8；
 *           此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{
    int16_t i, Height;

    Height = (FontSize == OLED_8X16) ? 16 : 8; // 字符的高度，用于换行时计算行间距

    for (i = 0; String[i] != '\0'; i++) // 遍历字符串
    {
        if (X > 128 - FontSize) // 如果当前位置的字符超出了屏幕右边界
        {
            X = 0;       // 横坐标回到最左侧
            Y += Height; // 纵坐标换到下一行
        }

        OLED_ShowChar(X, Y, String[i], FontSize); // 显示一个字符
        X += FontSize;                            // 横坐标偏移到下一个字符的位置
    }
}

/**
 * 函    数：OLED显示汉字
 * 参    数：X 指定汉字左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定汉字左上角的纵坐标，范围：-32768~32767
 * 参    数：Chinese 指定要显示的汉字，以UTF-8编码的字符串形式给出
 * 返 回 值：无
 * 说    明：汉字字模存放在Font.c的OLED_CF16x16数组中，尺寸固定为16x16；
 *           在数组中查找不到指定汉字时，会显示数组末尾的空索引/半角索引单元（默认图形）
 */
void OLED_ShowChinese(int16_t X, int16_t Y, char *Chinese)
{
    uint8_t i, j, k, n;
    char C;

    /*遍历汉字字模数据，查找与指定汉字相同的索引：
      索引必须是非ASCII的全角字符（见Font.c中的说明），数组最末尾用空索引或半角索引的单元
      作为未找到时的默认图形，因此一旦读到空索引或半角ASCII索引就说明列表已经结束，
      直接用它作为默认图形显示，这样即使表中没有相应数据也不会读到数组之外*/
    for (i = 0;; i++)
    {
        C = OLED_CF16x16[i].Index[0];

        if (C == '\0' || (uint8_t)C < 0x80)
        {
            break;
        }

        if (strncmp(OLED_CF16x16[i].Index, Chinese, strlen(OLED_CF16x16[i].Index)) == 0)
        {
            break;
        }
    }

    /*绘制16x16的点阵，字模数据格式：纵向8点，高位在下，先从左到右，再从上到下*/
    for (j = 0; j < 2; j++)
    {
        for (k = 0; k < 16; k++)
        {
            for (n = 0; n < 8; n++)
            {
                if (OLED_CF16x16[i].Data[j * 16 + k] & (0x01 << n))
                {
                    OLED_DrawPoint(X + k, Y + j * 8 + n);
                }
            }
        }
    }
}

/**
 * 函    数：OLED显示数字（十进制，正数）
 * 参    数：X 指定数字左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定数字左上角的纵坐标，范围：-32768~32767
 * 参    数：Number 指定要显示的数字，范围：0~4294967295
 * 参    数：Length 指定数字的长度，即显示多少位，范围：0~10
 * 参    数：FontSize 指定字体大小，范围：OLED_8X16、OLED_6X8
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t i;
    uint32_t Pow = 1;

    if (Length == 0 || Length > 10)
        return; // 长度不在有效范围内，直接退出

    for (i = 1; i < Length; i++) // 计算10的(Length-1)次幂
    {
        Pow *= 10;
    }

    for (i = 0; i < Length; i++) // 依次显示每一位数字，高位在前，不足位补0
    {
        OLED_ShowChar(X + i * FontSize, Y, (char)('0' + (Number / Pow) % 10), FontSize);
        Pow /= 10;
    }
}

/**
 * 函    数：OLED显示数字（十进制，带符号）
 * 参    数：X 指定数字左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定数字左上角的纵坐标，范围：-32768~32767
 * 参    数：Number 指定要显示的数字，范围：-2147483648~2147483647
 * 参    数：Length 指定数字的长度，即显示多少位，范围：0~10
 * 参    数：FontSize 指定字体大小，范围：OLED_8X16、OLED_6X8
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint32_t Number1;

    if (Number >= 0) // 数字大于等于0
    {
        OLED_ShowChar(X, Y, '+', FontSize); // 显示正号
        Number1 = (uint32_t)Number;         // 数字取绝对值（本身就是正数）
    }
    else // 数字小于0
    {
        OLED_ShowChar(X, Y, '-', FontSize); // 显示负号
        Number1 = (uint32_t)(-Number);      // 数字取绝对值
    }

    OLED_ShowNum(X + FontSize, Y, Number1, Length, FontSize); // 显示数字部分
}

/**
 * 函    数：OLED显示数字（十六进制，正数）
 * 参    数：X 指定数字左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定数字左上角的纵坐标，范围：-32768~32767
 * 参    数：Number 指定要显示的数字，范围：0~0xFFFFFFFF
 * 参    数：Length 指定数字的长度，即显示多少位，范围：0~8
 * 参    数：FontSize 指定字体大小，范围：OLED_8X16、OLED_6X8
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t i, SingleNumber;
    uint32_t Pow = 1;

    if (Length == 0 || Length > 8)
        return; // 长度不在有效范围内，直接退出

    for (i = 1; i < Length; i++) // 计算16的(Length-1)次幂
    {
        Pow *= 16;
    }

    for (i = 0; i < Length; i++) // 依次显示每一位数字，高位在前，不足位补0
    {
        SingleNumber = (uint8_t)((Number / Pow) % 16); // 取出当前位的数据
        if (SingleNumber < 10)                         // 小于10，显示0~9
        {
            OLED_ShowChar(X + i * FontSize, Y, (char)('0' + SingleNumber), FontSize);
        }
        else // 大于等于10，显示A~F
        {
            OLED_ShowChar(X + i * FontSize, Y, (char)('A' + SingleNumber - 10), FontSize);
        }
        Pow /= 16;
    }
}

/**
 * 函    数：OLED显示数字（二进制，正数）
 * 参    数：X 指定数字左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定数字左上角的纵坐标，范围：-32768~32767
 * 参    数：Number 指定要显示的数字，范围：0~0xFFFFFFFF
 * 参    数：Length 指定数字的长度，即显示多少位，范围：0~32
 * 参    数：FontSize 指定字体大小，范围：OLED_8X16、OLED_6X8
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t i;
    uint32_t Pow = 1;

    if (Length == 0 || Length > 32)
        return; // 长度不在有效范围内，直接退出

    for (i = 1; i < Length; i++) // 计算2的(Length-1)次幂
    {
        Pow *= 2;
    }

    for (i = 0; i < Length; i++) // 依次显示每一位数字，高位在前，不足位补0
    {
        OLED_ShowChar(X + i * FontSize, Y, (char)('0' + (Number / Pow) % 2), FontSize);
        Pow /= 2;
    }
}

/**
 * 函    数：OLED显示浮点数（十进制，带符号）
 * 参    数：X 指定数字左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定数字左上角的纵坐标，范围：-32768~32767
 * 参    数：Number 指定要显示的数字，范围：负数除外的浮点数
 * 参    数：IntLength 指定整数部分的长度，即显示多少位，范围：0~10
 * 参    数：FraLength 指定小数部分的长度，即显示多少位，范围：0~10
 * 参    数：FontSize 指定字体大小，范围：OLED_8X16、OLED_6X8
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize)
{
    uint8_t i;
    uint32_t IntNum, FraNum, Pow = 1;

    if (IntLength == 0 || IntLength > 10 || FraLength == 0 || FraLength > 10)
        return; // 长度不在有效范围内，直接退出

    /*显示符号位*/
    if (Number >= 0)
    {
        OLED_ShowChar(X, Y, '+', FontSize);
    }
    else
    {
        OLED_ShowChar(X, Y, '-', FontSize);
        Number = -Number;
    }
    X += FontSize;

    /*计算小数部分的精度*/
    for (i = 0; i < FraLength; i++)
    {
        Pow *= 10;
    }

    /*分离整数部分和小数部分，小数部分四舍五入*/
    IntNum = (uint32_t)Number;
    FraNum = (uint32_t)((Number - (double)IntNum) * (double)Pow + 0.5);
    if (FraNum >= Pow) // 小数部分进位到整数部分
    {
        IntNum += 1;
        FraNum = 0;
    }

    /*显示整数部分*/
    OLED_ShowNum(X, Y, IntNum, IntLength, FontSize);
    X += IntLength * FontSize;

    /*显示小数点和Fraction部分*/
    OLED_ShowChar(X, Y, '.', FontSize);
    OLED_ShowNum(X + FontSize, Y, FraNum, FraLength, FontSize);
}

/**
 * 函    数：OLED显示图像
 * 参    数：X 指定图像左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定图像左上角的纵坐标，范围：-32768~32767
 * 参    数：Width 指定图像的宽度，范围：0~65535
 * 参    数：Height 指定图像的高度，范围：0~65535
 * 参    数：Image 指定要显示的图像，数据格式：纵向8点，高位在下，先从左到右，再从上到下
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
    uint8_t i, j, k, Data;
    uint8_t PageCount;

    if (Width == 0 || Height == 0)
        return; // 尺寸为0，直接退出

    PageCount = (uint8_t)((Height - 1) / 8 + 1); // 计算图像占用的页数

    for (j = 0; j < PageCount; j++) // 遍历图像涉及的所有页
    {
        for (i = 0; i < Width; i++) // 遍历每一列
        {
            Data = Image[j * Width + i]; // 取出该列的数据
            for (k = 0; k < 8; k++)      // 逐位绘制，位0在最上方
            {
                if (Data & (0x01 << k))
                {
                    OLED_DrawPoint(X + i, Y + j * 8 + k);
                }
            }
        }
    }
}

/**
 * 函    数：OLED使用printf函数打印格式化字符串
 * 参    数：X 指定字符串左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定字符串左上角的纵坐标，范围：-32768~32767
 * 参    数：FontSize 指定字体大小，范围：OLED_8X16、OLED_6X8
 * 参    数：format 指定要显示的格式化字符串，范围：ASCII码可见字符组成的字符串
 * 参    数：... 格式化字符串参数列表
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...)
{
    char String[64]; // 定义字符数组，用于存储格式化后的字符串
    va_list arg;     // 定义输出参数列表

    va_start(arg, format);                          // 获取可变参数列表
    vsnprintf(String, sizeof(String), format, arg); // 格式化字符串，只保留数组能装下的部分
    va_end(arg);                                    // 释放可变参数列表

    OLED_ShowString(X, Y, String, FontSize); // 将字符串显示出来
}

/**
 * 函    数：OLED画线
 * 参    数：X0 指定一个端点的横坐标，范围：-32768~32767
 * 参    数：Y0 指定一个端点的纵坐标，范围：-32768~32767
 * 参    数：X1 指定另一个端点的横坐标，范围：-32768~32767
 * 参    数：Y1 指定另一个端点的纵坐标，范围：-32768~32767
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1)
{
    int16_t x = X0, y = Y0;
    int16_t dx, dy, sx, sy, err, e2;

    dx = (X1 > X0) ? (int16_t)(X1 - X0) : (int16_t)(X0 - X1); // 横坐标的增量
    dy = (Y1 > Y0) ? (int16_t)(Y0 - Y1) : (int16_t)(Y1 - Y0); // 纵坐标的增量（取负值，方便统一计算）
    sx = (X0 < X1) ? 1 : -1;                                  // 横坐标的步进方向
    sy = (Y0 < Y1) ? 1 : -1;                                  // 纵坐标的步进方向

    err = dx + dy; // 误差项的初始值

    while (1)
    {
        OLED_DrawPoint(x, y); // 绘制当前点

        if (x == X1 && y == Y1)
            break; // 到达另一个端点，绘制结束

        e2 = (int16_t)(2 * err); // 误差项的2倍
        if (e2 >= dy)
        {
            err += dy;
            x += sx;
        } // 误差项满足条件，横坐标步进
        if (e2 <= dx)
        {
            err += dx;
            y += sy;
        } // 误差项满足条件，纵坐标步进
    }
}

/**
 * 函    数：OLED画矩形
 * 参    数：X 指定矩形左上角的横坐标，范围：-32768~32767
 * 参    数：Y 指定矩形左上角的纵坐标，范围：-32768~32767
 * 参    数：Width 指定矩形的宽度，范围：0~65535
 * 参    数：Height 指定矩形的高度，范围：0~65535
 * 参    数：IsFilled 指定矩形是否填充，范围：OLED_UNFILLED、OLED_FILLED
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled)
{
    int16_t i, j;

    if (IsFilled == OLED_FILLED) // 填充矩形
    {
        for (i = X; i < X + Width; i++)
        {
            for (j = Y; j < Y + Height; j++)
            {
                OLED_DrawPoint(i, j);
            }
        }
    }
    else // 不填充矩形，绘制四条边
    {
        for (i = X; i < X + Width; i++)
        {
            OLED_DrawPoint(i, Y);              // 上边
            OLED_DrawPoint(i, Y + Height - 1); // 下边
        }
        for (j = Y; j < Y + Height; j++)
        {
            OLED_DrawPoint(X, j);             // 左边
            OLED_DrawPoint(X + Width - 1, j); // 右边
        }
    }
}

/**
 * 函    数：OLED画三角形
 * 参    数：X0 指定顶点1的横坐标，范围：-32768~32767
 * 参    数：Y0 指定顶点1的纵坐标，范围：-32768~32767
 * 参    数：X1 指定顶点2的横坐标，范围：-32768~32767
 * 参    数：Y1 指定顶点2的纵坐标，范围：-32768~32767
 * 参    数：X2 指定顶点3的横坐标，范围：-32768~32767
 * 参    数：Y2 指定顶点3的纵坐标，范围：-32768~32767
 * 参    数：IsFilled 指定三角形是否填充，范围：OLED_UNFILLED、OLED_FILLED
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t IsFilled)
{
    int16_t Y, YMin, YMax, XMin, XMax, XList[3];
    uint8_t Valid, Count, Last;
    int16_t i;

    if (IsFilled == OLED_UNFILLED) // 不填充三角形，绘制三条边
    {
        OLED_DrawLine(X0, Y0, X1, Y1);
        OLED_DrawLine(X1, Y1, X2, Y2);
        OLED_DrawLine(X2, Y2, X0, Y0);
        return;
    }

    /*填充三角形，逐行扫描：计算扫描线与三条边的交点，填充最左和最右交点之间的像素*/
    YMin = Y0;
    if (Y1 < YMin)
        YMin = Y1;
    if (Y2 < YMin)
        YMin = Y2;
    YMax = Y0;
    if (Y1 > YMax)
        YMax = Y1;
    if (Y2 > YMax)
        YMax = Y2;

    for (Y = YMin; Y <= YMax; Y++)
    {
        Count = 0;
        Last = (Y == YMax) ? 1 : 0; // 最后一行改用另一种半开区间，保证底边和底顶点能被填充

        XMin = OLED_LineIntersectX(X0, Y0, X1, Y1, Y, Last, &Valid);
        if (Valid)
            XList[Count++] = XMin;
        XMin = OLED_LineIntersectX(X1, Y1, X2, Y2, Y, Last, &Valid);
        if (Valid)
            XList[Count++] = XMin;
        XMin = OLED_LineIntersectX(X2, Y2, X0, Y0, Y, Last, &Valid);
        if (Valid)
            XList[Count++] = XMin;

        if (Count >= 2)
        {
            XMin = XList[0];
            XMax = XList[0];
            for (i = 1; i < Count; i++)
            {
                if (XList[i] < XMin)
                    XMin = XList[i];
                if (XList[i] > XMax)
                    XMax = XList[i];
            }
            for (i = 0; XMin + i <= XMax; i++)
            {
                OLED_DrawPoint(XMin + i, Y);
            }
        }
    }
}

/**
 * 函    数：OLED画圆
 * 参    数：X 指定圆的圆心的横坐标，范围：-32768~32767
 * 参    数：Y 指定圆的圆心的纵坐标，范围：-32768~32767
 * 参    数：Radius 指定圆的半径，范围：0~255
 * 参    数：IsFilled 指定圆是否填充，范围：OLED_UNFILLED、OLED_FILLED
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled)
{
    OLED_DrawEllipse(X, Y, Radius, Radius, IsFilled); // 圆是横纵半轴相等的椭圆
}

/**
 * 函    数：OLED画椭圆
 * 参    数：X 指定椭圆的圆心的横坐标，范围：-32768~32767
 * 参    数：Y 指定椭圆的圆心的纵坐标，范围：-32768~32767
 * 参    数：A 指定椭圆的横向半轴长度，范围：0~255
 * 参    数：B 指定椭圆的纵向半轴长度，范围：0~255
 * 参    数：IsFilled 指定椭圆是否填充，范围：OLED_UNFILLED、OLED_FILLED
 * 返 回 值：无
 * 说    明：此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B, uint8_t IsFilled)
{
    int16_t I, J, I0, I1, J0, J1;

    if (A == 0 || B == 0)
        return; // 半轴为0，直接退出

    /*扫描范围限制在屏幕内，既减少无效计算，也保证内部运算不会溢出*/
    OLED_ClipRange(X, Y, A, B, &I0, &I1, &J0, &J1);

    for (J = J0; J <= J1; J++)
    {
        for (I = I0; I <= I1; I++)
        {
            if (OLED_EllipseInside(I, J, A, B)) // 点在椭圆内
            {
                /*填充时绘制所有内部点，不填充时只绘制轮廓上的点*/
                if (IsFilled == OLED_FILLED || OLED_EllipseBoundary(I, J, A, B))
                {
                    OLED_DrawPoint(X + I, Y + J);
                }
            }
        }
    }
}

/**
 * 函    数：OLED画圆弧（填充时为扇形）
 * 参    数：X 指定圆弧的圆心的横坐标，范围：-32768~32767
 * 参    数：Y 指定圆弧的圆心的纵坐标，范围：-32768~32767
 * 参    数：Radius 指定圆弧的半径，范围：0~255
 * 参    数：StartAngle 指定圆弧的起始角度，范围：-360~360，单位为度
 * 参    数：EndAngle 指定圆弧的终止角度，范围：-360~360，单位为度，与起始角度相同时绘制整圆
 * 参    数：IsFilled 指定圆弧是否填充，范围：OLED_UNFILLED、OLED_FILLED
 * 返 回 值：无
 * 说    明：角度以度为单位，0度在右侧，逆时针方向为正；起始角度大于终止角度时，圆弧跨越0度；
 *           此函数会更改显存，但不会立即刷新到屏幕，需要调用OLED_Update()
 */
void OLED_DrawArc(int16_t X, int16_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled)
{
    int16_t I, J, I0, I1, J0, J1;
    int16_t Start, End;

    if (Radius == 0)
        return; // 半径为0，直接退出

    /*将角度归一化到0~360度，方便比较*/
    Start = (int16_t)(StartAngle % 360);
    if (Start < 0)
        Start += 360;
    End = (int16_t)(EndAngle % 360);
    if (End < 0)
        End += 360;

    /*扫描范围限制在屏幕内，既减少无效计算，也保证内部运算不会溢出*/
    OLED_ClipRange(X, Y, Radius, Radius, &I0, &I1, &J0, &J1);

    for (J = J0; J <= J1; J++)
    {
        for (I = I0; I <= I1; I++)
        {
            if (!OLED_EllipseInside(I, J, Radius, Radius))
                continue; // 点在圆外，跳过
            if (IsFilled == OLED_UNFILLED && !OLED_EllipseBoundary(I, J, Radius, Radius))
                continue; // 填充时画扇形，不填充时只画圆弧线
            if (Start != End && !OLED_AngleInRange(I, (int16_t)(-J), Start, End))
                continue; // 角度不在指定范围内，跳过

            OLED_DrawPoint(X + I, Y + J);
        }
    }
}

/*私有函数定义*********************/

/**
 * 函    数：把以(X,Y)为中心、横纵半轴为A、B的扫描范围裁剪到屏幕内
 * 参    数：I0 I1 输出参数，横向相对坐标的最小值和最大值
 * 参    数：J0 J1 输出参数，纵向相对坐标的最小值和最大值
 */
static void OLED_ClipRange(int16_t X, int16_t Y, uint8_t A, uint8_t B,
                           int16_t *I0, int16_t *I1, int16_t *J0, int16_t *J1)
{
    *I0 = (int16_t)(-X); // 屏幕左边界对应的相对横坐标
    if (*I0 < -(int16_t)A)
        *I0 = -(int16_t)A;
    *I1 = (int16_t)(127 - X); // 屏幕右边界对应的相对横坐标
    if (*I1 > (int16_t)A)
        *I1 = (int16_t)A;

    *J0 = (int16_t)(-Y); // 屏幕上边界对应的相对纵坐标
    if (*J0 < -(int16_t)B)
        *J0 = -(int16_t)B;
    *J1 = (int16_t)(63 - Y); // 屏幕下边界对应的相对纵坐标
    if (*J1 > (int16_t)B)
        *J1 = (int16_t)B;
}

/**
 * 函    数：判断相对坐标(I,J)是否在横纵半轴为A、B的椭圆内部（含边界）
 * 返 回 值：1表示在椭圆内部，0表示在椭圆外部
 * 说    明：判定式为 I²B² + J²A² <= A²B²，调用前需要保证|I|、|J|不超过屏幕范围，否则会溢出
 */
static uint8_t OLED_EllipseInside(int16_t I, int16_t J, uint8_t A, uint8_t B)
{
    uint32_t A2 = (uint32_t)A * A; // 横向半轴的平方
    uint32_t B2 = (uint32_t)B * B; // 纵向半轴的平方

    if ((uint32_t)(I * I) * B2 + (uint32_t)(J * J) * A2 <= A2 * B2)
    {
        return 1;
    }
    return 0;
}

/**
 * 函    数：判断相对坐标(I,J)是否在椭圆的轮廓上
 * 返 回 值：1表示在椭圆轮廓上，0表示不在椭圆轮廓上
 * 说    明：点在椭圆内部，且上下左右四个相邻点中至少有一个在椭圆外部，即可认为该点在轮廓上
 */
static uint8_t OLED_EllipseBoundary(int16_t I, int16_t J, uint8_t A, uint8_t B)
{
    if (!OLED_EllipseInside(I + 1, J, A, B))
        return 1;
    if (!OLED_EllipseInside(I - 1, J, A, B))
        return 1;
    if (!OLED_EllipseInside(I, J + 1, A, B))
        return 1;
    if (!OLED_EllipseInside(I, J - 1, A, B))
        return 1;

    return 0;
}

/**
 * 函    数：判断相对坐标(I,J)的角度是否在[Start,End]范围内
 * 参    数：I 相对于圆心的横坐标
 * 参    数：J 相对于圆心的纵坐标（以数学坐标系给出，即屏幕上方为正）
 * 参    数：Start 起始角度，范围：0~359，单位为度
 * 参    数：End 终止角度，范围：0~359，单位为度
 * 返 回 值：1表示在范围内，0表示不在范围内
 * 说    明：起始角度大于终止角度时，角度范围跨越0度
 */
static uint8_t OLED_AngleInRange(int16_t I, int16_t J, int16_t Start, int16_t End)
{
    int16_t Angle = (int16_t)(atan2f((float)J, (float)I) * 57.29578f); // 计算角度，单位为度，范围：-180~180

    if (Angle < 0)
        Angle += 360; // 归一化到0~360度

    if (Start <= End)
    {
        if (Angle >= Start && Angle <= End)
            return 1;
    }
    else
    {
        if (Angle >= Start || Angle <= End)
            return 1;
    }

    return 0;
}

/**
 * 函    数：计算水平扫描线Y与线段(X0,Y0)-(X1,Y1)交点的横坐标
 * 参    数：Y 水平扫描线的纵坐标
 * 参    数：Last 0表示普通扫描行，采用[Y0,Y1)的半开区间；1表示最后一行，采用(Y0,Y1]的半开区间
 * 参    数：Valid 输出参数，1表示有交点，0表示无交点
 * 返 回 值：交点的横坐标，无交点时返回0
 * 说    明：采用半开区间可以避免顶点被相邻两条边重复计数，最后一行的区间方向相反，
 *           使三角形的底边和底顶点也能被填充
 */
static int16_t OLED_LineIntersectX(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1,
                                   int16_t Y, uint8_t Last, uint8_t *Valid)
{
    int16_t X = 0;

    *Valid = 0;

    if (Last == 0)
    {
        if (((Y0 <= Y) && (Y1 > Y)) || ((Y1 <= Y) && (Y0 > Y)))
        {
            X = (int16_t)(X0 + (int32_t)(Y - Y0) * (X1 - X0) / (Y1 - Y0));
            *Valid = 1;
        }
    }
    else
    {
        if (((Y0 < Y) && (Y1 >= Y)) || ((Y1 < Y) && (Y0 >= Y)))
        {
            X = (int16_t)(X0 + (int32_t)(Y - Y0) * (X1 - X0) / (Y1 - Y0));
            *Valid = 1;
        }
    }

    return X;
}
/**
 * @brief 刷新某一行数据，数据格式默认6x8
 *
 * @param X 起始的x位置，可以使用roll定义
 * @param Y 起始的y位置，可以使用line定义
 * @param Data 要显示的数据
 */
void OLED_FastFresh(int16_t Y, int16_t X, uint8_t *Data)
{
    /*数据超过一行宽度时OLED_ShowString会自动换行，这里同步多清一行，避免换行处残留旧内容*/
    uint8_t Height = (X + (int16_t)strlen((char *)Data) * OLED_6X8 > 128) ? 16 : 8;

    OLED_ClearArea(X, Y, 128, Height);
    OLED_ShowString(X, Y, (char *)Data, OLED_6X8);
    OLED_Update();
}