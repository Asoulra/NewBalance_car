#include "Serial.h"

uint8_t Tempbuff[MAXLEN + 1];
uint8_t TempSenData;
volatile uint16_t Serial_RxLength;
volatile uint8_t Serial_RxReady;

/* 空闲中断只表示收到一个片段；']' 是协议帧尾，'\n' 作为兼容帧尾。 */
static uint8_t RxFrameBuffer[MAXLEN + 1];
static uint16_t RxFrameLength;
static uint8_t RxFrameOverflow;

void Serial_Init(void)
{
    Serial_RxLength = 0U;
    Serial_RxReady = 0U;
    RxFrameLength = 0U;
    RxFrameOverflow = 0U;
}

void Serial_SenData(uint8_t *Data, uint16_t len)
{
    /* USART3 当前未配置 DMA，使用阻塞发送避免访问空的 DMA 句柄。 */
    HAL_UART_Transmit(&huart3, Data, len, HAL_MAX_DELAY);
}

void Serial_ReciveData(void)
{
    /* USART3 使用空闲中断判定一帧数据结束，无需 DMA 配置。 */
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, Tempbuff, MAXLEN);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3)
    {
        uint16_t i;

        for (i = 0U; i < Size; i++)
        {
            uint8_t data = Tempbuff[i];

            if ((data == ']') || (data == '\n'))
            {
                /* ']' 属于 [slider,Kx,x] 帧本身，提交前要写入缓冲区。 */
                if ((data == ']') && !RxFrameOverflow)
                {
                    if (RxFrameLength < MAXLEN)
                    {
                        RxFrameBuffer[RxFrameLength++] = data;
                    }
                    else
                    {
                        RxFrameLength = 0U;
                        RxFrameOverflow = 1U;
                    }
                }

                if (!RxFrameOverflow)
                {
                    /* 同时兼容以 \r\n 结尾的数据。 */
                    if ((RxFrameLength > 0U) &&
                        (RxFrameBuffer[RxFrameLength - 1U] == '\r'))
                    {
                        RxFrameLength--;
                    }

                    if (RxFrameLength > 0U)
                    {
                        memcpy(ReciveData, RxFrameBuffer, RxFrameLength);
                        ReciveData[RxFrameLength] = '\0';
                        Serial_RxLength = RxFrameLength;
                        Serial_RxReady = 1U;
                    }
                }

                RxFrameLength = 0U;
                RxFrameOverflow = 0U;
            }
            else if (data == '[')
            {
                /* 新帧头可清除换行配置错误或线路噪声留下的残片。 */
                RxFrameBuffer[0] = data;
                RxFrameLength = 1U;
                RxFrameOverflow = 0U;
            }
            else if (!RxFrameOverflow)
            {
                if (RxFrameLength < MAXLEN)
                {
                    RxFrameBuffer[RxFrameLength++] = data;
                }
                else
                {
                    /* 超长帧丢弃到下一个换行，避免残片成为新帧。 */
                    RxFrameLength = 0U;
                    RxFrameOverflow = 1U;
                }
            }
        }

        Serial_ReciveData();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        /* 接收出错时 HAL 会结束本次接收；立即恢复下一帧接收。 */
        RxFrameLength = 0U;
        RxFrameOverflow = 0U;
        Serial_RxReady = 0U;
        Serial_ReciveData();
    }
}

int fputc(int ch, FILE *f)
{
    uint8_t data = (uint8_t)ch;
    HAL_UART_Transmit(&huart3, &data, 1, HAL_MAX_DELAY);
    return ch;
}

