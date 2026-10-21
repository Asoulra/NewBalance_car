#include "Serial.h"

uint8_t Tempbuff[MAXLEN + 1];
uint8_t TempSenData;
volatile uint16_t Serial_RxLength;
volatile uint8_t Serial_RxReady;

void Serial_Init(void)
{
    Serial_RxLength = 0U;
    Serial_RxReady = 0U;
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
        uint16_t rx_len = Size;

        if (rx_len > MAXLEN)
        {
            rx_len = MAXLEN;
        }
        memcpy(ReciveData, Tempbuff, rx_len);
        while ((rx_len > 0U) &&
               ((ReciveData[rx_len - 1U] == '\r') ||
                (ReciveData[rx_len - 1U] == '\n')))
        {
            rx_len--;
        }
        ReciveData[rx_len] = '\0';
        Serial_RxLength = rx_len;
        Serial_RxReady = 1U;
        Serial_ReciveData();
        
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        /* 接收出错时 HAL 会结束本次接收；立即恢复下一帧接收。 */
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

