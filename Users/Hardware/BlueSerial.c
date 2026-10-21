#include "BlueSerial.h"

/**
 * @brief 不需要用基本上
 *
 * @param baudrate 目标波特率
 */
void BlueSerial_Init(uint32_t baudrate)
{
    huart3.Init.BaudRate = baudrate; // 波特率
    HAL_UART_Init(&huart3);
}
