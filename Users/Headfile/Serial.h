#ifndef __SERIAL_H
#define __SERIAL_H

#include "AllHead.h"

#define MAXLEN 31
/* MAXLEN 是一帧有效载荷上限；数组额外保留一个 '\0' 用于文本显示。 */
extern uint8_t ReciveData[MAXLEN + 1];
extern uint8_t Tempbuff[MAXLEN + 1];
extern uint8_t TempSenData;
extern volatile uint16_t Serial_RxLength;
extern volatile uint8_t Serial_RxReady;
/* 接口声明 */
/* 在 MX_USART3_UART_Init() 之后调用，启动 USART3 的空闲中断接收。 */
void Serial_Init(void);

void Serial_SenData(uint8_t *Data, uint16_t len);
void Serial_ReciveData(void);
#endif /* __SERIAL_H */
