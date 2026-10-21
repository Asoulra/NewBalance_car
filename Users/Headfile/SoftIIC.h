#ifndef __SOFTIIC_H
#define __SOFTIIC_H

#include "AllHead.h"

/* 接口声明 */
/*引脚由CubeMX的MX_GPIO_Init()配置，不需要额外的初始化函数*/

/*通信层接口（供OLED等I2C从机驱动调用）*/
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t Ack);
uint8_t MyI2C_ReceiveAck(void);

#endif /* __SOFTIIC_H */
