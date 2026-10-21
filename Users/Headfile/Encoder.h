#ifndef __ENCODER_H
#define __ENCODER_H

#include "AllHead.h"

/* 接口声明 */
void Encoder_Clear(void);
void Encoder_Start(void);
int16_t Encoder_GetCount(uint8_t n);
#endif /* __ENCODER_H */
