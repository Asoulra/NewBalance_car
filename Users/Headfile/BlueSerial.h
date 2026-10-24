#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H

#include "AllHead.h"

#define BLUESERIAL_NAME_LEN 16U

/* BlueSerial_ProcessPacket() 返回值 */
#define BLUESERIAL_UPDATE_NONE     0x00U
#define BLUESERIAL_UPDATE_KEY      0x01U
#define BLUESERIAL_UPDATE_SLIDER   0x02U
#define BLUESERIAL_UPDATE_JOYSTICK 0x04U
#define BLUESERIAL_UPDATE_PID      0x08U

/* 最近一次按键事件，处理后可把 BlueSerial_KeyEventPending 清零。 */
extern uint8_t BlueSerial_KeyEventPending;
extern char BlueSerial_KeyName[BLUESERIAL_NAME_LEN];
extern char BlueSerial_KeyAction[BLUESERIAL_NAME_LEN];

/* 最近一次滑杆事件，供 OLED 或调试显示。 */
extern char BlueSerial_LastSliderName[BLUESERIAL_NAME_LEN];
extern float BlueSerial_LastSliderValue;

/* 摇杆原始值和转换后的控制目标。 */
extern volatile int8_t BlueSerial_Joystick_LH;
extern volatile int8_t BlueSerial_Joystick_LV;
extern volatile int8_t BlueSerial_Joystick_RH;
extern volatile int8_t BlueSerial_Joystick_RV;
extern volatile float BlueSerial_Target_Speed;
extern volatile int16_t BlueSerial_Target_Turn;

void BlueSerial_Init(uint32_t baudrate);

/**
 * @brief 解析一帧蓝牙数据。
 *        支持 key、slider、joystick，帧首尾的 [] 可省略。
 * @param packet 可修改的数据帧，函数内部会先复制再解析。
 * @return BLUESERIAL_UPDATE_xxx 位掩码。
 */
uint8_t BlueSerial_ProcessPacket(char *packet);

#endif /* __BLUESERIAL_H */