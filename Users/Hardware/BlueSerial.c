#include "BlueSerial.h"
#include <ctype.h>
#include <stdlib.h>

#define BLUESERIAL_SPEED_DIVISOR 16.0f  // 直行速度分频
#define BLUESERIAL_TURN_MULTIPLIER 5.0f // 旋转角度分频

uint8_t BlueSerial_KeyEventPending = 0U;
char BlueSerial_KeyName[BLUESERIAL_NAME_LEN] = {0};
char BlueSerial_KeyAction[BLUESERIAL_NAME_LEN] = {0};

char BlueSerial_LastSliderName[BLUESERIAL_NAME_LEN] = {0};
float BlueSerial_LastSliderValue = 0.0f;

// 手柄
volatile int8_t BlueSerial_Joystick_LH = 0;
volatile int8_t BlueSerial_Joystick_LV = 0;
volatile int8_t BlueSerial_Joystick_RH = 0;
volatile int8_t BlueSerial_Joystick_RV = 0;
volatile float BlueSerial_Target_Speed = 0.0f;
volatile int16_t BlueSerial_Target_Turn = 0;

static void BlueSerial_CopyName(char *dest, const char *src)
{
    if (dest == NULL)
        return;

    if (src == NULL)
    {
        dest[0] = '\0';
        return;
    }

    strncpy(dest, src, BLUESERIAL_NAME_LEN - 1U);
    dest[BLUESERIAL_NAME_LEN - 1U] = '\0';
}

static char *BlueSerial_TrimFrame(char *packet)
{
    char *start = packet;
    char *end;

    while ((*start != '\0') && isspace((unsigned char)*start))
        start++;

    if (*start == '[')
    {
        start++;
        while ((*start != '\0') && isspace((unsigned char)*start))
            start++;
    }

    end = start + strlen(start);
    while (end > start)
    {
        char last = end[-1];
        if (!(isspace((unsigned char)last) || (last == ']')))
            break;
        end--;
    }
    *end = '\0';

    return start;
}

static uint8_t BlueSerial_ParseFloat(const char *text, float *value)
{
    char *end = NULL;
    double parsed;

    if ((text == NULL) || (value == NULL))
        return 0U;

    parsed = strtod(text, &end);
    if (end == text)
        return 0U;

    while ((*end != '\0') && isspace((unsigned char)*end))
        end++;

    /* 拒绝尾随字符、NaN 和明显溢出的值。 */
    if ((*end != '\0') || (parsed != parsed) ||
        (parsed > 1.0e9) || (parsed < -1.0e9))
        return 0U;

    *value = (float)parsed;
    return 1U;
}

static uint8_t BlueSerial_ParseInt8(const char *text, int8_t *value)
{
    char *end = NULL;
    long parsed;

    if ((text == NULL) || (value == NULL))
        return 0U;

    parsed = strtol(text, &end, 10);
    if (end == text)
        return 0U;

    while ((*end != '\0') && isspace((unsigned char)*end))
        end++;

    if ((*end != '\0') || (parsed < -128L) || (parsed > 127L))
        return 0U;

    *value = (int8_t)parsed;
    return 1U;
}

static uint8_t BlueSerial_ApplySlider(const char *name, float value)
{
    if (name == NULL)
        return 0U;

    if (strcmp(name, "AngleKp") == 0)
        Vert.Kp = value;
    else if (strcmp(name, "AngleKi") == 0)
        Vert.Ki = value;
    else if (strcmp(name, "AngleKd") == 0)
        Vert.Kd = value;
    else if (strcmp(name, "SpeedKp") == 0)
        Velo.Kp = value;
    else if (strcmp(name, "SpeedKi") == 0)
        Velo.Ki = value;
    else if (strcmp(name, "SpeedKd") == 0)
        Velo.Kd = value;
    else if (strcmp(name, "TurnKp") == 0)
        Turn.Kp = value;
    else if (strcmp(name, "TurnKi") == 0)
        Turn.Ki = value;
    else if (strcmp(name, "TurnKd") == 0)
        Turn.Kd = value;
    /* 兼容当前短名称：Kp/Ki/Kd 仍调整速度环。 */
    else if (strcmp(name, "Kp") == 0)
        Velo.Kp = value;
    else if (strcmp(name, "Ki") == 0)
        Velo.Ki = value;
    else if (strcmp(name, "Kd") == 0)
        Velo.Kd = value;
    else
        return 0U;

    return 1U;
}

void BlueSerial_Init(uint32_t baudrate)
{
    huart3.Init.BaudRate = baudrate;
    HAL_UART_Init(&huart3);
}

uint8_t BlueSerial_ProcessPacket(char *packet)
{
    char packet_copy[MAXLEN + 1U];
    char *frame;
    char *tag;
    size_t length;
    uint8_t status = BLUESERIAL_UPDATE_NONE;

    if (packet == NULL)
        return BLUESERIAL_UPDATE_NONE;

    length = strlen(packet);
    if ((length == 0U) || (length > MAXLEN))
        return BLUESERIAL_UPDATE_NONE;

    memcpy(packet_copy, packet, length);
    packet_copy[length] = '\0';

    frame = BlueSerial_TrimFrame(packet_copy);
    tag = strtok(frame, ",");
    if (tag == NULL)
        return BLUESERIAL_UPDATE_NONE;

    if (strcmp(tag, "key") == 0)
    {
        char *name = strtok(NULL, ",");
        char *action = strtok(NULL, ",");

        if ((name != NULL) && (action != NULL))
        {
            BlueSerial_CopyName(BlueSerial_KeyName, name);
            BlueSerial_CopyName(BlueSerial_KeyAction, action);
            BlueSerial_KeyEventPending = 1U;
            status |= BLUESERIAL_UPDATE_KEY;
        }
    }
    else if (strcmp(tag, "slider") == 0)
    {
        char *name = strtok(NULL, ",");
        char *value_text = strtok(NULL, ",");
        float value;

        if ((name != NULL) && (value_text != NULL) &&
            BlueSerial_ParseFloat(value_text, &value))
        {
            BlueSerial_CopyName(BlueSerial_LastSliderName, name);
            BlueSerial_LastSliderValue = value;
            status |= BLUESERIAL_UPDATE_SLIDER;

            if (BlueSerial_ApplySlider(name, value))
                status |= BLUESERIAL_UPDATE_PID;
        }
    }
    else if (strcmp(tag, "joystick") == 0)
    {
        char *lh_text = strtok(NULL, ",");
        char *lv_text = strtok(NULL, ",");
        char *rh_text = strtok(NULL, ",");
        char *rv_text = strtok(NULL, ",");
        int8_t lh;
        int8_t lv;
        int8_t rh;
        int8_t rv;

        if ((lh_text != NULL) && (lv_text != NULL) &&
            (rh_text != NULL) && (rv_text != NULL) &&
            BlueSerial_ParseInt8(lh_text, &lh) &&
            BlueSerial_ParseInt8(lv_text, &lv) &&
            BlueSerial_ParseInt8(rh_text, &rh) &&
            BlueSerial_ParseInt8(rv_text, &rv))
        {
            BlueSerial_Joystick_LH = lh;
            BlueSerial_Joystick_LV = lv;
            BlueSerial_Joystick_RH = rh;
            BlueSerial_Joystick_RV = rv;

            BlueSerial_Target_Speed = (float)lv / BLUESERIAL_SPEED_DIVISOR;
            BlueSerial_Target_Turn = (int16_t)((float)rh * BLUESERIAL_TURN_MULTIPLIER);
            status |= BLUESERIAL_UPDATE_JOYSTICK;
        }
    }

    return status;
}