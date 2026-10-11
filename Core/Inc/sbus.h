#ifndef SBUS_H
#define SBUS_H

#include "main.h"

#define SBUS_FRAME_LENGTH 25  //25Byte

typedef struct
{
    uint16_t ch[16];

    uint8_t lost_frame;
    uint8_t failsafe;

} SBUS_Data_t;

extern volatile SBUS_Data_t sbus_data;

void SBUS_Init(void);
void SBUS_Parse(uint8_t *buf);

#endif

