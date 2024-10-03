/*****************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
*****************************************************************/
#ifndef _UNITREE_LEGGED_JOYSTICK_H_
#define _UNITREE_LEGGED_JOYSTICK_H_

#include <stdint.h>

// 16b
typedef union {
    struct {
        uint8_t R1          :1;
        uint8_t L1          :1;
        uint8_t start       :1;
        uint8_t select      :1;
        uint8_t R2          :1;
        uint8_t L2          :1;
        uint8_t F1          :1;
        uint8_t F2          :1;
        uint8_t A           :1;
        uint8_t B           :1;
        uint8_t X           :1;
        uint8_t Y           :1;
        uint8_t up          :1;
        uint8_t right       :1;
        uint8_t down        :1;
        uint8_t left        :1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

// Structure for RC input from Arduino
typedef struct {
    float channels[8];  // Assuming 8 channels, adjust if needed
} xRCInputStruct;

// 40 Byte
typedef struct {
    uint8_t head[2];
    xKeySwitchUnion btn;
    float lx;
    float rx;
    float ry;
    float L2;
    float ly;
    xRCInputStruct rc_input;
} xRockerBtnDataStruct;

// Function declarations for RC-specific operations
void InitializeRCUART(const char* port, int baud_rate);
void ReadRCInput(xRCInputStruct* rc_input);
void ConvertRCToRockerBtn(const xRCInputStruct* rc_input, xRockerBtnDataStruct* rocker_btn);

#endif  // _UNITREE_LEGGED_JOYSTICK_H_