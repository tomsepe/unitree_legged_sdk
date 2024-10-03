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
    // All channel values are expected to be in the range of -1.0 to 1.0
    // Joystick axes: -1.0 (full reverse) to 1.0 (full forward)
    // Buttons/Switches: 0.0 (off/released) or 1.0 (on/pressed)
    float channels[8];  // Assuming 8 channels, adjust if needed
} xRCInputStruct;

// 40 Byte
typedef struct {
    uint8_t head[2];
    xKeySwitchUnion btn;
    // The following values are expected to be in the range of -1.0 to 1.0
    float lx;  // Left joystick X-axis: -1.0 (full left) to 1.0 (full right)
    float rx;  // Right joystick X-axis: -1.0 (full left) to 1.0 (full right)
    float ry;  // Right joystick Y-axis: -1.0 (full down) to 1.0 (full up)
    float L2;  // Left trigger: 0.0 (not pressed) to 1.0 (fully pressed)
    float ly;  // Left joystick Y-axis: -1.0 (full down) to 1.0 (full up)
    xRCInputStruct rc_input;
} xRockerBtnDataStruct;

// Function declarations for RC-specific operations

/**
 * Initializes the UART connection for RC input.
 * @param port The UART port to use (e.g., "/dev/ttyACM0")
 * @param baud_rate The baud rate for the UART connection
 */
void InitializeRCUART(const char* port, int baud_rate);

/**
 * Reads RC input data from the UART connection.
 * @param rc_input Pointer to an xRCInputStruct to store the read data
 */
void ReadRCInput(xRCInputStruct* rc_input);

/**
 * Converts raw RC input data to the format used by the robot control system.
 * @param rc_input Pointer to the raw RC input data
 * @param rocker_btn Pointer to the xRockerBtnDataStruct to store the converted data
 */
void ConvertRCToRockerBtn(const xRCInputStruct* rc_input, xRockerBtnDataStruct* rocker_btn);

#endif  // _UNITREE_LEGGED_JOYSTICK_H_