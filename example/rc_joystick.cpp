/************************************************************************
 * Copyright (c) 2023, Your Company Name. All rights reserved.
 * 
 * This file implements the RCJoystick class, which handles the communication
 * with an RC joystick connected via Arduino. It provides thread-safe reading
 * of RC input and conversion to the format used by the Unitree SDK.
 ************************************************************************/

#include "unitree_legged_sdk/joystick.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

class RCJoystick {
private:
    int uart_fd;  // File descriptor for UART communication
    std::mutex uart_mutex;  // Mutex for thread-safe UART access
    std::queue<xRCInputStruct> data_queue;  // Queue to store RC input data
    std::mutex queue_mutex;  // Mutex for thread-safe queue access
    std::thread read_thread;  // Thread for continuous RC input reading
    bool should_stop;  // Flag to signal thread termination

    // Continuous reading loop for RC input
    void readLoop() {
        while (!should_stop) {
            xRCInputStruct rc_input;
            {
                std::lock_guard<std::mutex> lock(uart_mutex);
                int bytes_read = read(uart_fd, &rc_input.channels, sizeof(rc_input.channels));
                if (bytes_read != sizeof(rc_input.channels)) {
                    std::cerr << "Error reading RC data: " << bytes_read << " bytes read" << std::endl;
                    continue;
                }
                rc_input.timestamp = std::chrono::steady_clock::now();
                rc_input.deadManSwitchPressed = (rc_input.channels[5] > 0.5f);  // Assuming channel 6 is the dead man's switch
            }
            
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                data_queue.push(rc_input);
                if (data_queue.size() > 10) data_queue.pop();  // Limit queue size to prevent memory issues
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Adjust sleep time as needed
        }
    }

public:
    RCJoystick() : uart_fd(-1), should_stop(false) {}

    ~RCJoystick() {
        should_stop = true;
        if (read_thread.joinable()) read_thread.join();
        if (uart_fd != -1) close(uart_fd);
    }

    // Initialize UART communication with the Arduino
    bool InitializeUART(const char* port, int baud_rate) {
        std::lock_guard<std::mutex> lock(uart_mutex);
        uart_fd = open(port, O_RDWR | O_NOCTTY);
        if (uart_fd == -1) {
            std::cerr << "Error opening UART port: " << port << std::endl;
            return false;
        }

        struct termios options;
        tcgetattr(uart_fd, &options);
        cfsetispeed(&options, baud_rate);
        cfsetospeed(&options, baud_rate);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        tcsetattr(uart_fd, TCSANOW, &options);

        should_stop = false;
        read_thread = std::thread(&RCJoystick::readLoop, this);
        return true;
    }

    // Read the latest RC input data from the queue
    bool ReadRCInput(xRCInputStruct* rc_input, std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < timeout) {
            if (g_rcJoystick.ReadRCInput(rc_input)) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }
};

// Global RCJoystick instance
RCJoystick g_rcJoystick;

// Initialize UART communication for RC input
void InitializeRCUART(const char* port, int baud_rate) {
    if (!g_rcJoystick.InitializeUART(port, baud_rate)) {
        std::cerr << "Failed to initialize UART for RC input" << std::endl;
        // Consider adding error handling or program termination here
    }
}

// Read RC input data
void ReadRCInput(xRCInputStruct* rc_input) {
    while (!g_rcJoystick.ReadRCInput(rc_input)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// Convert raw RC input to Unitree SDK joystick format
void ConvertRCToRockerBtn(const xRCInputStruct* rc_input, xRockerBtnDataStruct* rocker_btn) {
    // Assuming Arduino sends values already in -1 to 1 range
    rocker_btn->lx = rc_input->channels[0];  // Left joystick X-axis
    rocker_btn->ly = rc_input->channels[1];  // Left joystick Y-axis
    rocker_btn->rx = rc_input->channels[2];  // Right joystick X-axis
    rocker_btn->ry = rc_input->channels[3];  // Right joystick Y-axis

    // Clearly identify the dead man's switch
    rocker_btn->btn.components.B = rc_input->deadManSwitchPressed ? 1 : 0;  // Use B as the dead man's switch

    // Convert button states (assuming 0.0 is off, 1.0 is on)
    rocker_btn->btn.components.A = rc_input->channels[4] > 0.5f ? 1 : 0;
    rocker_btn->btn.components.X = rc_input->channels[6] > 0.5f ? 1 : 0;
    rocker_btn->btn.components.Y = rc_input->channels[7] > 0.5f ? 1 : 0;

    // Copy raw RC input for reference
    std::memcpy(&rocker_btn->rc_input, rc_input, sizeof(xRCInputStruct));
}