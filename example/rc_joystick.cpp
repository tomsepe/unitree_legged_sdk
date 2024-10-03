#include "unitree_legged_sdk/joystick.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

static int uart_fd = -1;

void InitializeRCUART(const char* port, int baud_rate) {
    uart_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        std::cerr << "Error opening UART port" << std::endl;
        return;
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
}

void ReadRCInput(xRCInputStruct* rc_input) {
    if (uart_fd == -1) {
        std::cerr << "UART not initialized" << std::endl;
        return;
    }

    int bytes_read = read(uart_fd, rc_input->channels, sizeof(rc_input->channels));
    if (bytes_read != sizeof(rc_input->channels)) {
        std::cerr << "Error reading RC data: " << bytes_read << " bytes read" << std::endl;
    }
}

void ConvertRCToRockerBtn(const xRCInputStruct* rc_input, xRockerBtnDataStruct* rocker_btn) {
    // Convert PWM (0-255) to joystick range (-1 to 1)
    rocker_btn->lx = (rc_input->channels[0] / 127.5f) - 1.0f;
    rocker_btn->ly = (rc_input->channels[1] / 127.5f) - 1.0f;
    rocker_btn->rx = (rc_input->channels[2] / 127.5f) - 1.0f;
    rocker_btn->ry = (rc_input->channels[3] / 127.5f) - 1.0f;

    // Convert other channels to button states
    rocker_btn->btn.components.A = (rc_input->channels[4] > 127) ? 1 : 0;
    rocker_btn->btn.components.B = (rc_input->channels[5] > 127) ? 1 : 0;
    rocker_btn->btn.components.X = (rc_input->channels[6] > 127) ? 1 : 0;
    rocker_btn->btn.components.Y = (rc_input->channels[7] > 127) ? 1 : 0;

    // Copy raw RC input
    std::memcpy(&rocker_btn->rc_input, rc_input, sizeof(xRCInputStruct));
}