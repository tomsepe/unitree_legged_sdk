/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/joystick.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): 
        safe(LeggedType::B1), 
        udp(level, 8090, "192.168.123.10", 8007){
        udp.InitCmdData(cmd);
        InitializeRCUART("/dev/ttyS0", B115200);
    }
    void UDPSend();
    void UDPRecv();
    void RobotControl();
    void UpdateRCJoystick();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    xRockerBtnDataStruct _keyData;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    int uart_fd;
    uint8_t rc_data[8];  // Assuming 8 channels of RC data
};

void Custom::UDPRecv()
{ 
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::InitializeUART()
{
    uart_fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        std::cerr << "Error opening UART port" << std::endl;
        exit(1);
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(uart_fd, TCSANOW, &options);
}

void Custom::ReadRCJoystick()
{
    int bytes_read = read(uart_fd, rc_data, sizeof(rc_data));
    if (bytes_read != sizeof(rc_data)) {
        std::cerr << "Error reading RC data" << std::endl;
    }
}

void Custom::ConvertRCToKeyData()
{
    // Convert PWM (0-255) to joystick range (-1 to 1)
    _keyData.lx = (rc_data[0] / 127.5f) - 1.0f;
    _keyData.ly = (rc_data[1] / 127.5f) - 1.0f;
    _keyData.rx = (rc_data[2] / 127.5f) - 1.0f;
    _keyData.ry = (rc_data[3] / 127.5f) - 1.0f;

    // Convert other channels to button states
    _keyData.btn.components.A = (rc_data[4] > 127) ? 1 : 0;
    // ... map other buttons as needed ...
}

void Custom::UpdateRCJoystick()
{
    xRCInputStruct rc_input;
    ReadRCInput(&rc_input);
    ConvertRCToRockerBtn(&rc_input, &_keyData);
}

void Custom::RobotControl() 
{
    motiontime++;
    udp.GetRecv(state);

    UpdateRCJoystick();

    if(_keyData.btn.components.A == 1){
        std::cout << "The key A is pressed, and the value of lx is " << _keyData.lx << std::endl;
    }

    safe.PowerProtect(cmd, state, 1);
    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(LOWLEVEL);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    close(custom.uart_fd);  // Close UART port before exiting
    return 0; 
}
