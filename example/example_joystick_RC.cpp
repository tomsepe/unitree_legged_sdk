/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/joystick.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): 
        safe(LeggedType::B1), 
        udp(level, 8090, "192.168.123.10", 8007) {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    xRockerBtnDataStruct _keyData;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime++;
    udp.GetRecv(state);

    // Read RC input from Arduino and convert to joystick data
    xRCInputStruct rc_input;
    ReadRCInput(&rc_input);
    ConvertRCToRockerBtn(&rc_input, &_keyData);

    // Example of using the joystick data
    if(_keyData.btn.components.A == 1){
        std::cout << "Button A is pressed, lx: " << _keyData.lx << ", ly: " << _keyData.ly << std::endl;
    }

    // Add your robot control logic here using _keyData
    // For example:
    cmd.motorCmd[0].q = _keyData.lx * some_scaling_factor;
    cmd.motorCmd[1].q = _keyData.ly * some_scaling_factor;
    // ... control other motors or functions based on joystick input

    safe.PositionLimit(cmd);
    safe.PowerProtect(cmd, state, 1);
    
    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(LOWLEVEL);
    InitializeRCUART("/dev/ttyACM0", B115200);  // Adjust port and baud rate to match your Arduino setup

    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
