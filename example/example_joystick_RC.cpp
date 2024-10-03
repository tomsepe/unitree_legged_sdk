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
#include <thread>
#include <chrono>
#include <atomic>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): 
        safe(LeggedType::B1), 
        udp(level, 8090, "192.168.123.220", 8082),
        lastWatchdogReset(std::chrono::steady_clock::now()),
        watchdogTriggered(false) {  // Changed to high-level controller
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void WatchdogCheck();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    xRockerBtnDataStruct _keyData;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    bool deadManSwitchActive = false;
    float currentVelocity[3] = {0, 0, 0};  // x, y, yaw
    float decelerationRate = 2.0;  // units/second, adjust as needed

    std::chrono::steady_clock::time_point lastWatchdogReset;
    std::atomic<bool> watchdogTriggered;
    const std::chrono::milliseconds watchdogTimeout{10}; // 10ms timeout, adjust as needed
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

    // Reset the watchdog timer at the start of each control loop iteration
    lastWatchdogReset = std::chrono::steady_clock::now();

    xRCInputStruct rc_input;
    if (ReadRCInput(&rc_input)) {
        ConvertRCToRockerBtn(&rc_input, &_keyData);

        // Check dead man's switch (assuming button B is used)
        deadManSwitchActive = (_keyData.btn.components.B == 1);

        if (deadManSwitchActive) {
            // Dead man's switch is active, allow movement
            currentVelocity[0] = _keyData.lx;  // Forward/backward velocity
            currentVelocity[1] = _keyData.ly;  // Left/right velocity
            currentVelocity[2] = _keyData.rx;  // Rotational velocity

            // Example of using another button input
            if(_keyData.btn.components.A == 1){
                std::cout << "Button A is pressed, stopping movement" << std::endl;
                currentVelocity[0] = 0;
                currentVelocity[1] = 0;
                currentVelocity[2] = 0;
            }
        } else {
            // Dead man's switch is not active, gradually slow down
            for (int i = 0; i < 3; i++) {
                if (currentVelocity[i] > 0) {
                    currentVelocity[i] = std::max(0.0f, currentVelocity[i] - decelerationRate * dt);
                } else if (currentVelocity[i] < 0) {
                    currentVelocity[i] = std::min(0.0f, currentVelocity[i] + decelerationRate * dt);
                }
            }
        }

        // Apply the current velocity
        cmd.velocity[0] = currentVelocity[0];
        cmd.velocity[1] = currentVelocity[1];
        cmd.yawSpeed = currentVelocity[2];

        // If we've come to a complete stop, switch to standing mode
        if (currentVelocity[0] == 0 && currentVelocity[1] == 0 && currentVelocity[2] == 0) {
            cmd.mode = 1;  // Assuming mode 1 is for standing/stopping. Adjust as per SDK documentation.
        }

        // Logging the state for debugging and monitoring
        if (deadManSwitchActive) {
            std::cout << "Dead man's switch active. Movement allowed." << std::endl;
        } else {
            std::cout << "Dead man's switch inactive. Slowing down." << std::endl;
        }
    }
    else {
        // Handle the case where no new RC input is available
        std::cout << "No new RC input available" << std::endl;
        // Continue the gradual slow-down
        for (int i = 0; i < 3; i++) {
            if (currentVelocity[i] > 0) {
                currentVelocity[i] = std::max(0.0f, currentVelocity[i] - decelerationRate * dt);
            } else if (currentVelocity[i] < 0) {
                currentVelocity[i] = std::min(0.0f, currentVelocity[i] + decelerationRate * dt);
            }
        }
        cmd.velocity[0] = currentVelocity[0];
        cmd.velocity[1] = currentVelocity[1];
        cmd.yawSpeed = currentVelocity[2];

        if (currentVelocity[0] == 0 && currentVelocity[1] == 0 && currentVelocity[2] == 0) {
            cmd.mode = 1;  // Switch to standing mode when fully stopped
        }
    }

    // Check if watchdog has been triggered
    if (watchdogTriggered.load()) {
        // Perform safety action (e.g., stop the robot)
        cmd.velocity[0] = 0;
        cmd.velocity[1] = 0;
        cmd.yawSpeed = 0;
        cmd.mode = 1;  // Switch to standing mode
        std::cerr << "Watchdog triggered! Control loop frequency issue detected." << std::endl;
    }

    udp.SetSend(cmd);
}

void Custom::WatchdogCheck()
{
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastWatchdogReset);
    if (duration > watchdogTimeout) {
        watchdogTriggered.store(true);
    }
}

int main(void)
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    InitializeRCUART("/dev/ttyACM0", B115200);  // Adjust port and baud rate to match your Arduino setup

    // Give some time for UART initialization and initial data reading
    std::this_thread::sleep_for(std::chrono::seconds(1));

    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    std::thread watchdogThread([&custom]() {
        while (true) {
            custom.WatchdogCheck();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    while(1){
        sleep(10);
    };

    return 0; 
}
