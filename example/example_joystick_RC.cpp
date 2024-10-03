/************************************************************************
 * Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
 * Use of this source code is governed by the MPL-2.0 license, see LICENSE.
 * 
 * This example demonstrates how to control a Unitree robot using an RC
 * joystick connected via Arduino. It includes safety features such as a
 * dead man's switch and a watchdog timer.
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
        udp(level, 8090, "192.168.123.220", 8082), // Changed to high-level controller
        lastWatchdogReset(std::chrono::steady_clock::now()),
        watchdogTriggered(false) {
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
    float dt = 0.002;     // Control loop time step (seconds)
    bool deadManSwitchActive = false;
    float currentVelocity[3] = {0, 0, 0};  // Current velocity [x, y, yaw]
    float decelerationRate = 2.0;  // Deceleration rate (units/second)

    std::chrono::steady_clock::time_point lastWatchdogReset;
    std::atomic<bool> watchdogTriggered;
    const std::chrono::milliseconds watchdogTimeout{10}; // Watchdog timeout (milliseconds)
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

    // Reset watchdog timer at the start of each control loop iteration
    lastWatchdogReset = std::chrono::steady_clock::now();

    xRCInputStruct rc_input;
    if (ReadRCInput(&rc_input)) {
        ConvertRCToRockerBtn(&rc_input, &_keyData);

        // Check dead man's switch (Button B)
        deadManSwitchActive = (_keyData.btn.components.B == 1);

        if (deadManSwitchActive) {
            // Dead man's switch is active, allow movement
            currentVelocity[0] = _keyData.lx;  // Forward/backward velocity
            currentVelocity[1] = _keyData.ly;  // Left/right velocity
            currentVelocity[2] = _keyData.rx;  // Rotational velocity

            // Example: Use Button A to stop movement
            if(_keyData.btn.components.A == 1){
                std::cout << "Button A pressed: Emergency stop" << std::endl;
                std::fill(std::begin(currentVelocity), std::end(currentVelocity), 0);
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

        // Apply current velocity to robot command
        cmd.velocity[0] = currentVelocity[0];
        cmd.velocity[1] = currentVelocity[1];
        cmd.yawSpeed = currentVelocity[2];

        // Switch to standing mode if velocity is zero
        if (std::all_of(std::begin(currentVelocity), std::end(currentVelocity), 
                        [](float v) { return v == 0; })) {
            cmd.mode = 1;  // Standing mode
        }

        // Log current state
        std::cout << (deadManSwitchActive ? "Dead man's switch active. Moving." 
                                          : "Dead man's switch inactive. Slowing down.") << std::endl;
    }
    else {
        std::cout << "No new RC input available. Continuing deceleration." << std::endl;
        // Continue gradual slow-down when no new input is available
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

        if (std::all_of(std::begin(currentVelocity), std::end(currentVelocity), 
                        [](float v) { return v == 0; })) {
            cmd.mode = 1;  // Switch to standing mode when fully stopped
        }
    }

    // Check if watchdog has been triggered
    if (watchdogTriggered.load()) {
        // Perform safety action: stop the robot
        std::fill(std::begin(cmd.velocity), std::end(cmd.velocity), 0);
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
              << "WARNING: Ensure the robot is on a stable surface before proceeding." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    InitializeRCUART("/dev/ttyACM0", B115200);  // Initialize UART for RC input

    // Allow time for UART initialization
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Set up control loops
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    // Start control loops
    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    // Start watchdog thread
    std::thread watchdogThread([&custom]() {
        while (true) {
            custom.WatchdogCheck();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // Keep the program running
    while(1){
        sleep(10);
    };

    return 0; 
}