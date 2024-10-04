# Modifications to Unitree\_legged\_sdk

Here's a comprehensive summary of the changes made to utilize an RC joystick connected to an Arduino, along with the functions created and important considerations:

1. **RC Joystick Integration (rc\_joystick.cpp):**  
   1. Created RCJoystick class for thread-safe UART handling.  
   2. Implemented functions:  
      1. InitializeUART: Sets up UART communication with the Arduino.  
      2. ReadRCInput: Reads RC input data from a thread-safe queue.  
      3. ConvertRCToRockerBtn: Converts raw RC data to the SDK's joystick format.  
   3. Added a separate thread for continuous RC input reading.  
2. **Robot Control (example\_joystick\_RC.cpp):**  
   1. Modified Custom class to handle RC input and implement safety features.  
   2. Key functions:  
      1. RobotControl: Main control loop, processes RC input and sends commands to the robot.  
      2. WatchdogCheck: Ensures the control loop is running at the expected frequency.  
   3. Implemented a dead man's switch using one of the RC buttons.  
   4. Added gradual slow-down mechanism when the dead man's switch is released.  
3. **Safety Features:**  
   1. Dead man's switch: Robot only moves when the switch is actively held.  
   2. Gradual deceleration: Smooth slow-down when the dead man's switch is released or RC input is lost.  
   3. Watchdog timer: Detects if the control loop frequency drops or stalls.  
4. **Threading Model:**  
   1. RC input reading thread (in RCJoystick class)  
   2. Control loop thread (LoopFunc for RobotControl)  
   3. UDP send/receive threads (LoopFunc for UDPSend and UDPRecv)  
   4. Watchdog thread  
5. **Considerations and Potential Concerns:**  
   1. Thread Safety: Mixing std::thread and Boost threads might lead to issues. Consider unifying the threading model.  
6. **UART Communication:**  
   1. Ensure the Arduino sends data at a consistent, expected rate.  
   2. The current implementation might block if no data is available. Consider adding a timeout.  
7. **3\. RC Input Processing:**  
   1. The data queue is limited to 10 items. Adjust if needed based on your control loop and RC data frequencies.  
   2. Ensure that the Arduino's data format matches the expected xRCInputStruct.  
8. **Safety Mechanisms:**  
   1. Test the dead man's switch thoroughly in various scenarios, including connection loss.  
   2. Tune the deceleration rate for optimal safety and smoothness.  
   3. Verify that the watchdog timer doesn't cause false positives during normal operation.  
9. **Error Handling:**  
   1. Implement more robust error handling, especially for UART initialization and connection loss scenarios.  
   2. Consider adding a timeout for RC input to handle complete signal loss.  
10. **Performance:**  
    1. Monitor CPU usage, especially with the additional threads.  
    2. Ensure that the control loop frequency is maintained with the added processing.  
11. **Compatibility:**  
    1. Verify that all changes are compatible with the Unitree Legged SDK.  
    2. Ensure that the high-level control mode (cmd.mode \= 1 for standing) behaves as expected.  
12. **Testing:**  
    1. Thoroughly test all new features in a safe, controlled environment before real-world deployment.  
    2. Test edge cases, such as rapid toggling of the dead man's switch or intermittent RC signal.  
13. **Customization:**  
    1. The current implementation assumes specific button mappings (e.g., button B for dead man's switch). Ensure this matches your RC controller setup.  
    2. You may need to adjust scaling factors for the RC inputs depending on your joystick's range.  
14. **Documentation:**  
    1. Update any existing documentation to reflect the new RC joystick functionality and safety features.  
    2. Provide clear instructions for setting up the Arduino and RC controller.

### Changes to joystick.h:

**1\. Updated Structure Definitions:**

Modified xRCInputStruct to use float values for channels:

     typedef struct {

         float channels\[8\];  *// Assuming 8 channels, adjust if needed*

     } xRCInputStruct;

\- Updated xRockerBtnDataStruct to include the new xRCInputStruct:

     typedef struct {

         uint8\_t head\[2\];

         xKeySwitchUnion btn;

         float lx;

         float rx;

         float ry;

         float L2;

         float ly;

         xRCInputStruct rc\_input;

     } xRockerBtnDataStruct;

**2\. New Function Declarations:**

   void InitializeRCUART(const char\* *port*, int *baud\_rate*);

   void ReadRCInput(xRCInputStruct\* *rc\_input*);

   void ConvertRCToRockerBtn(const xRCInputStruct\* *rc\_input*, xRockerBtnDataStruct\* *rocker\_btn*);

### 

### Considerations for joystick.h changes:

1. Compatibility: Ensure that these changes don't break compatibility with existing code that might be using the original structures.  
2. Channel Count: The xRCInputStruct assumes 8 channels. Verify that this matches your RC controller's output. Adjust if necessary.  
3. Data Types: Using floats for RC input assumes that the Arduino is sending preprocessed, normalized values. Ensure your Arduino code aligns with this expectation.  
4. Button Mapping: The xKeySwitchUnion structure remains unchanged. Make sure your RC controller's buttons map correctly to this structure in the ConvertRCToRockerBtn function.  
5. Size Considerations: The size of xRockerBtnDataStruct has likely changed. Verify that this doesn't affect any part of the system that might depend on its size (e.g., UDP packet sizes, memory allocations).  
6. Header Guards: Ensure that proper header guards are in place to prevent multiple inclusions.  
7. Documentation: Consider adding comments to explain the expected ranges for the float values (e.g., \-1.0 to 1.0 for joystick axes).  
8. Potential Expansion: The current structure might not capture all possible RC controller inputs (e.g., switches, potentiometers). Consider if additional fields are needed for your specific use case.

These changes to joystick.h are crucial as they define the interface between the RC input system and the rest of the robot control code. They need to be consistent with both the Arduino's output and the expectations of the robot control software. Thorough testing should be done to ensure that data is correctly interpreted throughout the system after these changes.

### Additions and modifications for the rc\_joystick.cpp

1. Watchdog Integration:  
   1. We should add a timestamp to the xRCInputStruct to support watchdog functionality in the main control loop.  
2. Dead Man's Switch:  
   1. We should ensure that the dead man's switch button is clearly identified in the ConvertRCToRockerBtn function.  
3. Error Handling:  
   1. Implement a timeout mechanism in the ReadRCInput function to avoid indefinite blocking.  
4. Gradual Deceleration:  
   1. While this is mainly handled in the main control loop, we might want to add a flag in the xRCInputStruct to indicate if the dead man's switch is pressed.

These modifications accomplish the following:

1. Added a timestamp and dead man's switch flag to the xRCInputStruct.  
2. Modified the readLoop to set the timestamp and dead man's switch flag.  
3. Updated ReadRCInput to include a timeout mechanism.  
4. Clearly identified the dead man's switch in ConvertRCToRockerBtn.


# v3.8.3
The unitree_legged_sdk is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.

### Notice
support robot: B1

not support robot: Laikago, Aliengo, A1. (Check release [v3.3.1](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1) for support)

### Sport Mode
```bash
Legged_sport    >= v3.24
firmware H0.1.7 >= v0.1.35
         H0.1.9 >= v0.1.35
```

### Dependencies
* [Boost](http://www.boost.org) (version 1.71.0 or higher)
* [CMake](http://www.cmake.org) (version 3.16.3 or higher)
* [g++](https://gcc.gnu.org/) (version 9.4.0 or higher)


### Build
```bash
mkdir build
cd build
cmake ../
make
```

### Run

#### Cpp
Run examples with 'sudo' for memory locking.

#### Python
##### arm
change `sys.path.append('../lib/python/amd64')` to `sys.path.append('../lib/python/arm64')`
