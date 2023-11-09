# Self-Balancing-Robot

Here is the README in Markdown format:

# Self-Balancing Robot

This Arduino sketch implements a self-balancing robot using a stepper motor, IMU sensor, and distance sensor.

## Components

- Arduino Uno
- Stepper motor with driver
- JY901 IMU sensor
- VL53L0X time-of-flight distance sensor
- Wires and breadboard

## Libraries Needed

- JY901 library
- VL53L0X library
- Wire library 

## Overview

The self-balancing functionality is achieved using a PID control loop to keep the robot upright based on IMU sensor readings. 

The distance sensor is used to follow or back away from objects in front of the robot.

The stepper motors are driven at each control loop iteration to balance the robot and move forward/backward.

## Control Loop

- IMU sensor readings are acquired to get the current angle and angular velocity
- Distance sensor reading is acquired
- PID control loop calculates output to balance at the target angle 
- Stepper motors are driven based on PID output to balance and move
- Loop repeats every 2ms

## Operation Modes

- Normal balancing mode - balances upright at 0-degree angle
- Forward driving mode - balances slightly forward to follow an object in front
- Backward driving mode - balances upright and moves backward if the object is too close

## Customization

The PID gains can be tuned by modifying the Kp, Ki, and Kd constants. 

Changing the target angle variables can modify the balancing angle and distance thresholds.

## Hardware Setup

The stepper motor should be connected to the stepper driver, which is then connected to the Arduino as follows:

- Stepper driver DIR pin -> Arduino pin 12
- Stepper driver STEP pin -> Arduino pin 14 
- Second stepper driver DIR pin -> Arduino pin 27
- Second stepper driver STEP pin -> Arduino pin 26

The JY901 IMU sensor should be connected over I2C:

- JY901 TX pin -> Arduino pin 16
- JY901 RX pin -> Arduino pin 17

The VL53L0X distance sensor should also be connected over I2C:

- VL53L0X SDA pin -> Arduino SDA pin
- VL53L0X SCL pin -> Arduino SCL pin

See the datasheets for each component for full wiring details.

## Installation

1. Install the Arduino IDE
2. Install the required libraries (JY901, VL53L0X, Wire) using the Library Manager
3. Connect the components to Arduino as described in the wiring section
4. Upload the sketch to the Arduino
5. Open the Serial Monitor at 115200 baud to see sensor readings

## Usage

Once powered on, the robot will automatically start balancing. 

Wave an object or hand before the distance sensor to trigger the following mode. The robot will lean slightly forward and follow the object as it moves farther away.

Bring the object very close to the robot to trigger the back away mode. The robot will lean backward and move away until in the normal range again.

The PID gains can be tuned to adjust responsiveness and balance performance.

View the Serial Monitor output to see sensor readings and debug.

## Credits

Swetan Reddy S and Helen Lu
