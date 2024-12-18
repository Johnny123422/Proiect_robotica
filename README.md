# Bluetooth-Controlled and Autonomous Obstacle-Avoiding Robot
## Introduction
This project involves creating a robot that can be controlled via Bluetooth using a smartphone or operate autonomously to avoid obstacles. The goal is to combine manual and autonomous control in a versatile, user-friendly design. This project is useful for learning about embedded systems, robotics, and communication between hardware components. It also has potential applications in education and hobby projects.

# General Description
The robot is based on an Arduino microcontroller and includes the following features:
Bluetooth Control: The robot can be remotely operated using a smartphone app.
Autonomous Navigation: Using sensors, the robot detects and avoids obstacles.
Mobility: Powered by four motors, the robot has wheels for smooth movement.


# Hardware Design
## Components Used
### Arduino Board:
Main controller for processing commands and driving the motors.
### 4 DC Motors:
Provide movement for the robot.
### Motor Driver Module : 
Controls motor speed and direction.
### Bluetooth Module:
Facilitates wireless communication.
### Obstacle Sensor (e.g., Ultrasonic or IR):
Detects obstacles for autonomous mode.
### Battery Pack:
Supplies power to all components.
### Chassis and Wheels: 
Physical structure of the robot.

# Block diagram
![diagrama block](https://github.com/user-attachments/assets/8a34a1ad-a723-4d13-a826-3059217bc1bf)


| Component | Quantity | Description | Link/Datasheet |
| --- | --- | --- | --- |
| Arduino Uno | 1 | Microcontroller for main control | |
| DC Motors | 4 | 6V DC motors for robot movement | | 
| Motor Driver (L293D) | 1 | motor control | | 
| Bluetooth Module | 1 | wireless control | | 
| Ultrasonic Sensor(HC-SR04) | 1 |  for obstacle detection | | 
| Servo Motor | 1 | for sensor positioning | | 
| Battery Pack| 1 | 6V supply for robot power | | 

# Hardware Functionality
## Components and Interfaces
### Arduino Uno/Nano:

Central controller.
Interfaces:
Digital pins for motor driver, Bluetooth, and sensor communication.
Power pins for sensors and modules.
### Bluetooth Module (HC-05/HC-06):

Interface: UART communication (TX and RX pins).
Use: Receives commands from the smartphone.
### Motor Driver (L293D):

Interface: PWM signals from Arduino to control motor speed and direction.
Power: Connected to external battery for higher current output.
DC Motors:

### Driven by the motor driver.
Connected to the wheels for movement.
Ultrasonic Sensor (HC-SR04):

Interface: Trigger and Echo pins connected to Arduino A1 and A0 pins.
Use: Measures distance to obstacles for autonomous mode.
Battery Pack:

# Images and Proof of Functionality
![proof](https://github.com/user-attachments/assets/34524e55-052d-470f-8390-4df0902e23ea)

# Software Design
## Features
## Bluetooth Control Mode:
Commands from the smartphone are interpreted by the Arduino to move the robot forward, backward, left, or right.
## Autonomous Mode:
The obstacle sensor data is processed to navigate around obstacles.
## Mode Switching:
A button on the smartphone app or a hardware switch toggles between manual and autonomous modes.
# Code Overview
The Arduino code includes:

Serial communication setup for Bluetooth commands.
A control loop to process sensor data and drive motors accordingly.
Mode management for manual and autonomous operation.
Results
The robot successfully:

Responds to Bluetooth commands in real-time.
Avoids obstacles autonomously using the installed sensors.
Operates smoothly in both modes with seamless transitions.
# Conclusions
 It can successfully navigate through obstacles and can be adapted to solve mazes, making it a great tool for educational purposes and hobby projects. Furthermore, its design serves as an excellent starting point for developing more advanced robots, such as smart vacuum cleaners or autonomous delivery systems.

