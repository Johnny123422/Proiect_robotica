# Bluetooth-Controlled and Autonomous Obstacle-Avoiding Robot
## Introduction
This project involves creating a robot that can be controlled via Bluetooth using a smartphone or operate autonomously to avoid obstacles. The goal is to combine manual and autonomous control in a versatile, user-friendly design. This project is useful for learning about embedded systems, robotics, and communication between hardware components. It also has potential applications in education and hobby projects.

# General Description
The robot is based on an Arduino microcontroller and includes the following features:
Bluetooth Control: The robot can be remotely operated using a smartphone app.
Autonomous Navigation: Using sensors, the robot detects and avoids obstacles.
Mobility: Powered by four motors, the robot has wheels for smooth movement.


# Current State of Software Implementation
The project is functional and includes:

### Ultrasonic Sensor: 
Used to measure the distance to obstacles.
### Servo Motor: 
Redirects the ultrasonic sensor to measure distances on the left and right sides.
### DC Motors:  
Controlled via the AFMotor library to ensure smooth movement of the robot.
### Obstacle Avoidance Algorithm:  
Detects obstacles and decides the optimal avoidance direction (left or right) based on measured distances.
Software validation was performed through tests on actual hardware.

# Motivation for Library Choices
### <Servo.h>: 
Simplifies control of the servo motor, which is used for directing the ultrasonic sensor.
### <AFMotor.h>: 
A specific library for controlling DC motors through the Adafruit Motor Shield, providing an easy-to-use interface for managing speed and direction.
### <Arduino.h>: 
Includes standard Arduino functions for manipulating digital and analog pins.
These libraries were chosen due to their compatibility with the hardware and the ease they bring to the implementation process.

# Novelty of the Project
Intelligent Obstacle Avoidance: The robot determines the optimal avoidance direction (left or right) based on measured distances.
Integration of Multiple Motors: The project uses 8 DC motors, which are controlled simultaneously for complex mobility.
Modular Algorithm Design: Functionalities are implemented in separate modules (e.g., obstacle avoidance, distance measurement, movement), allowing for easy scalability.

# Justification for Using Lab Functionalities
### Ultrasonic Sensor Control: 
The HC-SR04 sensor is used to measure distances, implemented with custom trigger and reading functions.
### Servo and DC Motors: 
Lab practices for motor control were directly applied here to achieve complex movement patterns.
### Serial Communication: 
Used for debugging and monitoring real-time distance measurements.

# Project Structure, Interaction of Functionalities, and Validation
Distance Measurement Module:
### The ultrasonic() function measures distance using the HC-SR04 sensor.
leftsee() and rightsee() functions use the servo motor to direct the ultrasonic sensor for lateral measurements.
### Motor Control Module:
Functions like forward(), backward(), left(), and right() handle robot movement.
The Stop() function stops all motors.
### Obstacle Avoidance Algorithm:
The avoid_obstacle() function decides whether to move forward or avoid an obstacle by turning left or right.
### Validation: 
The software was validated through real-world testing, where the robot successfully navigated an obstacle-filled environment.

# Calibration of Sensors and Components
### Ultrasonic Sensor Calibration:
Sensor readings were cross-verified with a ruler to adjust the conversion formula for time to distance in centimeters.
The delay in the ultrasonic() function was fine-tuned to minimize signal noise.
### Servo Motor Calibration:
The center position (spoint) was adjusted to 85° to align the sensor with the robot’s forward direction.
### DC Motor Calibration:
Speed parameters (Speed and Speed_Cot) were calibrated to ensure stable and consistent movement.

# Optimizations Made
### Avoiding Code Blocking: 
Replaced unnecessary delay() calls with well-defined cycles to reduce latency (can be further optimized with non-blocking methods).
### Function Simplification: 
Refactored motor control functions for simplicity and readability.
Differentiated Motor Speeds: Set separate speeds for front and rear motors to enhance performance during directional changes.

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
![Screenshot 2024-12-22 145159](https://github.com/user-attachments/assets/1645c206-7a4f-484f-a560-dbcd27e254bc)



| Component | Quantity | Description | Link/Datasheet |
| --- | --- | --- | --- |
| Arduino Uno | 1 | Microcontroller for main control | (https://docs.arduino.cc/hardware/uno-rev3/) |
| DC Motors | 4 | 6V DC motors for robot movement | (https://www.contact-evolution.ch/files/DC_motors.pdf) | 
| Motor Driver (L293D) | 1 | motor control | (https://www.alldatasheet.com/view.jsp?Searchword=L293d%20datasheet&gad_source=1&gclid=CjwKCAiAgoq7BhBxEiwAVcW0LB3BW3rR4wXACkna_W92lFrL_49D-PYOrUpoYvx45-kNx5xpSzOgXRoCAKAQAvD_BwE) | 
| Bluetooth Module | 1 | wireless control | (https://www.electronicwings.com/sensors-modules/bluetooth-module-hc-05-) | 
| Ultrasonic Sensor(HC-SR04) | 1 |  for obstacle detection | (https://www.alldatasheet.com/view.jsp?Searchword=Hcsr04%20datasheet&gad_source=1&gclid=CjwKCAiAgoq7BhBxEiwAVcW0LNM13Y2I3MZllnghP2lJDl3hzTrROSy3NxhPAs9rYz3OBykpnaPe7hoCtQMQAvD_BwE) | 
| Servo Motor | 1 | for sensor positioning |  (http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf) | 
| Battery Pack| 1 | 6V supply for robot power | | 

# Electrical Schematic
![Screenshot 2024-12-22 144215](https://github.com/user-attachments/assets/6b1739d0-1a91-433e-bb48-43fc79e10129)


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

