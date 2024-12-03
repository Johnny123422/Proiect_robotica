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
Arduino Board: Main controller for processing commands and driving the motors.
4 DC Motors: Provide movement for the robot.
Motor Driver Module : Controls motor speed and direction.
Bluetooth Module: Facilitates wireless communication.
Obstacle Sensor (e.g., Ultrasonic or IR): Detects obstacles for autonomous mode.
Battery Pack: Supplies power to all components.
Chassis and Wheels: Physical structure of the robot.

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
This project demonstrates the integration of hardware and software in a functional robotic system. It can serve as a foundation for more advanced robotic applications, such as maze-solving robots or AI-driven navigation systems.

