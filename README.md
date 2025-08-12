Fuzzy Wall-Following and PID Object Tracking on Mobile Robot
This repository contains a mobile robot project with a hybrid control system combining a Fuzzy Logic Controller for wall-following navigation and a PID Controller for camera-based object tracking. The system uses a distributed architecture where the Raspberry Pi 4 Model B handles image processing and PID computation, while the ESP32 Devkit V1 controls the actuators in real-time based on fuzzy logic and commands from the Raspberry Pi.

Key Features
Wall-following navigation using Fuzzy Logic Controller based on VL53L0X distance sensor.

Camera-based object tracking using PID Controller.

Raspberry Pi integration for computer vision and ESP32 for motor control.

Multi-tasking system capable of performing navigation and object tracking simultaneously.

Object detection based on HSV (Hue, Saturation, Value) for high accuracy under various lighting conditions.

Technologies Used
Raspberry Pi 4 Model B – Computer vision processing and PID computation.

ESP32 Devkit V1 – Real-time actuator control and fuzzy logic execution.

Fuzzy Logic Controller (FLC) – Handles uncertainty and non-linearity in robot navigation.

PID Controller – Adjusts robot’s speed and direction during object tracking.

OpenCV (Python) – HSV-based object detection from Raspberry Pi camera.

Serial Communication (UART) – Connects Raspberry Pi with ESP32.

Main Components Used
Raspberry Pi 4 Model B

ESP32 Devkit V1

SG90 Servo Motor (for gripper control)

Pi Camera (Raspberry Pi camera for computer vision)

N30 Micro Gear DC Motor (wheel drive)

L298N Motor Driver

XL4016 Step Down Converter (voltage regulation)

18650 Battery (x3)

3S 18650 Battery Holder

Contact
📧 Email: zahidan54@gmail.com
📷 Instagram: @zhdnakhmad
