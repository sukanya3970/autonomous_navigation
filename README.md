
# Autonomous Rover Navigation Using ROS 2 Nav2

## Project Description
This project focuses on developing a four-wheel autonomous rover capable of mapping its environment, localizing itself, navigating to user-defined goal positions, and avoiding obstacles. The system uses **ROS 2 Jazzy**, **SLAM Toolbox**, and the **Nav2 Navigation Stack** to achieve autonomous navigation in indoor environments.

An **RPLIDAR A1** is used to collect laser scan data for mapping and obstacle detection. **Wheel encoders** and a **BNO055 IMU** provide odometry and orientation information, which can be fused using an Extended Kalman Filter (EKF) for improved localization. An **Arduino Mega** controls the four-wheel drive system through BTS7960 motor drivers.

Users can set navigation goals through RViz2, allowing the rover to plan paths, move toward target locations, and respond to obstacles using its navigation and recovery mechanisms.

## Key Features
- Autonomous navigation using ROS 2 Nav2.
- Environment mapping and localization using SLAM Toolbox.
- LiDAR-based obstacle detection using RPLIDAR A1.
- Wheel encoder and IMU sensor integration.
- Odometry and sensor fusion using an EKF.
- Four-wheel motor control using Arduino Mega and BTS7960 drivers.
- Goal-based navigation through RViz2.
- Obstacle-aware navigation and recovery behaviors.

## Technologies Used
- ROS 2 Jazzy
- Nav2 Navigation Stack
- SLAM Toolbox
- RViz2
- RPLIDAR A1
- Arduino Mega
- BNO055 IMU
- AMT103 Incremental Wheel Encoders
- Python and C++
- Extended Kalman Filter (EKF)

## Project Demonstration
**YouTube Video:** [Watch the Autonomous Rover Demo](https://youtube.com/@kotasaisukanya?si=-Iov9DFV2_BRGaOc)

## Project Objective
To develop a low-cost autonomous mobile robot that integrates sensor data, environment mapping, localization, path planning, and obstacle avoidance to navigate safely toward user-defined destinations.
