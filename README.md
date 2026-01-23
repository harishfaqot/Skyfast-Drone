# Skyfast-Drone

Skyfast-Drone is a robotics and UAV development repository focused on **high-speed autonomous drone systems**, system integration, and experimentation with **ROS-based perception, control, and navigation** pipelines.

This project is intended for research, prototyping, and competition-oriented development, emphasizing **modularity, reproducibility, and hardware integration**.

---

## Project Objectives

- Develop a **high-speed autonomous drone stack**
- Integrate **flight controller (FC)** with onboard computing
- Support **ROS / ROS Noetic** workflows
- Enable perception modules such as:
  - Visual SLAM / VIO
  - LiDAR or depth camera integration
- Support simulation and real-hardware deployment

---

## System Architecture (High-Level)

- **Flight Controller**
  - PX4 / ArduPilot (via MAVLink)
- **Companion Computer**
  - Ubuntu Linux
  - ROS Noetic
- **Sensors**
  - IMU
  - Camera / Depth Camera
  - LiDAR (optional)
- **Communication**
  - MAVROS
  - Serial / USB / Ethernet

---

## Requirements

### Software
- Ubuntu 20.04
- ROS Noetic
- Docker (optional)
- Git

### Hardware
- Flight Controller (PX4 / ArduPilot compatible)
- Companion computer (Jetson / x86 / Raspberry Pi)
- Camera / LiDAR
- RC or telemetry link

---

## Setup

### ROS Workspace Example
mkdir -p ~/skyfast_ws/src
ln -s ~/Skyfast-Drone/src ~/skyfast_ws/src/skyfast_drone
cd ~/skyfast_ws
skyfast_make
source devel/setup.bash

---

## Docker Support

Docker can be used for:
- Reproducible builds
- Isolated ROS environments
- Multi-platform deployment

Example:
docker build -t skyfast-drone .
docker run -it --net=host --privileged skyfast-drone

---

## Status

- Repository initialized
- ROS workspace structure ready
- MAVROS integration: in progress
- Sensor drivers: planned
- Autonomous mission logic: planned

---

## Author

Muhammad Harish  
Robotics | UAV Systems | ROS | SLAM
