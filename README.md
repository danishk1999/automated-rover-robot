# 🤖 Automated Rover Robot
**Course:** CMPT 312 — Intro to Robotics  
**Institution:** MacEwan University  
**Status:** 🚧 In Progress

## Overview
Designed and built a fully autonomous rover robot capable of navigating its 
environment independently using real-time sensor data, computer vision, and 
motor control. The robot integrates hardware and software components to detect 
obstacles and move without human input.

## Hardware Components
| Component | Purpose |
|---|---|
| Raspberry Pi 3B | Main computing unit / brain of the rover |
| Arduino Uno | Low-level motor control and hardware interface |
| Sabertooth 2x12 Motor Controller | Dual motor driver for rover chassis |
| Rover Chassis | Physical frame and wheel system |
| RPLidar | 360° laser distance sensor for obstacle detection |
| USB Camera | Visual input for computer vision processing |

## Software & Libraries
- **Python** — Primary programming language
- **OpenCV** — Computer vision and camera processing
- **NumPy** — Numerical computations and data handling
- **RPLidar Library** — Interfacing with the lidar sensor
- **Serial Communication** — Raspberry Pi to Arduino communication

## System Architecture
