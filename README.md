# Autonomous Underwater Vehicle (AUV)

An Autonomous Underwater Vehicle developed for underwater navigation, perception, and control through the integration of embedded systems, sensor technology, computer vision, and robotic propulsion.

## Overview

The Autonomous Underwater Vehicle (AUV) is an underwater robotic system integrating mechanical design, electronic control, sensor technologies, and thruster-based propulsion. It combines embedded processing, visual perception, acoustic sensing, and control algorithms to perform autonomous underwater operations.

## Key Features

- Six-thruster propulsion system for multi-directional movement.
- Depth and orientation sensing.
- Camera-based underwater visual perception.
- Obstacle detection and target identification.
- Acoustic pinger detection and localization.
- PID-based stabilization and motion control.
- Integrated waterproof electronic enclosure.

## Hardware Components

| Component | Application |
|---|---|
| Raspberry Pi 4B (8GB) | Main processing unit |
| Arduino Mega 2560 | Sensor interfacing and actuator control |
| Raspberry Pi Camera Module 3 | Underwater visual perception |
| MPU6050 IMU | Orientation sensing |
| MS5837-02BA | Pressure and depth measurement |
| Blue Robotics T200 Thrusters (6) | Underwater propulsion |
| Electronic Speed Controllers | Thruster control |
| Hydrophone and Preamplifier | Acoustic signal detection |
| Underwater Ultrasonic Sensor | Obstacle detection |
| Li-ion Battery | Power supply |
| Buck Converter | Voltage regulation |

## Software & Control

- Embedded controller communication between Raspberry Pi and Arduino Mega.
- Sensor data acquisition and processing.
- Computer vision and image processing.
- Acoustic signal processing using GCC-PHAT and Time Difference of Arrival (TDOA) estimation.
- Thruster allocation for distributing control forces and torques.
- PID-based stabilization and motion control.

## Repository Structure

```text
Autonomous-Underwater-Vehicle/
│
├── README.md
│
├── Hardware/
│   ├── Mechanical/
│   └── Electronics/
│
├── Software/
│
├── Documentation/
│
└── Media/
```

## Applications

- Underwater exploration.
- Marine environmental monitoring.
- Underwater infrastructure inspection.
- Object detection and localization.

## Project Status

**Completed**

## Documentation

Refer to the `Documentation/` directory for the detailed technical report covering mechanical design, structural analysis, electronics integration, system architecture, and control methodology.
