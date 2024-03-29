# WPILib HAL Simulation - XRP Edition
## Introduction
This repository contains a reference implementation of a XRP robot that can be controlled via the WPILib HALSim WebSocket extension.

The firmware implements (a subset) of the [WPILib Robot Hardware Interface WebSockets API spec](https://github.com/wpilibsuite/allwpilib/blob/main/simulation/halsim_ws_core/doc/hardware_ws_api.md). It behaves similarly to the [WPILib Romi Project](https://github.com/wpilibsuite/wpilib-ws-robot-romi) (with some functionality removed due to being an embedded system vs a Linux machine).

## Built-in IO Mapping

### Digital I/O Map
| DIO Port # | Function          |
|------------|-------------------|
| 0          | XRP User Button   |
| 1          | XRP Onboard LED   |
| 2          | RESERVED          |
| 3          | RESERVED          |
| 4          | Left Encoder A    |
| 5          | Left Encoder B    |
| 6          | Right Encoder A   |
| 7          | Right Encoder B   |
| 8          | Motor 3 Encoder A |
| 9          | Motor 3 Encoder B |
| 10         | Motor 4 Encoder A |
| 11         | Motor 4 Encoder B |

### PWM Map

| PWM Port # | Function    |
|------------|-------------|
| 0          | Left Motor  |
| 1          | Right Motor |
| 2          | Motor 3     |
| 3          | Motor 4     |
| 4          | Servo 1     |
| 5          | Servo 2     |