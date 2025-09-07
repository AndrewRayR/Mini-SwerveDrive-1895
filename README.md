# Mini-SwerveDrive-1895
FRC Team 1895's Mini RC Swerve Drive Controller

## Project Overview

This project implements a WiFi-controlled mini swerve drive robot using an ESP32 microcontroller running MicroPython. The system creates a wireless access point that serves a web-based joystick interface, allowing remote control of a four-wheel swerve drive mechanism. Each wheel can independently control both drive speed and steering angle, enabling advanced maneuvers like strafing and rotation in place.

### Key Features
- **WiFi Access Point**: Creates a standalone network for direct device connection
- **WebSocket Communication**: Real-time bidirectional communication for responsive control
- **Dual Joystick Interface**: Touch-friendly web UI with left and right virtual joysticks
- **Swerve Drive Control**: Independent control of drive and steering motors for each wheel
- **Sensor Integration**: Magnetic rotary encoders for precise wheel angle feedback
- **Multiple Drive Modes**: Normal driving, rotation mode, and demonstration mode
- **OLED Display**: Real-time status display on optional SSD1306 OLED screen
- **IMU Support**: Optional MPU6050 integration for motion sensing

## Hardware Requirements
- ESP32 development board
- 4x L298N motor driver modules (or equivalent dual H-bridge drivers)
- 8x DC motors (4 for drive, 4 for steering)
- 4x Magnetic rotary encoders (dual hall sensor type)
- Optional: SSD1306 OLED display (128x32, I2C)
- Optional: MPU6050 IMU sensor
- Power supply suitable for motors and ESP32

## Project Structure

### `main.py`
**Primary firmware file** - MicroPython code for ESP32 that implements the complete swerve drive control system.

**Key Components:**
- **WiFi Access Point Setup**: Configures ESP32 as WiFi AP with static IP (192.168.10.1)
- **WebSocket Server**: Handles real-time communication with web clients
- **Motor Control**: PWM-based speed control with direction management for L298N drivers
- **Sensor Reading**: Analog input processing for magnetic rotary encoders
- **Control Algorithms**: 
  - Swerve kinematics for coordinated wheel movement
  - PID-style heading control for wheel steering
  - Multiple drive modes (normal, rotation, demonstration)
- **Threading**: Separate threads for web server and main control loop
- **Hardware Abstraction**: Pin mapping and hardware initialization routines

**Pin Configuration Sections:**
- Analog sensor pins (GPIO32-39 for ADC compatibility with WiFi)
- Motor driver pins (direction and PWM enable pins)
- I2C pins for OLED and IMU communication
- Configurable pin mapping for different hardware setups

**Drive Modes:**
- **Normal Mode**: Standard swerve drive with coordinated wheel steering
- **Rotation Mode**: All wheels angled for in-place rotation
- **Demonstration Mode**: Automated sequence showing system capabilities

### `web/index.html`
**Web-based user interface** - Single-file HTML application served by the ESP32.

**Features:**
- **Responsive Design**: Works on desktop browsers and mobile devices
- **Dual Virtual Joysticks**: Touch-enabled controls for left and right sticks
- **Visual Feedback**: Active pad highlighting and knob position indication
- **WebSocket Integration**: Real-time communication with ESP32
- **Auto-reconnection**: Handles network interruptions gracefully
- **Status Display**: Shows connection state, active clients, and drive mode
- **Button Controls**: Home, Demo, and mode switching buttons

**Technical Implementation:**
- Pure HTML/CSS/JavaScript with no external dependencies
- Pointer Events API for cross-platform touch/mouse support
- Multi-touch gesture handling with pad exclusivity
- JSON message protocol for control data transmission
- Runtime SSID/password injection by firmware

**Control Mapping:**
- **Right Joystick**: Forward/backward movement (Y-axis) and steering (X-axis)
- **Left Joystick**: Rotation control when in rotation mode
- **Joystick Buttons**: Activated by pressing down on the virtual joysticks
- **Physical Buttons**: Home (wheel alignment) and Demo (automated sequence)

### `LICENSE`
Standard software license file defining usage terms and conditions for the project.

### `README.md`
This documentation file providing comprehensive project information, setup instructions, and technical details.

## Getting Started

1. **Hardware Setup**: Wire motors, sensors, and ESP32 according to pin mappings in `main.py`
2. **Flash Firmware**: Upload `main.py` to ESP32 using MicroPython
3. **Connect**: Join the "Mini-RC-SwerveDrive-AP" WiFi network (password: FRC_TEAM1895)
4. **Control**: Navigate to 192.168.10.1 in a web browser to access the joystick interface

## Configuration

Before first use, edit the PIN MAP section in `main.py` to match your specific wiring configuration. The default pin assignments are provided as a reference but must be verified against your hardware setup.

## Safety Notes

- Test with motors disconnected initially to verify web interface functionality
- Ensure proper power supply ratings for your specific motors
- Use ADC1 pins (GPIO32-39) for analog sensors when WiFi is active
- Verify L298N enable pins receive proper 3.3V logic levels from ESP32
