# ESP32 + Arduino Master-Slave Setup for TurtleBot

This guide explains how to connect your hand gesture control system to a real TurtleBot using ESP32 and Arduino in a master-slave configuration.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    System Architecture                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐      ┌──────────────┐                   │
│  │   Webcam     │ ───► │ hand_drive_  │                   │
│  └──────────────┘      │    node      │                   │
│                        └──────┬───────┘                   │
│                               │                            │
│                               ▼                            │
│                        ┌──────────────┐                    │
│                        │   /cmd_vel   │                    │
│                        │   (ROS2)     │                    │
│                        └──────┬───────┘                    │
│                               │                            │
│                               │ WiFi (micro-ROS)           │
│                               ▼                            │
│  ┌──────────────────────────────────────────────┐         │
│  │  ESP32 (Master)                               │         │
│  │  - micro-ROS subscriber                       │         │
│  │  - Receives /cmd_vel                          │         │
│  │  - Processes commands                         │         │
│  └──────┬───────────────────────────────────────┘         │
│         │ Serial/UART/I2C                                │
│         ▼                                                 │
│  ┌──────────────────────────────────────────────┐         │
│  │  Arduino (Slave)                             │         │
│  │  - Receives motor commands                   │         │
│  │  - Controls left/right motors                │         │
│  │  - Motor driver (L298N, TB6612, etc.)       │         │
│  └──────┬───────────────────────────────────────┘         │
│         │                                                 │
│         ▼                                                 │
│  ┌──────────────┐    ┌──────────────┐                   │
│  │  Left Motor  │    │ Right Motor  │                   │
│  └──────────────┘    └──────────────┘                   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Hardware Requirements

### Components Needed:
1. **ESP32 Development Board** (e.g., ESP32-DevKitC)
2. **Arduino Uno/Nano** (or compatible)
3. **Motor Driver** (L298N, TB6612FNG, or DRV8833)
4. **TurtleBot Motors** (2x DC motors with encoders)
5. **Power Supply** (12V battery for motors, 5V for boards)
6. **Jumper Wires**
7. **USB Cables** (for programming)

### Wiring Connections:

#### ESP32 ↔ Arduino Communication:
- **Option 1: Hardware Serial (Serial2)** (Recommended)
  - ESP32 GPIO 17 (TX2) → Arduino RX
  - ESP32 GPIO 16 (RX2) → Arduino TX (optional, if bidirectional)
  - GND → GND
  - Note: ESP32 Serial (USB) is used for micro-ROS, Serial2 is used for Arduino
  
- **Option 2: Software Serial** (Alternative)
  - Can use any GPIO pins if Serial2 conflicts
  - Use SoftwareSerial library on ESP32
  - GND → GND

- **Option 2: I2C**
  - ESP32 SDA → Arduino SDA (A4)
  - ESP32 SCL → Arduino SCL (A5)
  - GND → GND

#### Arduino → Motor Driver:
- **L298N Example:**
  - Arduino D5 → IN1 (Left motor forward)
  - Arduino D6 → IN2 (Left motor backward)
  - Arduino D9 → ENA (Left motor PWM)
  - Arduino D10 → IN3 (Right motor forward)
  - Arduino D11 → IN4 (Right motor backward)
  - Arduino D3 → ENB (Right motor PWM)
  - GND → GND
  - 5V → 5V (if needed)

#### Motor Driver → Motors:
- Motor Driver OUT1/OUT2 → Left Motor
- Motor Driver OUT3/OUT4 → Right Motor
- 12V Battery → Motor Driver VCC
- GND → GND

## Step-by-Step Setup

### Step 1: Install micro-ROS on ESP32

#### 1.1 Install Prerequisites

```bash
# Install ESP-IDF (version 4.4 or later)
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
. ./export.sh
```

#### 1.2 Install micro-ROS for ESP32

```bash
# Create micro-ROS workspace
mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install dependencies
sudo apt update
sudo apt install -y gcc-arm-none-eabi

# Build micro-ROS agent
cd ~/microros_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

#### 1.3 Flash micro-ROS Firmware to ESP32

```bash
# Connect ESP32 via USB
# Find the port (usually /dev/ttyUSB0 or /dev/ttyACM0)
ls /dev/ttyUSB* /dev/ttyACM*

# Flash firmware
ros2 run micro_ros_setup create_firmware_ws.sh host
ros2 run micro_ros_setup configure_firmware.sh subscriber --transport serial
ros2 run micro_ros_setup build_firmware.sh
ros2 run micro_ros_setup flash_firmware.sh /dev/ttyUSB0
```

### Step 2: Program ESP32 (Master)

#### 2.1 Install Arduino IDE or PlatformIO

**Option A: Arduino IDE**
1. Install Arduino IDE
2. Add ESP32 board support:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install

**Option B: PlatformIO (Recommended)**
```bash
# Install PlatformIO
pip install platformio
```

#### 2.2 Create ESP32 Code

See `firmware/esp32_master/esp32_master.ino` in this repository.

The ESP32 will:
- Connect to WiFi
- Connect to micro-ROS agent
- Subscribe to `/cmd_vel` topic
- Send motor commands to Arduino via Serial

### Step 3: Program Arduino (Slave)

#### 3.1 Install Arduino IDE

Download from: https://www.arduino.cc/en/software

#### 3.2 Create Arduino Code

See `firmware/arduino_slave/arduino_slave.ino` in this repository.

The Arduino will:
- Receive commands from ESP32 via Serial
- Control motors using PWM
- Implement differential drive kinematics

### Step 4: Set Up micro-ROS Agent on Your Computer

#### 4.1 Start micro-ROS Agent

```bash
# Terminal 1: Start micro-ROS agent
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash

# For Serial connection (USB)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# For WiFi connection (if configured)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

#### 4.2 Verify Connection

```bash
# Terminal 2: Check if ESP32 is connected
ros2 node list
ros2 topic list
ros2 topic echo /cmd_vel
```

### Step 5: Configure WiFi on ESP32 (Optional but Recommended)

If using WiFi instead of Serial for micro-ROS:

1. Edit ESP32 code to include WiFi credentials
2. Update micro-ROS agent to use UDP instead of Serial
3. Connect ESP32 and computer to same WiFi network

### Step 6: Test the System

#### 6.1 Start Hand Gesture Control

```bash
# Terminal 1: Start hand gesture node
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch hand_gesture_robot full_system.launch.py
```

#### 6.2 Monitor Topics

```bash
# Terminal 2: Monitor cmd_vel
ros2 topic echo /cmd_vel

# Terminal 3: Check node status
ros2 node list
ros2 topic hz /cmd_vel
```

#### 6.3 Test Robot Movement

1. Make hand gestures in front of camera
2. Verify `/cmd_vel` messages are being published
3. Check if ESP32 receives messages (via Serial monitor)
4. Verify Arduino receives commands (via Serial monitor)
5. Robot should move according to gestures

## Troubleshooting

### ESP32 Not Connecting to micro-ROS Agent

```bash
# Check serial port
ls -l /dev/ttyUSB* /dev/ttyACM*

# Check permissions
sudo usermod -a -G dialout $USER
# Log out and back in

# Test serial communication
screen /dev/ttyUSB0 115200
```

### Arduino Not Receiving Commands

- Check Serial baud rate matches (115200)
- Verify wiring (TX→RX, RX→TX, GND)
- Check Serial Monitor is not open (blocks communication)
- Add debug prints to ESP32 code

### Motors Not Moving

- Check motor driver power supply (12V)
- Verify motor driver connections
- Test motors directly with battery
- Check PWM pins are correct
- Verify motor driver enable pins

### ROS2 Topics Not Visible

```bash
# Check if micro-ROS agent is running
ros2 node list

# Restart micro-ROS agent
# Reboot ESP32
```

## Next Steps

1. **Calibration**: Adjust motor speed scaling in Arduino code
2. **Safety**: Add emergency stop functionality
3. **Feedback**: Add encoder feedback for closed-loop control
4. **Battery Monitoring**: Add voltage monitoring on ESP32
5. **LED Indicators**: Add status LEDs for debugging

## Files to Create

The following firmware files need to be created:
- `firmware/esp32_master/esp32_master.ino` - ESP32 master code
- `firmware/arduino_slave/arduino_slave.ino` - Arduino slave code
- `firmware/README.md` - Firmware documentation

See the `firmware/` directory for complete code examples.

