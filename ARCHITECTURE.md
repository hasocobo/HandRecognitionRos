# System Architecture

This project uses a client-server design with MQTT for robot control. There is no ROS2 in the runtime path.

## Diagram

```
Camera/RTSP -> Frame Gate -> MediaPipe -> Hand Control -> WebSocket
                                              |
                                              v
                                    Server Gateway
                                              |
                                              v
                                      MQTT (robot/cmd)
                                              |
                                              v
                                           ESP32
                                              |
                                              v
                                           Arduino
```

## Components

- `client_hand_control/`
  - Captures frames, validates them, detects hands, computes controls.
  - Sends JSON control messages over WebSocket.
- `server_gateway/`
  - Authenticates clients, enforces single-controller lock.
  - Publishes motor commands to MQTT and enforces a deadman timer.
- `hand-gesture-robot/firmware/`
  - ESP32 subscribes to MQTT and forwards left/right PWM to Arduino.
  - Arduino drives the motors.

## Data Flow

1. Client validates frames and computes linear/angular commands.
2. Client sends `{linear, angular, enable, ts_ms}` over WebSocket.
3. Server clamps values, applies deadman, publishes `robot/cmd`.
4. ESP32 receives `{"left": -255..255, "right": -255..255}` and forwards to Arduino.
5. Arduino drives motors and stops on timeout.

## MQTT Contract

- Command topic: `robot/cmd`
- Telemetry topic: `robot/telemetry` (optional)
- Command payload:

```json
{"left": -255, "right": 255, "ts": 1700000000000}
```

## Run (Local)

```bash
# Server
export CONTROL_TOKEN=test123
python -m server_gateway.main

# Client
python -m client_hand_control.main --server ws://127.0.0.1:8080/control --token test123 --camera 0 --preview
```
