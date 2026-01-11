# Hand Gesture Controlled Robot

Control a differential-drive robot using hand gestures from a webcam. The client runs locally with MediaPipe and sends validated control messages over WebSocket to a small server that forwards motor commands to an ESP32 via MQTT.

This workspace is MQTT/WebSocket only (no ROS2).

## Architecture

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
```

The server exposes:
- WebSocket control endpoint: `ws://<server>:8080/control`
- Health check: `http://<server>:8080/health`

## Project Structure

```
client_hand_control/   # Client app (no ROS2)
server_gateway/        # WebSocket server + MQTT bridge
hand-gesture-robot/    # Hardware + firmware (MQTT)
```

## Quick Start

### Prerequisites

- Python 3.10+
- Webcam or RTSP stream
- MQTT broker (e.g., Mosquitto) and ESP32 firmware for real hardware

### Install

```bash
cd client_hand_control
pip install -r requirements.txt

cd ../server_gateway
pip install -r requirements.txt
```

### Run (local test)

Terminal 1 (optional MQTT broker):
```bash
mosquitto -v
```

Terminal 2 (server):
```bash
# Bash
export CONTROL_TOKEN=test123

# PowerShell
$env:CONTROL_TOKEN="test123"

python -m server_gateway.main
```

Terminal 3 (client):
```bash
python -m client_hand_control.main --server ws://127.0.0.1:8080/control --token test123 --camera 0 --preview

# RTSP example
python -m client_hand_control.main --server ws://127.0.0.1:8080/control --token test123 --rtsp "rtsp://10.8.34.150:8554/handcam" --preview
```

## Configuration

### Client CLI

| Argument | Default | Description |
|---------|---------|-------------|
| `--server` | `ws://127.0.0.1:8080/control` | WebSocket server URL |
| `--token` | (required) | Authentication token |
| `--camera` | `0` | Camera device index |
| `--rtsp` | `None` | RTSP URL (overrides `--camera`) |
| `--max-linear` | `0.22` | Max linear velocity (m/s) |
| `--max-angular` | `2.84` | Max angular velocity (rad/s) |
| `--rate` | `15.0` | Control loop rate (Hz) |
| `--preview` | `false` | Show preview window |
| `--invalid-timeout` | `300` | Force-stop after invalid frames (ms) |
| `--debug` | `false` | Enable debug logging |

### Server Environment

| Variable | Default | Description |
|----------|---------|-------------|
| `CONTROL_TOKEN` | (required) | Client authentication token |
| `DEADMAN_MS` | `300` | Deadman timeout (ms) |
| `MAX_LINEAR` | `0.22` | Max linear velocity (m/s) |
| `MAX_ANGULAR` | `2.84` | Max angular velocity (rad/s) |
| `MQTT_HOST` | `localhost` | MQTT broker host |
| `MQTT_PORT` | `1883` | MQTT broker port |

## MQTT Contract

- Command topic: `robot/cmd`
- Telemetry topic: `robot/telemetry` (optional)
- Command payload (JSON):

```json
{"left": -255, "right": 255, "ts": 1700000000000}
```

## Controls

- Hold both hands still to calibrate the neutral position.
- Push both hands forward to accelerate; pull back to reverse.
- Twist the "handlebar" (left hand forward, right back) to steer.
- Hold both hands open for about 1.5s to toggle drive enable.
- Keyboard: `E` toggle enable, `C` recalibrate, `Q`/`Esc` quit.

## Safety Features

- Frame gate drops corrupted frames and forces stop after timeout.
- Hand gate requires both hands visible and calibrated.
- Message validation clamps out-of-range values.
- Deadman timer stops the robot if messages stop arriving.
- Single-controller lock prevents two clients from driving at once.
- Graceful shutdown sends a final stop.

## Documentation

- `ARCHITECTURE.md`
- `hand-gesture-robot/firmware/README.md` (ESP32 + Arduino firmware)

## Authors

- Esin Eltimor
- Hasan Coban
- Lilian L'helgoualc'h

CENG483 Behavioral Robotics - Fall 2025

## License

MIT License - see `LICENSE`.
