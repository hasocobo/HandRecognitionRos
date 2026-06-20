# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

Hand gesture **command publisher**. Webcam → MediaPipe (local) → `ControlMessage` → MQTT (`robot/cmd`) → robot. No ROS2, no gateway server — the client publishes straight to the broker. The robot (the 424pi rover, a separate repo) subscribes and drives itself in "onroad" mode. The wire contract for the robot side lives in **`GESTURES.md`**.

## Running the System

**MQTT broker** (local test):
```bash
mosquitto -v
```

**Client** (publishes to the broker):
```bash
# Webcam, broker on localhost
python -m client_hand_control.main --camera 0 --preview

# Broker elsewhere (env var or --broker)
MQTT_BROKER=10.42.0.243 python -m client_hand_control.main --camera 0 --preview
python -m client_hand_control.main --broker 10.42.0.243 --camera 0 --preview
```

**Install dependencies**:
```bash
pip install -r client_hand_control/requirements.txt
```

## Architecture

```
Camera/RTSP → FrameGate → MediaPipe → HandControl + GestureState → MQTT Publisher
                                                                         │  robot/cmd (JSON)
                                                                   MQTT broker
                                                                         │
                                                            robot subscriber (424pi rover)
```

**Client pipeline** (`client_hand_control/`):
- `frame_gate.py` — rejects corrupted/empty frames, force-stops on sustained bad stream (300ms)
- `hand_control.py` — MediaPipe landmarks → handlebar (continuous speed/steer), calibration, arming
- `gestures.py` — discrete commands: mode toggle (both thumbs up), start/stop (both fists), lane-change events (point left/right). E-stop is a keyboard key (SPACE), not a gesture.
- `message.py` — `ControlMessage` dataclass, bounds/NaN/monotonicity + field validation
- `mqtt_publisher.py` — paho publisher, background reconnect, fire-and-forget (QoS 0)
- `main.py` — async orchestration at 15 Hz

**Firmware** (`hand-gesture-robot/firmware/`): **legacy ESP32/Arduino** from the old direct-drive robot. Not used by the current rover. Kept for reference.

## Key Contract

**MQTT topic** `robot/cmd` (client → robot), the `ControlMessage` JSON:
```json
{"linear": 0.1, "angular": -0.5, "enable": true, "mode": "onroad", "run": true, "lane_change": "left", "lane_seq": 3, "estop": false, "ts_ms": 1700000000000}
```
Optional fields (`mode`/`run`/`lane_change`/`lane_seq`/`estop`) are omitted when unset. In onroad the rover drives itself and ignores `linear`/`angular`; it acts on `run`/`lane_change`/`estop`. See `GESTURES.md`.

## Safety Chain

Every layer has its own stop mechanism — intentional, do not remove:
1. Client `FrameGate`: bad stream → publishes stop
2. Client `MessageValidator`: NaN/bounds → drops message
3. Keyboard e-stop (SPACE): zeroes motion + sets `estop`
4. **Robot deadman (on the rover): no command → stop. This is mandatory and lives on the robot.** The real emergency stop is "hands out of frame" (FrameGate), which zeroes the stream so the deadman trips.

## `mediapipe` Is Pinned

`mediapipe==0.10.21` — do not bump without testing. Later versions changed the Hands API.
