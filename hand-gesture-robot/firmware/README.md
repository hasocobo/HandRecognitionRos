# ESP32 + Arduino Firmware (MQTT)

This firmware bridges MQTT motor commands to an Arduino motor controller.

## Directory Structure

```
firmware/
├── esp32_master/
│   └── esp32_master.ino
└── arduino_slave/
    └── arduino_slave.ino
```

## Quick Start

1. Flash `esp32_master/esp32_master.ino` to the ESP32.
2. Flash `arduino_slave/arduino_slave.ino` to the Arduino.
3. Wire the boards and motors (see below).
4. Configure WiFi and MQTT settings in `esp32_master.ino`.

## Wiring

- ESP32 TX2 (GPIO17) -> Arduino RX
- ESP32 RX2 (GPIO16) -> Arduino TX
- ESP32 GND -> Arduino GND

## MQTT Contract

- Command topic: `robot/cmd`
- Telemetry topic: `robot/telemetry` (optional)
- Payload:

```json
{"left": -255, "right": 255, "ts": 1700000000000}
```

## ESP32 Configuration

Edit these constants in `esp32_master.ino`:
- `WIFI_SSID`
- `WIFI_PASS`
- `MQTT_SERVER`
- `MQTT_PORT`

## Arduino Serial Protocol

The ESP32 sends commands to the Arduino over Serial2:

```
<L:xxx,R:yyy>
```

Where `xxx` and `yyy` are PWM values in `-255..255`.

## Safety

- ESP32 stops motors if no MQTT command arrives for `TIMEOUT_MS`.
- Arduino stops motors if no serial command arrives for `TIMEOUT_MS`.
