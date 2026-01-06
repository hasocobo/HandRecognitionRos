"""
Server Gateway - ROS2 bridge with WebSocket control interface.

This module runs on the robot server machine and:
- Accepts WebSocket connections from clients
- Publishes commands to ROS2 topics
- Bridges to MQTT for ESP32 communication
"""

__version__ = "1.0.0"

