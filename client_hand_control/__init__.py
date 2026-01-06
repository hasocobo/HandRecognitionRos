"""
Client Hand Control - Standalone hand gesture control client.

This module runs on a client machine (laptop), processes camera/RTSP frames
locally with MediaPipe, and sends validated JSON control messages to a
server gateway over WebSocket.

NO ROS2 DEPENDENCIES.
"""

__version__ = "1.0.0"

