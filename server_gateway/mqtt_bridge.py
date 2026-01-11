"""
MQTT Bridge for ESP32 communication.

Handles:
- Publishing motor commands to robot/cmd
- Subscribing to robot/telemetry for logging
- Differential drive mixing (Twist -> left/right PWM)
"""

import asyncio
import json
import logging
import time
from typing import Optional, Callable

import paho.mqtt.client as mqtt

logger = logging.getLogger(__name__)


class MQTTBridge:
    """
    MQTT bridge for ESP32 motor control.
    
    Converts Twist commands to differential drive PWM values
    and publishes them to the ESP32 via MQTT.
    """
    
    def __init__(
        self,
        host: str = "localhost",
        port: int = 1883,
        cmd_topic: str = "robot/cmd",
        telemetry_topic: str = "robot/telemetry",
        wheel_base: float = 0.16,  # meters between wheels
        max_linear: float = 0.22,  # m/s
        max_angular: float = 2.84,  # rad/s
        pwm_max: int = 255,
        on_telemetry: Optional[Callable[[dict], None]] = None,
    ):
        """
        Initialize MQTT bridge.
        
        Args:
            host: MQTT broker host
            port: MQTT broker port
            cmd_topic: Topic for motor commands
            telemetry_topic: Topic for telemetry data
            wheel_base: Distance between wheels (meters)
            max_linear: Maximum linear velocity for scaling
            max_angular: Maximum angular velocity for scaling
            pwm_max: Maximum PWM value
            on_telemetry: Callback for telemetry messages
        """
        self.host = host
        self.port = port
        self.cmd_topic = cmd_topic
        self.telemetry_topic = telemetry_topic
        self.wheel_base = wheel_base
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.pwm_max = pwm_max
        self.on_telemetry = on_telemetry
        
        # MQTT client
        self._client: Optional[mqtt.Client] = None
        self._connected = False
        self._running = False
        
        # Statistics
        self._messages_sent = 0
        self._messages_received = 0
        self._last_send_time: Optional[float] = None
        
    def start(self) -> bool:
        """
        Start the MQTT bridge.
        
        Returns:
            True if connection successful, False otherwise
        """
        if self._running:
            return True
            
        try:
            # Create client with unique ID
            client_id = f"hand_control_gateway_{int(time.time())}"
            self._client = mqtt.Client(client_id=client_id)
            
            # Set callbacks
            self._client.on_connect = self._on_connect
            self._client.on_disconnect = self._on_disconnect
            self._client.on_message = self._on_message
            
            # Connect
            logger.info(f"Connecting to MQTT broker at {self.host}:{self.port}")
            self._client.connect(self.host, self.port, keepalive=60)
            
            # Start loop in background thread
            self._running = True
            self._client.loop_start()
            
            # Wait for connection
            for _ in range(50):  # 5 second timeout
                if self._connected:
                    break
                time.sleep(0.1)
                
            if not self._connected:
                logger.warning("MQTT connection timeout - continuing without MQTT")
                return False
                
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to MQTT broker: {e}")
            return False
            
    def stop(self) -> None:
        """Stop the MQTT bridge."""
        if not self._running:
            return
            
        self._running = False
        
        # Send stop command
        if self._connected:
            self.publish_motor_command(0.0, 0.0)
            
        # Disconnect
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()
            self._client = None
            
        self._connected = False
        logger.info("MQTT bridge stopped")
        
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback."""
        if rc == 0:
            self._connected = True
            logger.info("Connected to MQTT broker")
            
            # Subscribe to telemetry
            client.subscribe(self.telemetry_topic)
            logger.info(f"Subscribed to {self.telemetry_topic}")
        else:
            logger.error(f"MQTT connection failed with code: {rc}")
            
    def _on_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback."""
        self._connected = False
        if rc != 0:
            logger.warning(f"Unexpected MQTT disconnection, code: {rc}")
        else:
            logger.info("Disconnected from MQTT broker")
            
    def _on_message(self, client, userdata, msg):
        """MQTT message callback."""
        self._messages_received += 1
        
        try:
            payload = json.loads(msg.payload.decode())
            logger.debug(f"Telemetry received: {payload}")
            
            if self.on_telemetry:
                self.on_telemetry(payload)
                
        except json.JSONDecodeError as e:
            logger.warning(f"Invalid telemetry JSON: {e}")
        except Exception as e:
            logger.error(f"Error processing telemetry: {e}")
            
    def publish_motor_command(
        self,
        linear: float,
        angular: float,
        gripper: Optional[int] = None,
    ) -> bool:
        """
        Publish motor command as differential drive PWM.
        
        Converts (linear, angular) velocity to left/right PWM values.
        
        Simple mixing:
        - linear=max, angular=0  → left=255, right=255 (full forward)
        - linear=0, angular=max  → left=255, right=-255 (spin right)
        - linear=max, angular=max → left=255, right=0 (sharp right turn)
        
        Args:
            linear: Linear velocity (m/s), positive = forward
            angular: Angular velocity (rad/s), positive = turn left
            gripper: Optional gripper command (0=open, 1=closed)
            
        Returns:
            True if published successfully
        """
        if not self._connected or not self._client:
            return False
            
        # Normalize to -1..1
        linear_pct = linear / self.max_linear if self.max_linear > 0 else 0.0
        angular_pct = angular / self.max_angular if self.max_angular > 0 else 0.0
        
        # Clamp inputs
        linear_pct = max(-1.0, min(1.0, linear_pct))
        angular_pct = max(-1.0, min(1.0, angular_pct))
        
        # Simple differential mixing
        # Positive angular = turn left = right wheel faster
        left_norm = linear_pct - angular_pct
        right_norm = linear_pct + angular_pct
        
        # Clamp outputs to -1..1
        left_norm = max(-1.0, min(1.0, left_norm))
        right_norm = max(-1.0, min(1.0, right_norm))
        
        # Convert to PWM values
        left_pwm = int(left_norm * self.pwm_max)
        right_pwm = int(right_norm * self.pwm_max)
        
        # Create command payload
        payload = {
            "left": left_pwm,
            "right": right_pwm,
            "ts": int(time.time() * 1000),
        }
        if gripper is not None:
            payload["gripper"] = 1 if int(gripper) != 0 else 0
        
        try:
            self._client.publish(
                self.cmd_topic, 
                json.dumps(payload),
                qos=0,  # Fire and forget for low latency
            )
            self._messages_sent += 1
            self._last_send_time = time.time()
            
            logger.debug(f"Published motor command: {payload}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to publish motor command: {e}")
            return False
            
    def publish_from_twist(
        self,
        linear: float,
        angular: float,
        enable: bool,
        gripper: Optional[int] = None,
    ) -> bool:
        """
        Publish motor command from Twist-like values.
        
        Args:
            linear: Linear velocity (m/s)
            angular: Angular velocity (rad/s)
            enable: If False, send zero command
            gripper: Optional gripper command (0=open, 1=closed)
            
        Returns:
            True if published successfully
        """
        if not enable:
            linear = 0.0
            angular = 0.0
            
        return self.publish_motor_command(linear, angular, gripper=gripper)
        
    @property
    def connected(self) -> bool:
        """Check if connected to MQTT broker."""
        return self._connected
        
    def get_stats(self) -> dict:
        """Get bridge statistics."""
        return {
            "connected": self._connected,
            "messages_sent": self._messages_sent,
            "messages_received": self._messages_received,
            "last_send_time": self._last_send_time,
        }


class AsyncMQTTBridge:
    """
    Async wrapper for MQTTBridge.
    
    Provides async-compatible methods for use with asyncio.
    """
    
    def __init__(self, **kwargs):
        """Initialize with same arguments as MQTTBridge."""
        self._bridge = MQTTBridge(**kwargs)
        
    async def start(self) -> bool:
        """Start the MQTT bridge."""
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self._bridge.start)
        
    async def stop(self) -> None:
        """Stop the MQTT bridge."""
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._bridge.stop)
        
    async def publish_from_twist(
        self,
        linear: float,
        angular: float,
        enable: bool,
        gripper: Optional[int] = None,
    ) -> bool:
        """Publish motor command from Twist values."""
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            None, self._bridge.publish_from_twist, linear, angular, enable, gripper
        )
        
    @property
    def connected(self) -> bool:
        """Check if connected."""
        return self._bridge.connected
        
    def get_stats(self) -> dict:
        """Get statistics."""
        return self._bridge.get_stats()

