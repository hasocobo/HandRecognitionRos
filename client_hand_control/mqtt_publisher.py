"""
MQTT publisher — the client publishes control messages straight to the broker.

Architecture: client → MQTT broker → robot. No gateway server in between; the
broker IS the queue/decoupler. Fire-and-forget (QoS 0) for low latency — the
robot's deadman handles dropped packets, so a lost message is harmless.
"""

import logging
import time
from typing import Optional

import paho.mqtt.client as mqtt

logger = logging.getLogger(__name__)


class MqttPublisher:
    """Thin paho wrapper: connect in the background, publish JSON strings."""

    def __init__(self, broker: str, port: int = 1883, topic: str = "robot/cmd"):
        self.broker = broker
        self.port = port
        self.topic = topic
        self._connected = False
        self._client = mqtt.Client(client_id=f"hand_control_{int(time.time())}")
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._connected = True
            logger.info("Connected to MQTT broker %s:%d", self.broker, self.port)
        else:
            logger.error("MQTT connect failed (rc=%s)", rc)

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        if rc != 0:
            logger.warning("Unexpected MQTT disconnect (rc=%s); paho will retry", rc)

    def start(self) -> None:
        # connect_async + loop_start so a broker that's down at launch doesn't crash
        # the client — paho keeps retrying in its background thread.
        self._client.connect_async(self.broker, self.port, keepalive=30)
        self._client.loop_start()
        logger.info("MQTT publisher started (%s:%d, topic=%s)", self.broker, self.port, self.topic)

    def stop(self) -> None:
        self._client.loop_stop()
        try:
            self._client.disconnect()
        except Exception:
            pass

    @property
    def connected(self) -> bool:
        return self._connected

    def publish(self, payload: str) -> bool:
        """Publish a JSON string. Returns True if handed to paho while connected."""
        if not self._connected:
            return False
        try:
            self._client.publish(self.topic, payload, qos=0)
            return True
        except Exception as e:
            logger.error("MQTT publish failed: %s", e)
            return False
