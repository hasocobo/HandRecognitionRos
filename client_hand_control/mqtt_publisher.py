"""
MQTT publisher — the client's outbound transport.

Replaces the WebSocket client on the active path: instead of sending control
messages to a server over WS, we publish them straight to an MQTT broker. The
robot subscribes to the same topic. Client and robot never talk directly — the
broker is the queue in the middle.

Connection/loop plumbing is the same pattern as the old
server_gateway/mqtt_bridge.py (paho with a background loop thread), minus all the
twist->PWM mixing — mixing now lives on the robot. We just ship the JSON.

The async surface (start/stop/send_async/connected) mirrors WebSocketClient so
main.py can swap one for the other without restructuring.
"""

import asyncio
import logging
import time
from typing import Awaitable, Callable, Optional

import paho.mqtt.client as mqtt

from .message import ControlMessage

logger = logging.getLogger(__name__)


class MQTTPublisher:
    """Thin synchronous paho wrapper: connect, publish JSON, reconnect."""

    def __init__(
        self,
        broker: str = "localhost",
        port: int = 1883,
        topic: str = "robot/cmd",
    ):
        self.broker = broker
        self.port = port
        self.topic = topic

        self._client: Optional[mqtt.Client] = None
        self._connected = False
        self._running = False
        self._messages_sent = 0

    def start(self) -> bool:
        if self._running:
            return self._connected
        try:
            client_id = f"hand_control_client_{int(time.time())}"
            self._client = mqtt.Client(client_id=client_id)
            self._client.on_connect = self._on_connect
            self._client.on_disconnect = self._on_disconnect
            # Auto-reconnect with backoff if the broker drops.
            self._client.reconnect_delay_set(min_delay=1, max_delay=16)

            logger.info(f"Connecting to MQTT broker at {self.broker}:{self.port}")
            self._client.connect(self.broker, self.port, keepalive=60)
            self._running = True
            self._client.loop_start()

            for _ in range(50):  # up to 5s for the initial connect
                if self._connected:
                    break
                time.sleep(0.1)
            if not self._connected:
                logger.warning("MQTT connect timeout — will keep retrying in background")
            return self._connected
        except Exception as e:
            logger.error(f"Failed to connect to MQTT broker: {e}")
            return False

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()
            self._client = None
        self._connected = False
        logger.info("MQTT publisher stopped")

    def publish(self, msg: ControlMessage) -> bool:
        if not self._connected or not self._client:
            return False
        try:
            self._client.publish(self.topic, msg.to_json(), qos=0)  # fire-and-forget
            self._messages_sent += 1
            return True
        except Exception as e:
            logger.error(f"Failed to publish: {e}")
            return False

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._connected = True
            logger.info("Connected to MQTT broker")
        else:
            logger.error(f"MQTT connection failed with code: {rc}")

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        if rc != 0:
            logger.warning(f"Unexpected MQTT disconnect (code {rc}); reconnecting")

    @property
    def connected(self) -> bool:
        return self._connected


class AsyncMQTTPublisher:
    """Async wrapper matching WebSocketClient's interface for a drop-in swap."""

    def __init__(
        self,
        broker: str = "localhost",
        port: int = 1883,
        topic: str = "robot/cmd",
        on_connected: Optional[Callable[[], Awaitable[None]]] = None,
        on_disconnected: Optional[Callable[[], Awaitable[None]]] = None,
    ):
        self._pub = MQTTPublisher(broker=broker, port=port, topic=topic)
        self._on_connected = on_connected
        self._on_disconnected = on_disconnected
        self._was_connected = False

    async def start(self) -> None:
        loop = asyncio.get_event_loop()
        ok = await loop.run_in_executor(None, self._pub.start)
        if ok and self._on_connected:
            self._was_connected = True
            await self._on_connected()

    async def stop(self) -> None:
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._pub.stop)

    async def send_async(self, msg: ControlMessage) -> bool:
        """Publish a control message. Also fires connect/disconnect callbacks on
        edges, since paho reconnects silently in the background."""
        ok = self._pub.publish(msg)
        if self._pub.connected and not self._was_connected:
            self._was_connected = True
            if self._on_connected:
                await self._on_connected()
        elif not self._pub.connected and self._was_connected:
            self._was_connected = False
            if self._on_disconnected:
                await self._on_disconnected()
        return ok

    @property
    def connected(self) -> bool:
        return self._pub.connected
