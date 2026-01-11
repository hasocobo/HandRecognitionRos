#!/usr/bin/env python3
"""
Server Gateway - Main Entry Point

This server receives control messages from hand gesture clients and:
- Bridges to MQTT for ESP32 motor control
- Implements deadman safety timer
- Enforces single-controller lock

Environment Variables:
    CONTROL_TOKEN: Required authentication token
    DEADMAN_MS: Deadman timeout in milliseconds (default: 300)
    MAX_LINEAR: Maximum linear velocity m/s (default: 0.22)
    MAX_ANGULAR: Maximum angular velocity rad/s (default: 2.84)
    MQTT_HOST: MQTT broker host (default: localhost)
    MQTT_PORT: MQTT broker port (default: 1883)
    
Usage:
    export CONTROL_TOKEN=mysecrettoken
    python -m server_gateway.main
"""

import asyncio
import logging
import os
import signal
import sys
from typing import Optional

import uvicorn

from .ws_server import WebSocketServer, ControlMessage
from .mqtt_bridge import AsyncMQTTBridge

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ServerGateway:
    """
    Main server gateway integrating WebSocket and MQTT.
    
    Architecture:
        Client -> WebSocket -> ServerGateway -> MQTT (robot/cmd)
    """
    
    def __init__(
        self,
        token: str,
        host: str = "0.0.0.0",
        port: int = 8080,
        deadman_ms: int = 300,
        max_linear: float = 0.22,
        max_angular: float = 2.84,
        mqtt_host: str = "localhost",
        mqtt_port: int = 1883,
        enable_mqtt: bool = True,
    ):
        """
        Initialize server gateway.
        
        Args:
            token: Authentication token for clients
            host: Server bind address
            port: Server port
            deadman_ms: Deadman timeout in milliseconds
            max_linear: Maximum linear velocity for validation
            max_angular: Maximum angular velocity for validation
            mqtt_host: MQTT broker host
            mqtt_port: MQTT broker port
            enable_mqtt: Whether to enable MQTT bridge
        """
        self.token = token
        self.host = host
        self.port = port
        self.deadman_ms = deadman_ms
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.enable_mqtt = enable_mqtt
        
        # Components
        self.ws_server: Optional[WebSocketServer] = None
        self.mqtt_bridge: Optional[AsyncMQTTBridge] = None
        
        # State
        self._running = False
        
    async def start(self) -> None:
        """Start all server components."""
        logger.info("Starting Server Gateway...")
        
        # Start MQTT bridge
        if self.enable_mqtt:
            try:
                self.mqtt_bridge = AsyncMQTTBridge(
                    host=self.mqtt_host,
                    port=self.mqtt_port,
                    max_linear=self.max_linear,
                    max_angular=self.max_angular,
                    on_telemetry=self._on_telemetry,
                )
                success = await self.mqtt_bridge.start()
                if success:
                    logger.info("MQTT bridge started")
                else:
                    logger.warning("MQTT bridge failed to connect")
            except Exception as e:
                logger.error(f"Failed to start MQTT bridge: {e}")
                logger.warning("Continuing without MQTT bridge")
                self.mqtt_bridge = None
        
        # Create WebSocket server
        self.ws_server = WebSocketServer(
            token=self.token,
            on_message=self._on_control_message,
            on_controller_connected=self._on_controller_connected,
            on_controller_disconnected=self._on_controller_disconnected,
        )
        
        self._running = True
        logger.info(f"Server Gateway started on {self.host}:{self.port}")
        
    async def stop(self) -> None:
        """Stop all server components."""
        logger.info("Stopping Server Gateway...")
        self._running = False
        
        # Stop MQTT bridge
        if self.mqtt_bridge:
            await self.mqtt_bridge.stop()
            logger.info("MQTT bridge stopped")
            
        logger.info("Server Gateway stopped")
        
    async def _on_control_message(self, msg: ControlMessage) -> None:
        """Handle incoming control message from client."""
        logger.debug(
            "Received: linear=%.3f, angular=%.3f, enable=%s, gripper=%s",
            msg.linear,
            msg.angular,
            msg.enable,
            msg.gripper,
        )
        
        # Validate bounds
        linear = max(-self.max_linear, min(self.max_linear, msg.linear))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular))
        
        # Publish to MQTT
        if self.mqtt_bridge and self.mqtt_bridge.connected:
            await self.mqtt_bridge.publish_from_twist(
                linear,
                angular,
                msg.enable,
                gripper=msg.gripper,
            )
            
    async def _on_controller_connected(self, client_id: str) -> None:
        """Handle controller connection."""
        logger.info(f"Controller connected: {client_id}")
        
    async def _on_controller_disconnected(self, client_id: str) -> None:
        """Handle controller disconnection - force stop."""
        logger.info(f"Controller disconnected: {client_id}")
        
        # Force stop on MQTT bridge
        if self.mqtt_bridge and self.mqtt_bridge.connected:
            await self.mqtt_bridge.publish_from_twist(0.0, 0.0, False)
            
    def _on_telemetry(self, data: dict) -> None:
        """Handle telemetry from ESP32."""
        logger.debug(f"Telemetry: {data}")
        # Hook for optional telemetry handling
        
    def get_app(self):
        """Get the FastAPI application for uvicorn."""
        return self.ws_server.app
        
    def get_stats(self) -> dict:
        """Get server statistics."""
        stats = {
            "ws_server": self.ws_server.get_stats() if self.ws_server else {},
            "mqtt_bridge": self.mqtt_bridge.get_stats() if self.mqtt_bridge else {},
        }
        return stats


async def run_server(gateway: ServerGateway) -> None:
    """Run the server with uvicorn."""
    config = uvicorn.Config(
        gateway.get_app(),
        host=gateway.host,
        port=gateway.port,
        log_level="info",
        access_log=True,
    )
    server = uvicorn.Server(config)
    await server.serve()


async def main_async() -> None:
    """Async main entry point."""
    # Load configuration from environment
    token = os.environ.get("CONTROL_TOKEN")
    if not token:
        logger.error("CONTROL_TOKEN environment variable is required")
        sys.exit(1)
        
    deadman_ms = int(os.environ.get("DEADMAN_MS", "300"))
    max_linear = float(os.environ.get("MAX_LINEAR", "0.22"))
    max_angular = float(os.environ.get("MAX_ANGULAR", "2.84"))
    mqtt_host = os.environ.get("MQTT_HOST", "localhost")
    mqtt_port = int(os.environ.get("MQTT_PORT", "1883"))
    
    # Check if MQTT is available
    enable_mqtt = True
    try:
        import paho.mqtt.client
    except ImportError:
        logger.warning("paho-mqtt not available, running without MQTT bridge")
        enable_mqtt = False
    
    # Create gateway
    gateway = ServerGateway(
        token=token,
        deadman_ms=deadman_ms,
        max_linear=max_linear,
        max_angular=max_angular,
        mqtt_host=mqtt_host,
        mqtt_port=mqtt_port,
        enable_mqtt=enable_mqtt,
    )
    
    # Setup signal handlers
    loop = asyncio.get_event_loop()
    
    shutdown_event = asyncio.Event()
    
    def signal_handler():
        logger.info("Shutdown signal received")
        shutdown_event.set()
        
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler)
    
    try:
        await gateway.start()
        
        # Run server until shutdown
        server_task = asyncio.create_task(run_server(gateway))
        shutdown_task = asyncio.create_task(shutdown_event.wait())
        
        done, pending = await asyncio.wait(
            [server_task, shutdown_task],
            return_when=asyncio.FIRST_COMPLETED,
        )
        
        # Cancel pending tasks
        for task in pending:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
                
    except Exception as e:
        logger.error(f"Server error: {e}")
    finally:
        await gateway.stop()


def main() -> None:
    """Main entry point."""
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        sys.exit(0)


if __name__ == "__main__":
    main()

