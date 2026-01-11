"""
WebSocket Client for server communication.

Handles:
- Async WebSocket connection with Bearer token auth
- Exponential backoff reconnection
- Direct message sending (no queue bottleneck)
- Clean shutdown with final STOP message
"""

import asyncio
import logging
import time
from typing import Optional, Callable, Awaitable
from dataclasses import dataclass

import websockets
from websockets.client import WebSocketClientProtocol
from websockets.exceptions import (
    ConnectionClosed,
    InvalidStatusCode,
    WebSocketException,
)

from .message import ControlMessage

logger = logging.getLogger(__name__)


@dataclass
class ConnectionStats:
    """Statistics about WebSocket connection."""
    connected: bool = False
    connect_time: Optional[float] = None
    disconnect_time: Optional[float] = None
    reconnect_attempts: int = 0
    messages_sent: int = 0
    messages_failed: int = 0
    last_send_time: Optional[float] = None


class WebSocketClient:
    """
    Async WebSocket client with automatic reconnection.
    
    Features:
    - Bearer token authentication
    - Exponential backoff on connection failure (1s -> 30s max)
    - Direct send (no queue to avoid bottleneck)
    - Graceful shutdown with final STOP message
    """
    
    def __init__(
        self,
        server_url: str,
        token: str,
        max_backoff_seconds: float = 30.0,
        initial_backoff_seconds: float = 1.0,
        on_connected: Optional[Callable[[], Awaitable[None]]] = None,
        on_disconnected: Optional[Callable[[], Awaitable[None]]] = None,
    ):
        """
        Initialize WebSocket client.
        
        Args:
            server_url: WebSocket server URL (e.g., ws://127.0.0.1:8080/control)
            token: Bearer token for authentication
            max_backoff_seconds: Maximum backoff time between reconnect attempts
            initial_backoff_seconds: Initial backoff time
            on_connected: Callback when connection is established
            on_disconnected: Callback when connection is lost
        """
        self.server_url = server_url
        self.token = token
        self.max_backoff = max_backoff_seconds
        self.initial_backoff = initial_backoff_seconds
        self.on_connected = on_connected
        self.on_disconnected = on_disconnected
        
        # Connection state
        self._ws: Optional[WebSocketClientProtocol] = None
        self._connected = False
        self._running = False
        self._shutdown_requested = False
        
        # Statistics
        self.stats = ConnectionStats()
        
        # Backoff state
        self._current_backoff = initial_backoff_seconds
        
        # Lock for thread-safe sends
        self._send_lock = asyncio.Lock()
        
        # Tasks
        self._connect_task: Optional[asyncio.Task] = None
        
    @property
    def connected(self) -> bool:
        """Check if currently connected."""
        return self._connected and self._ws is not None
    
    async def start(self) -> None:
        """Start the client and connection tasks."""
        if self._running:
            return
            
        self._running = True
        self._shutdown_requested = False
        
        # Start connection manager task
        self._connect_task = asyncio.create_task(self._connection_loop())
        
        logger.info(f"WebSocket client started, connecting to {self.server_url}")
        
    async def stop(self) -> None:
        """Stop the client and send final STOP message."""
        if not self._running:
            return
            
        logger.info("WebSocket client stopping...")
        self._shutdown_requested = True
        self._running = False
        
        # Send final STOP message if connected
        if self.connected:
            try:
                stop_msg = ControlMessage.stop_message()
                await self.send_async(stop_msg)
                logger.info("Sent final STOP message")
            except Exception as e:
                logger.warning(f"Failed to send final STOP: {e}")
        
        # Cancel tasks
        if self._connect_task:
            self._connect_task.cancel()
            try:
                await self._connect_task
            except asyncio.CancelledError:
                pass
        
        # Close WebSocket
        if self._ws:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None
            
        self._connected = False
        logger.info("WebSocket client stopped")
        
    def send(self, message: ControlMessage) -> bool:
        """
        Queue a message for sending (sync interface).
        
        For the async main loop, this schedules the send.
        
        Args:
            message: ControlMessage to send
            
        Returns:
            True if send was initiated, False if not connected
        """
        if not self.connected:
            self.stats.messages_failed += 1
            return False
            
        # Schedule async send
        asyncio.create_task(self._do_send(message.to_json()))
        return True
    
    async def send_async(self, message: ControlMessage) -> bool:
        """
        Send a message asynchronously.
        
        Args:
            message: ControlMessage to send
            
        Returns:
            True if sent successfully
        """
        if not self.connected:
            self.stats.messages_failed += 1
            return False
            
        return await self._do_send(message.to_json())
            
    async def _do_send(self, message: str) -> bool:
        """Actually send the message."""
        if not self._ws or not self._connected:
            self.stats.messages_failed += 1
            return False
            
        try:
            async with self._send_lock:
                await self._ws.send(message)
            self.stats.messages_sent += 1
            self.stats.last_send_time = time.time()
            return True
        except (ConnectionClosed, WebSocketException) as e:
            self.stats.messages_failed += 1
            logger.debug(f"Send failed: {e}")
            return False
        except Exception as e:
            self.stats.messages_failed += 1
            logger.warning(f"Send error: {e}")
            return False
            
    async def _connection_loop(self) -> None:
        """Main connection loop with automatic reconnection."""
        while self._running and not self._shutdown_requested:
            try:
                await self._connect()
                
                # Reset backoff on successful connection
                self._current_backoff = self.initial_backoff
                    
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Connection error: {e}")
                
            if not self._running or self._shutdown_requested:
                break
                
            # Exponential backoff before reconnect
            logger.info(f"Reconnecting in {self._current_backoff:.1f}s...")
            await asyncio.sleep(self._current_backoff)
            
            self._current_backoff = min(
                self._current_backoff * 2, 
                self.max_backoff
            )
            self.stats.reconnect_attempts += 1
            
    async def _connect(self) -> None:
        """Establish WebSocket connection with authentication."""
        headers = {
            "Authorization": f"Bearer {self.token}"
        }
        
        try:
            logger.info(f"Connecting to {self.server_url}...")
            
            self._ws = await websockets.connect(
                self.server_url,
                additional_headers=headers,
                ping_interval=20,
                ping_timeout=10,
                close_timeout=5,
            )
            
            self._connected = True
            self.stats.connected = True
            self.stats.connect_time = time.time()
            
            logger.info("WebSocket connected successfully")
            
            if self.on_connected:
                await self.on_connected()
                
            # Keep connection alive - just wait for close, don't block on recv
            # Use a ping-pong approach instead of blocking recv
            while self._connected and self._running and not self._shutdown_requested:
                try:
                    # Check if connection is still alive with short timeout
                    await asyncio.wait_for(
                        self._ws.recv(),
                        timeout=0.1
                    )
                except asyncio.TimeoutError:
                    # No message received, that's fine - connection still alive
                    continue
                except ConnectionClosed:
                    logger.info("Connection closed by server")
                    break
                    
        except InvalidStatusCode as e:
            logger.error(f"Authentication failed: {e.status_code}")
            raise
        except ConnectionRefusedError:
            logger.error("Connection refused - is the server running?")
            raise
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            raise
        finally:
            was_connected = self._connected
            self._connected = False
            self.stats.connected = False
            self.stats.disconnect_time = time.time()
            
            if was_connected and self.on_disconnected:
                await self.on_disconnected()
            
    def get_stats(self) -> dict:
        """Get connection statistics."""
        return {
            "connected": self.connected,
            "connect_time": self.stats.connect_time,
            "disconnect_time": self.stats.disconnect_time,
            "reconnect_attempts": self.stats.reconnect_attempts,
            "messages_sent": self.stats.messages_sent,
            "messages_failed": self.stats.messages_failed,
            "last_send_time": self.stats.last_send_time,
        }
