"""
WebSocket Client for server communication.

Handles:
- Async WebSocket connection with Bearer token auth
- Exponential backoff reconnection
- Message queue for decoupled sending
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
    ConnectionClosedError,
    ConnectionClosedOK,
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
    - Non-blocking message sending via queue
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
        
        # Message queue
        self._send_queue: asyncio.Queue[Optional[str]] = asyncio.Queue(maxsize=100)
        
        # Statistics
        self.stats = ConnectionStats()
        
        # Backoff state
        self._current_backoff = initial_backoff_seconds
        
        # Tasks
        self._connect_task: Optional[asyncio.Task] = None
        self._send_task: Optional[asyncio.Task] = None
        
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
        
        # Start connection manager and sender tasks
        self._connect_task = asyncio.create_task(self._connection_loop())
        self._send_task = asyncio.create_task(self._send_loop())
        
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
                await self._send_immediate(stop_msg.to_json())
                logger.info("Sent final STOP message")
            except Exception as e:
                logger.warning(f"Failed to send final STOP: {e}")
        
        # Signal send loop to exit
        await self._send_queue.put(None)
        
        # Cancel tasks
        if self._connect_task:
            self._connect_task.cancel()
            try:
                await self._connect_task
            except asyncio.CancelledError:
                pass
                
        if self._send_task:
            self._send_task.cancel()
            try:
                await self._send_task
            except asyncio.CancelledError:
                pass
        
        # Close WebSocket
        if self._ws:
            await self._ws.close()
            self._ws = None
            
        self._connected = False
        logger.info("WebSocket client stopped")
        
    def send(self, message: ControlMessage) -> bool:
        """
        Queue a message for sending.
        
        Non-blocking. Returns False if queue is full.
        
        Args:
            message: ControlMessage to send
            
        Returns:
            True if queued successfully, False if queue is full
        """
        try:
            self._send_queue.put_nowait(message.to_json())
            return True
        except asyncio.QueueFull:
            self.stats.messages_failed += 1
            logger.warning("Send queue full, dropping message")
            return False
            
    async def _connection_loop(self) -> None:
        """Main connection loop with automatic reconnection."""
        while self._running and not self._shutdown_requested:
            try:
                await self._connect()
                
                # Reset backoff on successful connection
                self._current_backoff = self.initial_backoff
                
                # Wait for disconnection
                while self._connected and self._running:
                    await asyncio.sleep(0.1)
                    
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
                
            # Keep connection alive by listening for messages
            try:
                async for message in self._ws:
                    # Server might send acknowledgements or status updates
                    logger.debug(f"Received from server: {message}")
            except ConnectionClosed:
                pass
                
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
            self._connected = False
            self.stats.connected = False
            self.stats.disconnect_time = time.time()
            
            if self.on_disconnected:
                await self.on_disconnected()
                
    async def _send_loop(self) -> None:
        """Process outgoing message queue."""
        while self._running:
            try:
                # Wait for message with timeout
                try:
                    message = await asyncio.wait_for(
                        self._send_queue.get(),
                        timeout=1.0
                    )
                except asyncio.TimeoutError:
                    continue
                    
                # None is shutdown signal
                if message is None:
                    break
                    
                # Only send if connected
                if self.connected:
                    try:
                        await self._ws.send(message)
                        self.stats.messages_sent += 1
                        self.stats.last_send_time = time.time()
                    except (ConnectionClosed, WebSocketException) as e:
                        self.stats.messages_failed += 1
                        logger.warning(f"Send failed: {e}")
                else:
                    # Not connected, drop message
                    self.stats.messages_failed += 1
                    
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Send loop error: {e}")
                
    async def _send_immediate(self, message: str) -> None:
        """Send a message immediately, bypassing queue."""
        if self._ws and self._connected:
            await self._ws.send(message)
            self.stats.messages_sent += 1
            self.stats.last_send_time = time.time()
            
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
            "queue_size": self._send_queue.qsize(),
        }


class SyncWebSocketClient:
    """
    Synchronous wrapper around WebSocketClient for use in non-async code.
    
    Runs the async client in a background thread.
    """
    
    def __init__(
        self,
        server_url: str,
        token: str,
        max_backoff_seconds: float = 30.0,
    ):
        """
        Initialize sync client wrapper.
        
        Args:
            server_url: WebSocket server URL
            token: Bearer token for authentication
            max_backoff_seconds: Maximum backoff for reconnection
        """
        self.server_url = server_url
        self.token = token
        self.max_backoff = max_backoff_seconds
        
        self._client: Optional[WebSocketClient] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[asyncio.Task] = None
        self._started = False
        
    def start(self) -> None:
        """Start the client in a background event loop."""
        if self._started:
            return
            
        # Create new event loop for background thread
        self._loop = asyncio.new_event_loop()
        
        self._client = WebSocketClient(
            server_url=self.server_url,
            token=self.token,
            max_backoff_seconds=self.max_backoff,
        )
        
        # Run event loop in background
        import threading
        self._bg_thread = threading.Thread(
            target=self._run_loop,
            daemon=True
        )
        self._bg_thread.start()
        self._started = True
        
        # Wait a moment for connection
        import time
        time.sleep(0.5)
        
    def _run_loop(self) -> None:
        """Run the async event loop."""
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._client.start())
        self._loop.run_forever()
        
    def stop(self) -> None:
        """Stop the client."""
        if not self._started:
            return
            
        # Schedule stop in the event loop
        if self._loop and self._client:
            future = asyncio.run_coroutine_threadsafe(
                self._client.stop(),
                self._loop
            )
            future.result(timeout=5.0)
            
            self._loop.call_soon_threadsafe(self._loop.stop)
            
        self._started = False
        
    def send(self, message: ControlMessage) -> bool:
        """
        Queue a message for sending.
        
        Args:
            message: ControlMessage to send
            
        Returns:
            True if queued successfully
        """
        if self._client:
            return self._client.send(message)
        return False
        
    @property
    def connected(self) -> bool:
        """Check if connected."""
        return self._client.connected if self._client else False
        
    def get_stats(self) -> dict:
        """Get connection statistics."""
        if self._client:
            return self._client.get_stats()
        return {}

