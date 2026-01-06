"""
WebSocket Server for control message reception.

Handles:
- FastAPI WebSocket endpoint at /control
- Bearer token authentication
- Single-controller lock (first client to send enable=true owns control)
- Message parsing and forwarding to ROS bridge
"""

import asyncio
import json
import logging
import os
import time
from typing import Optional, Callable, Awaitable, Dict, Any
from dataclasses import dataclass

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware

logger = logging.getLogger(__name__)


@dataclass
class ControllerState:
    """State of the active controller."""
    client_id: str
    connected_at: float
    last_message_at: float
    message_count: int = 0


@dataclass
class ControlMessage:
    """Parsed control message from client."""
    linear: float
    angular: float
    enable: bool
    ts_ms: int
    
    @classmethod
    def from_json(cls, data: str) -> 'ControlMessage':
        """Parse from JSON string."""
        d = json.loads(data)
        return cls(
            linear=float(d['linear']),
            angular=float(d['angular']),
            enable=bool(d['enable']),
            ts_ms=int(d['ts_ms']),
        )


class WebSocketServer:
    """
    WebSocket server for receiving control messages.
    
    Features:
    - Bearer token authentication
    - Single-controller lock
    - Message forwarding callback
    """
    
    def __init__(
        self,
        token: str,
        on_message: Optional[Callable[[ControlMessage], Awaitable[None]]] = None,
        on_controller_connected: Optional[Callable[[str], Awaitable[None]]] = None,
        on_controller_disconnected: Optional[Callable[[str], Awaitable[None]]] = None,
    ):
        """
        Initialize WebSocket server.
        
        Args:
            token: Required bearer token for authentication
            on_message: Callback for received control messages
            on_controller_connected: Callback when a controller connects
            on_controller_disconnected: Callback when controller disconnects
        """
        self.token = token
        self.on_message = on_message
        self.on_controller_connected = on_controller_connected
        self.on_controller_disconnected = on_controller_disconnected
        
        # Controller state
        self._active_controller: Optional[ControllerState] = None
        self._controller_lock = asyncio.Lock()
        
        # Connected clients (for monitoring)
        self._connected_clients: Dict[str, WebSocket] = {}
        self._client_counter = 0
        
        # Statistics
        self._total_messages = 0
        self._invalid_messages = 0
        
        # FastAPI app
        self.app = FastAPI(title="Hand Gesture Control Gateway")
        
        # Add CORS middleware
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        # Register routes
        self._setup_routes()
        
    def _setup_routes(self):
        """Set up FastAPI routes."""
        
        @self.app.get("/health")
        async def health_check():
            """Health check endpoint."""
            return {
                "status": "ok",
                "active_controller": self._active_controller.client_id if self._active_controller else None,
                "connected_clients": len(self._connected_clients),
                "total_messages": self._total_messages,
            }
            
        @self.app.websocket("/control")
        async def websocket_control(websocket: WebSocket):
            """WebSocket endpoint for control messages."""
            await self._handle_websocket(websocket)
            
    async def _handle_websocket(self, websocket: WebSocket) -> None:
        """Handle incoming WebSocket connection."""
        # Authenticate
        auth_header = websocket.headers.get("authorization", "")
        if not self._verify_token(auth_header):
            logger.warning(f"Authentication failed from {websocket.client}")
            await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
            return
            
        # Accept connection
        await websocket.accept()
        
        # Generate client ID
        self._client_counter += 1
        client_id = f"client_{self._client_counter}"
        self._connected_clients[client_id] = websocket
        
        logger.info(f"Client connected: {client_id} from {websocket.client}")
        
        try:
            await self._receive_messages(websocket, client_id)
        except WebSocketDisconnect:
            logger.info(f"Client disconnected: {client_id}")
        except Exception as e:
            logger.error(f"Error handling client {client_id}: {e}")
        finally:
            # Clean up
            self._connected_clients.pop(client_id, None)
            
            # Release controller lock if this client held it
            async with self._controller_lock:
                if self._active_controller and self._active_controller.client_id == client_id:
                    logger.info(f"Controller {client_id} disconnected, releasing lock")
                    self._active_controller = None
                    if self.on_controller_disconnected:
                        await self.on_controller_disconnected(client_id)
                        
    def _verify_token(self, auth_header: str) -> bool:
        """Verify Bearer token."""
        if not auth_header:
            return False
            
        parts = auth_header.split()
        if len(parts) != 2 or parts[0].lower() != "bearer":
            return False
            
        return parts[1] == self.token
        
    async def _receive_messages(self, websocket: WebSocket, client_id: str) -> None:
        """Receive and process messages from a client."""
        while True:
            try:
                data = await websocket.receive_text()
            except WebSocketDisconnect:
                raise
                
            self._total_messages += 1
            
            # Parse message
            try:
                msg = ControlMessage.from_json(data)
            except (json.JSONDecodeError, KeyError, ValueError) as e:
                self._invalid_messages += 1
                logger.warning(f"Invalid message from {client_id}: {e}")
                continue
                
            # Handle controller lock
            async with self._controller_lock:
                if msg.enable:
                    # Client wants to take control
                    if self._active_controller is None:
                        # No active controller, grant control
                        self._active_controller = ControllerState(
                            client_id=client_id,
                            connected_at=time.time(),
                            last_message_at=time.time(),
                        )
                        logger.info(f"Controller lock granted to {client_id}")
                        if self.on_controller_connected:
                            await self.on_controller_connected(client_id)
                    elif self._active_controller.client_id != client_id:
                        # Another client has control, ignore
                        logger.debug(f"Ignoring enable from {client_id}, controller is {self._active_controller.client_id}")
                        continue
                else:
                    # Client is releasing control or sending disabled state
                    if self._active_controller and self._active_controller.client_id == client_id:
                        # This client had control, release it
                        logger.info(f"Controller {client_id} released control")
                        self._active_controller = None
                        if self.on_controller_disconnected:
                            await self.on_controller_disconnected(client_id)
                            
                # Update last message time if this client has control
                if self._active_controller and self._active_controller.client_id == client_id:
                    self._active_controller.last_message_at = time.time()
                    self._active_controller.message_count += 1
                    
            # Forward message to callback
            # Only forward if client has control OR is sending a stop (enable=false)
            should_forward = (
                (self._active_controller and self._active_controller.client_id == client_id) or
                not msg.enable
            )
            
            if should_forward and self.on_message:
                try:
                    await self.on_message(msg)
                except Exception as e:
                    logger.error(f"Error in message callback: {e}")
                    
    def get_active_controller(self) -> Optional[str]:
        """Get the ID of the active controller."""
        return self._active_controller.client_id if self._active_controller else None
        
    def get_last_message_time(self) -> Optional[float]:
        """Get the time of the last message from active controller."""
        return self._active_controller.last_message_at if self._active_controller else None
        
    def get_stats(self) -> dict:
        """Get server statistics."""
        return {
            "connected_clients": len(self._connected_clients),
            "active_controller": self._active_controller.client_id if self._active_controller else None,
            "total_messages": self._total_messages,
            "invalid_messages": self._invalid_messages,
        }


def create_app(
    token: str,
    on_message: Optional[Callable[[ControlMessage], Awaitable[None]]] = None,
) -> FastAPI:
    """
    Create FastAPI application with WebSocket server.
    
    Args:
        token: Authentication token
        on_message: Callback for received messages
        
    Returns:
        Configured FastAPI application
    """
    server = WebSocketServer(token=token, on_message=on_message)
    return server.app, server

