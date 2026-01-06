"""
ROS2 Bridge for publishing control commands.

Handles:
- Publishing to /cmd_vel (geometry_msgs/Twist)
- Publishing to /drive_enabled (std_msgs/Bool)
- Deadman timer (force stop if no message received)
"""

import asyncio
import logging
import time
from typing import Optional, Callable
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

logger = logging.getLogger(__name__)


class ROSBridge(Node):
    """
    ROS2 node for publishing control commands.
    
    Features:
    - Publishes Twist messages to /cmd_vel
    - Publishes Bool messages to /drive_enabled
    - Deadman timer: publishes stop if no command for deadman_ms
    - Only publishes on state change or deadman trigger (no spam)
    """
    
    def __init__(
        self,
        deadman_ms: int = 300,
        node_name: str = "hand_control_gateway",
    ):
        """
        Initialize ROS bridge.
        
        Args:
            deadman_ms: Milliseconds without message before triggering deadman stop
            node_name: Name for the ROS node
        """
        super().__init__(node_name)
        
        self.deadman_ms = deadman_ms
        self._last_command_time: Optional[float] = None
        self._last_published_linear: float = 0.0
        self._last_published_angular: float = 0.0
        self._last_published_enable: bool = False
        self._deadman_triggered: bool = False
        
        # QoS profile for reliable delivery
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self.drive_enabled_pub = self.create_publisher(Bool, "/drive_enabled", qos)
        
        # Deadman timer (runs at 10 Hz)
        self._deadman_timer = self.create_timer(0.1, self._deadman_check)
        
        # Lock for thread safety
        self._lock = threading.Lock()
        
        logger.info(f"ROS bridge initialized with deadman timeout: {deadman_ms}ms")
        
    def publish_command(
        self, 
        linear: float, 
        angular: float, 
        enable: bool,
    ) -> None:
        """
        Publish a control command.
        
        Only publishes if values have changed or deadman was triggered.
        
        Args:
            linear: Linear velocity (m/s)
            angular: Angular velocity (rad/s)
            enable: Whether drive is enabled
        """
        with self._lock:
            now = time.time()
            self._last_command_time = now
            
            # Check if values changed
            changed = (
                abs(linear - self._last_published_linear) > 0.001 or
                abs(angular - self._last_published_angular) > 0.001 or
                enable != self._last_published_enable or
                self._deadman_triggered
            )
            
            if not changed:
                return
                
            # Reset deadman state
            self._deadman_triggered = False
            
            # Update last published values
            self._last_published_linear = linear
            self._last_published_angular = angular
            self._last_published_enable = enable
            
        # Publish Twist message
        twist = Twist()
        twist.linear.x = linear if enable else 0.0
        twist.angular.z = angular if enable else 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Publish enable state
        enabled_msg = Bool()
        enabled_msg.data = enable
        self.drive_enabled_pub.publish(enabled_msg)
        
        logger.debug(f"Published: linear={linear:.3f}, angular={angular:.3f}, enable={enable}")
        
    def _deadman_check(self) -> None:
        """Check deadman timer and publish stop if triggered."""
        with self._lock:
            if self._last_command_time is None:
                return
                
            elapsed_ms = (time.time() - self._last_command_time) * 1000
            
            if elapsed_ms >= self.deadman_ms and not self._deadman_triggered:
                logger.warning(f"Deadman triggered: {elapsed_ms:.0f}ms since last command")
                self._deadman_triggered = True
                
                # Update state
                self._last_published_linear = 0.0
                self._last_published_angular = 0.0
                self._last_published_enable = False
                
        # Publish stop outside lock
        if self._deadman_triggered:
            self._publish_stop()
            
    def _publish_stop(self) -> None:
        """Publish stop command."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        enabled_msg = Bool()
        enabled_msg.data = False
        self.drive_enabled_pub.publish(enabled_msg)
        
        logger.info("Published STOP (deadman)")
        
    def force_stop(self) -> None:
        """Force an immediate stop."""
        with self._lock:
            self._last_published_linear = 0.0
            self._last_published_angular = 0.0
            self._last_published_enable = False
            self._deadman_triggered = True
            
        self._publish_stop()
        logger.info("Force STOP published")
        
    def get_stats(self) -> dict:
        """Get bridge statistics."""
        with self._lock:
            elapsed_ms = 0.0
            if self._last_command_time:
                elapsed_ms = (time.time() - self._last_command_time) * 1000
                
            return {
                "last_linear": self._last_published_linear,
                "last_angular": self._last_published_angular,
                "last_enable": self._last_published_enable,
                "ms_since_command": elapsed_ms,
                "deadman_triggered": self._deadman_triggered,
            }


class ROSBridgeManager:
    """
    Manager for running ROS bridge in a separate thread.
    
    Since ROS2 spin() is blocking, we run it in a background thread
    and provide async-compatible methods for the main server.
    """
    
    def __init__(self, deadman_ms: int = 300):
        """
        Initialize ROS bridge manager.
        
        Args:
            deadman_ms: Deadman timeout in milliseconds
        """
        self.deadman_ms = deadman_ms
        self._bridge: Optional[ROSBridge] = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._running = False
        
    def start(self) -> None:
        """Start the ROS bridge in a background thread."""
        if self._running:
            return
            
        # Initialize ROS if not already done
        if not rclpy.ok():
            rclpy.init()
            
        # Create bridge node
        self._bridge = ROSBridge(deadman_ms=self.deadman_ms)
        
        # Start spin thread
        self._running = True
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        
        logger.info("ROS bridge started")
        
    def stop(self) -> None:
        """Stop the ROS bridge."""
        if not self._running:
            return
            
        self._running = False
        
        # Force a final stop
        if self._bridge:
            self._bridge.force_stop()
            self._bridge.destroy_node()
            self._bridge = None
            
        # Wait for spin thread
        if self._spin_thread:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None
            
        # Shutdown ROS
        if rclpy.ok():
            rclpy.shutdown()
            
        logger.info("ROS bridge stopped")
        
    def _spin_loop(self) -> None:
        """ROS spin loop running in background thread."""
        while self._running and rclpy.ok():
            rclpy.spin_once(self._bridge, timeout_sec=0.1)
            
    def publish_command(self, linear: float, angular: float, enable: bool) -> None:
        """
        Publish a control command.
        
        Thread-safe method callable from any thread.
        
        Args:
            linear: Linear velocity (m/s)
            angular: Angular velocity (rad/s)
            enable: Whether drive is enabled
        """
        if self._bridge:
            self._bridge.publish_command(linear, angular, enable)
            
    async def publish_command_async(self, linear: float, angular: float, enable: bool) -> None:
        """
        Async wrapper for publish_command.
        
        Runs the publish in the default executor to avoid blocking.
        """
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self.publish_command, linear, angular, enable)
        
    def force_stop(self) -> None:
        """Force an immediate stop."""
        if self._bridge:
            self._bridge.force_stop()
            
    async def force_stop_async(self) -> None:
        """Async wrapper for force_stop."""
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self.force_stop)
        
    def get_stats(self) -> dict:
        """Get bridge statistics."""
        if self._bridge:
            return self._bridge.get_stats()
        return {}

