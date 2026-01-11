"""
Message Schema and Validation for control messages.

Defines the JSON message format for client-server communication and
validates all outgoing messages before transmission.
"""

import math
import time
import json
import logging
from dataclasses import dataclass, asdict
from typing import Tuple, Optional

logger = logging.getLogger(__name__)


@dataclass
class ControlMessage:
    """
    Control message sent from client to server.
    
    Attributes:
        linear: Linear velocity in m/s (forward positive)
        angular: Angular velocity in rad/s (counter-clockwise positive)
        enable: Whether drive is enabled
        gripper: Optional gripper command (0=open, 1=closed)
        ts_ms: Timestamp in milliseconds (monotonic)
    """
    linear: float
    angular: float
    enable: bool
    ts_ms: int
    gripper: Optional[int] = None
    
    def to_json(self) -> str:
        """Serialize to JSON string."""
        payload = asdict(self)
        if payload.get("gripper") is None:
            payload.pop("gripper", None)
        return json.dumps(payload)
    
    @classmethod
    def from_json(cls, data: str) -> 'ControlMessage':
        """Deserialize from JSON string."""
        d = json.loads(data)
        gripper = d.get("gripper", None)
        if gripper is not None:
            gripper = int(gripper)
        return cls(
            linear=float(d['linear']),
            angular=float(d['angular']),
            enable=bool(d['enable']),
            gripper=gripper,
            ts_ms=int(d['ts_ms']),
        )
    
    @classmethod
    def stop_message(cls) -> 'ControlMessage':
        """Create a stop message (zero velocity, disabled)."""
        return cls(
            linear=0.0,
            angular=0.0,
            enable=False,
            gripper=None,
            ts_ms=int(time.monotonic() * 1000),
        )


class MessageValidator:
    """
    Validates outgoing control messages before transmission.
    
    Ensures:
    - linear and angular are finite (not NaN/Inf)
    - Values are within configured bounds
    - Timestamps are monotonic (non-decreasing)
    """
    
    def __init__(
        self,
        max_linear: float = 0.22,
        max_angular: float = 2.84,
    ):
        """
        Initialize validator.
        
        Args:
            max_linear: Maximum allowed linear velocity (m/s)
            max_angular: Maximum allowed angular velocity (rad/s)
        """
        self.max_linear = max_linear
        self.max_angular = max_angular
        self._last_ts: int = 0
        self._dropped_count: int = 0
        self._validated_count: int = 0
        
    def validate(self, msg: ControlMessage) -> Tuple[bool, str]:
        """
        Validate a control message.
        
        Args:
            msg: The message to validate
            
        Returns:
            Tuple of (is_valid, reason_string)
        """
        # Check 1: Linear velocity is finite
        if not math.isfinite(msg.linear):
            self._dropped_count += 1
            logger.warning(f"Invalid message: linear={msg.linear} is not finite")
            return False, "linear_not_finite"
        
        # Check 2: Angular velocity is finite
        if not math.isfinite(msg.angular):
            self._dropped_count += 1
            logger.warning(f"Invalid message: angular={msg.angular} is not finite")
            return False, "angular_not_finite"
        
        # Check 3: Linear velocity within bounds
        if abs(msg.linear) > self.max_linear:
            self._dropped_count += 1
            logger.warning(
                f"Invalid message: linear={msg.linear} exceeds max={self.max_linear}"
            )
            return False, "linear_out_of_bounds"
        
        # Check 4: Angular velocity within bounds
        if abs(msg.angular) > self.max_angular:
            self._dropped_count += 1
            logger.warning(
                f"Invalid message: angular={msg.angular} exceeds max={self.max_angular}"
            )
            return False, "angular_out_of_bounds"
        
        # Check 5: Timestamp is monotonic (allow equal for same-tick messages)
        if msg.ts_ms < self._last_ts:
            self._dropped_count += 1
            logger.warning(
                f"Invalid message: timestamp {msg.ts_ms} < previous {self._last_ts}"
            )
            return False, "timestamp_regression"
        
        # Check 6: enable is actually a boolean (type safety)
        if not isinstance(msg.enable, bool):
            self._dropped_count += 1
            logger.warning(f"Invalid message: enable={msg.enable} is not boolean")
            return False, "enable_not_boolean"

        # Check 7: gripper command (if provided) is 0 or 1
        if msg.gripper is not None and msg.gripper not in (0, 1):
            self._dropped_count += 1
            logger.warning(f"Invalid message: gripper={msg.gripper} is not 0/1")
            return False, "gripper_not_valid"
        
        # All checks passed
        self._last_ts = msg.ts_ms
        self._validated_count += 1
        return True, "ok"
    
    def clamp_and_validate(self, msg: ControlMessage) -> Tuple[ControlMessage, bool, str]:
        """
        Clamp values to bounds and then validate.
        
        This is a convenience method that first clamps the velocity values
        to the configured bounds, then validates the result.
        
        Args:
            msg: The message to clamp and validate
            
        Returns:
            Tuple of (clamped_message, is_valid, reason)
        """
        # Clamp values
        gripper = msg.gripper
        if gripper is not None:
            gripper = 1 if int(gripper) != 0 else 0

        clamped = ControlMessage(
            linear=max(-self.max_linear, min(self.max_linear, msg.linear)),
            angular=max(-self.max_angular, min(self.max_angular, msg.angular)),
            enable=msg.enable,
            gripper=gripper,
            ts_ms=msg.ts_ms,
        )
        
        # Handle NaN/Inf by setting to zero
        if not math.isfinite(clamped.linear):
            clamped.linear = 0.0
        if not math.isfinite(clamped.angular):
            clamped.angular = 0.0
            
        valid, reason = self.validate(clamped)
        return clamped, valid, reason
    
    def get_stats(self) -> dict:
        """Get validation statistics."""
        total = self._validated_count + self._dropped_count
        return {
            "total_messages": total,
            "validated": self._validated_count,
            "dropped": self._dropped_count,
            "drop_rate": self._dropped_count / total if total > 0 else 0.0,
        }
    
    def reset_stats(self) -> None:
        """Reset statistics counters."""
        self._dropped_count = 0
        self._validated_count = 0


def create_control_message(
    linear: float,
    angular: float,
    enable: bool,
    gripper: Optional[int] = None,
) -> ControlMessage:
    """
    Create a control message with current timestamp.
    
    Args:
        linear: Linear velocity in m/s
        angular: Angular velocity in rad/s
        enable: Whether drive is enabled
        gripper: Optional gripper command (0=open, 1=closed)
        
    Returns:
        ControlMessage instance
    """
    return ControlMessage(
        linear=linear,
        angular=angular,
        enable=enable,
        gripper=gripper,
        ts_ms=int(time.monotonic() * 1000),
    )

