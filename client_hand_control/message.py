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
    """Minimal control message: just run flag. One boolean."""
    run: bool

    def to_json(self) -> str:
        """Serialize to JSON string."""
        return json.dumps({"run": self.run})

    @classmethod
    def from_json(cls, data: str) -> 'ControlMessage':
        """Deserialize from JSON string."""
        d = json.loads(data)
        return cls(run=bool(d.get("run", False)))

    @classmethod
    def stop_message(cls) -> 'ControlMessage':
        """Create a stop message."""
        return cls(run=False)


class MessageValidator:
    """Validates outgoing control messages. Minimal: just ensure run is boolean."""

    def __init__(self):
        pass

    def validate(self, msg: ControlMessage) -> Tuple[bool, str]:
        """Check that run is a boolean."""
        if not isinstance(msg.run, bool):
            logger.warning(f"Invalid message: run={msg.run} is not boolean")
            return False, "run_not_boolean"
        return True, "ok"

    def clamp_and_validate(self, msg: ControlMessage) -> Tuple[ControlMessage, bool, str]:
        """Validate the message."""
        valid, reason = self.validate(msg)
        return msg, valid, reason

    def get_stats(self) -> dict:
        """Dummy stats."""
        return {"total_messages": 0, "validated": 0, "dropped": 0, "drop_rate": 0.0}


def create_control_message(run: bool) -> ControlMessage:
    """Create a control message with just the run flag."""
    return ControlMessage(run=run)

