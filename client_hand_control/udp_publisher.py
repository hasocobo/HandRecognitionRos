"""
UDP publisher — the client fires control-message JSON straight at the rover.

Architecture: client → UDP datagram → rover (purePursuit.py listens on its port).
No broker, no connection, no background thread. UDP is connectionless
fire-and-forget, which is exactly what we want here: the message format already
carries everything as idempotent state (mode/run/estop are levels, lane changes
are deduped by lane_seq), so a dropped packet is harmless — the next frame
resends the same state. This is also why there's no 1s reconnect stutter the
way paho had: there's nothing to (re)connect.
"""

import logging
import socket

logger = logging.getLogger(__name__)


class UdpPublisher:
    """Thin UDP sender: open a socket, sendto JSON strings. Same interface as the
    old MqttPublisher so main.py doesn't care which transport it's holding."""

    def __init__(self, host: str, port: int = 5001):
        self.host = host
        self.port = port
        self._sock = None

    def start(self) -> None:
        # SOCK_DGRAM = UDP. No connect() — we sendto() each packet, so a rover
        # that isn't up yet just means the datagrams hit the floor. No error,
        # no blocking, no retry loop.
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logger.info("UDP publisher started -> %s:%d", self.host, self.port)

    def stop(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    @property
    def connected(self) -> bool:
        # UDP has no connection; "ready to send" is the closest honest answer.
        return self._sock is not None

    def publish(self, payload: str) -> bool:
        """Send a JSON string as one datagram. Returns True if handed to the OS."""
        if self._sock is None:
            return False
        try:
            self._sock.sendto(payload.encode("utf-8"), (self.host, self.port))
            return True
        except OSError as e:
            logger.error("UDP send failed: %s", e)
            return False
