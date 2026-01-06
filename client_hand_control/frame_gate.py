"""
Frame Quality Gate - Validates frames before processing.

This is the critical safety fix: do NOT compute or transmit velocity commands
based on corrupted/broken frames from RTSP streams or failed camera reads.
"""

import time
import logging
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class FrameValidationResult:
    """Result of frame validation."""
    valid: bool
    reason: str
    frame: Optional[np.ndarray] = None


class FrameGate:
    """
    Frame quality gate for validating camera/RTSP frames.
    
    Validates:
    - cap.read() success
    - Frame not empty/None
    - Frame has correct shape (H, W, 3)
    - Shape consistency across frames (detect decode corruption)
    
    Tracks consecutive invalid frames and triggers force-stop when
    invalid frames exceed timeout threshold.
    """
    
    def __init__(
        self,
        invalid_timeout_ms: int = 300,
        allow_shape_change: bool = False,
    ):
        """
        Initialize FrameGate.
        
        Args:
            invalid_timeout_ms: Time in ms after which consecutive invalid
                frames trigger a force-stop condition.
            allow_shape_change: If True, don't treat shape changes as invalid.
                Set to False for RTSP streams where shape changes indicate
                decode corruption.
        """
        self.invalid_timeout_ms = invalid_timeout_ms
        self.allow_shape_change = allow_shape_change
        
        # Tracking state
        self._consecutive_invalid_start: Optional[float] = None
        self._last_valid_shape: Optional[Tuple[int, int, int]] = None
        self._total_invalid_count: int = 0
        self._total_valid_count: int = 0
        self._force_stop_sent: bool = False
        
    def validate(self, ok: bool, frame: Optional[np.ndarray]) -> FrameValidationResult:
        """
        Validate a frame from cap.read().
        
        Args:
            ok: The boolean return value from cap.read()
            frame: The frame array from cap.read()
            
        Returns:
            FrameValidationResult with valid flag, reason, and frame if valid.
        """
        now = time.monotonic()
        
        # Check 1: cap.read() returned False
        if not ok:
            self._mark_invalid(now)
            return FrameValidationResult(False, "read_failed")
        
        # Check 2: Frame is None
        if frame is None:
            self._mark_invalid(now)
            return FrameValidationResult(False, "frame_none")
        
        # Check 3: Frame is empty (size == 0)
        if frame.size == 0:
            self._mark_invalid(now)
            return FrameValidationResult(False, "empty_frame")
        
        # Check 4: Frame has wrong number of dimensions
        if len(frame.shape) != 3:
            self._mark_invalid(now)
            return FrameValidationResult(False, "invalid_dims")
        
        # Check 5: Frame doesn't have 3 channels (BGR)
        if frame.shape[2] != 3:
            self._mark_invalid(now)
            return FrameValidationResult(False, "invalid_channels")
        
        # Check 6: Shape consistency (detect H264 decode corruption)
        # Corrupted RTSP frames can produce frames with wrong dimensions
        if not self.allow_shape_change:
            if self._last_valid_shape is not None:
                if frame.shape != self._last_valid_shape:
                    self._mark_invalid(now)
                    logger.warning(
                        f"Frame shape changed from {self._last_valid_shape} to {frame.shape} "
                        "- possible decode corruption"
                    )
                    return FrameValidationResult(False, "shape_changed")
        
        # Check 7: Basic sanity check on frame data
        # Corrupted frames sometimes have all-zero or all-same values
        if self._is_likely_corrupted(frame):
            self._mark_invalid(now)
            return FrameValidationResult(False, "likely_corrupted")
        
        # Frame is valid
        self._mark_valid(frame.shape)
        return FrameValidationResult(True, "ok", frame)
    
    def _is_likely_corrupted(self, frame: np.ndarray) -> bool:
        """
        Heuristic check for likely corrupted frames.
        
        Corrupted H264 frames often have:
        - Large blocks of identical values
        - Extreme color values
        
        This is a lightweight check - we sample a few pixels rather than
        analyzing the entire frame.
        """
        # Sample a few pixels from different regions
        h, w = frame.shape[:2]
        try:
            # Check if frame is entirely black (common corruption pattern)
            # Sample 5 points
            samples = [
                frame[h // 4, w // 4],
                frame[h // 4, 3 * w // 4],
                frame[h // 2, w // 2],
                frame[3 * h // 4, w // 4],
                frame[3 * h // 4, 3 * w // 4],
            ]
            
            # If all samples are identical and very dark, likely corrupted
            if all(np.array_equal(s, samples[0]) for s in samples):
                if np.mean(samples[0]) < 5:  # Nearly black
                    return True
                    
        except (IndexError, ValueError):
            # Frame too small or malformed
            return True
            
        return False
    
    def _mark_invalid(self, now: float) -> None:
        """Mark current frame as invalid and track timing."""
        self._total_invalid_count += 1
        
        if self._consecutive_invalid_start is None:
            self._consecutive_invalid_start = now
            self._force_stop_sent = False
            logger.debug("Starting invalid frame tracking")
    
    def _mark_valid(self, shape: Tuple[int, int, int]) -> None:
        """Mark current frame as valid and reset invalid tracking."""
        self._total_valid_count += 1
        self._last_valid_shape = shape
        self._consecutive_invalid_start = None
        self._force_stop_sent = False
    
    def should_force_stop(self) -> bool:
        """
        Check if we should force a stop command due to prolonged invalid frames.
        
        Returns:
            True if invalid frames have exceeded the timeout threshold and
            we haven't sent a force-stop yet.
        """
        if self._consecutive_invalid_start is None:
            return False
            
        elapsed_ms = (time.monotonic() - self._consecutive_invalid_start) * 1000
        
        if elapsed_ms >= self.invalid_timeout_ms and not self._force_stop_sent:
            self._force_stop_sent = True
            logger.warning(
                f"Force stop triggered: {elapsed_ms:.0f}ms of consecutive invalid frames"
            )
            return True
            
        return False
    
    def get_invalid_duration_ms(self) -> float:
        """Get the duration of current invalid streak in milliseconds."""
        if self._consecutive_invalid_start is None:
            return 0.0
        return (time.monotonic() - self._consecutive_invalid_start) * 1000
    
    def reset(self) -> None:
        """Reset all tracking state."""
        self._consecutive_invalid_start = None
        self._last_valid_shape = None
        self._force_stop_sent = False
        
    def get_stats(self) -> dict:
        """Get validation statistics."""
        total = self._total_valid_count + self._total_invalid_count
        return {
            "total_frames": total,
            "valid_frames": self._total_valid_count,
            "invalid_frames": self._total_invalid_count,
            "valid_rate": self._total_valid_count / total if total > 0 else 0.0,
            "current_invalid_duration_ms": self.get_invalid_duration_ms(),
            "last_valid_shape": self._last_valid_shape,
        }


class MediaPipeGate:
    """
    Gate for MediaPipe processing errors.
    
    Wraps MediaPipe hand processing to catch exceptions and track
    processing failures that indicate frame corruption.
    """
    
    def __init__(self, max_consecutive_failures: int = 5):
        """
        Initialize MediaPipeGate.
        
        Args:
            max_consecutive_failures: Number of consecutive processing failures
                before marking the stream as problematic.
        """
        self.max_consecutive_failures = max_consecutive_failures
        self._consecutive_failures = 0
        self._total_failures = 0
        self._total_successes = 0
        
    def process(self, hands, rgb_frame: np.ndarray):
        """
        Process frame with MediaPipe hands, catching exceptions.
        
        Args:
            hands: MediaPipe Hands instance
            rgb_frame: RGB frame to process
            
        Returns:
            Tuple of (success: bool, result: MediaPipe result or None)
        """
        try:
            result = hands.process(rgb_frame)
            self._consecutive_failures = 0
            self._total_successes += 1
            return True, result
        except Exception as e:
            self._consecutive_failures += 1
            self._total_failures += 1
            logger.warning(f"MediaPipe processing error: {e}")
            return False, None
    
    def is_stream_problematic(self) -> bool:
        """Check if the stream has too many consecutive failures."""
        return self._consecutive_failures >= self.max_consecutive_failures
    
    def reset(self) -> None:
        """Reset failure tracking."""
        self._consecutive_failures = 0
        
    def get_stats(self) -> dict:
        """Get processing statistics."""
        total = self._total_successes + self._total_failures
        return {
            "total_processed": total,
            "successes": self._total_successes,
            "failures": self._total_failures,
            "success_rate": self._total_successes / total if total > 0 else 0.0,
            "consecutive_failures": self._consecutive_failures,
        }

