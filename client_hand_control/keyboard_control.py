"""
Keyboard control via pynput — a second input source alongside hand gestures.

Hybrid split with the cv2 preview window:
- pynput (this module) owns *driving* (held WASD + Shift nitro) and the new
  discrete keys (M = mode, Left/Right = lane, Enter = start/stop autonomy).
- the preview's cv2.waitKey still owns q/e/c/Space (quit/enable/calibrate/estop).

Driving overrides gestures only while WASD is held (offroad); release and the
gesture pipeline resumes.

Caveats worth knowing:
- pynput captures keys GLOBALLY while the client runs. That's what lets you
  drive without the preview window focused — but it also means WASD will move the
  robot even if you alt-tab to another app. Don't type elsewhere while driving.
- macOS requires Accessibility permission for your terminal:
  System Settings -> Privacy & Security -> Accessibility -> enable it.
- If pynput isn't installed, the client still runs (gestures only); driving is
  just disabled with a warning.
"""

import logging
import threading
from typing import Callable

logger = logging.getLogger(__name__)

try:
    from pynput import keyboard as _kb
    _HAVE_PYNPUT = True
except Exception:  # ImportError, or platform backend failure
    _HAVE_PYNPUT = False


class KeyboardController:
    """Tracks held WASD/Shift in a thread-safe set and fires one-shot callbacks
    for the discrete keys. The listener runs on its own thread (pynput); the
    control loop reads driving()/drive_axes() each tick."""

    def __init__(
        self,
        on_mode_toggle: Callable[[], None],
        on_lane: Callable[[str], None],
        on_auto_toggle: Callable[[], None],
    ):
        self._on_mode_toggle = on_mode_toggle
        self._on_lane = on_lane
        self._on_auto_toggle = on_auto_toggle

        self._held = set()
        self._lock = threading.Lock()
        self._listener = None

    @property
    def available(self) -> bool:
        return _HAVE_PYNPUT

    def start(self) -> None:
        if not _HAVE_PYNPUT:
            logger.warning(
                "pynput not available — keyboard driving disabled. "
                "Install it (pip install pynput) and grant macOS Accessibility permission."
            )
            return
        self._listener = _kb.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.daemon = True
        self._listener.start()
        logger.info(
            "Keyboard control active: WASD drive, Shift nitro, M mode, <-/-> lane, Enter start/stop"
        )

    def stop(self) -> None:
        if self._listener:
            self._listener.stop()
            self._listener = None

    # ---- pynput callbacks (run on the listener thread) ----
    @staticmethod
    def _char(key):
        try:
            return key.char.lower() if key.char else None
        except AttributeError:
            return None  # a special key (shift, arrows, enter, ...)

    def _on_press(self, key):
        c = self._char(key)
        if c in ("w", "a", "s", "d"):
            with self._lock:
                self._held.add(c)
            return
        if key in (_kb.Key.shift, _kb.Key.shift_r, _kb.Key.shift_l):
            with self._lock:
                self._held.add("shift")
            return
        # Discrete one-shots (pynput already gives one event per physical press).
        if c == "m":
            self._on_mode_toggle()
        elif key == _kb.Key.left:
            self._on_lane("left")
        elif key == _kb.Key.right:
            self._on_lane("right")
        elif key == _kb.Key.enter:
            self._on_auto_toggle()

    def _on_release(self, key):
        c = self._char(key)
        if c in ("w", "a", "s", "d"):
            with self._lock:
                self._held.discard(c)
        elif key in (_kb.Key.shift, _kb.Key.shift_r, _kb.Key.shift_l):
            with self._lock:
                self._held.discard("shift")

    # ---- read by the control loop ----
    def driving(self) -> bool:
        """True if any movement key is currently held (keyboard takes over)."""
        with self._lock:
            return bool(self._held & {"w", "a", "s", "d"})

    def drive_axes(self):
        """(linear_dir, angular_dir, nitro): dirs in {-1,0,1}, nitro bool.

        angular positive = turn left (CCW), matching the message convention.
        """
        with self._lock:
            h = set(self._held)
        linear = (1.0 if "w" in h else 0.0) - (1.0 if "s" in h else 0.0)
        angular = (1.0 if "a" in h else 0.0) - (1.0 if "d" in h else 0.0)
        return linear, angular, ("shift" in h)
