#!/usr/bin/env python3
"""
Hand Gesture Robot Control - ROS2 Node

This node uses MediaPipe to detect hand gestures and publishes
velocity commands to control a robot.

Controls:
- Left hand: Speed control (openness determines magnitude, palm facing = forward, back = reverse)
- Right hand: Direction control (fist roll angle for steering)
- Both hands open for 1.5s: Toggle drive enable/disable
"""

import time
import math

import cv2
import numpy as np
import mediapipe as mp

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


# -------- MediaPipe setup --------
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

# Landmark indices (MediaPipe Hands)
WRIST = 0
INDEX_MCP, INDEX_PIP, INDEX_DIP, INDEX_TIP = 5, 6, 7, 8
MIDDLE_MCP, MIDDLE_PIP, MIDDLE_DIP, MIDDLE_TIP = 9, 10, 11, 12
RING_MCP, RING_PIP, RING_DIP, RING_TIP = 13, 14, 15, 16
PINKY_MCP, PINKY_PIP, PINKY_DIP, PINKY_TIP = 17, 18, 19, 20


def val_gap(prev, new, gap=0.05):
    """Apply gap filtering to smooth value changes."""
    if abs(new - prev) < gap:
        return prev
    return new


def clamp(v, lo, hi):
    """Clamp value between lo and hi."""
    return max(lo, min(hi, v))


def deadzone(x, dz):
    """Apply deadzone to input value."""
    return 0.0 if abs(x) < dz else (x - np.sign(x) * dz) / (1 - dz)


def draw_progress_bar(img, x, y, w, h, p, color=(0, 255, 255)):
    """Draw a progress bar on the image."""
    p = clamp(p, 0.0, 1.0)
    cv2.rectangle(img, (x, y), (x + w, y + h), (50, 50, 50), 1)
    cv2.rectangle(img, (x, y), (x + int(w * p), y + h), color, -1)


def _v(lm, i):
    """Get 3D vector from landmark."""
    p = lm[i]
    return np.array([p.x, p.y, p.z], dtype=np.float32)


def _norm(v):
    """Normalize vector."""
    n = np.linalg.norm(v)
    return v / (n + 1e-8)


def _angle_deg(u, v):
    """Calculate angle in degrees between two vectors."""
    u = _norm(u)
    v = _norm(v)
    d = float(np.clip(np.dot(u, v), -1.0, 1.0))
    return math.degrees(math.acos(d))


def _hand_size_ref(lm):
    """Get reference hand size for normalization."""
    w = _v(lm, WRIST)
    idx = _v(lm, INDEX_MCP)
    mid = _v(lm, MIDDLE_MCP)
    pink = _v(lm, PINKY_MCP)
    return max(
        (np.linalg.norm(idx - w) + np.linalg.norm(mid - w) + np.linalg.norm(pink - w))
        / 3.0,
        1e-3,
    )


def _palm_center(lm):
    """Get center of palm."""
    pts = [
        _v(lm, WRIST),
        _v(lm, INDEX_MCP),
        _v(lm, MIDDLE_MCP),
        _v(lm, RING_MCP),
        _v(lm, PINKY_MCP),
    ]
    return np.mean(pts, axis=0)


# -------- Finger joint tuples --------
INDEX = (INDEX_MCP, INDEX_PIP, INDEX_DIP, INDEX_TIP)
MIDDLE = (MIDDLE_MCP, MIDDLE_PIP, MIDDLE_DIP, MIDDLE_TIP)
RING = (RING_MCP, RING_PIP, RING_DIP, RING_TIP)
PINKY = (PINKY_MCP, PINKY_PIP, PINKY_DIP, PINKY_TIP)


def _curl_pip_deg(lm, joint_ids):
    """Calculate finger curl angle at PIP joint."""
    MCP, PIP, DIP, TIP = joint_ids
    v1 = _v(lm, PIP) - _v(lm, MCP)
    v2 = _v(lm, TIP) - _v(lm, PIP)
    return _angle_deg(v1, v2)


def _tip_palm_dist(lm, tip_idx):
    """Get distance from fingertip to palm center."""
    c = _palm_center(lm)
    return np.linalg.norm(_v(lm, tip_idx) - c)


def _finger_features(lm, joint_ids):
    """Extract finger features for gesture recognition."""
    ref = _hand_size_ref(lm)
    MCP, PIP, DIP, TIP = joint_ids
    return {"tip_palm_n": _tip_palm_dist(lm, TIP) / ref}


def _all_finger_feats(lm):
    """Get features for all fingers."""
    return {
        "index": _finger_features(lm, INDEX),
        "middle": _finger_features(lm, MIDDLE),
        "ring": _finger_features(lm, RING),
        "pinky": _finger_features(lm, PINKY),
    }


def _map01(x, a0, a1):
    """Map value to 0-1 range."""
    return float(np.clip((x - a0) / (a1 - a0), 0.0, 1.0))


def finger_openness_01(f):
    """Calculate finger openness (0=closed, 1=open)."""
    return _map01(f["tip_palm_n"], 0.50, 0.90)


def hand_openness_01(lm):
    """Calculate overall hand openness."""
    feats = _all_finger_feats(lm)
    vals = [finger_openness_01(f) for f in feats.values()]
    return float(np.mean(vals)) if vals else 0.0


def palm_facing_sign(lm):
    """Determine if palm is facing camera (+1) or away (-1)."""
    tips = [INDEX_TIP, MIDDLE_TIP, RING_TIP, PINKY_TIP]
    mcps = [INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP]
    tip_z = float(np.mean([lm[i].z for i in tips]))
    mcp_z = float(np.mean([lm[i].z for i in mcps]))
    dz = mcp_z - tip_z  # >0: tips closer than knuckles -> palm to camera
    THR = 0.015
    if dz > THR:
        return +1, dz
    if dz < -THR:
        return -1, dz
    return 0, dz


def direction_from_right_fist_roll(lm):
    """Get steering direction from right hand fist roll angle."""
    a = lm[INDEX_MCP]
    b = lm[PINKY_MCP]
    dx = b.x - a.x
    dy = b.y - a.y
    ang_deg = math.degrees(math.atan2(dy, dx))
    ang_deg = ((ang_deg - 90.0 + 180.0) % 360.0) - 180.0
    ang_deg_clamped = clamp(ang_deg, -50.0, 50.0)
    val = deadzone(ang_deg_clamped / 50.0, 4.0 / 50.0)
    return clamp(val, -1.0, 1.0), ang_deg


def right_is_fist_simple(lm):
    """Check if right hand is making a fist gesture."""
    curls = [_curl_pip_deg(lm, fids) for fids in (INDEX, MIDDLE, RING, PINKY)]
    return sum(1 for c in curls if c >= 45.0) >= 3


class DriveState:
    """State machine for drive control."""

    def __init__(self):
        self.enabled = False
        self._arming = False
        self._t0 = None
        self._hold = 1.5

        self.speed = 0.0
        self.direction = 0.0
        self._f_speed = 0.0
        self._f_dir = 0.0

        self.left_speed_sign = +1  # hold last sign when facing ambiguous

    def update_arming(self, both_open):
        """Update arming state based on whether both hands are open."""
        if both_open:
            if not self._arming:
                self._arming = True
                self._t0 = time.time()
            elapsed = time.time() - self._t0
            if elapsed >= self._hold:
                self.enabled = not self.enabled
                self._arming = False
                self._t0 = None
                return None
            return clamp(elapsed / self._hold, 0.0, 1.0)
        else:
            self._arming = False
            self._t0 = None
            return None

    def set_speed_dir(self, spd, direc):
        """Set speed and direction with gap filtering."""
        self._f_speed = val_gap(self._f_speed, spd)
        self._f_dir = val_gap(self._f_dir, direc)
        self.speed = clamp(self._f_speed, -1, 1)
        self.direction = clamp(self._f_dir, -1, 1)


def speed_from_left_openness_facing(lm, state):
    """Calculate speed from left hand openness and facing direction."""
    openv = hand_openness_01(lm)
    mag = clamp(1.0 - openv, 0.0, 1.0)
    sign, dz = palm_facing_sign(lm)
    if sign != 0:
        state.left_speed_sign = sign
    s = state.left_speed_sign * mag
    s = 0.0 if abs(s) < 0.03 else s
    return s, openv, dz, state.left_speed_sign


class HandDriveNode(Node):
    """ROS2 node for hand gesture robot control."""

    def __init__(self):
        super().__init__("hand_drive_node")

        # Declare parameters
        self.declare_parameter("max_linear_vel", 0.22)  # TurtleBot3 max: 0.22 m/s
        self.declare_parameter("max_angular_vel", 2.84)  # TurtleBot3 max: 2.84 rad/s
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("camera_url", "")
        self.declare_parameter("show_preview", True)
        self.declare_parameter("publish_rate", 15.0)  # Hz

        # Get parameters
        self.max_linear_vel = (
            self.get_parameter("max_linear_vel").get_parameter_value().double_value
        )
        self.max_angular_vel = (
            self.get_parameter("max_angular_vel").get_parameter_value().double_value
        )
        self.camera_id = (
            self.get_parameter("camera_id").get_parameter_value().integer_value
        )
        self.show_preview = (
            self.get_parameter("show_preview").get_parameter_value().bool_value
        )
        self.publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        self.camera_url = self.get_parameter("camera_url").get_parameter_value().string_value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.drive_enabled_pub = self.create_publisher(Bool, "drive_enabled", 10)

        # Initialize camera (device index OR RTSP/URL)
        if self.camera_url:
            self.get_logger().info(f"Opening camera_url: {self.camera_url}")
            self.cap = cv2.VideoCapture(self.camera_url, cv2.CAP_FFMPEG)
            src = self.camera_url
        else:
            self.get_logger().info(f"Opening camera_id: {self.camera_id}")
            self.cap = cv2.VideoCapture(self.camera_id)
            src = str(self.camera_id)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera source: {src}")
            raise RuntimeError(f"Cannot open camera source: {src}")

        self.get_logger().info(f"Camera source opened: {src}")

        # Initialize state
        self.state = DriveState()
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # Initialize MediaPipe hands
        self.hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            model_complexity=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        # Create timer for main loop
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Hand Drive Node initialized")
        self.get_logger().info(
            f"Max velocities: linear={self.max_linear_vel} m/s, angular={self.max_angular_vel} rad/s"
        )

    def timer_callback(self):
        """Main processing loop."""
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Failed to read frame from camera")
            return

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)

        left = right = None
        
        if res.multi_hand_landmarks and res.multi_handedness:
            packs = []
            for lm, hd in zip(res.multi_hand_landmarks, res.multi_handedness):
                label = hd.classification[0].label
                packs.append((lm, label))
            for lm, label in packs:
                if label == "Left" and left is None:
                    left = (lm, label)
                elif label == "Right" and right is None:
                    right = (lm, label)

            if self.show_preview:
                for pack in (left, right):
                    if pack:
                        mp_draw.draw_landmarks(
                            frame, pack[0], mp_hands.HAND_CONNECTIONS
                        )

            # Compute openness for arming
            left_open_val = (
                hand_openness_01(left[0].landmark) >= 0.85 if left else False
            )
            right_open_val = (
                hand_openness_01(right[0].landmark) >= 0.85 if right else False
            )
            both_open = left_open_val and right_open_val

            # Update arming state
            p = self.state.update_arming(both_open)

            # Compute speed and direction
            spd_raw = 0.0
            dir_raw = 0.0

            # LEFT: speed from openness + facing
            if left:
                lmL = left[0].landmark
                spd_raw, _, _, _ = speed_from_left_openness_facing(lmL, self.state)

            # RIGHT: direction from fist roll
            right_is_fist = False
            if right:
                lmR = right[0].landmark
                if right_is_fist_simple(lmR):
                    right_is_fist = True
                    dir_raw, _ = direction_from_right_fist_roll(lmR)
                else:
                    dir_raw = 0.0

            # Update filtered state
            self.state.set_speed_dir(spd_raw, dir_raw)

            # Draw UI if preview enabled
            if self.show_preview:
                self._draw_ui(frame, h, w, p, both_open, left, right, right_is_fist)
        else:
            # No hands detected
            if self.show_preview:
                self._draw_no_hands_ui(frame, h, w)

        # Publish messages
        self._publish_commands()

        # Show preview window
        if self.show_preview:
            cv2.imshow("Hand Gesture Robot Control", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                self.get_logger().info("Quit requested, shutting down...")
                rclpy.shutdown()
            elif key in (ord("e"), ord("E")):
                self.state.enabled = not self.state.enabled
                self.get_logger().info(
                    f"Drive {'ENABLED' if self.state.enabled else 'DISABLED'} (keyboard toggle)"
                )

    def _publish_commands(self):
        """Publish velocity commands and drive state."""
        twist = Twist()

        if self.state.enabled:
            # Scale normalized values to actual velocities
            twist.linear.x = self.state.speed * self.max_linear_vel
            twist.angular.z = -self.state.direction * self.max_angular_vel
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

        # Publish drive enabled state
        enabled_msg = Bool()
        enabled_msg.data = self.state.enabled
        self.drive_enabled_pub.publish(enabled_msg)

    def _draw_ui(self, frame, h, w, p, both_open, left, right, right_is_fist):
        """Draw UI overlay on frame."""
        # Status text and progress bar
        status_txt = f"Drive {'ENABLED' if self.state.enabled else 'DISABLED'}"
        txt_org = (20, 40)

        # Determine colors based on state
        if p is not None and both_open:
            future_enabled = not self.state.enabled
            txt_color = (0, 255, 0) if future_enabled else (0, 0, 255)
            bar_color = txt_color
            p_disp = p
        else:
            txt_color = (0, 255, 0) if self.state.enabled else (0, 0, 255)
            bar_color = txt_color
            p_disp = 1.0 if self.state.enabled else 0.0

        cv2.putText(frame, status_txt, txt_org, self.font, 0.9, txt_color, 2)

        # Progress bar
        (tw, th), _ = cv2.getTextSize(status_txt, self.font, 0.9, 2)
        bar_w, bar_h = 140, 18
        bar_x = txt_org[0] + tw + 12
        bar_y = txt_org[1] - th + 2
        draw_progress_bar(frame, bar_x, bar_y, bar_w, bar_h, p_disp, color=bar_color)

        # Speed display
        speed_y = h - 50
        if left:
            cv2.putText(
                frame,
                f"Speed = {self.state.speed:+.2f}",
                (20, speed_y),
                self.font,
                0.7,
                (255, 0, 0),
                2,
            )
        else:
            cv2.putText(
                frame,
                "Speed = N/A (no left hand)",
                (20, speed_y),
                self.font,
                0.7,
                (255, 0, 0),
                2,
            )

        # Direction display
        dir_y = h - 20
        if right:
            if right_is_fist:
                cv2.putText(
                    frame,
                    f"Direction = {self.state.direction:+.2f}",
                    (20, dir_y),
                    self.font,
                    0.7,
                    (0, 255, 255),
                    2,
                )
            else:
                cv2.putText(
                    frame,
                    "Direction = 0.00 (right not fist)",
                    (20, dir_y),
                    self.font,
                    0.7,
                    (0, 255, 255),
                    2,
                )
        else:
            cv2.putText(
                frame,
                "Direction = N/A (no right hand)",
                (20, dir_y),
                self.font,
                0.7,
                (0, 255, 255),
                2,
            )

        # ROS2 info
        cv2.putText(
            frame,
            f"Publishing: linear={self.state.speed * self.max_linear_vel:.2f} m/s, "
            f"angular={-self.state.direction * self.max_angular_vel:.2f} rad/s",
            (20, 70),
            self.font,
            0.5,
            (200, 200, 200),
            1,
        )

    def _draw_no_hands_ui(self, frame, h, w):
        """Draw UI when no hands are detected."""
        status_txt = f"Drive {'ENABLED' if self.state.enabled else 'DISABLED'}"
        txt_org = (20, 40)
        cv2.putText(frame, status_txt, txt_org, self.font, 0.9, (0, 255, 255), 2)

        (tw, th), _ = cv2.getTextSize(status_txt, self.font, 0.9, 2)
        bar_w, bar_h = 140, 18
        bar_x = txt_org[0] + tw + 12
        bar_y = txt_org[1] - th + 2
        p_disp = 1.0 if self.state.enabled else 0.0
        bar_color = (0, 255, 0) if self.state.enabled else (0, 0, 255)
        draw_progress_bar(frame, bar_x, bar_y, bar_w, bar_h, p_disp, color=bar_color)

        cv2.putText(
            frame,
            "Speed = N/A (no left hand)",
            (20, h - 50),
            self.font,
            0.7,
            (255, 0, 0),
            2,
        )
        cv2.putText(
            frame,
            "Direction = N/A (no right hand)",
            (20, h - 20),
            self.font,
            0.7,
            (0, 255, 255),
            2,
        )

    def destroy_node(self):
        """Clean up resources."""
        self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = HandDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

