import select
import sys
import termios
import tty
from typing import Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from . import JOINT_ORDER


KEY_BINDINGS = {
    "q": ("shoulder_pitch", 1),
    "a": ("shoulder_pitch", -1),
    "w": ("shoulder_yaw", 1),
    "s": ("shoulder_yaw", -1),
    "e": ("shoulder_roll", 1),
    "d": ("shoulder_roll", -1),
    "r": ("elbow_flex", 1),
    "f": ("elbow_flex", -1),
    "t": ("wrist_pitch", 1),
    "g": ("wrist_pitch", -1),
    "y": ("wrist_roll", 1),
    "h": ("wrist_roll", -1),
}


class KeyboardTeleop(Node):
    """Publishes small joint deltas from keyboard input."""

    def __init__(self):
        super().__init__("arm_keyboard_teleop")
        self.declare_parameter("step", 0.05)
        self.declare_parameter("rate_hz", 20.0)
        self.step = float(self.get_parameter("step").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.joint_delta_pub = self.create_publisher(JointState, "joint_delta", 10)
        self.enable_pub = self.create_publisher(Bool, "enable", 10)

        self._settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        self.get_logger().info(self._help_text())
        self.create_timer(1.0 / self.rate_hz, self._read_keyboard)

    def _help_text(self) -> str:
        bindings = ", ".join([f"{k}:{v[0]}" for k, v in KEY_BINDINGS.items()])
        return (
            f"Teleop ready. Keys -> joints [{bindings}]. "
            f"'+'/'-' adjust step ({self.step:.3f} rad), "
            "'p' enable, 'k' disable, space = emergency disable."
        )

    def _read_keyboard(self):
        key = self._get_key()
        if key is None:
            return

        if key in ["-", "_"]:
            self.step = max(0.005, self.step - 0.005)
            self.get_logger().info("Step size: %.3f rad", self.step)
            return
        if key in ["+", "="]:
            self.step = min(0.3, self.step + 0.005)
            self.get_logger().info("Step size: %.3f rad", self.step)
            return
        if key == "p":
            self.enable_pub.publish(Bool(data=True))
            self.get_logger().warn("Enable requested")
            return
        if key == "k" or key == " ":
            self.enable_pub.publish(Bool(data=False))
            self.get_logger().warn("Emergency disable issued")
            return
        if key in KEY_BINDINGS:
            joint, direction = KEY_BINDINGS[key]
            delta = self.step * direction
            msg = JointState()
            msg.name = [joint]
            msg.position = [delta]
            self.joint_delta_pub.publish(msg)
        elif key == "\x03":  # Ctrl-C
            raise KeyboardInterrupt

    def _get_key(self) -> str | None:
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
