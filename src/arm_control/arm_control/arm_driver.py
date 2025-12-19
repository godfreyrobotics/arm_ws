import logging
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from . import JOINT_ORDER
from .joints import (
    FakeJoint,
    CyberGearJoint,
    RobstrideJoint,
    DifferentialWristJoint,
    DifferentialWristState,
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class ArmDriver(Node):
    """Receives joint deltas, enforces limits, and commands joints safely."""

    def __init__(self):
        super().__init__("arm_driver")
        self.get_logger().info("Starting arm driver (safe bring-up mode)")

        # Parameters
        self.declare_parameter("joint_configs", {})
        self.declare_parameter("max_delta_default", 0.05)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("motors_enabled", False)

        self.joint_configs: Dict[str, dict] = self.get_parameter("joint_configs").get_parameter_value().to_python()
        self.max_delta_default: float = float(self.get_parameter("max_delta_default").value)
        self.publish_rate_hz: float = float(self.get_parameter("publish_rate_hz").value)
        self.motors_enabled: bool = bool(self.get_parameter("motors_enabled").value)

        self._wrist_state: Optional[DifferentialWristState] = None
        self.joints = self._create_joints()
        self.commanded_positions = {name: 0.0 for name in JOINT_ORDER}

        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.joint_delta_sub = self.create_subscription(JointState, "joint_delta", self._delta_callback, 10)
        self.enable_sub = self.create_subscription(Bool, "enable", self._enable_callback, 10)

        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0 else 0.05
        self.create_timer(timer_period, self._publish_joint_states)

        # Allow runtime parameter update for enable flag.
        self.add_on_set_parameters_callback(self._on_param_update)

    def _on_param_update(self, params):
        for param in params:
            if param.name == "motors_enabled":
                self.motors_enabled = bool(param.value)
                self.get_logger().warn("motors_enabled set to %s via parameter", self.motors_enabled)
        return SetParametersResult(successful=True)

    def _enable_callback(self, msg: Bool):
        self.motors_enabled = msg.data
        self.get_logger().warn("Motor enable flag -> %s", self.motors_enabled)

    def _create_joints(self):
        joints = {}
        for name in JOINT_ORDER:
            cfg = self.joint_configs.get(name, {})
            joint_type = cfg.get("type", "fake")
            direction = float(cfg.get("direction", 1.0))
            offset = float(cfg.get("offset", 0.0))
            if joint_type == "fake":
                joints[name] = FakeJoint(name, direction=direction, offset=offset)
            elif joint_type == "cybergear":
                motor_id = int(cfg.get("motor_id", 0))
                joints[name] = CyberGearJoint(name, motor_id=motor_id, direction=direction, offset=offset)
            elif joint_type == "robstride":
                motor_id = int(cfg.get("motor_id", 0))
                joints[name] = RobstrideJoint(name, motor_id=motor_id, direction=direction, offset=offset)
            elif joint_type == "wrist_differential":
                if self._wrist_state is None:
                    self._wrist_state = DifferentialWristState(
                        servo_a_id=int(cfg.get("servo_a_id", 0)),
                        servo_b_id=int(cfg.get("servo_b_id", 1)),
                        servo_a_direction=float(cfg.get("servo_a_direction", 1.0)),
                        servo_b_direction=float(cfg.get("servo_b_direction", 1.0)),
                        servo_a_offset=float(cfg.get("servo_a_offset", 0.0)),
                        servo_b_offset=float(cfg.get("servo_b_offset", 0.0)),
                    )
                role = cfg.get("role", "pitch" if name == "wrist_pitch" else "roll")
                joints[name] = DifferentialWristJoint(
                    name=name,
                    role=role,
                    state=self._wrist_state,
                    direction=direction,
                    offset=offset,
                )
            else:
                self.get_logger().warn("Unknown joint type '%s' for %s, using fake", joint_type, name)
                joints[name] = FakeJoint(name, direction=direction, offset=offset)
        return joints

    def _delta_callback(self, msg: JointState):
        # Process each incoming delta and update commanded positions with safety constraints.
        for name, delta in zip(msg.name, msg.position):
            if name not in self.joints:
                self.get_logger().warn("Received delta for unknown joint %s", name)
                continue

            cfg = self.joint_configs.get(name, {})
            lower = float(cfg.get("lower_limit", -3.14))
            upper = float(cfg.get("upper_limit", 3.14))
            max_delta = float(cfg.get("max_delta", self.max_delta_default))

            delta = clamp(delta, -max_delta, max_delta)
            current = self.commanded_positions.get(name, 0.0)
            target = clamp(current + delta, lower, upper)
            self.commanded_positions[name] = target

            joint = self.joints[name]
            if self.motors_enabled or isinstance(joint, FakeJoint):
                joint.command_position(target)

    def _publish_joint_states(self):
        msg = JointState()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now

        for name in JOINT_ORDER:
            joint = self.joints[name]
            feedback = joint.read_position()
            if feedback is not None:
                self.commanded_positions[name] = feedback
            msg.name.append(name)
            msg.position.append(self.commanded_positions[name])

        self.joint_state_pub.publish(msg)


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    node = ArmDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
