from dataclasses import dataclass
from typing import Optional

from .base import Joint


@dataclass
class DifferentialWristState:
    servo_a_id: int
    servo_b_id: int
    servo_a_direction: float = 1.0
    servo_b_direction: float = 1.0
    servo_a_offset: float = 0.0
    servo_b_offset: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    def desired_servo_positions(self) -> tuple[float, float]:
        servo_a = self.servo_a_direction * (self.roll + self.pitch) + self.servo_a_offset
        servo_b = self.servo_b_direction * (self.roll - self.pitch) + self.servo_b_offset
        return servo_a, servo_b


class DifferentialWristJoint(Joint):
    """Adapter that maps logical wrist_pitch/wrist_roll to the two Feetech servos."""

    def __init__(
        self,
        name: str,
        role: str,
        state: DifferentialWristState,
        direction: float = 1.0,
        offset: float = 0.0,
    ):
        super().__init__(name, direction=direction, offset=offset)
        self.role = role  # "pitch" or "roll"
        self._state = state

    def _write_servos(self) -> None:
        servo_a, servo_b = self._state.desired_servo_positions()
        # TODO: call Feetech SDK for both servos atomically
        self._logger.debug(
            "Feetech[%s,%s] -> servo_a=%.3f, servo_b=%.3f (hw frame)",
            self._state.servo_a_id,
            self._state.servo_b_id,
            servo_a,
            servo_b,
        )

    def read_position(self) -> Optional[float]:
        value = self._state.pitch if self.role == "pitch" else self._state.roll
        return self.hardware_to_logical(value)

    def command_position(self, target_angle: float) -> None:
        logical_target = self.logical_to_hardware(target_angle)
        if self.role == "pitch":
            self._state.pitch = logical_target
        else:
            self._state.roll = logical_target
        self._write_servos()
