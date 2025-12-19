from typing import Optional

from .base import Joint


class FakeJoint(Joint):
    """In-memory joint used for simulation and bring-up without hardware."""

    def __init__(self, name: str, direction: float = 1.0, offset: float = 0.0):
        super().__init__(name, direction=direction, offset=offset)
        self._hardware_angle = self.logical_to_hardware(0.0)

    def read_position(self) -> Optional[float]:
        return self.hardware_to_logical(self._hardware_angle)

    def command_position(self, target_angle: float) -> None:
        self._hardware_angle = self.logical_to_hardware(target_angle)
