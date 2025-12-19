from typing import Optional

from .base import Joint


class CyberGearJoint(Joint):
    """Adapter for CyberGear (ODrive-style) motors. Hardware I/O intentionally stubbed."""

    def __init__(self, name: str, motor_id: int, direction: float = 1.0, offset: float = 0.0):
        super().__init__(name, direction=direction, offset=offset)
        self.motor_id = motor_id
        self._last_logical = 0.0

    def _write_hardware(self, hardware_angle: float) -> None:
        # TODO: integrate CyberGear motor SDK command here
        self._logger.debug("CyberGear[%s] command -> %s rad (hw frame)", self.motor_id, hardware_angle)

    def _read_hardware(self) -> Optional[float]:
        # TODO: integrate encoder readback
        return None

    def read_position(self) -> Optional[float]:
        hardware_angle = self._read_hardware()
        if hardware_angle is None:
            return self._last_logical
        return self.hardware_to_logical(hardware_angle)

    def command_position(self, target_angle: float) -> None:
        hardware_angle = self.logical_to_hardware(target_angle)
        self._write_hardware(hardware_angle)
        self._last_logical = target_angle
