import abc
import logging
from typing import Optional


class Joint(abc.ABC):
    """Interface for a logical joint. Hardware specifics remain hidden."""

    def __init__(self, name: str, direction: float = 1.0, offset: float = 0.0):
        self.name = name
        self.direction = direction
        self.offset = offset
        self._logger = logging.getLogger(f"joint.{name}")

    def logical_to_hardware(self, logical_angle: float) -> float:
        return self.direction * logical_angle + self.offset

    def hardware_to_logical(self, hardware_angle: float) -> float:
        return (hardware_angle - self.offset) * self.direction

    @abc.abstractmethod
    def read_position(self) -> Optional[float]:
        """Return the current logical joint angle in radians, or None if unavailable."""

    @abc.abstractmethod
    def command_position(self, target_angle: float) -> None:
        """Command the joint to the logical target angle in radians."""
