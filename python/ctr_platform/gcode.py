"""G-code formatting for the CTR platform, ported from Pose.m."""

from dataclasses import dataclass

# linear axis scale baked into the g-code, see Pose.m
LINEAR_SCALE = 16


@dataclass
class Pose:
    """Six-axis carriage pose, linear X/Y/Z and rotary A/B/C."""

    lin1: float = 0.0
    lin2: float = 0.0
    lin3: float = 0.0
    rot1: float = 0.0
    rot2: float = 0.0
    rot3: float = 0.0

    def to_gcode(self) -> str:
        """Return the axis words for a move, like 'X160 Y0 Z0 A90 B0 C0'."""
        return (
            f"X{self.lin1 * LINEAR_SCALE:g} "
            f"Y{self.lin2 * LINEAR_SCALE:g} "
            f"Z{self.lin3 * LINEAR_SCALE:g} "
            f"A{self.rot1:g} B{self.rot2:g} C{self.rot3:g}"
        )
