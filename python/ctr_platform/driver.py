"""
Serial G-code driver for the CTR platform, ported from Drive.m.

The transport sits behind a small protocol so the driver can run without
hardware. SerialTransport talks to the Octopus board and DryRunTransport
records the lines for tests.

Drive.m opens the port at 250000 baud while the lab notes say 115200, so
confirm against the board. Baud is a parameter defaulting to the MATLAB value.
"""

from typing import Protocol

import serial

from .gcode import Pose


class Transport(Protocol):
    """Anything the driver can push G-code lines to."""

    def write_line(self, line: str) -> None: ...

    def close(self) -> None: ...


class DryRunTransport:
    """Records G-code instead of sending it, for tests and demos."""

    def __init__(self) -> None:
        self.lines: list[str] = []

    def write_line(self, line: str) -> None:
        self.lines.append(line)

    def close(self) -> None:
        pass


class SerialTransport:
    """USB-serial link to the Octopus board."""

    def __init__(self, port: str, baud: int = 250000) -> None:
        self._serial = serial.Serial(port, baud)

    def write_line(self, line: str) -> None:
        self._serial.write((line + "\n").encode("ascii"))

    def close(self) -> None:
        self._serial.close()


class GCodeDriver:
    """
    Drives the CTR actuation unit over G-code, ported from the Drive class.

    The robot is open-loop with no encoders, limit switches, or e-stop. The
    operator hand-poses it, zeroes with G92, then moves inside known travel.
    set_home_as_pose from the MATLAB is left out because it had an apparent
    sign error and nothing here needs it.
    """

    def __init__(self, transport: Transport, start_pose: Pose | None = None) -> None:
        self._transport = transport
        self.current_pose = start_pose or Pose()
        self.set_current_pose(self.current_pose)

    def set_current_pose(self, pose: Pose) -> None:
        """Tell the firmware the robot is already at this pose, without moving it."""
        self.current_pose = pose
        self.send_command("G92 " + pose.to_gcode())

    def set_current_pose_as_home(self) -> None:
        """Make the current position the new zero origin."""
        self.set_current_pose(Pose())

    def travel_for(
        self,
        lin1: float = 0.0, lin2: float = 0.0, lin3: float = 0.0,
        rot1: float = 0.0, rot2: float = 0.0, rot3: float = 0.0,
    ) -> None:
        """Move by a delta on each axis, relative to where the robot is now."""
        now = self.current_pose
        target = Pose(
            now.lin1 + lin1, now.lin2 + lin2, now.lin3 + lin3,
            now.rot1 + rot1, now.rot2 + rot2, now.rot3 + rot3,
        )
        self._travel_to_pose(target)

    def travel_to(
        self,
        lin1: float = 0.0, lin2: float = 0.0, lin3: float = 0.0,
        rot1: float = 0.0, rot2: float = 0.0, rot3: float = 0.0,
    ) -> None:
        """Move to an absolute pose measured from the last zero."""
        self._travel_to_pose(Pose(lin1, lin2, lin3, rot1, rot2, rot3))

    def _travel_to_pose(self, target: Pose) -> None:
        self.current_pose = target
        self.send_command("G0 " + target.to_gcode())

    def send_command(self, command: str) -> None:
        self._transport.write_line(command)

    def close(self) -> None:
        self._transport.close()
