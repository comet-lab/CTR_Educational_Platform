# CTR Educational Platform - Python port

A clean Python port of the MATLAB driver in this repo, so the concentric tube
robot can be run from Python over the same G-code interface.

## Layout

- `ctr_platform/gcode.py` - Pose and its G-code formatting (from Pose.m)
- `ctr_platform/driver.py` - serial G-code driver (from Drive.m)
- `ctr_platform/tube.py` - tube geometry and elastic constants (from Tube.m)
- `ctr_platform/robot.py` - forward kinematics scaffold (from Robot.m)
- `ctr_platform/jointspace.py` - random joint-space pose generator
- `ctr_platform/demo.py` - runnable demo mirroring test.m

The constant curvature kinematics in `robot.py` are left as exercises, the same
way the MATLAB leaves them as TODOs. Students fill in `get_links`,
`calculate_phi_and_kappa`, and `calculate_transform`.

## Install

    pip install -r requirements.txt

## Run the demo

    python -m ctr_platform.demo

It prints the G-code the driver would send. To drive real hardware, build the
driver with a `SerialTransport` instead of `DryRunTransport`:

    from ctr_platform import GCodeDriver, SerialTransport, Pose

    bot = GCodeDriver(SerialTransport("/dev/ttyACM0"), Pose())
    bot.travel_for(lin1=10)

## Tests

    pytest

## Hardware notes

The robot is open loop, with no encoders, limit switches, or e-stop. The
operator hand-poses it, zeroes with G92, then commands moves inside the known
travel. Linear axes are scaled by 16 in the G-code because the firmware runs
with uncalibrated steps-per-unit (see `LINEAR_SCALE` in `gcode.py`). Drive.m
opened the port at 250000 baud while the lab notes mention 115200, so a tester
will need to confirm the baud rate against the board.
