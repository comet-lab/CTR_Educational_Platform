"""Tests for the standalone CTR platform Python port."""

import math

import numpy as np
import pytest
from ctr_platform.driver import DryRunTransport, GCodeDriver
from ctr_platform.gcode import Pose
from ctr_platform.jointspace import JointspaceGenerator
from ctr_platform.robot import Robot
from ctr_platform.tube import Tube


def test_pose_gcode_scales_linear_axes_by_16():
    assert Pose(10, 0, 0, 0, 0, 0).to_gcode() == "X160 Y0 Z0 A0 B0 C0"
    assert Pose(0, 0, 0, 90, 0, 0).to_gcode() == "X0 Y0 Z0 A90 B0 C0"
    assert Pose(1.5, 0, 0, 0, 0, 0).to_gcode() == "X24 Y0 Z0 A0 B0 C0"


def test_tube_derives_elastic_constants():
    # tube1 from test.m, args are Tube(id, od, r, l, d, E)
    tube = Tube(3.046e-3, 3.3e-3, 1 / 17, 90e-3, 50e-3, 1935e6)
    assert tube.curvature == pytest.approx(17.0)
    expected_i = math.pi / 64 * (3.3e-3**4 - 3.046e-3**4)
    assert tube.second_moment_area == pytest.approx(expected_i)
    assert tube.polar_moment_area == pytest.approx(2 * expected_i)


def test_driver_zeroes_on_init():
    transport = DryRunTransport()
    GCodeDriver(transport, Pose())
    assert transport.lines == ["G92 X0 Y0 Z0 A0 B0 C0"]


def test_travel_for_accumulates():
    transport = DryRunTransport()
    bot = GCodeDriver(transport, Pose())
    bot.travel_for(lin1=10)
    bot.travel_for(lin2=10, rot1=90)
    assert transport.lines[-1] == "G0 X160 Y160 Z0 A90 B0 C0"


def test_travel_to_is_absolute():
    transport = DryRunTransport()
    bot = GCodeDriver(transport, Pose(lin1=5))
    bot.travel_to(lin2=10)
    assert bot.current_pose == Pose(0, 10, 0, 0, 0, 0)
    assert transport.lines[-1] == "G0 X0 Y160 Z0 A0 B0 C0"


def test_set_current_pose_as_home_resets_state():
    transport = DryRunTransport()
    bot = GCodeDriver(transport, Pose())
    bot.travel_for(lin1=10)
    bot.set_current_pose_as_home()
    assert bot.current_pose == Pose()
    assert transport.lines[-1] == "G92 X0 Y0 Z0 A0 B0 C0"


def test_robot_unpacks_joint_vector():
    # tube1 and tube2 from test.m
    tubes = [
        Tube(3.046e-3, 3.3e-3, 1 / 17, 90e-3, 50e-3, 1935e6),
        Tube(2.386e-3, 2.64e-3, 1 / 22, 170e-3, 50e-3, 1935e6),
    ]
    robot = Robot(tubes)
    q = np.array([20, 50, 45, -45])
    assert robot.get_rho_values(q) == pytest.approx([0.0, 0.03])
    assert robot.get_theta(q) == pytest.approx(np.deg2rad([45, -45]))


def test_robot_kinematics_left_as_student_exercise():
    with pytest.raises(NotImplementedError):
        Robot([]).get_links(np.zeros(2))


def test_jointspace_samples_within_limits_and_accumulates():
    poses = JointspaceGenerator().random_poses(50, rng=np.random.default_rng(0))
    for pose in poses:
        assert 0 <= pose.lin1 <= 30  # cart1 linear range
        assert pose.lin1 <= pose.lin2 <= pose.lin3  # cumulative carriages
        assert -45 <= pose.rot1 <= 45  # cart1 rotary range
        assert -90 <= pose.rot2 <= 90  # cart2 rotary range
        assert pose.rot3 == 0  # cart3 is fixed at zero
