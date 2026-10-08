"""Random joint-space pose generator, ported from Jointspace_Generator.m."""

from dataclasses import dataclass

import numpy as np

from .gcode import Pose


@dataclass
class Cart:
    """Travel limits for one carriage, linear and rotary."""

    linear_min: float
    linear_max: float
    rotary_min: float
    rotary_max: float


class JointspaceGenerator:
    """Draws random carriage poses within the platform's travel limits."""

    # linear and rotary travel limits per carriage, from Jointspace_Generator.m
    CARTS = (
        Cart(0, 30, -45, 45),
        Cart(0, 30, -90, 90),
        Cart(0, 0, 0, 0),
    )

    def random_poses(
        self, count: int, rng: np.random.Generator | None = None
    ) -> list[Pose]:
        """Return count random poses, linear carriages accumulated base to tip."""
        rng = rng or np.random.default_rng()
        poses = []
        for _ in range(count):
            linear_samples = [
                rng.integers(cart.linear_min, cart.linear_max + 1) for cart in self.CARTS
            ]
            rotations = [
                int(rng.integers(cart.rotary_min, cart.rotary_max + 1)) for cart in self.CARTS
            ]
            cumulative = np.cumsum(linear_samples)
            poses.append(
                Pose(int(cumulative[0]), int(cumulative[1]), int(cumulative[2]),
                     rotations[0], rotations[1], rotations[2])
            )
        return poses
