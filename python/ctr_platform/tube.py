"""
Tube geometry and material properties, ported from Tube.m.

A tube holds its measured dimensions along with the bending and torsional
constants that follow from them, so the kinematics can read stiffness
straight off the tube.
"""

import math
from dataclasses import dataclass, field

# nitinol Poisson ratio used in the MATLAB
POISSON_RATIO = 0.217


@dataclass
class Tube:
    """A CTR tube's measured geometry and the elastic constants derived from it."""

    inner_diameter: float
    outer_diameter: float
    radius_of_curvature: float
    length: float
    straight_length: float
    youngs_modulus: float

    curvature: float = field(init=False)
    second_moment_area: float = field(init=False)
    polar_moment_area: float = field(init=False)
    shear_modulus: float = field(init=False)

    def __post_init__(self) -> None:
        self.curvature = 1.0 / self.radius_of_curvature
        self.second_moment_area = (
            math.pi / 64 * (self.outer_diameter**4 - self.inner_diameter**4)
        )
        self.polar_moment_area = 2 * self.second_moment_area
        self.shear_modulus = self.youngs_modulus / (2 * (1 + POISSON_RATIO))
