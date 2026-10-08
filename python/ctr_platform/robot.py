"""
CTR forward kinematics scaffold, ported from Robot.m.

The joint-unpacking helpers are implemented, but the constant-curvature
kinematics are left as exercises to match the TODOs in the MATLAB. Students
fill in get_links, calculate_phi_and_kappa, and calculate_transform.
"""

import numpy as np

from .tube import Tube


class Robot:
    """A concentric tube robot built from nested tubes, outermost first."""

    def __init__(self, tubes: list[Tube]) -> None:
        self.tubes = tubes
        self.num_tubes = len(tubes)

    def fkin(self, q: np.ndarray) -> np.ndarray:
        """Base-to-tip transform for a joint vector [translations..., rotations...]."""
        rho = self.get_rho_values(q)
        theta = self.get_theta(q)
        link_lengths = self.get_links(rho)
        phi, kappa = self.calculate_phi_and_kappa(theta)
        return self.calculate_transform(link_lengths, phi, kappa)

    def get_rho_values(self, q: np.ndarray) -> np.ndarray:
        """Tube translations relative to the outermost tube, in meters."""
        rho = np.zeros(self.num_tubes)
        # q translations arrive in mm, convert to meters
        rho[1:] = (np.asarray(q[1:self.num_tubes]) - q[0]) * 1e-3
        return rho

    def get_theta(self, q: np.ndarray) -> np.ndarray:
        """Tube rotation angles in radians, from the back half of q."""
        return np.deg2rad(np.asarray(q[self.num_tubes:2 * self.num_tubes]))

    def get_links(self, rho: np.ndarray) -> np.ndarray:
        """Link lengths for each constant-curvature section."""
        # TODO student exercise, see Robot.m
        raise NotImplementedError

    def calculate_phi_and_kappa(
        self, theta: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Per-link base angle phi and curvature kappa from the tube angles."""
        # TODO student exercise, see Robot.m
        raise NotImplementedError

    def calculate_transform(
        self, link_lengths: np.ndarray, phi: np.ndarray, kappa: np.ndarray
    ) -> np.ndarray:
        """Base-to-tip SE(3) transform from the arc parameters."""
        # TODO student exercise, see Robot.m
        raise NotImplementedError
