"""
 dubins_parameters
   - Dubins parameters that define path between two configurations

 mavsim_matlab
     - Beard & McLain, PUP, 2012
     - Update history:
         3/26/2019 - RWB
         4/2/2020 - RWB
         12/21 - GND
"""

from typing import cast

import numpy as np
from mav_sim.tools import types
from mav_sim.tools.types import NP_MAT  # Type for a numpy matrix


class DubinsParameters:
    """Class for storing the parameters for a Dubin's path
    """
    def __init__(self, ps: NP_MAT =9999*np.ones((3,1)), chis: float =9999,
                 pe: NP_MAT =9999*np.ones((3,1)), chie: float =9999, R: float =9999) -> None:
        """ Initialize the Dubin's path

        Args:
            ps: starting position
            chis: starting course angle
            pe: ending position
            chie: ending course angle
            R: radius of Dubin's path arcs
        """
        if R == 9999: # Infinite radius case - straight line
            L = R
            cs = ps
            lams = R
            ce = ps
            lame = R
            w1 = ps
            q1 = ps
            w2 = ps
            w3 = ps
            q3 = ps
        else:
            L, cs, lams, ce, lame, w1, q1, w2, w3, q3 \
                = compute_parameters(ps, chis, pe, chie, R)
        self.p_s = ps           # starting position
        self.chi_s = chis       # starting course angle
        self.p_e = pe           # ending position
        self.chi_e = chie       # ending course angle
        self.radius = R         # radius of Dubin's paths arcs
        self.length = L         # Dubin's path length
        self.center_s = cs      # starting circle center
        self.dir_s = lams       # direction of the start circle (1 => "CW", and "CCW" otherwise)
        self.center_e = ce      # ending circle center
        self.dir_e = lame       # direction of the end circle (1 => "CW", and "CCW" otherwise)
        self.r1 = w1            # Point on halfspace 1
        self.n1 = q1            # Normal vector for halfspace 1
        self.r2 = w2            # Point on halfspace 2 (note that normal vector is same as halfpace 1)
        self.r3 = w3            # Point on halfspace 3
        self.n3 = q3            # Normal vector for halfspace 3

    def update(self, ps: NP_MAT, chis: float, pe: NP_MAT, chie: float, R: float) -> None:
        """Create a Dubins path with new parameters

        Args:
            ps: starting position
            chis: starting course angle
            pe: ending position
            chie: ending course angle
            R: radius of Dubin's path arcs

        Returns:
            L: path length
            cs: starting circle center
            lams: direction of the start circle
            ce: ending circle center
            lame: direction of the end circle
            w1: Point on halfspace 1
            q1: Normal vector for halfspace 1
            w2: Point on halfspace 2 (note that normal vector is same as halfpace 1)
            w3: Point on halfspace 3
            q3: Normal vector for halfspace 3

        """
        pass


def compute_parameters(ps: NP_MAT, chis: float, pe: NP_MAT, chie: float, R: float) \
    -> tuple[float, NP_MAT, float, NP_MAT, float, NP_MAT, NP_MAT, NP_MAT, NP_MAT, NP_MAT]: \
    #           L     cs     lams    ce    lame    w1       q1      w2      w3     q3
    """Calculate the dubins paths parameters

    Args:
        ps: starting position
        chis: starting course angle
        pe: ending position
        chie: ending course angle
        R: radius of Dubin's paths arcs

    Returns:
        L: path length
        cs: starting circle center
        lams: direction of the start circle
        ce: ending circle center
        lame: direction of the end circle
        w1: Point on halfspace 1
        q1: Normal vector for halfspace 1
        w2: Point on halfspace 2 (note that normal vector is same as halfpace 1)
        w3: Point on halfspace 3
        q3: Normal vector for halfspace 3
    """
    pass


def rotz(theta: float) -> types.RotMat:
    """Calculates the rotation matrix about the z axis

    Args:
        theta: Angle of rotation

    Returns:
        Resulting rotation about z-axis
    """

    pass


def mod(x: float) -> float:
    """Computes the modulus of x with respect to 2 pi

    Args:
        x: Angle

    Returns:
        x: Angle modified to be between 0 and 2pi
    """
    pass
