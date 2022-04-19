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

import numpy as np
from mav_sim.tools import types
from mav_sim.tools.rotations import rotz  # Function for rotation about z-axis
from mav_sim.tools.types import NP_MAT  # Type for a numpy matrix


class DubinsParamsStruct:
    """Class for passing out calculated parameters
    """

    __slots__ = [
        "L",
        "c_s",
        "lam_s",
        "c_e",
        "lam_e",
        "z1",
        "q1",
        "z2",
        "z3",
        "q3",
    ]
    def __init__(self) -> None:
        self.L: float           # path length
        self.c_s: types.NP_MAT  # starting circle center
        self.lam_s: int         # direction of the start circle (+1 for CW/right, -1 for CCW/left)
        self.c_e: types.NP_MAT  # ending circle center
        self.lam_e: int         # direction of the end circle (+1 for CW/right, -1 for CCW/left)
        self.z1: types.NP_MAT   # Point on halfspace 1 boundary
        self.q1: types.NP_MAT   # Normal vector for halfspace 1
        self.z2: types.NP_MAT   # Point on halfspace 2 boundary (note that normal vector is same as halfpace 1)
        self.z3: types.NP_MAT   # Point on halfspace 3 boundary
        self.q3: types.NP_MAT   # Normal vector for halfspace 3

    def print(self) -> None:
        """Print the commands to the console."""
        print(
            "\n=L",self.L,
            "\n=c_s",self.c_s,
            "\n=lam_s",self.lam_s,
            "\n=c_e",self.c_e,
            "\n=lam_e",self.lam_e,
            "\n=z1",self.z1,
            "\n=q1",self.q1,
            "\n=z2",self.z2,
            "\n=z3",self.z3,
            "\n=q3",self.q3,
        )

class DubinsPoints:
    """Struct for storing points and radius used for calculating Dubins path
    """
    __slots__ = [
        "p_s",
        "chi_s",
        "p_e",
        "chi_e",
        "radius",
    ]
    def __init__(self, p_s: NP_MAT, chi_s: float, p_e: NP_MAT, chi_e: float, radius: float) -> None:
        self.p_s = p_s          # starting position
        self.chi_s = chi_s      # starting course angle
        self.p_e = p_e          # ending position
        self.chi_e = chi_e      # ending course angle
        self.radius = radius    # radius of Dubin's paths arcs

    def extract(self) -> tuple[NP_MAT, float, NP_MAT, float, float]:
        """Extracts all of the elements into a tuple
        """
        return (self.p_s, self.chi_s, self.p_e, self.chi_e, self.radius)

class DubinsParameters:
    """Class for storing the parameters for a Dubin's path
    """
    def __init__(self, p_s: NP_MAT =9999*np.ones((3,1)), chi_s: float =9999,
                 p_e: NP_MAT =9999*np.ones((3,1)), chi_e: float =9999, R: float =9999) -> None:
        """ Initialize the Dubin's path

        Args:
            p_s: starting position
            chi_s: starting course angle
            p_e: ending position
            chi_e: ending course angle
            R: radius of Dubin's path arcs
        """
        # Store input parameters
        self.p_s = p_s      # starting position
        self.chi_s = chi_s  # starting course angle
        self.p_e = p_e      # ending position
        self.chi_e = chi_e  # ending course angle
        self.radius = R     # radius of Dubin's paths arcs

        # Initialize calculated parameters
        self.length: float          # Dubin's path length
        self.center_s: types.NP_MAT # starting circle center
        self.dir_s: float           # direction of the start circle (1 => "CW", and "CCW" otherwise)
        self.center_e: types.NP_MAT # ending circle center
        self.dir_e: float           # direction of the end circle (1 => "CW", and "CCW" otherwise)
        self.r1: types.NP_MAT       # Point on halfspace 1
        self.n1: types.NP_MAT       # Normal vector for halfspace 1
        self.r2: types.NP_MAT       # Point on halfspace 2 (note that normal vector is same as halfpace 1)
        self.r3: types.NP_MAT       # Point on halfspace 3
        self.n3: types.NP_MAT       # Normal vector for halfspace 3

        if R == 9999: # Infinite radius case - straight line
            dubin = DubinsParamsStruct()
            dubin.L = R
            dubin.c_s = p_s
            dubin.lam_s = 1
            dubin.c_e = p_s
            dubin.lam_e = 1
            dubin.z1 = p_s
            dubin.q1 = p_s
            dubin.z2 = p_s
            dubin.z3 = p_s
            dubin.q3 = p_s
        else:
            points = DubinsPoints(p_s=p_s, chi_s=chi_s, p_e=p_e, chi_e=chi_e, radius=R)
            dubin = compute_parameters(points)
        self.set(dubin)

    def set(self, vals: DubinsParamsStruct) -> None:
        """Sets the class variables based upon the Dubins parameter struct

        Args:
            vals: Values to be stored in the class
        """
        self.length = vals.L         # Dubin's path length
        self.center_s = vals.c_s     # starting circle center
        self.dir_s = vals.lam_s      # direction of the start circle (1 => "CW", and "CCW" otherwise)
        self.center_e = vals.c_e     # ending circle center
        self.dir_e = vals.lam_e      # direction of the end circle (1 => "CW", and "CCW" otherwise)
        self.r1 = vals.z1            # Point on halfspace 1
        self.n1 = vals.q1            # Normal vector for halfspace 1
        self.r2 = vals.z2            # Point on halfspace 2 (note that normal vector is same as halfpace 1)
        self.r3 = vals.z3            # Point on halfspace 3
        self.n3 = vals.q3            # Normal vector for halfspace 3

def compute_parameters(points: DubinsPoints) -> DubinsParamsStruct:
    """Calculate the dubins paths parameters. Returns the parameters defining the shortest
       path between two oriented waypoints

    Args:
        points: Struct defining the oriented points and radius for the Dubins path

    Returns:
        dubin: variables for the shortest Dubins path
    """

    # Check to ensure sufficient distance between points
    (p_s, _, p_e, _, R) = points.extract()
    ell = np.linalg.norm(p_s[0:2] - p_e[0:2])
    if ell < 2 * R:
        raise ValueError('Error in Dubins Parameters: The distance between nodes must be larger than 2R.')

    # Compute the parameter for the Dubins paths and return the variables with shortest length
    dubin = calculate_rsr(points) # compute parameters for Right-Straight-Right case
    dubin_rsl = calculate_rsl(points) # compute parameters for the Right-Straight-Left case
    if dubin_rsl.L < dubin.L:
        dubin = dubin_rsl
    dubin_lsr = calculate_lsr(points) # Compute parameters for the Left-Straight-Right case
    if dubin_lsr.L < dubin.L:
        dubin = dubin_lsr
    dubin_lsl = calculate_lsl(points) # Compute parameters for the Left-Straight-Left case
    if dubin_lsl.L < dubin.L:
        dubin = dubin_lsl
    return dubin

def calculate_rsr(points: DubinsPoints) -> DubinsParamsStruct:
    """Calculates the Dubins parameters for the right-straight-right case

    Args:
        points: Struct defining the oriented points and radius for the Dubins path

    Returns:
        dubin: variables for the Dubins path
    """

    # Initialize output and extract inputs
    dubin = DubinsParamsStruct()
    (p_s, chi_s, p_e, chi_e, R) = points.extract()

    # Compute the start and end circles
    c_rs = p_s + R * np.array([[np.cos(chi_s+np.pi/2.)],[np.sin(chi_s+np.pi/2.)], [0.]])
    c_re = p_e + R * np.array([[np.cos(chi_e+np.pi/2.)],[np.sin(chi_e+np.pi/2.)], [0.]])

    # compute length for R-S-R case (Section 11.2.2.1)
    theta = np.arctan2(c_re.item(1) - c_rs.item(1), c_re.item(0) - c_rs.item(0)) # Angle of line connecting centers
    dubin.L = np.linalg.norm(c_rs - c_re) \
            + R * mod(2 * np.pi + mod(theta - np.pi / 2) - mod(chi_s - np.pi / 2)) \
            + R * mod(2 * np.pi + mod(chi_e - np.pi / 2) - mod(theta - np.pi / 2))

    # Compute the dubins parameters (lines 7-11, 34-35 of Algorithm 9)
    e1 = np.array([[1, 0, 0]]).T
    dubin.c_s = c_rs                                                            # Line 8
    dubin.lam_s = 1
    dubin.c_e = c_re
    dubin.lam_e = 1
    dubin.q1 = (dubin.c_e - dubin.c_s) / np.linalg.norm(dubin.c_e - dubin.c_s)  # Line 9
    dubin.z1 = dubin.c_s + R * rotz(-np.pi / 2) @ dubin.q1                      # Line 10
    dubin.z2 = dubin.c_e + R * rotz(-np.pi / 2) @ dubin.q1                      # Line 11
    dubin.z3 = p_e                                                              # Line 34
    dubin.q3 = rotz(chi_e) @ e1                                                 # Line 35

    return dubin

def calculate_rsl(points: DubinsPoints) -> DubinsParamsStruct:
    """Calculates the Dubins parameters for the right-straight-left case

    Args:
        points: Struct defining the oriented points and radius for the Dubins path

    Returns:
        dubin: variables for the Dubins path
    """

    # Initialize output and extract inputs
    dubin = DubinsParamsStruct()
    (p_s, chi_s, p_e, chi_e, R) = points.extract()

    # compute start and end circles (equations (11.5) and (11.6) )
    c_rs = p_s + R * np.array([[np.cos(chi_s+np.pi/2.)],[np.sin(chi_s+np.pi/2.)], [0.]])
    c_le = p_e + R * np.array([[np.cos(chi_e-np.pi/2.)],[np.sin(chi_e-np.pi/2.)], [0.]])

    # compute L2 (Section 11.2.2.2)
    ell = np.linalg.norm(c_le - c_rs) # Little l in book, distance between center points
    theta = np.arctan2(c_le.item(1) - c_rs.item(1), c_le.item(0) - c_rs.item(0))
    theta2 = theta - np.pi / 2 + np.arcsin(2 * R / ell)
    #if not np.isreal(theta2): # Occurs when 2R > ell
    if np.isnan(theta2): # Occurs when 2R > ell
        dubin.L = np.inf
    else:
        # Equation (11.10)
        dubin.L = np.sqrt(ell ** 2 - 4 * R ** 2) \
                + R * mod(2 * np.pi + mod(theta2) - mod(chi_s - np.pi / 2)) \
                + R * mod(2 * np.pi + mod(theta2 + np.pi) - mod(chi_e + np.pi / 2))


    # Halfplane parameters (Algorithm 9 lines 12-19 and 34-35)
    e1 = np.array([[1, 0, 0]]).T
    dubin.c_s = c_rs                                        # Line 13
    dubin.lam_s = 1
    dubin.c_e = c_le
    dubin.lam_e = -1
    ell = np.linalg.norm(dubin.c_e - dubin.c_s)             # Line 14
    dubin.q1 = rotz(theta2 + np.pi / 2) @ e1                # Line 17
    dubin.z1 = dubin.c_s + R * rotz(theta2) @ e1            # Line 18
    dubin.z2 = dubin.c_e + R * rotz(theta2 + np.pi) @ e1    # Line 19
    dubin.z3 = p_e                                          # Line 34
    dubin.q3 = rotz(chi_e) @ e1                             # Line 35

    return dubin

def calculate_lsr(points: DubinsPoints) -> DubinsParamsStruct:
    """Calculates the Dubins parameters for the left-straight-right case

    Args:
        points: Struct defining the oriented points and radius for the Dubins path

    Returns:
        dubin: variables for the Dubins path
    """

    # Initialize output and extract inputs
    dubin = DubinsParamsStruct()
    (p_s, chi_s, p_e, chi_e, R) = points.extract()

    # compute start and end circles (equations (11.5) and (11.6) )
    c_ls = p_s + R * np.array([[np.cos(chi_s-np.pi/2.)],[np.sin(chi_s-np.pi/2.)], [0.]])
    c_re = p_e + R * np.array([[np.cos(chi_e+np.pi/2.)],[np.sin(chi_e+np.pi/2.)], [0.]])

    # compute L3 (Section 11.2.2.3 )
    ell = np.linalg.norm(c_re - c_ls)
    theta = np.arctan2(c_re.item(1) - c_ls.item(1), c_re.item(0) - c_ls.item(0))
    theta2 = np.arccos(2 * R / ell)
    #if not np.isreal(theta2): # Case occurs when 2R > ell
    if np.isnan(theta2): # Occurs when 2R > ell
        dubin.L = np.inf
    else:
        # Equation (11.11)
        dubin.L = np.sqrt(ell ** 2 - 4 * R ** 2) \
                + R * mod(2 * np.pi - mod(theta + theta2) + mod(chi_s + np.pi / 2)) \
                + R * mod(2 * np.pi - mod(theta + theta2 - np.pi) + mod(chi_e - np.pi / 2))

    # Compute halfplane parameters (Algorithm 9 lines 20-27, 34-35)
    e1 = np.array([[1, 0, 0]]).T
    dubin.c_s = c_ls                                                # Line 21
    dubin.lam_s = -1
    dubin.c_e = c_re
    dubin.lam_e = 1
    ell = np.linalg.norm(dubin.c_e - dubin.c_s)                     # Line 22
    dubin.q1 = rotz(theta + theta2 - np.pi / 2) @ e1                # Line 25
    dubin.z1 = dubin.c_s + R * rotz(theta + theta2) @ e1            # Line 26
    dubin.z2 = dubin.c_e + R * rotz(theta + theta2 - np.pi) @ e1    # Line 27
    dubin.z3 = p_e                                                  # Line 34
    dubin.q3 = rotz(chi_e) @ e1                                     # Line 35

    return dubin

def calculate_lsl(points: DubinsPoints) -> DubinsParamsStruct:
    """Calculates the Dubins parameters for the left-straight-left case

    Args:
        points: Struct defining the oriented points and radius for the Dubins path

    Returns:
        dubin: variables for the Dubins path
    """

    # Initialize output and extract inputs
    dubin = DubinsParamsStruct()
    (p_s, chi_s, p_e, chi_e, R) = points.extract()

    # compute start and end circles (equations (11.5) and (11.6) )
    c_ls = p_s + R * np.array([[np.cos(chi_s-np.pi/2.)],[np.sin(chi_s-np.pi/2.)], [0.]])
    c_le = p_e + R * np.array([[np.cos(chi_e-np.pi/2.)],[np.sin(chi_e-np.pi/2.)], [0.]])

    # compute L4 (Section 11.2.2.4, Equation (11.12) )
    theta = np.arctan2(c_le.item(1) - c_ls.item(1), c_le.item(0) - c_ls.item(0))
    dubin.L = np.linalg.norm(c_ls - c_le) \
            + R * mod(2 * np.pi - mod(theta + np.pi / 2) + mod(chi_s + np.pi / 2)) \
            + R * mod(2 * np.pi - mod(chi_e + np.pi / 2) + mod(theta + np.pi / 2))

    # Compute halfplane parameters (Algorithm 9, Lines 28-35)
    e1 = np.array([[1, 0, 0]]).T
    dubin.c_s = c_ls                                                            # Line 29
    dubin.lam_s = -1
    dubin.c_e = c_le
    dubin.lam_e = -1
    dubin.q1 = (dubin.c_e - dubin.c_s) / np.linalg.norm(dubin.c_e - dubin.c_s)  # Line 30
    dubin.z1 = dubin.c_s + R * rotz(np.pi / 2.) @ dubin.q1                      # Line 31
    dubin.z2 = dubin.c_e + R * rotz(np.pi / 2.) @ dubin.q1                      # Line 32
    dubin.z3 = p_e                                                              # Line 34
    dubin.q3 = rotz(chi_e) @ e1                                                 # Line 35

    return dubin

def mod(x: float) -> float:
    """Computes the modulus of x with respect to 2 pi

    Args:
        x: Angle

    Returns:
        x: Angle modified to be between 0 and 2pi
    """
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x
