"""path_follower.py implements a class for following a path with a mav
"""
from math import cos, sin

import numpy as np
from mav_sim.message_types.msg_autopilot import MsgAutopilot
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_state import MsgState
from mav_sim.tools.wrap import wrap


class PathFollower:
    """Class for path following
    """
    def __init__(self) -> None:
        """Initialize path following class
        """
        self.chi_inf = np.radians(50)  # approach angle for large distance from straight-line path
        self.k_path = 0.01 #0.05  # path gain for straight-line path following
        self.k_orbit = 1.# 10.0  # path gain for orbit following
        self.gravity = 9.8
        self.autopilot_commands = MsgAutopilot()  # message sent to autopilot

    def update(self, path: MsgPath, state: MsgState) -> MsgAutopilot:
        """Update the control for following the path

        Args:
            path: path to be followed
            state: current state of the mav

        Returns:
            autopilot_commands: Commands to autopilot for following the path
        """
        if path.type == 'line':
            self.autopilot_commands = follow_straight_line(path=path, state=state, k_path=self.k_path, chi_inf=self.chi_inf)
        elif path.type == 'orbit':
            self.autopilot_commands = follow_orbit(path=path, state=state, k_orbit=self.k_orbit, gravity=self.gravity)
        return self.autopilot_commands

##===============================================================================
#
def follow_straight_line(path: MsgPath, state: MsgState, k_path: float, chi_inf: float) -> MsgAutopilot:
    """Calculate the autopilot commands for following a straight line

    Args:
        path: straight-line path to be followed
        state: current state of the mav
        k_path: convergence gain for converging to the path
        chi_inf: Angle to take towards path when at an infinite distance from the path

    Returns:
        autopilot_commands: the commands required for executing the desired line
    """
    # Initialize the output
    autopilot_commands = MsgAutopilot()

    # Create autopilot commands here

    # Update course command
    autopilot_commands.course_command = calc_lat_cmd(path, state, k_path, chi_inf)

    # Update altitude command
    autopilot_commands.altitude_command = calc_alt_cmd(path, state, k_path, chi_inf)

    # Update velocity command
    autopilot_commands.airspeed_command = state.Va

    return autopilot_commands


##===============================================================================
##
def calc_lat_cmd(path, state, k_path, chi_inf):
    # Variables
    p  = np.array([state.north, state.east, state.altitude]).T # Position of MAV
    q  = path.line_direction                                   # Direction of line
    r  = path.line_origin                                      # Origin of path

    # Calculate Xq and ep
    Xq, ep = calc_chiq_ep(q, state, p, r)

    # Calculate commanded course angle
    Xc = Xq - chi_inf * 2/np.pi * np.arctan(k_path*ep.item(1))

    return Xc

##===============================================================================
##
def calc_alt_cmd(path, state, k_path, chi_inf):
    # Variables
    p = np.array([[state.north, state.east, state.altitude]]).T # Position of MAV
    q = path.line_direction                                     # Direction of line
    r = path.line_origin                                        # Origin of path
    ki = np.array([[0,0,1]]).T                                  # Inertial down unit vector

    # Calculate Xq and ep
    Xq, ep = calc_chiq_ep(q, state, p, r)

    # Calcualte normal vector to plane
    n = np.cross(q.T,ki.T)/np.linalg.norm(np.cross(q.T,ki.T))
    n = n.T

    # Calculate projection of relative error vector
    s = ep - np.dot(ep[:,0], n[:,0])*n

    # Calculate altitude command
    hc = -r.item(2) - np.sqrt(s.item(0)**2 + s.item(1)**2) * \
                      (q.item(2)/np.sqrt(q.item(0)**2 + q.item(1)**2))

    return hc

##===============================================================================
##
def calc_chiq_ep(q, state, p, r):
    # Course angle as measured from north
    Xq = np.arctan2(q.item(1),q.item(0))
    Xq = wrap(Xq, state.chi)


    # Transformation matrix from inertial frame to path frame
    Rip = np.array([[np.cos(Xq)  , np.sin(Xq) , 0],
                    [-np.sin(Xq) , cos(Xq)    , 0],
                    [0           , 0          , 1]])

    # Path error
    ep = Rip@(p-r)

    return Xq, ep

##===============================================================================
##
def follow_orbit(path: MsgPath, state: MsgState, k_orbit: float, gravity: float) -> MsgAutopilot:
    """Calculate the autopilot commands for following a circular path

    Args:
        path: circular orbit to be followed
        state: current state of the mav
        k_orbit: Convergence gain for reducing error to orbit
        gravity: Gravity constant

    Returns:
        autopilot_commands: the commands required for executing the desired orbit
    """

    # Initialize the output
    autopilot_commands = MsgAutopilot()

    # Variables
    p   = np.array([[state.north, state.east, state.altitude]]).T # Position of MAV
    rho = path.orbit_radius
    c   = path.orbit_center
    l   = 1 if path.orbit_direction == "CW" else -1

    # Calculate distance from center of circle to MAV
    d = np.sqrt((p.item(0) - c.item(0))**2 + (p.item(1) - c.item(1))**2)

    # Calculate orbit angle
    phi = np.arctan2(p.item(1) - c.item(1), p.item(0) - c.item(0))
    phi = wrap(phi, state.chi)

    # Calculate course command
    Xc = phi + l*(np.pi/2 + np.arctan(k_orbit * (d-rho)/rho ))

    if (d-rho)/rho < 10:
        pff = l*np.arctan(state.Va**2/gravity*rho)
        autopilot_commands.phi_feedforward = pff
    else:
        autopilot_commands.phi_feedforward = 0

    # Calculate altitude command
    hc = -c.item(2)

    # Update command
    autopilot_commands.airspeed_command = state.Va
    autopilot_commands.course_command   = Xc
    autopilot_commands.altitude_command = hc

    return autopilot_commands
