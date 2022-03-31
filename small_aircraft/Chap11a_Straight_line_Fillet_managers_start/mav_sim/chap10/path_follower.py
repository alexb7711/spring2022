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

    ### Airspeed - just path airspeed
    autopilot_commands.airspeed_command = path.airspeed

    ### feedforward roll angle
    # For straight line, the roll angle should be zero
    autopilot_commands.phi_feedforward = 0.0

    ### Course command (Equation (10.8) )
    # Course angle of path (Equation (10.1) and wrapping per discussion below (10.8) )
    chi_q = np.arctan2(path.line_direction.item(1),
                        path.line_direction.item(0))
    chi_q = wrap(chi_q, state.chi)

    # Calculate they y-component of ep as defined above (10.2) - the path error
    # Note this is simplified by understanding that you only want the y component
    #  so you don't need to calculate the full rotation matrix
    path_error = -sin(chi_q) * (state.north - path.line_origin.item(0)) \
                    + cos(chi_q) * (state.east - path.line_origin.item(1))

    # course command - equation (10.8)
    autopilot_commands.course_command \
        = chi_q - chi_inf * (2 / np.pi) * np.arctan(k_path * path_error)

    ### altitude command (from Equation (10.5) - note that the normal below is greatly simplified
    ### by using the matrix form of the cross product and noting that k^i is [0,0,1]^T)
    # Compute the normal vector to the q-k plane
    qn = path.line_direction.item(0)
    qe = path.line_direction.item(1)
    n = (1/np.sqrt(qn**2 + qe**2)) * np.array([[qe],[-qn],[0]])

    # Calculate s, the projection of the relative error vector onto q-k plane
    p = np.array([[state.north], [state.east], [-state.altitude]])
    ep = p - np.reshape(path.line_origin, (3,1))
    s = ep - (ep.transpose()@n)*n

    # Calculate the autopilot command - Equation (10.5)
    autopilot_commands.altitude_command \
        = -path.line_origin.item(2) \
            - np.sqrt(s.item(0)**2 + s.item(1)**2) * path.line_direction.item(2) \
            / np.sqrt(path.line_direction.item(0)**2 + path.line_direction.item(1)**2)

    return autopilot_commands


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

    # Define the direction (lambda in book) based on whether to go CW or CCW
    if path.orbit_direction == 'CW':
        direction = 1.0
    else:
        direction = -1.0

    ### altitude command - assumes orbit is in north-east plane
    # Keep in mind that altitude is the negative of the down component
    autopilot_commands.altitude_command = -path.orbit_center.item(2)

    ### airspeed command - Take the airspeed from the path
    autopilot_commands.airspeed_command = path.airspeed

    ### Compute the course command (Equation (10.14) )
    # distance from orbit center (Equation above (10.9) )
    d = np.sqrt((state.north - path.orbit_center.item(0))**2
                + (state.east - path.orbit_center.item(1))**2)

    # Compute the phase angle of the relative position (Equation (10.9) )
    # wrapped per discussion below (10.14)
    varphi = np.arctan2(state.east - path.orbit_center.item(1),
                        state.north - path.orbit_center.item(0))
    varphi = wrap(varphi, state.chi)

    # compute normalized orbit error (d-p)/p, used in (10.14)
    orbit_error = (d - path.orbit_radius) / path.orbit_radius

    # course command (Equation (10.14) )
    autopilot_commands.course_command \
        = varphi + direction * (np.pi/2.0 + np.arctan(k_orbit * orbit_error))

    ### Roll feedforward command
    # Only do the feedforward when close
    if orbit_error < 10:
        # Feedforward caculated assuming no wind (Equation (10.15) )
        autopilot_commands.phi_feedforward \
            = direction * np.arctan(path.airspeed**2 / gravity / path.orbit_radius)
    else:
        autopilot_commands.phi_feedforward = 0.0

    return autopilot_commands
