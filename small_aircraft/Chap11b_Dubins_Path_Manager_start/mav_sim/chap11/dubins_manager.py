"""Provides an implementation of the dubins path manager for waypoint following as described in
   Chapter 11 Algorithms 9 and 10
"""

import numpy as np
from mav_sim.chap11.dubins_parameters import DubinsParameters
from mav_sim.chap11.path_manager_utilities import (
    HalfSpaceParams,
    WaypointIndices,
    get_airspeed,
    inHalfSpace,
)
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_state import MsgState
from mav_sim.message_types.msg_waypoints import MsgWaypoints

# pylint: disable=too-many-arguments

def dubins_manager(state: MsgState, waypoints: MsgWaypoints, ptr_prv: WaypointIndices,
                path_prv: MsgPath, hs_prv: HalfSpaceParams, radius: float, manager_state: int, \
                dubins_path_prv: DubinsParameters) \
            -> tuple[MsgPath, HalfSpaceParams, WaypointIndices, int, DubinsParameters]:
    """Update for the Dubins path manager.
       Updates state machine if the MAV enters into the next halfspace.

    Args:
        state: current state of the vehicle
        waypoints: The waypoints to be followed
        ptr_prv: The indices that were being used on the previous iteration (i.e., current waypoint
                 inidices being followed when manager called)
        hs_prv: The previous halfspace being looked for (i.e., the current halfspace when manager called)
        radius: minimum radius circle for the mav
        manager_state: Integer state of the manager
                1: First portion of beginning circle
                2: Second portion of beginning circle, up to H_1
                3: Straight-line segment, up to H_2
                4: First portion of ending circle
                5: Second portion of the ending circle, up to H_3

    Returns:
        path (MsgPath): Path to be followed
        hs (HalfSpaceParams): Half space parameters corresponding to the next change in state
        ptr (WaypointIndices): Indices of the current waypoint being followed
        manager_state (int): The current state of the manager
    """
    # Default the outputs to be the inputs
    path = path_prv
    hs = hs_prv
    ptr = ptr_prv
    dubins_path = dubins_path_prv

    # Manage the Dubins sections

    return (path, hs, ptr, manager_state, dubins_path)

def construct_dubins_circle_start(waypoints: MsgWaypoints, ptr: WaypointIndices, dubins_path: DubinsParameters) \
    -> tuple[MsgPath, HalfSpaceParams]:
    """ Create the starting orbit for the dubin's path

    Args:
        waypoints: The waypoints to be followed
        ptr: The indices of the waypoints being used for the path
        dubins_Path: The parameters that make-up the Dubin's path between waypoints

    Returns:
        path: The first circle of the Dubin's path
        hs: The halfspace for switching to the next waypoint (H_1)
    """
    # Create the orbit
    path = MsgPath()
    path.type = 'orbit'
    path.plot_updated = False
    path.airspeed = get_airspeed(waypoints, ptr)

    # Fill in remaining parameters

    # Define the switching halfspace
    hs = HalfSpaceParams()

    return (path, hs)


def construct_dubins_line(waypoints: MsgWaypoints, ptr: WaypointIndices, dubins_path: DubinsParameters) \
    -> tuple[MsgPath, HalfSpaceParams]:
    """ Create the straight line segment for the dubin's path

    Args:
        waypoints: The waypoints to be followed
        ptr: The indices of the waypoints being used for the path
        dubins_Path: The parameters that make-up the Dubin's path between waypoints

    Returns:
        path: The straight-line path to be followed
        hs: The halfspace for switching to the next waypoint (H_2)
    """
    # Create the line
    path = MsgPath()
    path.type = 'line'
    path.plot_updated = False
    path.airspeed = get_airspeed(waypoints, ptr)

    # Fill in remaining parameters

    # Define the switching halfspace
    hs = HalfSpaceParams()

    return (path, hs)


def construct_dubins_circle_end(waypoints: MsgWaypoints, ptr: WaypointIndices, dubins_path: DubinsParameters) \
    -> tuple[MsgPath, HalfSpaceParams]:
    """ Create the ending orbit for the dubin's path

    Args:
        waypoints: The waypoints to be followed
        ptr: The indices of the waypoints being used for the path
        dubins_Path: The parameters that make-up the Dubin's path between waypoints

    Returns:
        path: The straight-line path to be followed
        hs: The halfspace for switching to the next waypoint
    """
    # Create the orbit
    path = MsgPath()
    path.plot_updated = False
    path.type = 'orbit'
    path.airspeed = get_airspeed(waypoints, ptr)

    # Fill in remaining parameters

    # Define the switching halfspace
    hs = HalfSpaceParams()

    return (path, hs)
