"""Provides an implementation of the fillet path manager for waypoint following as described in
   Chapter 11 Algorithm 8
"""
import numpy as np
from mav_sim.chap11.path_manager_utilities import (
    EPSILON,
    HalfSpaceParams,
    WaypointIndices,
    extract_waypoints,
    get_airspeed,
    inHalfSpace,
)
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_state import MsgState
from mav_sim.message_types.msg_waypoints import MsgWaypoints


def fillet_manager(state: MsgState, waypoints: MsgWaypoints, ptr_prv: WaypointIndices,
                 path_prv: MsgPath, hs_prv: HalfSpaceParams, radius: float, manager_state: int) \
                -> tuple[MsgPath, HalfSpaceParams, WaypointIndices, int]:

    """Update for the fillet manager

    Update waypoints if needed and check to see if in new halfspace.
    Update state machine accordingly.

    Args:
        waypoints: The waypoints to be followed
        radius: minimum radius circle for the mav
        state: current state of the vehicle
    """
    # Default the outputs to be the inputs
    path = path_prv
    hs = hs_prv
    ptr = ptr_prv

    # Insert code here

    return (path, hs, ptr, manager_state)

def construct_fillet_line(waypoints: MsgWaypoints, ptr: WaypointIndices, radius: float) \
    -> tuple[MsgPath, HalfSpaceParams]:
    """Define the line on a fillet and a halfspace for switching to the next fillet curve.

    The line is created from the previous and current waypoints with halfspace defined for
    switching once a circle of the specified radius can be used to transition to the next line segment.

    Args:
        waypoints: The waypoints to be followed
        ptr: The indices of the waypoints being used for the path
        radius: minimum radius circle for the mav

    Returns:
        path: The straight-line path to be followed
        hs: The halfspace for switching to the next waypoint
    """
    # Extract the waypoints (w_{i-1}, w_i, w_{i+1})
    (previous, current, next_wp) = extract_waypoints(waypoints=waypoints, ptr=ptr)



    # Construct the path
    path = MsgPath()
    path.plot_updated = False

    # Construct the halfspace
    hs = HalfSpaceParams()

    return (path, hs)

def construct_fillet_circle(waypoints: MsgWaypoints, ptr: WaypointIndices, radius: float) \
    -> tuple[MsgPath, HalfSpaceParams]:
    """Define the circle on a fillet

    Args:
        waypoints: The waypoints to be followed
        ptr: The indices of the waypoints being used for the path
        radius: minimum radius circle for the mav

    Returns:
        path: The straight-line path to be followed
        hs: The halfspace for switching to the next waypoint
    """
    # Extract the waypoints (w_{i-1}, w_i, w_{i+1})
    (previous, current, next_wp) = extract_waypoints(waypoints=waypoints, ptr=ptr)

    # Construct the path
    path = MsgPath()
    path.plot_updated = False

    # Define the switching halfspace
    hs = HalfSpaceParams()

    return (path, hs)
