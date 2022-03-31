"""Provides an implementation of the straight line path manager for waypoint following as described in
   Chapter 11 Algorithm 7
"""

import numpy as np
from mav_sim.chap11.path_manager_utilities import (
    HalfSpaceParams,
    WaypointIndices,
    extract_waypoints,
    get_airspeed,
    inHalfSpace,
)
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_state import MsgState
from mav_sim.message_types.msg_waypoints import MsgWaypoints


def line_manager(state: MsgState, waypoints: MsgWaypoints, ptr_prv: WaypointIndices,
                 path_prv: MsgPath, hs_prv: HalfSpaceParams) \
                -> tuple[MsgPath, HalfSpaceParams, WaypointIndices]:
    """Update for the line manager. Only updates the path and next halfspace under two conditions:
        1) The waypoints are new
        2) In a new halfspace

    Args:
        state: current state of the vehicle
        waypoints: The waypoints to be followed
        ptr_prv: The indices of the waypoints being used for the previous path
        path_prv: The previously commanded path
        hs_prv: The currently active halfspace for switching

    Returns:
        path: The updated path to follow
        hs: The updated halfspace for the next switch
        ptr: The updated index pointer
    """
    # Default the outputs to be the inputs
    path = path_prv
    hs   = hs_prv
    pos  = np.array([[state.north, state.east, -state.altitude]]).T
    ptr  = ptr_prv

    if inHalfSpace(pos, hs):
        ptr.increment_pointers(1)

        # Create manager here
        path, hs = construct_line(waypoints, ptr)

    # Output the updated path, halfspace, and index pointer
    return (path, hs, ptr)

def construct_line(waypoints: MsgWaypoints, ptr: WaypointIndices) \
    -> tuple[MsgPath, HalfSpaceParams]:
    """Creates a line and switching halfspace. The halfspace assumes that the aggregate
       path will consist of a series of straight lines.

    The line is created from the previous and current waypoints with halfspace defined for
    switching once the current waypoint is reached.

    Args:
        waypoints: The waypoints from which to construct the path
        ptr: The indices of the waypoints being used for the path

    Returns:
        path: The straight-line path to be followed
        hs: The halfspace for switching to the next waypoint
    """

    # norm
    n = lambda a,b : np.linalg.norm(a-b)

    # Extract the waypoints (w_{i-1}, w_i, w_{i+1})
    (previous, current, next_wp) = extract_waypoints(waypoints = waypoints, ptr = ptr)
    wp                           = previous
    w                            = current
    wn                           = next_wp

    # Construct the path
    path                = MsgPath()         # Path object
    path.plot_updated   = False             # Update plot
    path.line_origin    = previous          # Origin of line
    path.line_direction = (w-wp)/n(w,wp)    # Direction of line
    path.airspeed       = waypoints.airspeed[ptr.current]

    # Construct the halfspace
    hs        = HalfSpaceParams()
    hs.point  = w
    q         = (wn-w)/n(wn,w)
    qp        = (w-wp)/n(w,wp)
    hs.normal = (qp+q)/n(qp,-q)

    return (path, hs)
