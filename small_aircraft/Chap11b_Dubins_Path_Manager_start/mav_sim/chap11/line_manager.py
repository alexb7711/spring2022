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
    hs = hs_prv
    ptr = ptr_prv

    # Extract the position
    mav_pos = np.array([[state.north, state.east, -state.altitude]]).T

    # if the waypoints have changed, update the waypoint pointer
    if waypoints.flag_waypoints_changed is True:
        waypoints.flag_waypoints_changed = False
        ptr = WaypointIndices() # Resets the pointers - Line 2 of Algorithm 7

        # Lines 4-7 of Algorithm 7
        (path, hs) = construct_line(waypoints=waypoints, ptr=ptr)

    # entered into the half plane separating waypoint segments
    if inHalfSpace(mav_pos, hs_prv):
        ptr.increment_pointers(waypoints.num_waypoints)
        (path, hs) = construct_line(waypoints=waypoints, ptr=ptr)

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

    # Extract the waypoints (w_{i-1}, w_i, w_{i+1})
    (previous, current, next_wp) = extract_waypoints(waypoints=waypoints, ptr=ptr)

    # Define unit vector from previous waypoint to current waypoint (w_{i-1} to w_i) - Equation (11.2)
    q_previous = (current - previous)/ np.linalg.norm(current - previous)

    # Define unit vector from current waypoint to next waypoint (w_i to w_{i+1}) - Equation (11.2)
    q_next = (next_wp - current) / np.linalg.norm(next_wp - current)

    # Construct the path - Described in beginning of section 11.1
    path = MsgPath()
    path.type = 'line'
    path.plot_updated = False
    path.airspeed = get_airspeed(waypoints, ptr)
    path.line_origin = previous # Either previous or current should work
    path.line_direction = q_previous # Points from previous waypoint to current waypoint

    # Construct the halfspace - page 224 of the book
    hs = HalfSpaceParams()
    normal = (q_previous + q_next)
    mag_normal = np.linalg.norm(q_previous + q_next)
    if mag_normal > 0:
        normal = normal / mag_normal # Equation after (11.2)
    else:
        normal = q_previous # Avoid dividing by zero
    hs.set(normal=normal, point=current)

    return (path, hs)
