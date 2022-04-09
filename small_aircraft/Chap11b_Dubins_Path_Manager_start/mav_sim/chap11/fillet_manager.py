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

    """Update for the fillet manager.
       Updates state machine if the MAV enters into the next halfspace.

    Args:
        state: current state of the vehicle
        waypoints: The waypoints to be followed
        ptr_prv: The indices that were being used on the previous iteration (i.e., current waypoint
                 inidices being followed when manager called)
        hs_prv: The previous halfspace being looked for (i.e., the current halfspace when manager called)
        radius: minimum radius circle for the mav
        manager_state: Integer state of the manager
                Value of 1 corresponds to following the straight line path
                Value of 2 corresponds to following the arc between straight lines

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

    # Extract the position
    mav_pos = np.array([[state.north, state.east, -state.altitude]]).T

    # if the waypoints have changed, update the waypoint pointer
    # Lines 8-11 of Algorithm 8
    if waypoints.flag_waypoints_changed is True:
        waypoints.flag_waypoints_changed = False
        ptr = WaypointIndices() # Resets the pointers
        (path, hs) = \
                construct_fillet_line(waypoints=waypoints, ptr=ptr, radius=radius)
        manager_state = 1

    # state machine for fillet path
    if manager_state == 1:
        # follow straight line path from previous to current
        # Lines 12-14 of Algorithm 8
        if inHalfSpace(mav_pos, hs_prv):
            # Update to state 2 as described in Algorithm 8 line 13
            manager_state = 2

            # entered into the half plane H1 - construct the orbit
            # Lines 17-20 of Algorithm 8
            (path, hs) = \
                construct_fillet_circle(waypoints=waypoints, ptr=ptr, radius=radius)

    elif manager_state == 2:
        # follow orbit until in H2
        # Lines 21-24 of Algorithm 8
        if inHalfSpace(mav_pos, hs_prv):
            # Increment waypoint - line 22-23 of Algorithm 8
            ptr.increment_pointers(waypoints.num_waypoints)
            manager_state = 1

            # Get next line - lines 9-11 of Algorithm 8
            (path, hs) = \
                construct_fillet_line(waypoints=waypoints, ptr=ptr, radius=radius)

    else:
        raise ValueError("Invalid manager state")

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

    # Construct the halfspace - page 227 of the book
    varrho = np.arccos(-q_previous.T @ q_next) # Equation (11.3)
    if abs(np.tan(varrho/2)) > EPSILON:
        r1 = current - radius / np.tan(varrho/2) * q_previous # r_1 as defined on page 227
        normal=q_previous
    else: # Special case - moving back along the same line,
          # Allow the UAV to proceed to the waypoint
        r1 = current
        normal = q_previous
    hs = HalfSpaceParams() # H_1 defined on page 227
    hs.set(normal=normal, point=r1)

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

    # Define unit vector from previous waypoint to current waypoint (w_{i-1} to w_i) - Equation (11.2)
    q_previous = (current - previous)/ np.linalg.norm(current - previous)

    # Define unit vector from current waypoint to next waypoint (w_i to w_{i+1}) - Equation (11.2)
    q_next = (next_wp - current) / np.linalg.norm(next_wp - current)

    # Compute the normal vector for calculating center while considering the singularity
    difference = (q_previous - q_next)
    mag = np.linalg.norm(difference)
    if mag > 0: # Special case: stright line paths
        normal =  difference / mag # Normal vector as defined on page 226
    else: # Special case - moving in a straight line
        J = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]]) # A rotation by pi/2 about z-axis
        normal = J@q_previous

    # Calculate the orbit center as defined on page 226
    varrho = np.arccos(-q_previous.T @ q_next) # Equation (11.3)
    is_reverse = abs(np.sin(varrho/2)) < EPSILON
    if is_reverse: # Special case - moving back along the same line
        center = current - radius * normal # place circle to the side
    else:
        center = current - radius / np.sin(varrho/2.) * normal # Main equation on page 226

    # Calculate the orbit direction - right-hand rule with cross product.
    # Positive z => CW, negative z => CCW (Line 19 in Algorithm 8)
    path = MsgPath()
    if np.sign(q_previous.item(0)*q_next.item(1) - q_previous.item(1)*q_next.item(0)) > 0:
        path.orbit_direction = 'CW'
    else:
        path.orbit_direction = 'CCW'

    # Construct the orbit
    path.plot_updated = False
    path.type = 'orbit'
    path.airspeed = get_airspeed(waypoints, ptr)
    path.orbit_radius = radius
    path.orbit_center = center

    # Define the switching halfspace - page 227 of the book
    hs = HalfSpaceParams()
    if is_reverse: # Switching halfspace that allows vehicle to move to the current point
        normal = q_previous
        point = current
    else: # H_2 defined on page 227
        r2 = current + radius / np.tan(varrho/2.0) * q_next # r_2 defined on page 227
        normal = q_next
        point = r2

    hs.set(normal=normal, point=point )

    return (path, hs)
