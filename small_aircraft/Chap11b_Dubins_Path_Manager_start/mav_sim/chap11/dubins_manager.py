"""Provides an implementation of the dubins path manager for waypoint following as described in
   Chapter 11 Algorithms 9 and 10
"""

import numpy as np
from mav_sim.chap11.dubins_parameters import DubinsParameters
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
    path        = path_prv
    hs          = hs_prv
    ptr         = ptr_prv
    dubins_path = dubins_path_prv

    # If a new set of waypoints have been set
    if waypoints.flag_waypoints_changed is True:
        ## Unset waypoint flag
        waypoints.flag_waypoints_changed = False
        ## Resets the pointers
        ptr = WaypointIndices()
        ## Initialize to first state
        manager_state = 1
        ## Calculate parameters
        c = waypoints.get_waypoint(ptr.previous)
        n = waypoints.get_waypoint(ptr.current)
        ### Calc parameters
        dubins_path = DubinsParameters(p_s=c.ned, chi_s=c.course, p_e=n.ned, chi_e=n.course, R=radius)
        ### Start turn
        path,hs = construct_dubins_circle_start(waypoints=waypoints, ptr=ptr, dubins_path=dubins_path)

    # Store position of vehicle
    pos = np.array([[state.north, state.east, -state.altitude]]).T

    ## Circle Start
    if manager_state == 1:
        ### Start turn
        path,hs = construct_dubins_circle_start(waypoints=waypoints, ptr=ptr, dubins_path=dubins_path)

        ## Wait until behind first half plane k
        nhs = HalfSpaceParams(normal=-dubins_path.n1, point=dubins_path.r1)
        if inHalfSpace(pos, nhs):
            manager_state = 2

    ## Second portion of the start circle
    elif manager_state == 2:
        if inHalfSpace(pos, hs):
            manager_state = 3

    ## Straight line segment
    elif manager_state == 3:
        path,hs = construct_dubins_line(waypoints=waypoints, ptr=ptr, dubins_path=dubins_path)

        if inHalfSpace(pos, hs):
            manager_state = 4

    ## First portion of ending circle
    elif manager_state == 4:
        path,hs = construct_dubins_circle_end(waypoints=waypoints, ptr=ptr, dubins_path=dubins_path)


        ## Wait until behind last half plane
        nhs = HalfSpaceParams(normal=-dubins_path.n3, point=dubins_path.r3)
        if inHalfSpace(pos, nhs):
            manager_state = 5

    ## Second portion of ending circle
    elif manager_state == 5:
        if inHalfSpace(pos, hs):
            # Increment waypoint
            ptr.increment_pointers(waypoints.num_waypoints)
            ## Extract waypoints
            c = waypoints.get_waypoint(ptr.previous)
            n = waypoints.get_waypoint(ptr.current)
            ### Calc parameters
            dubins_path = DubinsParameters(p_s=c.ned, chi_s=c.course, p_e=n.ned, chi_e=n.course)
            # Reset back to start
            manager_state = 1
            print("swap")
    else:
        raise ValueError("Invalid manager state")

    # print("previous: ", waypoints.get_waypoint(ptr.previous).ned)
    # print("current: ", waypoints.get_waypoint(ptr.current).ned)
    # print("next: ", waypoints.get_waypoint(ptr.next).ned)
    # print("pos: ", pos.tolist())

    # print(path)
    # print(hs)
    # print(ptr)
    # print(manager_state)
    #input(dubins_path)

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
    path.orbit_center = dubins_path.center_s
    path.orbit_direction = dubins_path.dir_s
    path.orbit_radius = dubins_path.radius

    # Define the switching halfspace
    hs = HalfSpaceParams(normal=dubins_path.n1, point=dubins_path.r1)

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
    path.line_direction = dubins_path.n1
    path.line_origin    = dubins_path.r1

    # Define the switching halfspace
    hs = HalfSpaceParams(normal=dubins_path.n1, point=dubins_path.r2)

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
    path.orbit_center = dubins_path.center_e
    path.orbit_direction = dubins_path.dir_e
    path.orbit_radius = dubins_path.radius

    # Define the switching halfspace
    hs = HalfSpaceParams(normal=dubins_path.n3, point=dubins_path.r3)

    return (path, hs)
