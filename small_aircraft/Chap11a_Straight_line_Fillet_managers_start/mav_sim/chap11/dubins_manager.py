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
    """Update for the Dubin's path manager

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
    dubins_path = dubins_path_prv

    return (path, hs, ptr, manager_state, dubins_path)
