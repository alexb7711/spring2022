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

    """
    Update for the fillet manager

    Update waypoints if needed and check to see if in new halfspace.
    Update state machine accordingly.

    Args:
        waypoints: The waypoints to be followed
        radius: minimum radius circle for the mav
        state: current state of the vehicle
    """
    # Default the outputs to be the inputs
    path = path_prv
    hs   = hs_prv
    pos  = np.array([[state.north, state.east, -state.altitude]]).T
    ptr  = ptr_prv

    # Checks
    ## If waypoints are new
    if waypoints.flag_waypoints_changed:
        ## Unset waypoint flag
        waypoints.flag_waypoints_changed = False
        ## Create new waypoint indices
        ptr                              = WaypointIndices()
        ## Set state to fillet line
        manager_state = 1

    ## Check if in half space. Change to fillet circle
    if inHalfSpace(pos, hs):
        manager_state = 2

        ## Indrement pointers
        ptr.increment_pointers(waypoints.num_waypoints)
 
    ## Go back to line fillet
    else:
        manager_state = 1

    # Update
    ## If state is asking for a line
    if manager_state == 1:
        path, hs = construct_fillet_line(waypoints, ptr, radius)
    ## Else if the state is asking for a circle
    if manager_state == 2:
        path, hs = construct_fillet_circle(waypoints, ptr, radius)

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
    # norm
    n = lambda a,b=None : np.linalg.norm(a) if b is None else np.linalg.norm(a-b)

    # Extract the waypoints (w_{i-1}, w_i, w_{i+1})
    (previous, current, next_wp) = extract_waypoints(waypoints = waypoints, ptr = ptr)
    wp                           = previous
    w                            = current
    wn                           = next_wp

    # Construct the path
    path              = MsgPath()
    path.plot_updated = False

    q                    = (wn-w)/n(wn,w)                # q_i
    qp                   = (w-wp)/n(w,wp)                # q_{i-1}
    phi                  = np.arccos(-qp.T@q)            # angle

    # If straight line singularity case
    #  if (all(np.isclose(qp,q))):
        #  J = np.array([[0., 1., 0.],
                      #  [-1., 0., 0.],
                      #  [0., 0., 0.]]) # A rotation by pi/2 about z-axis
        #  path.orbit_center = w + J@qp*(radius/np.sin(phi/2))
        #  path.orbit_direction = 1
        #  path.orbit_radius = radius
    #  else:
    path.line_direction = (w-wp)/n(w,wp) # Direction of line
    path.line_origin    = wp             # Origin of line

    # Construct the halfspace
    hs        = HalfSpaceParams()
    # If singularity
    if (all(np.isclose([np.tan(phi/2)],[0]))):
        hs.point  = w                            # Half plane point
    # Else behave normally
    else:
        hs.point  = w - (radius/np.tan(phi/2))*qp # Half plane point

    hs.normal = qp/n(qp)                         # Normal vector

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
    # norm
    n = lambda a,b=None : np.linalg.norm(a) if b is None else np.linalg.norm(a-b)

    # Extract the waypoints (w_{i-1}, w_i, w_{i+1})
    (previous, current, next_wp) = extract_waypoints(waypoints=waypoints, ptr=ptr)
    wp                           = previous
    w                            = current
    wn                           = next_wp

    q                    = (wn-w)/n(wn,w) # q_i
    qp                   = (w-wp)/n(w,wp) # q_{i-1}
    phi                  = radius         # angle

    # Construct the path
    path                 = MsgPath()
    path.plot_updated    = False

    # If straight line singularity case
    if (all(np.isclose(qp,q))):
        J = np.array([[0., 1., 0.],
                      [-1., 0., 0.],
                      [0., 0., 0.]]) # A rotation by pi/2 about z-axis
        path.orbit_center = w + J@qp*(radius/np.sin(phi/2))
        path.orbit_direction = 1
        path.orbit_radius = radius
    # Else if the next waypoint is directly behind the vehicle
    elif (all(np.isclose([np.sin(phi/2)],[0]))):
        path.orbit_center    = w                                         # Calculate orbit center
        rot_dir              = np.cross(qp.T, q.T).item(2)               # Variable to calculate orbit direction
        path.orbit_direction = np.sign(rot_dir)                          # Orbit direction
        path.orbit_radius    = radius                                    # Orbit radius
    # Else calculate normal route
    else:
        path.orbit_center    = w - (radius/np.sin(phi/2))*(qp-q)/n(qp,q) # Calculate orbit center
        rot_dir              = np.cross(qp.T, q.T).item(2)               # Variable to calculate orbit direction
        path.orbit_direction = np.sign(rot_dir)                          # Orbit direction
        path.orbit_radius    = radius                                    # Orbit radius

    # Construct the path
    path = MsgPath()
 
    # Define the switching halfspace
    hs = HalfSpaceParams()
    # If singularity
    if (all(np.isclose([np.tan(phi/2)],[0]))):
        hs.point  = w                             # Half plane point
    # Else behave normally
    else:
        hs.point  = w + (radius/np.tan(phi/2))*qp # Half plane point

    hs.normal = q/n(q)                            # Normal vector

    return (path, hs)
