"""
msg_waypoints
    - messages type for input to path manager

part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        3/26/2019 - RWB
        3/31/2020 - RWB
        12/21 - GND
"""
from typing import Any, Literal, Optional, cast

import numpy as np
import numpy.typing as npt
from mav_sim.tools.types import NP_MAT


class Waypoint:
    """Structure for storing information for a single Waypoint
    """
    def __init__(self) -> None:
        """Initializes the object. Does not set values for the class member variables.
        """
        self.ned: NP_MAT # North-east-down position
        self.airspeed: float
        self.course: float
        self.cost: float
        self.parent: int
        self.connect_to_goal: int

class MsgWaypoints:
    """Message containing the waypoints
    """
    def __init__(self) -> None:
        """Create an empty message
        """
        # the first two flags are used for interacting with the path planner
        #
        # flag to indicate waypoints recently changed (set by planner)
        self.flag_waypoints_changed: bool = True
        self.plot_updated: bool = False  # used to plot waypoints

        # type of waypoint following:
        #   - straight line following
        #   - fillets between straight lines
        #   - follow dubins paths
        self.type: Literal['straight_line', 'fillet', 'dubins'] = 'straight_line'

        # current number of valid waypoints in memory
        self.num_waypoints: int = 0

        # [n, e, d] - coordinates of waypoints
        self.ned = np.array([[],[],[]])

        # the airspeed that is commanded along the waypoints
        self.airspeed = np.array([])

        # the desired course at each waypoint (used only for Dubins paths)
        self.course = np.array([])

        # these last three variables are used by the path planner running cost at each node
        self.cost = np.array([])

        # index of the parent to the node
        self.parent = np.array([])

        # can this node connect to the goal?
        self.connect_to_goal = np.array([])

    def add(self, ned: npt.NDArray[Any] =np.array([[0, 0, 0]]).T, airspeed: float=0,
            course: float =np.inf, cost: float=0, parent: int =0, connect_to_goal: int =0) -> bool:
        """Add a waypoint

        Args:
            ned: North-east-down position of the waypoint
            airspeed: Desired airspeed in meters/second
            course: Course angle of the waypoint
            cost: The cost of a particular waypoint
            parent: Parent of waypoint - used for storing a tree structure within the waypoints
            connect_to_goal: Integer indicating whether the waypoint is connected to the goal

        Returns:
            True if point added, False otherwise
        """
        # Check to see if the position is a duplicate of the previous waypoint
        if self.num_waypoints > 0:
            ned_prev = self.get_ned(self.num_waypoints-1) # Previously added waypoint
            mag_diff = np.linalg.norm(ned_prev - ned)
            if mag_diff < .000001:
                print("msg_waypoints::add() new point, ", ned, ", is nearly equal to prev point, ",
                 ned_prev, ", so not adding the waypoint")
                return False

        # Add waypoint
        self.num_waypoints = self.num_waypoints + 1
        self.ned = np.append(self.ned, ned, axis=1)
        self.airspeed = np.append(self.airspeed, airspeed)
        self.course = np.append(self.course, course)
        self.cost = np.append(self.cost, cost)
        self.parent = np.append(self.parent, parent)
        self.connect_to_goal = np.append(self.connect_to_goal, connect_to_goal)
        return True

    def add_waypoint(self, wp: Waypoint, parent: Optional[int] = None, cost: Optional[float] = None) -> bool:
        """ Adds a copy of the waypoint to the set of waypoints

        Args:
            wp: waypoint to be added
            parent: (optional) value for the parent index.
                    A value of "-1" uses the previously added waypoint as the parent
            cost: (optional) value for the cost to arrive at the node

        Returns:
            True if point added, False otherwise
        """
        # Set default waypoint parameters
        if parent is None:
            parent = wp.parent
        elif parent == -1:
            parent = self.num_waypoints-1
        if cost is None:
            cost = wp.cost

        # Add the waypoint
        return self.add(ned=wp.ned, airspeed=wp.airspeed, course=wp.course, \
            cost=cost, parent=parent, connect_to_goal=wp.connect_to_goal)


    def get_ned(self, index: int) -> npt.NDArray[Any]:
        """Quick function for extracting a single point from the waypoints list

        Args:
            index: column index into the ned matrix

        Returns:
            column of the ned matrix
        """
        if index < 0:
            return cast(npt.NDArray[Any], self.ned[:, self.num_waypoints-1:self.num_waypoints])
        return cast(npt.NDArray[Any], self.ned[:, index:index+1]) # Ensures the output is a column vector

    def terminal_direction(self) -> npt.NDArray[Any]:
        """Returns a vector pointing in the direction formed by the final two waypoints
        """
        direction = self.get_ned(self.num_waypoints-1) - self.get_ned(self.num_waypoints-2)
        direction = direction / np.linalg.norm(direction)
        return cast(npt.NDArray[Any], direction)

    def get_waypoint(self, index: int) -> Waypoint:
        """ Returns the waypoint at the given index

        Args:
            index: The index of the waypoint to return

        Returns:
            Waypoint at the index
        """
        waypoint = Waypoint()
        waypoint.ned = self.get_ned(index)
        waypoint.airspeed = self.airspeed.item(index)
        waypoint.course = self.course.item(index)
        waypoint.cost = self.cost.item(index)
        waypoint.parent = self.parent.item(index)
        waypoint.connect_to_goal = self.connect_to_goal.item(index)
        return waypoint
