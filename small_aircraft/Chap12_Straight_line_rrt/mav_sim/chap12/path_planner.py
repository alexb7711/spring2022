"""
path planner for mavsim_python

mavsim_python
    - Beard & McLain, PUP, 2012
    - Last updated:
        4/3/2019 - BGM
"""
from typing import Optional, cast

import mav_sim.parameters.planner_parameters as PLAN
import numpy as np
from mav_sim.chap12.rrt_straight_line import RRTStraightLine
from mav_sim.message_types.msg_state import MsgState
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from mav_sim.message_types.msg_world_map import MsgWorldMap
from mav_sim.tools.types import NP_MAT


class PlannerType:
    """Defines the planner to be used
    """
    simple_fillet = 1 # Fixed waypoint path connected by fillet motion primitives
    simple_dubins = 2 # Fixed dubins waypoint path
    rrt_straight = 3 # Straight-line rrt. Waypoints defined as fillets, but fillet constraints not considered
    rrt_dubins = 4 # plan path through city using dubins RRT


class PathPlanner:
    """Path planning object
    """
    def __init__(self) -> None:
        """Initilize path planner
        """
        # waypoints definition
        self.waypoints = MsgWaypoints()
        self.rrt_straight_line = RRTStraightLine()
        self.radius = PLAN.R_min

    def update(self, world_map: MsgWorldMap, state: MsgState, \
        end_pose: Optional[NP_MAT] = None, planner_type: int = PlannerType.rrt_straight, \
        desired_airspeed: float = 25., desired_altitude: float = 100.) -> MsgWaypoints:
        """Creates a plan based upon the current map, mav state, and minimum radius

        Args:
            world_map: definition of the world for planning
            state: latest mav state
            end_pose: 3x1 or 4x1 vector giving the desired end location.
                        3x1 for straight line
                        4x1 for dubins where final element is the end orientation
            desired_airspeed: Airspeed to assign to the waypoints
            desired_altitude: The alitude that the aircraft should maintain

        Returns:
            waypoints: resulting plan
        """
        # Create the start pose
        start_pose = np.array([[state.north], [state.east],
                               [-desired_altitude], [state.chi]])

        # Create the default end position
        if end_pose is None:
            # Move to top right corder
            if np.linalg.norm(start_pose[0:2]) < world_map.city_width / 2:
                end_pose = np.array([[world_map.city_width], [world_map.city_width],
                            [-desired_altitude]])
            else:  # or to the bottom-left corner of world_map
                end_pose = np.array([[0], [0], [-desired_altitude]])

            # Add in a final orientation for the dubins path
            if planner_type==PlannerType.rrt_dubins:
                end_pose = np.append(end_pose, state.chi)

        print('planning...')
        if planner_type == PlannerType.simple_fillet:
            self.waypoints.type = 'fillet'
            self.waypoints.add(np.array([[0, 0, -100]]).T, desired_airspeed, np.inf, np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 0, -100]]).T, desired_airspeed, np.inf, np.inf, 0, 0)
            self.waypoints.add(np.array([[0, 1000, -100]]).T, desired_airspeed, np.inf, np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 1000, -100]]).T, desired_airspeed, np.inf, np.inf, 0, 0)

        elif planner_type == PlannerType.simple_dubins:
            self.waypoints.type = 'dubins'
            self.waypoints.add(np.array([[0, 0, -100]]).T, desired_airspeed, np.radians(0), np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 0, -100]]).T, desired_airspeed, np.radians(45), np.inf, 0, 0)
            self.waypoints.add(np.array([[0, 1000, -100]]).T, desired_airspeed, np.radians(45), np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 1000, -100]]).T, desired_airspeed, np.radians(-135), np.inf, 0, 0)

        elif planner_type == PlannerType.rrt_straight:
            self.waypoints = self.rrt_straight_line.update(start_pose[0:3], cast(NP_MAT, end_pose),
                                                           desired_airspeed, world_map)

        else:
            raise ValueError("Error in Path Planner: Undefined planner type.")
        self.waypoints.plot_updated = False
        print('...done planning.')
        return self.waypoints
