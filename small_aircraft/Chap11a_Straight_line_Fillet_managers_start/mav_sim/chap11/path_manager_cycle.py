"""path_manager_cycle.py Creates a path manager for repeatedly visiting a number of waypoints
"""
import numpy as np
from mav_sim.chap11.dubins_parameters import DubinsParameters
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_state import MsgState
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from mav_sim.tools.types import NP_MAT


class PathManager:
    """Class for managing the path for following waypoints
    """
    def __init__(self) -> None:
        """Initialize class variables
        """
        # message sent to path follower
        self.path = MsgPath()
        # pointers to previous, current, and next waypoints
        self.ptr_previous = 0   # Previous waypoint
        self.ptr_current = 1    # Waypoint currently navigating towards
        self.ptr_next = 2       # The waypoint that will come after the current
        self.num_waypoints = 0  # Number of waypoints

        # Halfspace determines when to switch to the next waypoint
        self.halfspace_n = np.inf * np.ones((3,1)) # Normal to the halfspace
        self.halfspace_r = np.inf * np.ones((3,1)) # Point on halfspace boundary

        # state of the manager state machine, used for fillet and dubin's paths
        self.manager_state = 1

        # Flag indicating that new waypoints are needed, but have not been received
        self.manager_requests_waypoints = True

        # Storate for dubins parameters
        self.dubins_path = DubinsParameters()

    def update(self, waypoints: MsgWaypoints, radius: float, state: MsgState) -> MsgPath:
        """Provide update to the waypoing manager

        Args:
            waypoints: The waypoints to be followed
            radius: minimum radius circle for the mav
            state: current state of the vehicle

        Returns:
            path: The path to be followed
        """
        if waypoints.num_waypoints == 0:
            self.manager_requests_waypoints = True
        if self.manager_requests_waypoints is True \
                and waypoints.flag_waypoints_changed is True:
            self.manager_requests_waypoints = False

        # Process waypoints
        if waypoints.type == 'straight_line':
            self.line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self.fillet_manager(waypoints, radius, state)
        elif waypoints.type == 'dubins':
            self.dubins_manager(waypoints, radius, state)
        else: # Provide safety if a new type is added without updating the path manager
            raise ValueError('Error in Path Manager: Undefined waypoint type.')
        return self.path

    def line_manager(self, waypoints: MsgWaypoints, state: MsgState) -> None:
        """Update for the line manager

        Update waypoints if needed and check to see if in new halfspace

        Args:
            waypoints: The waypoints to be followed
            state: current state of the vehicle
        """
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed is True:
            waypoints.flag_waypoints_changed = False
            self.num_waypoints = waypoints.num_waypoints
            self.initialize_pointers()
            self.construct_line(waypoints)

        # entered into the half plane separating waypoint segments
        if self.inHalfSpace(mav_pos):
            self.increment_pointers()
            self.construct_line(waypoints)
            # requests new waypoints when reach end of current list
            if self.ptr_current == 0:
                self.manager_requests_waypoints = True

    def fillet_manager(self,waypoints: MsgWaypoints, radius: float, state: MsgState) -> None:
        """Update for the fillet manager

        Update waypoints if needed and check to see if in new halfspace.
        Update state machine accordingly.

        Args:
            waypoints: The waypoints to be followed
            radius: minimum radius circle for the mav
            state: current state of the vehicle
        """
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed is True:
            waypoints.flag_waypoints_changed = False
            self.num_waypoints = waypoints.num_waypoints
            self.initialize_pointers()
            self.construct_fillet_line(waypoints, radius)
            self.manager_state = 1
        # state machine for fillet path
        if self.manager_state == 1:
            # follow straight line path from previous to current
            if self.inHalfSpace(mav_pos):
                # entered into the half plane H1
                self.construct_fillet_circle(waypoints, radius)
                self.manager_state = 3
        elif self.manager_state == 2:
            # follow start orbit until out of H2
            if not self.inHalfSpace(mav_pos):
                self.manager_state = 3
        elif self.manager_state == 3:
            # follow orbit from previous->current to current->next
            if self.inHalfSpace(mav_pos):
                # entered into the half plane H2
                self.increment_pointers()
                self.construct_fillet_line(waypoints, radius)
                self.manager_state = 1
                # requests new waypoints when reach end of current list
                if self.ptr_current == 0:
                    self.manager_requests_waypoints = True

    def dubins_manager(self, waypoints: MsgWaypoints, radius: float, state: MsgState) -> None:
        """Update for the Dubin's path manager

        Update waypoints if needed and check to see if in new halfspace.
        Update state machine accordingly.

        Args:
            waypoints: The waypoints to be followed
            radius: minimum radius circle for the mav
            state: current state of the vehicle
        """
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed is True:
            #waypoints.flag_waypoints_changed = False
            self.num_waypoints = waypoints.num_waypoints
            self.initialize_pointers()
            # dubins path parameters
            self.dubins_path.update(
                ps=waypoints.ned[:, self.ptr_previous:self.ptr_previous+1],
                chis=waypoints.course.item(self.ptr_previous),
                pe=waypoints.ned[:, self.ptr_current:self.ptr_current+1],
                chie=waypoints.course.item(self.ptr_current),
                R=radius)
            self.construct_dubins_circle_start(waypoints, self.dubins_path)
            if self.inHalfSpace(mav_pos):
                self.manager_state = 1
            else:
                self.manager_state = 2
        # state machine for dubins path
        if self.manager_state == 1:
            # follow start orbit until out of H1
            if not self.inHalfSpace(mav_pos):
                self.manager_state = 2
        elif self.manager_state == 2:
            # follow start orbit until cross into H1
            if self.inHalfSpace(mav_pos):
                self.construct_dubins_line(waypoints, self.dubins_path)
                self.manager_state = 3
        elif self.manager_state == 3:
            # follow start orbit until cross into H2
            if self.inHalfSpace(mav_pos):
                self.construct_dubins_circle_end(waypoints, self.dubins_path)
                if self.inHalfSpace(mav_pos):
                    self.manager_state = 4
                else:
                    self.manager_state = 5
        elif self.manager_state == 4:
            # follow start orbit until out of H3
            if not self.inHalfSpace(mav_pos):
                self.manager_state = 5
        elif self.manager_state == 5:
            # follow start orbit until cross into H3
            if self.inHalfSpace(mav_pos):
                self.increment_pointers()
                self.dubins_path.update(
                    waypoints.ned[:, self.ptr_previous:self.ptr_previous+1],
                    waypoints.course.item(self.ptr_previous),
                    waypoints.ned[:, self.ptr_current:self.ptr_current+1],
                    waypoints.course.item(self.ptr_current),
                    radius)
                self.construct_dubins_circle_start(waypoints, self.dubins_path)
                if self.inHalfSpace(mav_pos):
                    self.manager_state = 1
                else:
                    self.manager_state = 2
                # requests new waypoints when reach end of current list
                if self.ptr_current == 0:
                    self.manager_requests_waypoints = True

    def initialize_pointers(self)-> None:
        """Starts the pointers for path following. Value error raised if there are insufficient
        waypoints
        """
        if self.num_waypoints >= 3:
            self.ptr_previous = 0
            self.ptr_current = 1
            self.ptr_next = 2
        else:
            raise ValueError("Path Manager: need at least three waypoints")

    def increment_pointers(self) -> None:
        """Updates the pointers to point at the next waypoint

        A value of IND_OOB is assigned to pointers when the number of waypoints has been exceeded
        """
        self.ptr_previous = self.ptr_current
        if self.ptr_current == self.num_waypoints-2:
            self.ptr_current = self.num_waypoints-1
            self.ptr_next = 0
        elif self.ptr_current == self.num_waypoints-1:
            self.ptr_current = 0
            self.ptr_next = 1
        else:
            self.ptr_current = self.ptr_next
            self.ptr_next = self.ptr_next + 1

    def construct_line(self, waypoints: MsgWaypoints) -> None:
        """Creates a line and updates the halfspace

        The line is created from the previous and current waypoints.
        The halfspace for switching is defined at the current waypoint position with the
        switching surface being defined as the average of the normal to the current and from
        the current to the next.

        Args:
            waypoints: The waypoints from which to construct the line
        """
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next_wp = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        self.path.type = 'line'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        self.path.line_origin = previous
        q_previous = (current - previous)/ np.linalg.norm(current - previous)
        self.path.line_direction = q_previous
        q_next = (next_wp - current) / np.linalg.norm(next_wp - current)
        self.halfspace_n = (q_previous + q_next) / 2
        self.halfspace_n = self.halfspace_n / np.linalg.norm(self.halfspace_n)
        self.halfspace_r = current
        self.path.plot_updated = False

    def construct_fillet_line(self, waypoints: MsgWaypoints, radius: float) -> None:
        """Define the line on a fillet

        Args:
            waypoints: The waypoints to be followed
            radius: minimum radius circle for the mav
        """
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next_wp = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        self.path.type = 'line'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        self.path.line_origin = previous
        q_previous = (current - previous) / np.linalg.norm(current - previous)
        self.path.line_direction = q_previous
        q_next = (next_wp - current) / np.linalg.norm(next_wp - current)
        beta = np.arccos(-q_previous.T @ q_next)
        self.halfspace_n = q_previous
        self.halfspace_r = current - radius / np.tan(beta/2) * q_previous
        self.path.plot_updated = False

    def construct_fillet_circle(self, waypoints: MsgWaypoints, radius: float) -> None:
        """Define the circle on a fillet

        Args:
            waypoints: The waypoints to be followed
            radius: minimum radius circle for the mav
        """
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next_wp = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        self.path.type = 'orbit'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        q_previous = (current - previous) / np.linalg.norm(current - previous)
        q_next = (next_wp - current) / np.linalg.norm(next_wp - current)
        varrho = np.arccos(-q_previous.T @ q_next)
        q_tmp = (q_previous - q_next) / np.linalg.norm(q_previous - q_next)
        self.path.orbit_center = current - radius / np.sin(varrho/2.0) * q_tmp
        self.path.orbit_radius = radius
        if np.sign(q_previous.item(0)*q_next.item(1) - q_previous.item(1)*q_next.item(0)) > 0:
            self.path.orbit_direction = 'CW'
        else:
            self.path.orbit_direction = 'CCW'
        self.halfspace_n = q_next
        self.halfspace_r = current + radius / np.tan(varrho/2.0) * q_next
        self.path.plot_updated = False

    def construct_dubins_circle_start(self, waypoints: MsgWaypoints, dubins_path: DubinsParameters) -> None:
        """ Create the starting orbit for the dubin's path

        Args:
            waypoints: The waypoints to be followed
            dubins_Path: The parameters that make-up the Dubin's path between waypoints
        """
        self.path.type = 'orbit'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        self.path.orbit_radius = dubins_path.radius
        self.path.orbit_center = dubins_path.center_s
        if dubins_path.dir_s == 1:
            self.path.orbit_direction = 'CW'
        else:
            self.path.orbit_direction = 'CCW'
        self.halfspace_n = dubins_path.n1
        self.halfspace_r = dubins_path.r1
        self.path.plot_updated = False

    def construct_dubins_line(self, waypoints: MsgWaypoints, dubins_path: DubinsParameters) -> None:
        """ Create the straight line segment for the dubin's path

        Args:
            waypoints: The waypoints to be followed
            dubins_Path: The parameters that make-up the Dubin's path between waypoints
        """
        self.path.type = 'line'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        self.path.line_origin = dubins_path.r1
        self.path.line_direction = dubins_path.n1
        self.halfspace_n = dubins_path.n1
        self.halfspace_r = dubins_path.r2
        self.path.plot_updated = False

    def construct_dubins_circle_end(self, waypoints: MsgWaypoints, dubins_path: DubinsParameters) -> None:
        """ Create the ending orbit for the dubin's path

        Args:
            waypoints: The waypoints to be followed
            dubins_Path: The parameters that make-up the Dubin's path between waypoints
        """
        self.path.type = 'orbit'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        self.path.orbit_radius = dubins_path.radius
        self.path.orbit_center = dubins_path.center_e
        if dubins_path.dir_e == 1:
            self.path.orbit_direction = 'CW'
        else:
            self.path.orbit_direction = 'CCW'
        self.halfspace_n = dubins_path.n3
        self.halfspace_r = dubins_path.r3
        self.path.plot_updated = False

    def inHalfSpace(self, pos: NP_MAT) -> bool:
        """Determine whether the position is in the next half-space

        Args:
            pos: Position being evaluated

        Returns:
            True if pos is in the halfpace, False otherwise
        """
        if (pos-self.halfspace_r).T @ self.halfspace_n >= 0:
            return True
        return False
