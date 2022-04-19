"""
Utilities that can be used throughout a variety of path planners

mavsim_python
    - Beard & McLain, PUP, 2012
    - Last updated:
        4/3/2019 - Brady Moon
        4/11/2019 - RWB
        3/31/2020 - RWB
        4/2022 - GND
"""
from typing import Optional, cast

import numpy as np
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from mav_sim.message_types.msg_world_map import MsgWorldMap
from mav_sim.tools.types import NP_MAT

##==============================================================================
##
def smooth_path(waypoints: MsgWaypoints, world_map: MsgWorldMap) -> MsgWaypoints:
    """smooth the waypoint path - Implementation of Algorithm 13

    Args:
        waypoints: The path to be smoothed
        world_map: definition of the world for planning
    Returns:
        smooth_path: List of smoothed waypoints
    """

    # Local variables
    i           = 0                                     # Start at root node
    j           = 1                                     # Start at next node in path
    smooth_path = MsgWaypoints()                        # Smoothed path
    smooth_path.add_waypoint(waypoints.get_waypoint(i)) # Add root waypoint to smooth path

    # While we have not reached the objective
    for j in range(waypoints.num_waypoints):
        ## Check if we are going out of bound
        if j+1 == waypoints.num_waypoints:
            break

        ## Update waypoints
        ws = smooth_path.get_waypoint(i) # Starting waypoint is set to index i
        wp = waypoints.get_waypoint(j+1) # End waypoint is the is looking one after j

        ## If waypoint j+1 collides with an object, use the previous node
        if not exist_feasible_path(ws.ned, wp.ned, world_map):
            ### Add node j to path
            smooth_path.add_waypoint(waypoints.get_waypoint(j))
            ### Make node j the new starting waypoint
            i = j
            
        ## Increment j to the next waypoint
        j += 1

    # Add last node to waypoint
    smooth_path.add_waypoint(waypoints.get_ned(waypoints.num_waypoints-1))

    return smooth_waypoints

##==============================================================================
##
def find_shortest_path(tree: MsgWaypoints, end_pose: NP_MAT) -> MsgWaypoints:
    """Find the lowest cost path to the end node.

    findShortestPath(...) from Algorithm 12

    Args:
        tree: Current set of waypoints in rrt search tree. Note that
              tree.connect_to_goal.item(i) == 1 iff the node is connected to
              the goal
        end_pose: Desired end pose of the path

    Returns:
        shortest_path: The shortest path
    """
    # Local variables
    candiate_end_nodes = [] # All the waypoints that are connected to the goal
    candidate_paths    = [] # All the paths from root to the goal
    candidate_costs    = [] # Cost to use path

    # Find all the nodes connected to the goal
    for i in range(tree.num_waypoints):
        if tree.get_waypoint(i).connect_to_goal == True:
            candidate_end_nodes.append(tree.get_waypoint(i)

    # For each candidate, calculate the cost of the path
    for i in range(len(candidate_end_nodes)):
        cost = 0               # Initialize cost
        wp   = []              # Array of waypoints
                                       
        ## Initialize wp with end node
        wp.append(candidate_end_nodes[i])
        cost += wp[0].cost

        ## Loop backwards until the root node is found
        while True:
            ### If we are at the beginning break
            if wp[0].parent == False:
                break
                                       
            ### Insert the parent node
            parent_node = tree.get_waypoint(int(wp[0].parent))
            wp.insert(0,parent_node)
                                       
            ### Add cost of waypoint
            cost += wp[0].cost
                                       
        ## Update candidates
        candidate_paths.append(wp.copy())
        candidate_costs.append(cost)
                                       
    # Find shortest path
    idx = index(min(candidate_costs))

    # Create shortest path
    shortest_path = MsgWaypoints()
              
    ## Loop through each waypoint and add it to MsgWaypoints
    for w in (candidate_paths[idx]): shortest_path.add_waypoint(w)
        
    ## Add final node (may not be necessary?)
    shortest_path.add(end_pose, airspeed=waypoints[0].airspeed, parent=i)

    return shortest_path

##==============================================================================
##
def generate_random_configuration(world_map: MsgWorldMap, pd: float) -> NP_MAT:
    """Generates a random pose in the world.

    The generated pose is generated randomly in the 2D plane with the
    down element (altitude) fixed. Note that the city is assumed square
    with world_map.city_width providing the lenght of one side of the square.

    generateRandomConfiguration() routine in Algorithm 12

    Args:
        world_map: definition of the world for planning
        pd: The down position (i.e., altitude) to use for the search

    Returns:
        pose: 3x1 vector with (pn, pe) defined using a random distribution over the width
              of map.
    """
    # Local variables
    pose        = np.array()
    city_width  = world_map.city_width;
    block_width = city_width/world_map.num_city_blocks
    scale       = 1

    # Loop until a new valid position is found
    while (True):
        ## generate a random pose
        ### Generate random step
        D  = np.randint(scale*block_width, scale*city_width)

        ### Generate random north, east position
        pn = D*np.random.rand(1)
        pe = D*np.random.rand(1)

        ### Create position
        pose = np.array([[pn], [pe], [pd]])

    return pose

##==============================================================================
## PASS
def find_closest_configuration(tree: MsgWaypoints, pos_in: NP_MAT) -> tuple[NP_MAT, int, float]:
    """ Returns the closest waypoint in tree to the passed in 3x1 position

        findClosestConfiguration() routine used in Algorithm 12

    Args:
        tree: Current set of waypoints in rrt search tree
        pos_in: The position to be used for comparison

    Returns:
        pos_closest: The 3x1 position that is closest to pos_in
        idx: The index of pos_closest in tree.ned
        dist: The distance from tree.get_ned(idx)
    """
    ## Local variables
    pos_closest = 99999
    idx         = 99999
    dist        = 99999

    # Loop through each waypoint
    for i in range(tree.num_waypoints):
        ## If the distance between the waypoint and the position is less
        ## than pos_closest, update variables and keep searching
        if distance(tree.get_ned(i), pos_in) < dist:
            pos_closest = tree.get_ned(i)
            dist         = distance(pos_closest, pos_in)
            idx          = i

    return (pos_closest, idx, dist)

##==============================================================================
## PASS
def plan_path(start_point: NP_MAT, desired_point: NP_MAT, max_edge_length: float, \
    dist: Optional[float] = None) -> tuple[NP_MAT, float]:
    """ Returns a point along the line formed between the two input points that is
        at most the max edge length away from the starting point

        planPath() routine from Algorithm 12

    Args:
        start_point: Starting point of the new line
        desired_point: The point that is in the direction of the tree extension
        max_edge_length: The maximum edge length allowed
        dist: The distance between start and desired points.
              If None is passed in then the distance is calculated.
              Note that dist must be positive

    Returns:
        new_point: The along the line between start and desired points
        dist_to_new: The distance to the new point
    """
    # Local variables
    new_point   = start_point
    dist_to_new = 0

    # If dist is None type, calculate the distance
    if dist is None:
        dist = distance(start_point, desired_point)

    # Calculate where on the line the point will be
    sigma = 1

    if dist > max_edge_length:
        sigma = max_edge_length/dist

    # Find new point
    new_point = (1-sigma)*start_point + sigma*desired_point

    # Update distance
    dist_to_new = distance(start_point, new_point)

    return (new_point, dist_to_new)

##==============================================================================
##
def exist_feasible_path(start_pose: NP_MAT, end_pose: NP_MAT, world_map: MsgWorldMap) -> bool:
    """ check to see of path from start_pose to end_pose colliding with map

    existFeasiblePath() routine from Algorithm 12

    Args:
        start_pose: starting point on a line
        end_pose: ending point on a line
        world_map: definition of the world for planning

    Returns:
        True => path is feasible, False => path collides with obstacle
    """
    # Local variables
    valid_pose = True
    points     = points_along_path(start_pose, end_pose, 100)

    # Loop through each obstacle
    for i in range(points.shape[1]):
        ## If any point is obstructed, return false
        if not valid_pose or height_above_ground(world_map, column(points, i)) <= 0:
            return False

        ## Get next point
        pn = points[i].item(0)
        pe = points[i].item(1)
        pd = points[i].item(2)

        ## Loop through each building
        for j in range(len(world_map.building_east)):
            ### Building dimensions
            b_pe   = world_map.building_east.item(j)  + world_map.city_width/2
            b_ne   = world_map.building_east.item(j)  - world_map.city_width/2
            b_pn   = world_map.building_north.item(j) + world_map.city_width/2
            b_nn   = world_map.building_north.item(j) - world_map.city_width/2

            ### Check for collision
            if (pn >= b_nn or pn <= b_pn) and \
               (pe >= b_ne or pe <= b_pe):
                # print(b_pe)
                # print(b_ne)
                # print(b_pn)
                # print(b_nn)
                # print(pn)
                # print(pe)
                # input(pd)
                valid_pose = False
                break

    return True

##==============================================================================
##
def distance(start_pose: NP_MAT, end_pose: NP_MAT) -> float:
    """compute distance between start and end pose

    Args:
        start_pose: pose one
        end_pose: pose two

    Returns:
        d: distance between start and end poses
    """
    d = np.linalg.norm(start_pose - end_pose)
    return float(d)

##==============================================================================
##
def height_above_ground(world_map: MsgWorldMap, point: NP_MAT) -> float:
    """find the altitude of point above ground level

    Args:
        world_map: definition of the world for planning
        point: location to calculate height

    Returns:
        h_agl: Height at the position (A negative value implies a collision)
    """
    point_height = -point.item(2)
    tmp          = np.abs(point.item(0)-world_map.building_north)
    d_n          = np.min(tmp)
    idx_n        = np.argmin(tmp)
    tmp          = np.abs(point.item(1)-world_map.building_east)
    d_e          = np.min(tmp)
    idx_e        = np.argmin(tmp)
                                       
    if (d_n<world_map.building_width) and (d_e<world_map.building_width):
        map_height = world_map.building_height[idx_n, idx_e]
    else:
        map_height = 0
                                       
    h_agl = point_height - map_height
    return float(h_agl)

##==============================================================================
##
def points_along_path(start_pose: NP_MAT, end_pose: NP_MAT, N: int) -> NP_MAT:
    """Returns N points along path defined by the starting and ending poses

    Args:
        start_pose: starting point on a line
        end_pose: ending point on a line
        N: Number of points to return

    Returns:
        points: Points along line between start and end pose
    """
    points    = start_pose
    q: NP_MAT = (end_pose - start_pose)
    L         = np.linalg.norm(q)
    q         = q / L
    w         = start_pose
                                       
    for _ in range(1, N):
        w = w + (L / N) * q
        points = np.append(points, w, axis=1)
                                       
    return points

##==============================================================================
##
def column(A: NP_MAT, i: int) -> NP_MAT:
    """Extracts the ith column of A and return column vector
    """
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return cast(NP_MAT, col)
