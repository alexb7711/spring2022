"""transforms.py implements the basic transforms from chapter 2

There are seven frames defined in chapter two:
    Iniertial frame (i)         - Fixed to earth
    Vehicle frame (v)           - Parellel to (i), but fixed to uav
    Vehicle 1 frame (v1)        - Rotate yaw (psi) about k^v
    Vehicle 2 frame (v2)        - Rotate pitch (theta) about j^{v1}
    Body frame (b)              - Rotate roll (phi) about i^{v2}
    Stability frame (s)         - Rotate left hand alpha about j^b
    Wind frame (w)              - Rotate beta about k^s

Functions:
    This module contains a number of functions for calculating basic rotations
        rot_x: calculate elementary rotation matrix about x-axis
        rot_y: calculate elementary rotation matrix about y-axis
        rot_z: calculate elementary rotation matrix about z-axis

    This module also contains a number of functions for calculating basic rotation matrices
        rot_v_to_v1: calculates the rotation from frame v to v1
        rot_v1_to_v2: calculates the rotation from frame v1 to v2
        rot_v2_to_b: calculates the rotation from v2 to body frame
        rot_b_to_s: calculates the rotation from body to stability frame
        rot_s_to_w: calculates the rotation from stability to wind frame
        rot_v_to_b: calculates the rotation from vehicle to body frame
        rot_b_to_v: calculates the rotation from body frame to vehicle frame

    This module also computes a number of functions for calculating the transforms of points
        trans_i_to_v: transforms a point from inertial frame to the vehicle frame
        trans_v_to_i: transforms a point from vehicle frame to the inertial frame
        trans_i_to_b: transforms a point from inertial frame to the body frame
        trans_b_to_i: transforms a point from the body frame to the inertial frame
"""

from typing import cast

import numpy as np
from mav_sim.tools.types import Points, Pose, RotMat


# Elementary rotation matrices
def rot_x(angle: float) -> RotMat:
    """Elementary rotation about x-axis

    Args:
        angle: angle of rotation

    Returns:
        rot: rotation matrix about x-axis
    """
    # Calculate angles
    c = np.cos(angle)
    s = np.sin(angle)

    # Calculate rotaiton matrix
    rot = np.array([    [1.,  0.,  0.],
                        [0.,  c,   s],
                        [0., -s,   c] ])
    return cast(RotMat, rot)

def rot_y(angle: float) -> RotMat:
    """Elementary rotation about y-axis

    Args:
        angle: angle of rotation

    Returns:
        rot: rotation matrix about y-axis
    """
    # Calculate angles
    c = np.cos(angle)
    s = np.sin(angle)

    # Calculate rotaiton matrix
    rot = np.array([    [c,   0., -s],
                        [0.,  1.,  0.],
                        [s,   0.,  c] ])
    return cast(RotMat, rot)

def rot_z(angle: float) -> RotMat:
    """Elementary rotation about z-axis

    Args:
        angle: angle of rotation

    Returns:
        rot: rotation matrix about z-axis
    """
    # Calculate angles
    c = np.cos(angle)
    s = np.sin(angle)

    # Calculate rotaiton matrix
    rot = np.array([    [c,   s,  0.],
                        [-s,  c,  0.],
                        [0., 0.,  1.] ])
    return cast(RotMat, rot)

# Rotation matrices between concentric frames
def rot_v_to_v1(psi: float) -> RotMat:
    """
    calculates the rotation from frame v to v1

    Args:
        psi: yaw angle about k^v axis

    Returns:
        rot: Rotation matrix from frame v to v1
    """
    print('Gets here')
    rot = rot_z(psi)
    return rot

def rot_v1_to_v2(theta: float) -> RotMat:
    """
    calculates the rotation from frame v1 to v2

    Args:
        theta: pitch angle about j^{v1} axis

    Returns:
        rot: Rotation matrix from frame v1 to v2
    """
    rot = rot_y(theta)
    return rot

def rot_v2_to_b(phi: float) -> RotMat:
    """
    calculates the rotation from frame v2 to body

    Args:
        phi: roll angle about i^{v2} axis

    Returns:
        rot: Rotation matrix from frame v2 to b
    """
    rot = rot_x(phi)
    return rot

def rot_b_to_s(alpha: float) -> RotMat:
    """
    calculates the rotation from body frame to stability frame

    Args:
        alpha: left-hand rotation angle about j^b to align i^s with projection of V_a onto i^b-k^b plane

    Returns:
        rot: Rotation matrix from body frame to stability frame
    """
    rot = rot_y(-alpha)
    return rot

def rot_s_to_w(beta: float) -> RotMat:
    """
    calculates the rotation from stability frame to wind frame

    Args:
        beta: rotation about k^s axis to align i^w with V_a

    Returns:
        rot: Rotation matrix from body frame to stability frame
    """
    rot = rot_z(beta)
    return rot

def rot_v_to_b(psi: float, theta: float, phi: float) -> RotMat:
    """
    calculates the rotation matrix from vehicle frame to body frame

    Args:
        psi: yaw angle about k^v axis
        theta: pitch angle about j^{v1} axis
        phi: roll angle about i^{v2} axis

    Returns:
        rot: Rotation matrix from vehicle frame to body frame
    """
    # Calculate the trig functions
    cpsi = np.cos(psi)
    spsi = np.sin(psi)
    cth = np.cos(theta)
    sth = np.sin(theta)
    cphi = np.cos(phi)
    sphi = np.sin(phi)

    # Calculate the rotation matrix
    rot = np.array([    [cth*cpsi,                  cth*spsi,                   -sth],
                        [sphi*sth*cpsi-cphi*spsi,   sphi*sth*spsi+cphi*cpsi,    sphi*cth],
                        [cpsi*sth*cphi+sphi*spsi,   cphi*sth*spsi-sphi*cpsi,    cphi*cth]])
    return cast(RotMat, rot)

def rot_b_to_v(psi: float, theta: float, phi: float) -> RotMat:
    """
    calculates the rotation matrix from body frame to vehicle frame

    Args:
        psi: yaw angle about k^v axis
        theta: pitch angle about j^{v1} axis
        phi: roll angle about i^{v2} axis

    Returns:
        rot: Rotation matrix from body frame to vehicle frame
    """
    # Calculate the rotation from v to b
    R_v_to_b = rot_v_to_b(psi, theta, phi)

    # rot_b_to_v is the inverse / transpose of the previous matrix
    rot = R_v_to_b.T
    return rot

# Calculating the transforms of points
def trans_i_to_v(pose: Pose, p_i: Points) -> Points:
    """
    Transforms a point from inertial frame to the vehicle frame

    Args:
        p_i: Point represented in the inertial frame

    Returns:
        p_v: Point represented in the vehicle frame
    """
    p_v = np.array([[p_i[0,0] - pose.north ],
                    [p_i[1,0] - pose.east  ],
                    [p_i[2,0] + pose.altitude]]) # Note "+" is due to altitude = -down
    return cast(Points, p_v)

def trans_v_to_i(pose: Pose, p_v: Points) -> Points:
    """
    Transforms a point from vehicle frame to the inertial frame

    Args:
        p_v: Point represented in the vehicle frame

    Returns:
        p_i: Point represented in the inertial frame
    """
    p_i = np.array([[p_v[0,0] + pose.north ],
                    [p_v[1,0] + pose.east  ],
                    [p_v[2,0] - pose.altitude]]) # Note "-" is due to altitude = -down
    return cast(Points, p_i)

def trans_i_to_b(pose: Pose, p_i: Points) -> Points:
    """
    Transforms a point from inertial frame to the body frame

    Args:
        p_i: Point represented in the inertial frame

    Returns:
        p_b: Point represented in the body frame
    """

    # Transform the point to the vehicle frame
    p_v = trans_i_to_v(pose, p_i)

    # Calculate the transform from the vehicle frame to the body frame
    R_v_to_b = rot_v_to_b(pose.psi, pose.theta, pose.phi)

    # Transform the point from the vehicle frame to the body frame
    p_b = R_v_to_b @ p_v
    return cast(Points, p_b)

def trans_b_to_i(pose: Pose, p_b: Points) -> Points:
    """
    Transforms a point from body frame to the inertial frame

    Args:
        p_b: Point represented in the body frame

    Returns:
        p_i: Point represented in the inertial frame
    """

    # Calculate the transform from the body frame to the vehicle frame
    R_b_to_v = rot_b_to_v(pose.psi, pose.theta, pose.phi)

    # Transform the point to the vehicle frame
    p_v = R_b_to_v @ p_b

    # Transform the point from the vehicle frame to the inertial frame
    return trans_v_to_i(pose, cast(Points, p_v) )
