"""
msg_path
    - messages type for input to path follower

part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        3/11/2019 - RWB
        12/2021 - GND
"""
from typing import Literal

import numpy as np


class MsgPath:
    """Defines input to the path follower
    """
    def __init__(self) -> None:
        # type='line' means straight line following, type='orbit' means orbit following
        self.type: str = 'line' #self.type = 'orbit'

        # flag that indicates that path has been plotted
        self.plot_updated: bool = False

        #######################################
        ###  Parameters for defining a line ###
        #######################################
        # desired airspeed along the path
        self.airspeed: float = 25

        # origin of the straight path line (r)
        self.line_origin = np.array([[0.0, 0.0, 0.0]]).T

        # direction of line -unit vector- (q)
        self.line_direction = np.array([[1.0, 0.0, 0.0]]).T

        #########################################
        ###  Parameters for defining an orbit ###
        #########################################
        # center of the orbit (c)
        self.orbit_center = np.array([[0.0, 0.0, 0.0]]).T

        # radius of the orbit (rho)
        self.orbit_radius: float = 50

        # orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise
        self.orbit_direction: Literal['CW', 'CCW']  = 'CW'
