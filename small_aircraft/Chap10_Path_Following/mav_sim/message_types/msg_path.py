"""msg_path: messages type for input to path follower.

part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        3/11/2019 - RWB
        12/2021 - GND
"""
# pylint: disable=too-many-arguments
from typing import Any, Literal

import numpy as np
import numpy.typing as npt


class MsgPath:
    """Defines input to the path follower."""

    __slots__ = [
        "type",
        "plot_updated",
        "airspeed",
        "line_origin",
        "line_direction",
        "orbit_center",
        "orbit_radius",
        "orbit_direction",
    ]

    def __init__(
        self,
        type: Literal["line", "orbit"] = "line", #pylint: disable=redefined-builtin
        plot_updated: bool = False,
        airspeed: float = 25,
        line_origin: npt.NDArray[Any] = np.array([[0.0, 0.0, 0.0]]).T,
        line_direction: npt.NDArray[Any] = np.array([[1.0, 0.0, 0.0]]).T,
        orbit_center: npt.NDArray[Any] = np.array([[0.0, 0.0, 0.0]]).T,
        orbit_radius: float = 50,
        orbit_direction: Literal["CW", "CCW"] = "CW",
    ) -> None:
        # type='line' means straight line following, type='orbit' means orbit following
        self.type: Literal["line", "orbit"] = type

        # flag that indicates that path has been plotted
        self.plot_updated: bool = plot_updated

        #######################################
        ###  Parameters for defining a line ###
        #######################################
        # desired airspeed along the path
        self.airspeed: float = airspeed

        # origin of the straight path line (r)
        self.line_origin = line_origin

        # direction of line -unit vector- (q)
        self.line_direction = line_direction

        #########################################
        ###  Parameters for defining an orbit ###
        #########################################
        # center of the orbit (c)
        self.orbit_center = orbit_center

        # radius of the orbit (rho)
        self.orbit_radius: float = orbit_radius

        # orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise
        self.orbit_direction: Literal["CW", "CCW"] = orbit_direction
