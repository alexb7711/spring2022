"""
msg_delta
    - messages type for input to the aircraft

part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/27/2020 - RWB
        12/2021 - GND
"""
from typing import Any

import numpy as np
import numpy.typing as npt


class MsgDelta:
    """Message inputs for the aircraft
    """
    def __init__(self,
                 elevator: float =0.0,
                 aileron: float =0.0,
                 rudder: float =0.0,
                 throttle: float =0.5) -> None:
        """Set the commands to default values
        """
        self.elevator: float = elevator  # elevator command
        self.aileron: float = aileron  # aileron command
        self.rudder: float = rudder  # rudder command
        self.throttle: float = throttle  # throttle command

    def to_array(self) -> npt.NDArray[Any]:
        """Convert the command to a numpy array
        """
        return np.array([[self.elevator],
                         [self.aileron],
                         [self.rudder],
                         [self.throttle]])

    def from_array(self, u: npt.NDArray[Any]) -> None:
        """Extract the commands from a numpy array
        """
        self.elevator = u.item(0)
        self.aileron = u.item(1)
        self.rudder = u.item(2)
        self.throttle = u.item(3)

    def print(self) -> None:
        """Print the commands to the console
        """
        print('elevator=', self.elevator,
              'aileron=', self.aileron,
              'rudder=', self.rudder,
              'throttle=', self.throttle)
