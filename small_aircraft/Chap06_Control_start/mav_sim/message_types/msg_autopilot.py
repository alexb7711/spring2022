"""
msg_autopilot
    - messages type for input to the autopilot

part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/5/2019 - RWB
"""


class MsgAutopilot:
    """
    Message class for commanding the autopilot
    """
    def __init__(self) -> None:
        """
        Default parameters to zero
        """
        self.airspeed_command : float = 0.0  # commanded airspeed m/s
        self.course_command   : float = 0.0  # commanded course angle in rad
        self.altitude_command : float = 0.0  # commanded altitude in m
        self.phi_feedforward  : float = 0.0  # feedforward command for roll angle
