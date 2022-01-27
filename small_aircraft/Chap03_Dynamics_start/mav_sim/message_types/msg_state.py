"""
msgState
    - messages type for state, that will be passed between blocks in the architecture

part of mavPySim
    - Beard & McLain, PUP, 2012
    - Update history:
        1/9/2019 - RWB
        12/21 - GND
"""


class MsgState:
    """Defines the state of the aircraft
    """
    def __init__(self) -> None:
        self.north: float = 0.      # inertial north position in meters
        self.east: float = 0.      # inertial east position in meters
        self.altitude: float = 0.       # inertial altitude in meters
        self.phi: float = 0.     # roll angle in radians
        self.theta: float = 0.   # pitch angle in radians
        self.psi: float = 0.     # yaw angle in radians
        self.Va: float = 0.      # airspeed in meters/sec
        self.alpha: float = 0.   # angle of attack in radians
        self.beta: float = 0.    # sideslip angle in radians
        self.p: float = 0.       # roll rate in radians/sec
        self.q: float = 0.       # pitch rate in radians/sec
        self.r: float = 0.       # yaw rate in radians/sec
        self.Vg: float = 0.      # groundspeed in meters/sec
        self.gamma: float = 0.   # flight path angle in radians
        self.chi: float = 0.     # course angle in radians
        self.wn: float = 0.      # inertial windspeed in north direction in meters/sec
        self.we: float = 0.      # inertial windspeed in east direction in meters/sec
        self.bx: float = 0.      # gyro bias along roll axis in radians/sec
        self.by: float = 0.      # gyro bias along pitch axis in radians/sec
        self.bz: float = 0.      # gyro bias along yaw axis in radians/sec
