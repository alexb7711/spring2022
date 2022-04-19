"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
        12/21 - GND
"""
import mav_sim.parameters.control_parameters as AP
import numpy as np
from mav_sim.chap6.pd_control_with_rate import PDControlWithRate
from mav_sim.chap6.pi_control import PIControl
from mav_sim.chap6.tf_control import TFControl
from mav_sim.message_types.msg_autopilot import MsgAutopilot
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.message_types.msg_state import MsgState

# from mav_sim.tools.transfer_function import TransferFunction
from mav_sim.tools.wrap import saturate, wrap


class Autopilot:
    """Creates an autopilot for controlling the mav to desired values
    """
    def __init__(self, ts_control: float) -> None:
        """Initialize the lateral and longitudinal controllers

        Args:
            ts_control: time step for the control
        """
        # instantiate lateral-directional controllers
        self.roll_from_aileron = PDControlWithRate( # Section 6.1.1.1
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit=np.radians(45))
        self.course_from_roll = PIControl( # Section 6.1.1.2
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit=np.radians(30))

        self.yaw_damper = TFControl( # Note that this is not in the book. This is a replacement for yawDamper function on page 111
                        k=AP.yaw_damper_kr,
                        n0=0.0,
                        n1=1.0,
                        d0=AP.yaw_damper_p_wo,
                        d1=1,
                        Ts=ts_control)

        # instantiate longitudinal controllers
        self.pitch_from_elevator = PDControlWithRate( # Section 6.1.2.1
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        self.altitude_from_pitch = PIControl( # Section 6.1.2.2
                        kp=AP.altitude_kp,
                        ki=AP.altitude_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.airspeed_from_throttle = PIControl( # Section 6.1.2.3
                        kp=AP.airspeed_throttle_kp,
                        ki=AP.airspeed_throttle_ki,
                        Ts=ts_control,
                        limit=1.0)
        self.commanded_state = MsgState()

    def update(self, cmd: MsgAutopilot, state: MsgState) -> tuple[MsgDelta, MsgState]:
        """Given a state and autopilot command, compute the control to the mav

        Args:
            cmd: command to the autopilot
            state: current state of the mav

        Returns:
            delta: low-level flap commands
            commanded_state: the state being commanded
        """

        # lateral autopilot
        chi_c = wrap(cmd.course_command, state.chi)
        phi_c = saturate( # course hold loop, last equation in 6.1.1.2 with addition of feedforward term
            cmd.phi_feedforward + self.course_from_roll.update(chi_c, state.chi), # course_from_roll is a PI_Control
            -np.radians(30), np.radians(30)) # Values for saturation +/- 30 degrees

        delta_a = self.roll_from_aileron.update(phi_c, state.phi, state.p) # Section 6.1.1.1 (last equation),
                                                                           # roll_from_aileron is a PDControlWithRate

        delta_r = self.yaw_damper.update(state.r) # yaw_damper is instance of TFControl, variation on Section 6.1.1.4 equations

        # longitudinal autopilot
        # saturate the altitude command
        altitude_c = saturate(cmd.altitude_command, state.altitude - AP.altitude_zone, # Saturate command altitude to be
                            state.altitude + AP.altitude_zone)                         # close to current altitude

        theta_c = self.altitude_from_pitch.update(altitude_c, state.altitude) # Section 6.1.2.2 (last equation),
                                                                              # altitude_from_pitch is PIControl

        delta_e = self.pitch_from_elevator.update(theta_c, state.theta, state.q) # Section 6.1.2.1 (last equation),
                                                                                 # pitch_from_elevator is a PDControlWithRate

        delta_t = self.airspeed_from_throttle.update(cmd.airspeed_command, state.Va) # Section 6.1.2.3 (last equation),
                                                                                     # airspeed_from_throttle is PIControl
        delta_t = saturate(delta_t, 0.0, 1.0)

        # construct control outputs and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         throttle=delta_t)
        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.course_command
        return delta, self.commanded_state
