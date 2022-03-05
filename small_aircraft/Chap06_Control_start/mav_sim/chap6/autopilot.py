"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
        12/21 - GND
"""
import mav_sim.parameters.control_parameters as AP
import numpy as np

from mav_sim.chap6.pd_control_with_rate  import PDControlWithRate
from mav_sim.chap6.pi_control            import PIControl
from mav_sim.chap6.tf_control            import TFControl
from mav_sim.message_types.msg_autopilot import MsgAutopilot
from mav_sim.message_types.msg_delta     import MsgDelta
from mav_sim.message_types.msg_state     import MsgState

# from mav_sim.tools.transfer_function import TransferFunction
from mav_sim.tools.wrap import saturate, wrap


class Autopilot:
    """
    Creates an autopilot for controlling the mav to desired values
    """

    ##===========================================================================
    #
    def __init__(self, ts_control: float) -> None:
        """
        Initialize the lateral and longitudinal controllers

        Args:
            ts_control: time step for the control
        """
        # instantiate lateral-directional controllers
        self.roll_from_aileron = PDControlWithRate(kp=AP.roll_kp,
                                                   kd=AP.roll_kd,
                                                   limit=np.radians(45))

        self.course_from_roll  = PIControl(kp=AP.course_kp,
                                           ki=AP.course_ki,
                                           Ts=ts_control,
                                           limit=np.radians(30))

        self.yaw_damper        = TFControl(k=AP.yaw_damper_kr,
                                           n0=0,
                                           n1=1,
                                           d0=AP.yaw_damper_p_wo,
                                           d1=1,
                                           Ts=ts_control)

        # instantiate longitudinal controllers
        self.pitch_from_elevator    = PDControlWithRate(kp=AP.pitch_kp,
                                                        kd=AP.pitch_kd,
                                                        limit=np.radians(45))

        self.altitude_from_pitch    = PIControl(kp=AP.altitude_kp,
                                                ki=AP.altitude_ki,
                                                Ts=ts_control,
                                                limit=np.radians(30))

        self.airspeed_from_throttle = PIControl(kp=AP.airspeed_throttle_kp,
                                                ki=AP.airspeed_throttle_ki,
                                                Ts=ts_control,
                                                limit=1)

        self.commanded_state = MsgState()

        return

    ##===========================================================================
    #
    def update(self, cmd: MsgAutopilot, state: MsgState) -> tuple[MsgDelta, MsgState]:
        """
        Given a state and autopilot command, compute the control to the mav

        Args:
            cmd: command to the autopilot
            state: current state of the mav

        Returns:
            delta: low-level flap commands
            commanded_state: the state being commanded
        """

        ##-----------------------------------------------------------------------
        # lateral autopilot

        ## Fix chi: bounded between (-pi, pi)
        chi_ = wrap(cmd.course_command, state.chi)

        ## Fix altitude: Don't let the altitude command more than
        ## +- altitude_zone
        alt_ = saturate(cmd.altitude_command, state.altitude-AP.altitude_zone,
                                              state.altitude+AP.altitude_zone)
        ## Calculate control commands
        ### Roll control command
        phi_c   = cmd.phi_feedforward + \
                  self.course_from_roll.update(chi_, state.chi)
        phi_c   = saturate(phi_c, -np.radians(30), np.radians(30))

        ### Pitch control command
        theta_c = self.altitude_from_pitch.update(alt_, state.altitude)

        ## Calculate unsaturated control
        delta_a = self.roll_from_aileron.update(phi_c, state.phi, state.p)
        delta_r = self.yaw_damper.update(state.r)

        ##-----------------------------------------------------------------------
        # longitudinal autopilot

        ## Calculate unsaturated control
        delta_e = self.pitch_from_elevator.update(theta_c,
                                                  state.theta,
                                                  state.q)

        delta_t = self.airspeed_from_throttle.update(cmd.airspeed_command,
                                                     state.Va)

        ## Saturate output
        delta_t = saturate(delta_t, 0.0, 1.0)

        ##-----------------------------------------------------------------------
        # construct control outputs and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         throttle=delta_t)

        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va       = cmd.airspeed_command
        self.commanded_state.phi      = phi_c
        self.commanded_state.theta    = theta_c
        self.commanded_state.chi      = cmd.course_command

        return delta, self.commanded_state
