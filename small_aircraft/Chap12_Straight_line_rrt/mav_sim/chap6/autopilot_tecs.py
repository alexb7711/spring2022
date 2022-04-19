"""
autopilot block for mavsim_python - Total Energy Control System
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/14/2020 - RWB
"""

import mav_sim.parameters.aerosonde_parameters as MAV
import mav_sim.parameters.control_parameters as AP
import numpy as np
from mav_sim.chap6.pd_control_with_rate import PDControlWithRate
from mav_sim.chap6.pi_control import PIControl
from mav_sim.message_types.msg_autopilot import MsgAutopilot
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.message_types.msg_state import MsgState
from mav_sim.tools.transfer_function import TransferFunction
from mav_sim.tools.wrap import saturate, wrap


class Autopilot:
    """Creates an autopilot for controlling the mav to desired values
    using "Total Energy Control System"
    """
    def __init__(self, ts_control: float):
        """Initialize the lateral and longitudinal controllers

        Args:
            ts_control: time step for the control
        """
        # instantiate lateral controllers
        self.roll_from_aileron = PDControlWithRate(
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit=np.radians(45))
        self.course_from_roll = PIControl(
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.yaw_damper = TransferFunction(
                        num=np.array([[AP.yaw_damper_kr, 0]]),
                        den=np.array([[1, AP.yaw_damper_p_wo]]),
                        Ts=ts_control)

        # instantiate TECS controllers
        self.pitch_from_elevator = PDControlWithRate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        # throttle gains (unitless)
        self.E_kp = 1
        self.E_ki = .5
        # pitch gains
        self.L_kp = 1
        self.L_ki = .1
        # saturated altitude error
        self.h_error_max = 50.  # meters
        self.E_integrator = 0.
        self.L_integrator = 0.
        self.E_error_d1 = 0.
        self.L_error_d1 = 0.
        self.delta_t_d1 = 0.
        self.theta_c_d1 = 0.
        self.theta_c_max = np.radians(30)
        self.Ts = ts_control
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
        phi_c = saturate(
            cmd.phi_feedforward + self.course_from_roll.update(chi_c, state.chi),
            -np.radians(30), np.radians(30))
        delta_a = self.roll_from_aileron.update(phi_c, state.phi, state.p)
        delta_r = self.yaw_damper.update(state.r)

        # longitudinal TECS autopilot
        # error in kinetic energy
        K_error = 0.5 * MAV.mass * (cmd.airspeed_command**2 - state.Va**2)
        K_ref = 0.5 * MAV.mass * cmd.airspeed_command**2

        # (saturated) error in potential energy
        U_error = MAV.mass * MAV.gravity * \
                  saturate(cmd.altitude_command - state.altitude,
                                -self.h_error_max, self.h_error_max)

        # (normalized) error in total energy and energy difference
        E_error = (K_error + U_error) / K_ref
        L_error = (U_error - K_error) / K_ref

        #  update the integrator(with anti - windup)
        if (self.delta_t_d1 > 0.) and (self.delta_t_d1 < 1.):
            self.E_integrator = self.E_integrator \
                                + (self.Ts / 2) * (E_error + self.E_error_d1)

        if (self.theta_c_d1 > -self.theta_c_max) and (self.theta_c_d1 < self.theta_c_max):
            self.L_integrator = self.L_integrator \
                                + (self.Ts / 2) * (L_error + self.L_error_d1)

        delta_t = saturate(self.E_kp * E_error
                                + self.E_ki * self.E_integrator, 0, 1)
        theta_c = saturate(self.L_kp * L_error
                                + self.L_ki * self.L_integrator,
                                -self.theta_c_max, self.theta_c_max)
        delta_e = self.pitch_from_elevator.update(theta_c, state.theta, state.q)
        self.E_error_d1 = E_error
        self.L_error_d1 = L_error
        self.delta_t_d1 = delta_t
        self.theta_c_d1 = theta_c

        # construct output and commanded states
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
