"""
mavDynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state

part of mavPySim
    - Beard & McLain, PUP, 2012
    - Update history:
        12/20/2018 - RWB
"""
from typing import Optional

import mav_sim.parameters.aerosonde_parameters as MAV
import numpy as np

# load mav dynamics from previous chapter
from mav_sim.chap3.mav_dynamics import IND, DynamicState, derivatives
from mav_sim.message_types.msg_delta import MsgDelta

# load message types
from mav_sim.message_types.msg_state import MsgState
from mav_sim.tools import types
from mav_sim.tools.rotations import Quaternion2Euler, Quaternion2Rotation


class MavDynamics:
    """Implements the dynamics of the MAV using vehicle inputs and wind
    """

    def __init__(self, Ts: float, state: Optional[DynamicState] = None):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        if state is None:
            self._state = DynamicState().convert_to_numpy()
        else:
            self._state = state.convert_to_numpy()

        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec

        # update velocity data
        (self._Va, self._alpha, self._beta, self._wind) = update_velocity_data(self._state)

        # Update forces and moments data
        self._forces = np.array([[0.], [0.], [0.]]) # store forces to avoid recalculation in the sensors function (ch 7)
        forces_moments_vec = forces_moments(self._state, MsgDelta(), self._Va, self._beta, self._alpha)
        self._forces[0] = forces_moments_vec.item(0)
        self._forces[1] = forces_moments_vec.item(1)
        self._forces[2] = forces_moments_vec.item(2)

        # initialize true_state message
        self.true_state = MsgState()

    ###################################
    # public functions
    def update(self, delta: MsgDelta, wind: types.WindVector) -> None:
        """
        Integrate the differential equations defining dynamics, update sensors

        Args:
            delta : (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind: the wind vector in inertial coordinates
        """
        # get forces and moments acting on rigid bod
        forces_moments_vec = forces_moments(self._state, delta, self._Va, self._beta, self._alpha)
        self._forces[0] = forces_moments_vec.item(0)
        self._forces[1] = forces_moments_vec.item(1)
        self._forces[2] = forces_moments_vec.item(2)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = derivatives(self._state, forces_moments_vec)
        k2 = derivatives(self._state + time_step/2.*k1, forces_moments_vec)
        k3 = derivatives(self._state + time_step/2.*k2, forces_moments_vec)
        k4 = derivatives(self._state + time_step*k3, forces_moments_vec)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(IND.E0)
        e1 = self._state.item(IND.E1)
        e2 = self._state.item(IND.E2)
        e3 = self._state.item(IND.E3)
        norm_e = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[IND.E0][0] = self._state.item(IND.E0)/norm_e
        self._state[IND.E1][0] = self._state.item(IND.E1)/norm_e
        self._state[IND.E2][0] = self._state.item(IND.E2)/norm_e
        self._state[IND.E3][0] = self._state.item(IND.E3)/norm_e

        # update the airspeed, angle of attack, and side slip angles using new state
        (self._Va, self._alpha, self._beta, _) = update_velocity_data(self._state, wind)

        # update the message class for the true state
        self._update_true_state()

    def external_set_state(self, new_state: types.DynamicState) -> None:
        """Loads a new state
        """
        self._state = new_state

    def get_state(self) -> types.DynamicState:
        """Returns the state
        """
        return self._state

    ###################################
    # private functions
    def _update_true_state(self) -> None:
        """ update the class structure for the true state:

        [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        """
        phi, theta, psi = Quaternion2Euler(self._state[IND.QUAT])
        pdot = Quaternion2Rotation(self._state[6:10]) @ self._state[IND.VEL]
        self.true_state.north = self._state.item(IND.NORTH)
        self.true_state.east = self._state.item(IND.EAST)
        self.true_state.altitude = -self._state.item(IND.DOWN)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(IND.P)
        self.true_state.q = self._state.item(IND.Q)
        self.true_state.r = self._state.item(IND.R)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)

def forces_moments(state: types.DynamicState, delta: MsgDelta, Va: float, beta: float, alpha: float) -> types.ForceMoment:
    """
    Return the forces on the UAV based on the state, wind, and control surfaces

    Args:
        state: current state of the aircraft
        delta: flap and thrust commands
        Va: Airspeed
        beta: Side slip angle
        alpha: Angle of attack


    Returns:
        Forces and Moments on the UAV (in body frame) np.matrix(fx, fy, fz, Mx, My, Mz)
    """
    # Extract angular rates
    p = state.item(IND.P)
    q = state.item(IND.Q)
    r = state.item(IND.R)

    # compute gravitaional forces
    R = Quaternion2Rotation(state[IND.QUAT]) # rotation from body to world frame
    f_g = R.T @ np.array([[0.], [0.], [MAV.mass * MAV.gravity]]) # Force of gravity in body frame
    fx = f_g.item(0)
    fy = f_g.item(1)
    fz = f_g.item(2)

    # intermediate variables
    qbar = 0.5 * MAV.rho * Va**2
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    p_nondim = p * MAV.b / (2 * Va)  # nondimensionalize p
    q_nondim = q * MAV.c / (2 * Va)  # nondimensionalize q
    r_nondim = r * MAV.b / (2 * Va)  # nondimensionalize r

    # compute Lift and Drag coefficients
    tmp1 = np.exp(-MAV.M * (alpha - MAV.alpha0))
    tmp2 = np.exp(MAV.M * (alpha + MAV.alpha0))
    sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))
    CL = (1 - sigma) * (MAV.C_L_0 + MAV.C_L_alpha * alpha) \
            + sigma * 2 * np.sign(alpha) * sa**2 * ca
    CD = MAV.C_D_p + ((MAV.C_L_0 + MAV.C_L_alpha * alpha)**2)/(np.pi * MAV.e * MAV.AR)
    # compute Lift and Drag Forces
    F_lift = qbar * MAV.S_wing * (
            CL
            + MAV.C_L_q * q_nondim
            + MAV.C_L_delta_e * delta.elevator
    )
    F_drag = qbar * MAV.S_wing * (
            CD
            + MAV.C_D_q * q_nondim
            + MAV.C_D_delta_e * delta.elevator
    )
    # compute longitudinal forces in body frame
    fx = fx - ca * F_drag + sa * F_lift
    fz = fz - sa * F_drag - ca * F_lift
    # compute lateral forces in body frame
    fy += qbar * MAV.S_wing * (
            MAV.C_Y_0
            + MAV.C_Y_beta * beta
            + MAV.C_Y_p * p_nondim
            + MAV.C_Y_r * r_nondim
            + MAV.C_Y_delta_a * delta.aileron
            + MAV.C_Y_delta_r * delta.rudder
    )
    # compute logitudinal torque in body frame
    My = qbar * MAV.S_wing * MAV.c * (
            MAV.C_m_0
            + MAV.C_m_alpha * alpha
            + MAV.C_m_q * q_nondim
            + MAV.C_m_delta_e * delta.elevator
    )
    # compute lateral torques in body frame
    Mx = qbar * MAV.S_wing * MAV.b * (
            MAV.C_ell_0
            + MAV.C_ell_beta * beta
            + MAV.C_ell_p * p_nondim
            + MAV.C_ell_r * r_nondim
            + MAV.C_ell_delta_a * delta.aileron
            + MAV.C_ell_delta_r * delta.rudder
    )
    Mz = qbar * MAV.S_wing * MAV.b * (
            MAV.C_n_0 + MAV.C_n_beta * beta
            + MAV.C_n_p * p_nondim
            + MAV.C_n_r * r_nondim
            + MAV.C_n_delta_a * delta.aileron
            + MAV.C_n_delta_r * delta.rudder
    )

    thrust_prop, torque_prop = motor_thrust_torque(Va, delta.throttle)
    fx += thrust_prop
    Mx += -torque_prop

    return types.ForceMoment( np.array([[fx, fy, fz, Mx, My, Mz]]).T )

def motor_thrust_torque(Va: float, delta_t: float) -> tuple[float, float]:
    """ compute thrust and torque due to propeller  (See addendum by McLain)

    Args:
        Va: Airspeed
        delta_t: Throttle command

    Returns:
        T_p: Propeller thrust
        Q_p: Propeller torque
    """
    # map delta_t throttle command(0 to 1) into motor input voltage
    v_in = MAV.V_max * delta_t
    # Quadratic formula to solve for motor speed
    a = MAV.C_Q0 * MAV.rho * np.power(MAV.D_prop, 5) \
        / ((2.*np.pi)**2)
    b = (MAV.C_Q1 * MAV.rho * np.power(MAV.D_prop, 4)
            / (2.*np.pi)) * Va + MAV.KQ**2/MAV.R_motor
    c = MAV.C_Q2 * MAV.rho * np.power(MAV.D_prop, 3) \
        * Va**2 - (MAV.KQ / MAV.R_motor) * v_in + MAV.KQ * MAV.i0

    # Angular speed of propeller
    omega_p = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)

    # thrust and torque due to propeller
    thrust_prop = (MAV.rho * np.power(MAV.D_prop,4) * MAV.C_T0 / (4 * np.pi**2)) * omega_p**2\
                    + (MAV.rho * np.power(MAV.D_prop, 3) * MAV.C_T1 * Va / 2 / np.pi) * omega_p\
                    + (MAV.rho * MAV.D_prop**2 * MAV.C_T2 * Va**2)
    torque_prop = (MAV.rho * np.power(MAV.D_prop, 5) * MAV.C_Q0 / (4 * np.pi**2)) * omega_p**2\
                    + (MAV.rho * np.power(MAV.D_prop, 4) * MAV.C_Q1 * Va / (2 * np.pi)) * omega_p\
                    + (MAV.rho * np.power(MAV.D_prop, 3) * MAV.C_Q2 * Va**2)
    return thrust_prop, torque_prop

def update_velocity_data(state: types.DynamicState, \
    wind: types.WindVector = types.WindVector( np.zeros((6,1)) ) \
    )  -> tuple[float, float, float, types.NP_MAT]:
    """Calculates airspeed, angle of attack, sideslip, and velocity wrt wind

    Args:
        state: current state of the aircraft

    Returns:
        Va: Airspeed
        alpha: Angle of attack
        beta: Side slip angle
        wind_inertial_frame: Wind vector in inertial frame
    """
    steady_state = wind[0:3]
    gust = wind[3:6]

    # convert wind vector from world to body frame
    R = Quaternion2Rotation(state[IND.QUAT]) # rotation from body to world frame
    wind_body_frame = R.T @ steady_state  # rotate steady state wind to body frame
    wind_body_frame += gust  # add the gust
    wind_inertial_frame = R @ wind_body_frame # Wind in the world frame

    # velocity vector relative to the airmass
    v_air = state[IND.VEL] - wind_body_frame
    ur = v_air.item(0)
    vr = v_air.item(1)
    wr = v_air.item(2)

    # compute airspeed
    Va: float = np.sqrt(ur**2 + vr**2 + wr**2)

    # compute angle of attack
    alpha: float
    if ur == 0:
        alpha = np.sign(wr)*np.pi/2.
    else:
        alpha = np.arctan(wr/ur)

    # compute sideslip angle
    beta: float
    if Va == 0:
        beta = np.sign(vr)*np.pi/2.
    else:
        beta = np.arcsin(vr/Va)

    # Return computed values
    return (Va, alpha, beta, wind_inertial_frame)
