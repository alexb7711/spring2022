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

##===============================================================================
#
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
    ##---------------------------------------------------------------------------
    # extract the states
    x = DynamicState(state)

    ##---------------------------------------------------------------------------
    # Compute gravitational forces
    fg = gravity(state)

    ##---------------------------------------------------------------------------
    # Compute aerodynamic forces
    fa = aerodynamic_forces(x, Va, delta, alpha, beta)

    ##---------------------------------------------------------------------------
    # Compute propulsion forces
    Tp, Qp = motor_thrust_torque(Va, delta.throttle)

    ##---------------------------------------------------------------------------
    # Combine forces
    f  = fg + fa + np.array([Tp,0,0])

    ##---------------------------------------------------------------------------
    # Separate forces
    fx = f[0]
    fy = f[1]
    fz = f[2]

    ##---------------------------------------------------------------------------
    # Compute logitudinal torque in body frame
    pm = 0.5*MAV.rho*np.power(Va,2)*MAV.S_wing

    m  = pm*MAV.c*(MAV.C_m_0 + MAV.C_m_alpha*alpha + MAV.C_m_q*MAV.c*x.q/(2*Va) + \
                   MAV.C_m_delta_e*delta.elevator)

    ##---------------------------------------------------------------------------
    # Compute l,m,n
    l = pm*MAV.b*(MAV.C_ell_0 + MAV.C_ell_beta*beta + MAV.C_ell_p*MAV.b*x.p/(2*Va) + \
                  MAV.C_ell_r*MAV.b*x.r/(2*Va) + MAV.C_ell_delta_a*delta.aileron + \
                  MAV.C_ell_delta_r*delta.rudder)

    n = pm*MAV.b*(MAV.C_n_0 + MAV.C_n_beta*beta + MAV.C_n_p*MAV.b*x.p/(2*Va) + \
                  MAV.C_n_r*MAV.b*x.r/(2*Va) + MAV.C_n_delta_a*delta.aileron + \
                  MAV.C_n_delta_r*delta.rudder)

    ##---------------------------------------------------------------------------
    # Combine
    M  = np.array([l,m,n]) - np.array([Qp,0,0])
    Mx = M[0]
    My = M[1]
    Mz = M[2]

    return types.ForceMoment( np.array([[fx, fy, fz, Mx, My, Mz]]).T )

##===============================================================================
#
def gravity(state):
    ##---------------------------------------------------------------------------
    # extract the attitude
    phi, theta, psi = Quaternion2Euler(state[IND.QUAT])

    ##---------------------------------------------------------------------------
    # Local variables
    st = np.sin(theta)
    ct = np.cos(theta)
    sp = np.sin(phi)
    cp = np.cos(phi)

    ##---------------------------------------------------------------------------
    # Calculate gravity
    fg = MAV.mass*MAV.gravity*np.array([-st, ct*sp, ct*cp])

    return fg

##===============================================================================
#
def aerodynamic_forces(x, Va, delta, alpha, beta):
    ##---------------------------------------------------------------------------
    # Calculate blending function
    s  = sigma(alpha)

    ##---------------------------------------------------------------------------
    # Lift and drag functions
    CL = lambda alpha : (1-s)*(MAV.C_L_0 + MAV.C_L_alpha*alpha) + \
         s*(2*np.sign(alpha)*np.power(np.sin(alpha),2)*np.cos(alpha))

    CD = lambda alpha : MAV.C_D_p + np.power(MAV.C_L_0 + MAV.C_L_alpha*alpha,2)/(np.pi*MAV.e*MAV.AR)

    ##---------------------------------------------------------------------------
    # Calculate 2D rotation matrix
    sa = np.sin(alpha)
    ca = np.cos(alpha)
    sc = np.array([[ca, -sa],
                   [sa,  ca]])

    ##---------------------------------------------------------------------------
    # Calculate lift and drag
    pm     = 0.5*MAV.rho*np.power(Va,2)*MAV.S_wing
    f_lift = pm*(CL(alpha) + MAV.C_L_q*MAV.c*x.q/(2*Va) + MAV.C_L_delta_e*delta.elevator)
    f_drag = pm*(CD(alpha) + MAV.C_D_q*MAV.c*x.q/(2*Va) + MAV.C_D_delta_e*delta.elevator)

    ##---------------------------------------------------------------------------
    # Calculate longintudinal forces
    fx,fz = sc@np.array([-f_drag, -f_lift])

    ##---------------------------------------------------------------------------
    # Calculate lateral forces
    fy = pm*(MAV.C_Y_0 + MAV.C_Y_beta*beta + MAV.C_Y_p*MAV.b*x.p/(2*Va) + \
             MAV.C_Y_r*MAV.b*x.r/(2*Va) + MAV.C_Y_delta_a*delta.aileron + \
             MAV.C_Y_delta_r*delta.rudder)

    ##---------------------------------------------------------------------------
    # Combine aerodynamic forces
    f = np.array([fx,fy,fz])

    return f

##===============================================================================
#
def sigma(alpha):
    ##---------------------------------------------------------------------------
    # Local variables
    a  = alpha
    a0 = MAV.alpha0
    M  = MAV.M

    e  = lambda x: np.exp(x)

    ##---------------------------------------------------------------------------
    # Calculate blending function
    s = (1 + e(-M*(a-a0)) + e(M*(a+a0)))/ \
       ((1+e(-M*(a-a0)))*(1+e(M*(a+a0))))

    return s

##===============================================================================
#
def motor_thrust_torque(Va: float, delta_t: float) -> tuple[float, float]:
    """ compute thrust and torque due to propeller  (See addendum by McLain)

    Args:
        Va: Airspeed
        delta_t: Throttle command

    Returns:
        T_p: Propeller thrust
        Q_p: Propeller torque
    """
    ##---------------------------------------------------------------------------
    ## Local variables
    r   = MAV.rho
    D   = MAV.D_prop
    pow = lambda x,y : np.power(x,y)

    ##---------------------------------------------------------------------------
    # Calculate propeller speed
    n = prop_speed(Va, delta_t)

    ##---------------------------------------------------------------------------
    # Calculate coefficients
    J  = Va/(n*D)
    CT = MAV.C_T2*pow(J,2) + MAV.C_T1*J + MAV.C_T0
    CQ = MAV.C_Q2*pow(J,2) + MAV.C_Q1*J + MAV.C_Q0

    ##---------------------------------------------------------------------------
    # thrust and torque due to propeller
    thrust_prop = r*pow(n,2)*pow(D,4)*CT
    torque_prop = r*pow(n,2)*pow(D,5)*CQ

    return thrust_prop, torque_prop

##===============================================================================
#
def prop_speed(Va, dt):
    ##---------------------------------------------------------------------------
    ## Local variables
    r   = MAV.rho
    D   = MAV.D_prop
    p   = np.pi
    kq  = MAV.KQ
    kv  = MAV.KV
    R   = MAV.R_motor
    i   = MAV.i0
    Vin = MAV.V_max*dt

    pow = lambda x,y : np.power(x,y)

    ##---------------------------------------------------------------------------
    # Calculate a, b, and c
    a = (r*pow(D,5))/(pow(2*p,2))*MAV.C_Q0
    b = ((MAV.rho*MAV.D_prop**4*MAV.C_Q1*Va)/(2*np.pi)) + ((MAV.KQ**2)/(MAV.R_motor))
    c = r*pow(D,3)*MAV.C_Q2*pow(Va,2) - kq/R*Vin + kq*i

    ##---------------------------------------------------------------------------
    # Calculate propeller speed [rad/sec]
    ohm = (-b + np.sqrt(pow(b,2) - 4*a*c))/(2*a)

    ##---------------------------------------------------------------------------
    # Calculate propeller speed [rev/sec]
    n = ohm/(2*p)

    return n

##===============================================================================
#
def update_velocity_data(state: types.DynamicState, \
    wind: types.WindVector = types.WindVector( np.zeros((6,1)) ) \
    )  -> tuple[float, float, float, types.NP_MAT]:
    """Calculates airspeed, angle of attack, sideslip, and velocity wrt wind

    Args:
        state: current state of the aircraft

    Returns:
        Va: Airspeed
        beta: Side slip angle
        alpha: Angle of attack
        wind_inertial_frame: Wind vector in inertial frame
    """
    steady_state = wind[0:3]
    gust = wind[3:6]

    ##---------------------------------------------------------------------------
    # convert wind vector from world to body frame
    R = Quaternion2Rotation(state[IND.QUAT]) # rotation from body to world frame
    wind_body_frame = R.T @ steady_state  # rotate steady state wind to body frame
    wind_body_frame += gust  # add the gust
    wind_inertial_frame = R @ wind_body_frame # Wind in the world frame

    ##---------------------------------------------------------------------------
    # extract the states
    x = DynamicState(state)

    ##---------------------------------------------------------------------------
    # compute airspeed
    s      = np.array([[x.u], [x.v], [x.w]])
    ur     = x.u - wind_body_frame.item(0)
    vr     = x.v - wind_body_frame.item(1)
    wr     = x.w - wind_body_frame.item(2)
    Va     = np.sqrt(np.power(ur,2) + np.power(vr,2) + np.power(wr,2))

    ##---------------------------------------------------------------------------
    # compute angle of attack
    alpha = np.arctan2(wr,ur)

    ##---------------------------------------------------------------------------
    # compute sideslip angle
    beta = np.arctan2(vr,np.sqrt(np.power(ur,2) + np.power(wr,2)))

    return (Va, alpha, beta, wind_inertial_frame)
