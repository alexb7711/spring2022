"""
mavDynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state

mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:
        2/24/2020 - RWB
"""
from tokenize import Floatnumber
from typing import Optional, cast

import mav_sim.parameters.aerosonde_parameters as MAV
import mav_sim.parameters.sensor_parameters as SENSOR
import numpy as np
import numpy.typing as npt

# load mav dynamics from previous chapter
from mav_sim.chap3.mav_dynamics import IND, DynamicState, derivatives
from mav_sim.chap4.mav_dynamics import forces_moments, update_velocity_data
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.message_types.msg_sensors import MsgSensors

# load message types
from mav_sim.message_types.msg_state import MsgState
from mav_sim.tools import types
from mav_sim.tools.rotations import (
    Euler2Rotation,
    Quaternion2Euler,
    Quaternion2Rotation,
)


class GpsTransient:
    """Struct for storing the gps transient (represent a Guass-Markov process)
    """

    def __init__(self, nu_n: float, nu_e: float, nu_h: float) -> None:
        """Initialize the gps transient elements

            Args:
                nu_n: Transient in the north direction
                nu_e: Transient in the east direction
                nu_h: Transient in the altitude
        """
        self.n = nu_n
        self.e = nu_e
        self.h = nu_h

    def to_array(self) -> npt.NDArray[Floatnumber]:
        """Convert the command to a numpy array."""
        return np.array(
            [
                [self.n],
                [self.e],
                [self.h],
            ],
            dtype=float,
        )

    def print(self) -> None:
        """Print the commands to the console
        """
        print('n=', self.n,
              'e=', self.e,
              'h=', self.h)


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

        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = MAV.u0
        self._alpha: float = 0
        self._beta: float = 0

        # initialize true_state message
        self.true_state = MsgState()

        # initialize the sensors message
        self._sensors = MsgSensors()

        # random walk parameters for GPS
        self._gps_nu_n: float = 0.
        self._gps_nu_e: float = 0.
        self._gps_nu_h: float = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.

        # update velocity data
        (self._Va, self._alpha, self._beta, self._wind) = update_velocity_data(self._state)

        # Update forces and moments data
        forces_moments_vec = forces_moments(self._state, MsgDelta(), self._Va, self._beta, self._alpha)
        self._forces[0] = forces_moments_vec.item(0)
        self._forces[1] = forces_moments_vec.item(1)
        self._forces[2] = forces_moments_vec.item(2)


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
        (self._Va, self._alpha, self._beta, self._wind) = update_velocity_data(self._state, wind)

        # update the message class for the true state
        self._update_true_state()

        # update the gps timer
        if self._t_gps >= SENSOR.ts_gps:
            self._t_gps = 0.
        else:
            self._t_gps += self._ts_simulation

    def sensors(self, noise_scale: float = 1.) -> MsgSensors:
        """ Return the values of the sensors given the current state. Note that GPS
        is only updated periodically according to the period SENSOR.ts_gps

        Args:
            noise_scale: Scaling on the random white noise

        Returns:
            sensors: The resulting sensor readings
        """
        # Create the GPS transient variable
        nu = GpsTransient(self._gps_nu_n, self._gps_nu_e, self._gps_nu_h)

        # Calculate sensor readings
        self._sensors, nu_update = calculate_sensor_readings(self._state, self._forces, \
            nu, self._Va, self._sensors, self._t_gps == 0., noise_scale)

        # Extract values and return sensor readings
        self._gps_nu_n = nu_update.n
        self._gps_nu_e = nu_update.e
        self._gps_nu_h = nu_update.h
        return self._sensors

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
        self.true_state.bx = SENSOR.gyro_x_bias
        self.true_state.by = SENSOR.gyro_y_bias
        self.true_state.bz = SENSOR.gyro_z_bias

def calculate_sensor_readings(state: types.DynamicState, forces: types.NP_MAT, \
    nu: GpsTransient, Va: float, sensors_prev: MsgSensors, update_gps: bool, noise_scale: float = 1.) -> \
    tuple[MsgSensors, GpsTransient]:
    """ Calculates the sensor readings. This involves calculating the sensor data without
        noise and then adding in the noise.

        Sensor values: gyros, accelerometers, absolute_pressure, dynamic_pressure
                       and GPS

        Args:
            state: current state of the aircraft
            forces: 3x1 forces vector acting on UAV (in body frame)
            nu: Latest GPS transients
            Va: airspeed
            sensors_prev: previous readings of the sensors
            update_gps: true - update the gps sensor, false - use the previous gps reading
            noise_scale: Scaling on the random white noise

        Returns:
            sensors: The resulting sensor readings
            nu_update: The updated gps transients
    """
    # Intialize the sensor reading
    sensors = MsgSensors()

    # Calculate Euler angles
    phi, theta, psi = Quaternion2Euler(state[IND.QUAT])

    # Populate all other sensors
    sensors.accel_x, sensors.accel_y, sensors.accel_z = accelerometer(phi, theta, forces, noise_scale)
    sensors.gyro_x, sensors.gyro_y, sensors.gyro_z    = gyro(state.item(IND.P), state.item(IND.Q), state.item(IND.R), noise_scale)
    sensors.abs_pressure, sensors.diff_press         = pressure(state.item(IND.DOWN), Va, noise_scale)
    sensors.mag_x, sensors.mag_y, sensors.mag_z       = magnetometer(state[IND.QUAT], noise_scale)
    sensors.gps_n, sensors.gps_e, sensors.gps_h, gps_Vg, gps_course \
                                                   = gps(state[0:3], state[IND.VEL], state[IND.QUAT], nu, noise_scale)

    # simulate GPS sensor
    if update_gps:
        # Update the gps transient bias
        nu_update = gps_error_trans_update(nu, noise_scale)

    else:
        # Output previous values
        nu_update = GpsTransient(nu.n, nu.e, nu.h)

        sensors.gps_n      = sensors_prev.gps_n
        sensors.gps_e      = sensors_prev.gps_e
        sensors.gps_h      = sensors_prev.gps_h
        sensors.gps_Vg     = sensors_prev.gps_Vg
        sensors.gps_course = sensors_prev.gps_course

    return sensors, nu_update

##==============================================================================
#
def accelerometer(phi: float, theta: float, forces: types.NP_MAT, noise_scale: float ) -> tuple[float, float, float] :
    """Calculates the accelerometer measurement based upon the current state data

        Args:
            phi: roll angle
            theta: pitch angle
            forces: 3x1 forces vector acting on UAV (in body frame)
            noise_scale: Scaling on the random white noise

        Returns:
            accel_x, accel_y, accel_z: body frame x-y-z acceleration measurements
    """
    accel_x = forces.item(0)/MAV.mass + MAV.gravity*np.sin(theta) + noise_scale*np.random.normal(0., SENSOR.accel_sigma)
    accel_y = forces.item(1)/MAV.mass - MAV.gravity*np.cos(theta)*np.sin(phi) + noise_scale*np.random.normal(0., SENSOR.accel_sigma)
    accel_z = forces.item(2)/MAV.mass - MAV.gravity*np.cos(phi)*np.cos(theta) + noise_scale*np.random.normal(0., SENSOR.accel_sigma)

    return accel_x, accel_y, accel_z

##==============================================================================
#
def gyro(p: float, q: float, r: float, noise_scale: float = 1. ) -> tuple[float, float, float] :
    """Calculates the gyro measurement based upon the current state data

        Args:
            (p,q,r): body x-y-z roll rates (rad/sec)
            noise_scale: Scaling on the random white noise

        Returns:
            gyro_x, gyro_y, gyro_z: body frame x-y-z gyro measurements
    """
    gyro_x = p + SENSOR.gyro_x_bias + noise_scale*np.random.normal(0., SENSOR.gyro_sigma)
    gyro_y = q + SENSOR.gyro_y_bias + noise_scale*np.random.normal(0., SENSOR.gyro_sigma)
    gyro_z = r + SENSOR.gyro_z_bias + noise_scale*np.random.normal(0., SENSOR.gyro_sigma)

    return gyro_x, gyro_y, gyro_z

##==============================================================================
#
def pressure(down: float, Va: float, noise_scale: float = 1. ) -> tuple[float, float] :
    """Calculates the pressure sensor measurement based upon the current state data

        Args:
            down: down position
            Va: airspeed
            noise_scale: Scaling on the random white noise

        Returns:
            abs_pressure: Absolute pressure measurement
            diff_pressure: Differential pressure measurement
    """
    abs_pressure  = -MAV.rho*MAV.gravity*down + SENSOR.abs_pres_bias + noise_scale*np.random.normal(0., SENSOR.abs_pres_bias)
    diff_pressure = MAV.rho*Va**2/2 + SENSOR.diff_pres_bias + noise_scale*np.random.normal(0., SENSOR.diff_pres_bias)

    return abs_pressure, diff_pressure

##==============================================================================
#
def magnetometer(quat_b_to_i: types.Quaternion, noise_scale: float = 1. ) -> tuple[float, float, float] :
    """Calculates the magnetometer measurement based upon the current state. The resulting
    output is a magnetometer reading in the body frame with additive noise. Note that this
    is not in the book

        Args:
            quat_b_to_i: body to inertial quaternion
            noise_scale: Scaling on the random white noise

        Returns:
            mag_x, mag_y, mag_z: body frame x-y-z magnetometer measurements
    """

    # Calculate mi
    mi = Euler2Rotation(0, SENSOR.mag_inc, SENSOR.mag_dec).T@np.array([1,0,0], dtype=float).T

    # Calculate mb
    mb = Quaternion2Rotation(quat_b_to_i).T@mi

    mag_x = mb.item(0) + noise_scale*np.random.normal(0., SENSOR.mag_sigma)
    mag_y = mb.item(1) + noise_scale*np.random.normal(0., SENSOR.mag_sigma)
    mag_z = mb.item(2) + noise_scale*np.random.normal(0., SENSOR.mag_sigma)

    return mag_x, mag_y, mag_z

##==============================================================================
#
def gps_error_trans_update(nu: GpsTransient, noise_scale: float = 1. ) -> GpsTransient :
    """Calculates the transient update of the gps error which is based upon a Gauss-Markov process

        Args:
            nu: the previous value of the transient error
            noise_scale: Scaling on the random white noise

        Returns:
            abs_pressure: Absolute pressure measurement
            diff_pressure: Differential pressure measurement
    """

    nu_n = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*nu.n + noise_scale*np.random.normal(0., SENSOR.gps_n_sigma)
    nu_e = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*nu.e + noise_scale*np.random.normal(0., SENSOR.gps_e_sigma)
    nu_h = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*nu.h + noise_scale*np.random.normal(0., SENSOR.gps_h_sigma)

    return GpsTransient(nu_n, nu_e, nu_h)

##==============================================================================
#
def gps(position: types.NP_MAT, V_g_b: types.NP_MAT, e_quat: types.Quaternion, nu: GpsTransient, \
    noise_scale: float = 1. ) -> tuple[float, float, float, float, float] :
    """Calculates the transient update of the gps error which is based upon a Gauss-Markov process

        Args:
            position: 3x1 vector of position (north, east, down)
            V_g_b: 3x1 vector of velocities in body frame
            e_quat: 4x1 vector consisting of the quaternion (e0, ex, ey, ez)
            nu: the current value of transient error
            noise_scale: Scaling on the random white noise

        Returns:
            (gps_n, gps_e, gps_h): (n,e,-d) measurement of position
            gps_Vg: Ground velocity measurement from GPS
            gps_course: GPS measurement of course angle
    """

    # Calculate position
    gps_n = position.item(0)  + nu.n
    gps_e = position.item(1)  + nu.e
    gps_h = -position.item(2) + nu.h

    # Calculate velocity
    ## Extract Euler angles
    phi, theta, psi = Quaternion2Euler(e_quat)

    ## Velocity directions
    gps_Vg = np.sqrt(V_g_b.item(0)**2 + V_g_b.item(1)**2) + noise_scale*np.random.normal(0., SENSOR.gps_Vg_sigma)

    # Calculate course
    gps_course = np.arctan2(V_g_b.item(1), V_g_b.item(0)) + noise_scale*np.random.normal(0., SENSOR.gps_course_sigma)


    return gps_n, gps_e, gps_h, gps_Vg, gps_course
