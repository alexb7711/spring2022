"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
        12/21 - GND
"""
# pylint: disable=no-self-use
from typing import Any, cast, no_type_check

import mav_sim.parameters.control_parameters as CTRL
import mav_sim.parameters.sensor_parameters as SENSOR
import mav_sim.parameters.simulation_parameters as SIM
import numpy as np
import numpy.typing as npt
from mav_sim.message_types.msg_sensors import MsgSensors
from mav_sim.message_types.msg_state import MsgState
from mav_sim.tools import types
from mav_sim.tools.wrap import wrap
from scipy import stats


class Observer:
    """Defines an observer for the Mav state
    """
    def __init__(self, ts_control: float, initial_state: MsgState = MsgState(), initial_measurements: MsgSensors = MsgSensors()) \
         -> None:
        """Initialize the observer

        Args:
            ts_control: time step for the controller
            initial_state: the initial state of the system
            initial_measurements: initial measurements received
        """
        # store the control time step
        self.ts_control = ts_control
        # initialized estimated state message
        self.estimated_state = initial_state
        # use alpha filters to low pass filter gyros and accels
        # alpha = Ts/(Ts + tau) where tau is the LPF time constant
        self.lpf_gyro_x = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_x)
        self.lpf_gyro_y = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_y)
        self.lpf_gyro_z = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_z)
        self.lpf_accel_x = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_x)
        self.lpf_accel_y = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_y)
        self.lpf_accel_z = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_z)
        # use alpha filters to low pass filter absolute and differential pressure
        self.lpf_abs = AlphaFilter(alpha=0.9, y0=initial_measurements.abs_pressure)
        self.lpf_diff = AlphaFilter(alpha=0.7, y0=initial_measurements.diff_pressure)
        # ekf for phi and theta
        self.attitude_ekf = EkfAttitude()
        # ekf for pn, pe, Vg, chi, wn, we, psi
        self.position_ekf = EkfPosition()

    def update(self, measurement: MsgSensors) -> MsgState:
        """Update the state estimates

        Args:
            measurement: The latest measurement received

        Returns:
            estimated_state: The latest estimate of the state
        """

        # estimates for p, q, r are low pass filter of gyro minus bias estimate
        self.estimated_state.p = self.lpf_gyro_x.update(measurement.gyro_x) - self.estimated_state.bx
        self.estimated_state.q = self.lpf_gyro_y.update(measurement.gyro_y) - self.estimated_state.by
        self.estimated_state.r = self.lpf_gyro_z.update(measurement.gyro_z) - self.estimated_state.bz

        # invert sensor model to get altitude and airspeed
        abs_pressure = self.lpf_abs.update(measurement.abs_pressure)
        diff_pressure = self.lpf_diff.update(measurement.diff_pressure)
        self.estimated_state.altitude = abs_pressure/CTRL.rho/CTRL.gravity
        self.estimated_state.Va = np.sqrt(2 * diff_pressure / CTRL.rho)

        # estimate phi and theta with simple ekf
        self.attitude_ekf.update(measurement, self.estimated_state)

        # estimate pn, pe, Vg, chi, wn, we, psi
        self.position_ekf.update(measurement, self.estimated_state)

        # not estimating these
        self.estimated_state.alpha = self.estimated_state.theta
        self.estimated_state.beta = 0.0
        self.estimated_state.bx = 0.0
        self.estimated_state.by = 0.0
        self.estimated_state.bz = 0.0
        return self.estimated_state


class AlphaFilter:
    """
    alpha filter implements a simple low pass filter
    y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    """
    def __init__(self, alpha: float =0.5, y0: float =0.0) -> None:
        """
        Initialize the low pass filter

        Args:
            alpha: filter parameter
            y0: initial condition
        """
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u: float) -> float:
        """Update teh filter

        Args:
            u: latest input

        Returns:
            y: filtered value
        """
        self.y = self.alpha * self.y + (1-self.alpha) * u
        return self.y


class EkfAttitude:
    """
    Implement continous-discrete EKF to estimate roll and pitch angles

    i.e., maintains an estimate for xhat = [phi, theta]
    """
    def __init__(self)->None:
        """ Initialize EKF
        """
        self.Q = 1e-6 * np.diag([1.0, 1.0])
        self.Q_gyro = SENSOR.gyro_sigma**2 * np.diag([1.0, 1.0, 1.0])
        self.R_accel = 100*SENSOR.accel_sigma**2 * np.diag([1.0, 1.0, 100.0])
        self.N = 2  # number of prediction step per sample
        self.xhat = np.array([[0.0], [0.0]]) # initial state: phi, theta
        self.P = np.diag([1.0, 1.0])
        self.Ts = SIM.ts_control/self.N
        self.gate_threshold = stats.chi2.isf(q=0.01, df=3)

    def update(self, measurement: types.AttitudeMeasurement, state: types.AttitudeState) -> None:
        """Update the attitude state estimate

        Args:
            measurement: latest attitude sensor data
            state: latest state
        """
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.phi = self.xhat.item(0)
        state.theta = self.xhat.item(1)

    def f(self, x: npt.NDArray[Any], measurement: types.AttitudeMeasurement, \
          state: types.AttitudeState) -> npt.NDArray[Any]:
        """
        system dynamics for propagation model: xdot = f(x, u)

        Args:
            x: vector with [phi, theta]
            measurement: latest attitude sensor data
            state: latest state (for biases)

        Returns:
            f_: Dynamics for state [phi, theta]
        """
        phi = x.item(0)
        theta = x.item(1)
        p = measurement.gyro_x - state.bx
        q = measurement.gyro_y - state.by
        r = measurement.gyro_z - state.bz
        G = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                      [0.0, np.cos(phi), -np.sin(phi)]])
        f_ = G @ np.array([[p], [q], [r]])
        return cast(npt.NDArray[Any], f_)

    def h(self, x: npt.NDArray[Any], measurement: types.AttitudeMeasurement, \
          state: types.AttitudeState) ->npt.NDArray[Any]:
        """
        measurement model y

        Args:
            x: vector with [phi, theta]
            measurement: latest attitude sensor data
            state: latest state (for biases)

        Returns:
            h_: Measurements vector [x-accel, y-accel, z-accel]
        """
        phi = x.item(0)
        theta = x.item(1)
        p = measurement.gyro_x - state.bx
        q = measurement.gyro_y - state.by
        r = measurement.gyro_z - state.bz
        Va = np.sqrt(2 * measurement.diff_pressure / CTRL.rho)
        h_ = np.array([
            [q * Va * np.sin(theta) + CTRL.gravity * np.sin(theta)],  # x-accel
            [r * Va * np.cos(theta) - p * Va * np.sin(theta) - CTRL.gravity * np.cos(theta) * np.sin(phi)],
            # y-accel
            [-q * Va * np.cos(theta) - CTRL.gravity * np.cos(theta) * np.cos(phi)],  # z-accel
        ])
        return h_

    def propagate_model(self, measurement: types.AttitudeMeasurement, state: types.AttitudeState) -> None:
        """Propagates the attitude state estimate forward in time

        Args:
            measurement: latest attitude sensor data
            state: latest state (for biases)
        """
        # model propagation
        for _ in range(0, self.N):
            phi = self.xhat.item(0)
            theta = self.xhat.item(1)
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
            # compute G matrix for gyro noise
            G = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                          [0.0, np.cos(phi), -np.sin(phi)]])
            # convert to discrete time models
            A_d = np.eye(2) + self.Ts * A + ((self.Ts ** 2)/2.) * A @ A
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + self.Ts**2 * (self.Q + G @ self.Q_gyro @ G.T)

    def measurement_update(self, measurement: types.AttitudeMeasurement, state: types.AttitudeState) -> None:
        """ Updates estimate based upon the latest attitude measurement

        Args:
            measurement: latest attitude sensor data
            state: latest state (for biases)
        """
        # measurement updates
        h = self.h(self.xhat, measurement, state)
        C = jacobian(self.h, self.xhat, measurement, state)
        y = np.array([[measurement.accel_x, measurement.accel_y, measurement.accel_z]]).T
        S_inv = np.linalg.inv(self.R_accel + C @ self.P @ C.T)
        if (y-h).T @ S_inv @ (y-h) < self.gate_threshold:
            L = self.P @ C.T @ S_inv
            tmp = np.eye(2) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R_accel @ L.T
            self.xhat = self.xhat + L @ (y - h)
            #print('updating')


class EkfPosition:
    """
    implement continous-discrete EKF to estimate pn, pe, Vg, chi, wn, we, psi
    """
    def __init__(self) -> None:
        """Initialize Position EKF parameters
        """
        self.Q = np.diag([
                    0.1,  # pn
                    0.1,  # pe
                    10.0,  # Vg
                    0.0001, # chi
                    0.1, # wn
                    0.1, # we
                    1.0, #0.0001, # psi
                    ])
        self.R_gps = np.diag([
                    SENSOR.gps_n_sigma**2,  # y_gps_n
                    SENSOR.gps_e_sigma**2,  # y_gps_e
                    SENSOR.gps_Vg_sigma**2,  # y_gps_Vg
                    SENSOR.gps_course_sigma**2,  # y_gps_course
                    ])
        self.R_pseudo = np.diag([
                    0.000001,  # pseudo measurement #1
                    0.000001,  # pseudo measurement #2
                    ])
        self.N = 10  # number of prediction step per sample
        self.Ts = (SIM.ts_control / self.N)

        # Initial estimate for pn, pe, Vg, chi, wn, we, psi
        self.xhat = np.array([[0.0], [0.0], [25.0], [0.0], [0.0], [0.0], [0.0]])

        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.gps_n_old = 9999.
        self.gps_e_old = 9999.
        self.gps_Vg_old = 9999.
        self.gps_course_old = 9999.
        self.pseudo_threshold = stats.chi2.isf(q=0.01, df=2)
        #self.gps_threshold = stats.chi2.isf(q=0.01, df=4)
        self.gps_threshold = 100000 # don't gate GPS

    def update(self, measurement: types.PositionMeasurement, state: types.PositionState) -> None:
        """Update the position state estimate

        Args:
            measurement: latest position sensor data
            state: latest state
        """
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.north = self.xhat.item(0)
        state.east = self.xhat.item(1)
        state.Vg = self.xhat.item(2)
        state.chi = self.xhat.item(3)
        state.wn = self.xhat.item(4)
        state.we = self.xhat.item(5)
        state.psi = self.xhat.item(6)

    def f(self, x: npt.NDArray[Any], measurement: types.PositionMeasurement, state: types.PositionState) -> npt.NDArray[Any]: # pylint: disable=unused-argument
        """
        system dynamics for propagation model: xdot = f(x, u)

        Args:
            x: vector with [_, _, Vg, chi, wn, we, psi]
            measurement: latest attitude sensor data
            state: latest state

        Outputs:
            f_: Dynamics for state [pn, pe, Vg, chi, wn, we, psi]
        """
        # system dynamics for propagation model: xdot = f(x, u)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)
        psidot = (state.q * np.sin(state.phi) + state.r * np.cos(state.phi)) / np.cos(state.theta)
        Vgdot = ((state.Va * np.cos(psi) + wn) * (-psidot * state.Va * np.sin(psi))
                 + (state.Va * np.sin(psi) + we) * (psidot * state.Va * np.cos(psi))) / Vg
        f_ = np.array([[Vg * np.cos(chi)],
                       [Vg * np.sin(chi)],
                       [Vgdot],
                       [(CTRL.gravity / Vg) * np.tan(state.phi) * np.cos(chi - psi)],
                       [0.0],
                       [0.0],
                       [psidot],
                       ])
        return f_

    def h_gps(self, x: npt.NDArray[Any], measurement: types.PositionMeasurement, state: types.PositionState) -> npt.NDArray[Any]: # pylint: disable=unused-argument
        """
        measurement model for gps measurements

        Args:
            x: vector with [pn, pe, Vg, chi, _, _, _]
            measurement: latest attitude sensor data
            state: latest state

        Outputs:
            h_: Measurement [pn, pe, Vg, chi]
        """
        pn = x.item(0)
        pe = x.item(1)
        Vg = x.item(2)
        chi = x.item(3)
        h_ = np.array([
            [pn],
            [pe],
            [Vg],
            [chi],
        ])
        return h_

    def h_pseudo(self, x: npt.NDArray[Any], measurement: types.PositionMeasurement, state: types.PositionState) \
        -> npt.NDArray[Any]: # pylint: disable=unused-argument
        """
        measurement model for wind triangale pseudo measurement

        Args:
            x: vector with [pn, pe, Vg, chi, _, _, _]
            measurement: latest attitude sensor data
            state: latest state

        Outputs:
            h_: Measurement [wind_triangle_x, wind_triangle_y]
        """
        # pn = x.item(0)
        # pe = x.item(1)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)
        h_ = np.array([
            [state.Va * np.cos(psi) + wn - Vg * np.cos(chi)],  # wind triangle x
            [state.Va * np.sin(psi) + we - Vg * np.sin(chi)],  # wind triangle y
        ])
        return h_

    def propagate_model(self, measurement: types.PositionMeasurement, state: types.PositionState)-> None:
        """Propagates the position state estimate forward in time

        Args:
            measurement: latest position sensor data
            state: latest state
        """
        # model propagation
        for _ in range(0, self.N):
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
            # convert to discrete time models
            A_d = np.eye(7) + self.Ts * A + (self.Ts ** 2) * A @ A / 2.0
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + self.Ts**2 * self.Q

    def measurement_update(self, measurement: types.PositionMeasurement, state: types.PositionState)-> None:
        """ Updates estimate based upon the latest position measurement

        Args:
            measurement: latest position sensor data
            state: latest state
        """
        # always update based on wind triangle pseudo measurement
        h = self.h_pseudo(self.xhat, measurement, state)
        C = jacobian(self.h_pseudo, self.xhat, measurement, state)
        y = np.array([[0, 0]]).T
        S_inv = np.linalg.inv(self.R_pseudo + C @ self.P @ C.T)
        if (y-h).T @ S_inv @ (y-h) < self.pseudo_threshold:
            L = self.P @ C.T @ S_inv
            tmp = np.eye(7) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R_pseudo @ L.T
            self.xhat = self.xhat + L @ (y - h)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = self.h_gps(self.xhat, measurement, state)
            C = jacobian(self.h_gps, self.xhat, measurement, state)
            y_chi = wrap(measurement.gps_course, h[3, 0])
            y = np.array([[measurement.gps_n,
                           measurement.gps_e,
                           measurement.gps_Vg,
                           y_chi]]).T
            S_inv = np.linalg.inv(self.R_gps + C @ self.P @ C.T)
            if (y-h).T @ S_inv @ (y-h) < self.gps_threshold:
                L = self.P @ C.T @ S_inv
                self.xhat = self.xhat + L @ (y - h)
                tmp = np.eye(7) - L @ C
                self.P = tmp @ self.P @ tmp.T + L @ self.R_gps @ L.T

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course

@no_type_check
def jacobian(fun: types.SensorJacobianFnc, x: npt.NDArray[Any], measurement: types.JacobianMeasurement, \
    state: types.JacobianState) -> npt.NDArray[Any]:
    """
    compute jacobian of fun with respect to x

    Args:
        fun: function about which jacobian is being taken
        x: state about which to compute jacobian
        measurement: current sensor measurements
        state: current state values

    Returns:
        J: jacobian of "fun" with respect to the state
    """
    f = fun(x, measurement, state)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.0001  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, measurement, state)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J
