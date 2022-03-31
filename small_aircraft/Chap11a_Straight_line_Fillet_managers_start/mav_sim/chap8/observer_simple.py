"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
        3/2/2021 - TWM  Added analytical Jacobians. Simplified GPS smoother (work in progress).
"""
# pylint: disable=no-self-use
from typing import Any, cast

import mav_sim.parameters.control_parameters as CTRL
import mav_sim.parameters.sensor_parameters as SENSOR
import mav_sim.parameters.simulation_parameters as SIM
import numpy as np
import numpy.typing as npt
from mav_sim.chap8.observer import AlphaFilter
from mav_sim.message_types.msg_sensors import MsgSensors
from mav_sim.message_types.msg_state import MsgState
from mav_sim.tools import types
from mav_sim.tools.wrap import wrap
from scipy import stats


class Observer:
    """Defines an oberver for the Mav state
    """
    def __init__(self, ts_control: float):
        """Initialize the observer

        Args:
            ts_control: time step for the controller
        """
        # store the control time step
        self.ts_control = ts_control



        # initialized estimated state message
        self.estimated_state = MsgState()
        # use alpha filters to low pass filter gyros and accels
        # alpha = Ts/(Ts + tau) where tau is the LPF time constant
        self.lpf_gyro_x = AlphaFilter(alpha=0.5)
        self.lpf_gyro_y = AlphaFilter(alpha=0.5)
        self.lpf_gyro_z = AlphaFilter(alpha=0.5)
        self.lpf_accel_x = AlphaFilter(alpha=0.5)
        self.lpf_accel_y = AlphaFilter(alpha=0.5)
        self.lpf_accel_z = AlphaFilter(alpha=0.5)
        # use alpha filters to low pass filter absolute and differential pressure
        self.lpf_abs = AlphaFilter(alpha=0.9)
        self.lpf_diff = AlphaFilter(alpha=0.5)
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
        return cast(npt.NDArray[Any],  f_)

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
        # p = measurement.gyro_x - state.bx
        q = measurement.gyro_y - state.by
        r = measurement.gyro_z - state.bz
        for _ in range(0, self.N):
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement, state)
            phi = self.xhat.item(0)
            theta = self.xhat.item(1)
            # compute Jacobian filter dynamics
            A = np.array([[ q * np.cos(phi) * np.tan(theta) - r * np.sin(phi) * np.tan(theta),
                            (q * np.sin(phi) + r * np.cos(phi)) / (np.cos(theta)**2) ],
                          [ -q * np.sin(phi) - r * np.cos(phi), 0.0 ]])
            # compute G matrix for gyro noise
            G = np.array([[1.0, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
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
        # inputs to EKF: p, q, r, Va
        p = measurement.gyro_x - state.bx
        q = measurement.gyro_y - state.by
        r = measurement.gyro_z - state.bz
        # Va = np.sqrt(2 * measurement.diff_pressure / CTRL.rho)
        Va = state.Va
        # state estimates: phi, theta
        phi = self.xhat.item(0)
        theta = self.xhat.item(1)
        # measurement model
        h = self.h(self.xhat, measurement, state)
        # compute Jacobian of measurement model
        C = np.array([[ 0.0, q * Va * np.cos(theta) + CTRL.gravity * np.cos(theta) ],
                      [ - CTRL.gravity * np.cos(phi) * np.cos(theta),
                        - r * Va * np.sin(theta) - p * Va * np.cos(theta)
                        + CTRL.gravity * np.sin(phi) * np.sin(theta) ],
                      [ CTRL.gravity * np.sin(phi) * np.cos(theta),
                        (q * Va + CTRL.gravity * np.cos(phi)) * np.sin(theta) ]])
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
                    1.0, #0.0001, # psi
                    ])
        self.R_gps = np.diag([
                    SENSOR.gps_n_sigma**2,  # y_gps_n
                    SENSOR.gps_e_sigma**2,  # y_gps_e
                    SENSOR.gps_Vg_sigma**2,  # y_gps_Vg
                    SENSOR.gps_course_sigma**2,  # y_gps_course
                    ])
        self.N = 10  # number of prediction step per sample
        self.Ts = (SIM.ts_control / self.N)
        self.xhat = np.array([[0.0], [0.0], [25.0], [0.0], [0.0]])
        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 1.0])
        self.gps_n_old = 9999.
        self.gps_e_old = 9999.
        self.gps_Vg_old = 9999.
        self.gps_course_old = 9999.
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
        state.psi = self.xhat.item(4)

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
        psi = x.item(4)
        psidot = (state.q * np.sin(state.phi) + state.r * np.cos(state.phi)) / np.cos(state.theta)
        Vgdot = (psidot * state.Va *  np.sin(chi - psi))
        f_ = np.array([[Vg * np.cos(chi)],
                       [Vg * np.sin(chi)],
                       [Vgdot],
                       [(CTRL.gravity / Vg) * np.tan(state.phi) * np.cos(chi - psi)],
                       [psidot]])
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

    def propagate_model(self, measurement: types.PositionMeasurement, state: types.PositionState)-> None:
        """Propagates the position state estimate forward in time

        Args:
            measurement: latest position sensor data
            state: latest state
        """
        # estimates q, r
        q = state.q
        r = state.r
        # estimates Va, phi, theta
        Va = state.Va
        phi = state.phi
        theta = state.theta
        for _ in range(0, self.N):
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement, state)
            # compute Jacobian
            # state estimates used in Jacobian evaluation
            Vg = self.xhat.item(2)
            chi = self.xhat.item(3)
            psi = self.xhat.item(4)
            # compute elements of Jacobian
            psidot = q * np.sin(phi) / np.cos(theta) + r * np.cos(phi) / np.cos(theta)
            Vgdot = Va * psidot * np.sin(chi-psi)
            p_pn_Vg = np.cos(chi)
            p_pn_chi = -Vg * np.sin(chi)
            p_pe_Vg = np.sin(chi)
            p_pe_chi = Vg * np.cos(chi)
            p_Vg_Vg = -Vgdot / Vg
            p_Vg_chi = psidot * Va * np.cos(chi-psi)
            p_Vg_psi = -psidot * Va * np.cos(chi-psi)
            p_chi_Vg = -CTRL.gravity * np.tan(phi) * np.cos(chi - psi) / Vg**2
            p_chi_chi = -CTRL.gravity * np.tan(phi) * np.sin(chi - psi) / Vg
            p_chi_psi = CTRL.gravity * np.tan(phi) * np.sin(chi - psi) / Vg
            A = np.array([[ 0.0, 0.0, p_pn_Vg, p_pn_chi, 0.0 ],
                          [ 0.0, 0.0, p_pe_Vg, p_pe_chi, 0.0 ],
                          [ 0.0, 0.0, p_Vg_Vg, p_Vg_chi, p_Vg_psi ],
                          [ 0.0, 0.0, p_chi_Vg, p_chi_chi, p_chi_psi ],
                          [ 0.0, 0.0, 0.0, 0.0, 0.0 ]])
            # convert to discrete time models
            A_d = np.eye(5) + self.Ts * A + (self.Ts ** 2) * A @ A / 2.0
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + self.Ts**2 * self.Q
            # # calculate wn and we estimates
            # wn = Vg * np.cos(chi) - Va * np.cos(psi)
            # we = Vg * np.sin(chi) - Va * np.sin(psi)

    def measurement_update(self, measurement: types.PositionMeasurement, state: types.PositionState)-> None:
        """ Updates estimate based upon the latest position measurement

        Args:
            measurement: latest position sensor data
            state: latest state
        """
        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = self.h_gps(self.xhat, measurement, state)
            # compute Jacobian
            C = np.array([[ 1.0, 0.0, 0.0, 0.0, 0.0 ],
                          [ 0.0, 1.0, 0.0, 0.0, 0.0 ],
                          [ 0.0, 0.0, 1.0, 0.0, 0.0 ],
                          [ 0.0, 0.0, 0.0, 1.0, 0.0 ]])
            y_chi = wrap(measurement.gps_course, h[3, 0])
            y = np.array([[measurement.gps_n,
                           measurement.gps_e,
                           measurement.gps_Vg,
                           y_chi]]).T
            S_inv = np.linalg.inv(self.R_gps + C @ self.P @ C.T)
            if (y-h).T @ S_inv @ (y-h) < self.gps_threshold:
                L = self.P @ C.T @ S_inv
                self.xhat = self.xhat + L @ (y - h)
                tmp = np.eye(5) - L @ C
                self.P = tmp @ self.P @ tmp.T + L @ self.R_gps @ L.T

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course
