"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/4/2019 - RWB
"""
# pylint: disable=no-self-use
# pylint: disable=assigning-non-slot
# mypy: ignore-errors

from typing import Any, Callable, cast, no_type_check

import mav_sim.parameters.aerosonde_parameters as MAV
import mav_sim.parameters.control_parameters as CTRL
import mav_sim.parameters.sensor_parameters as SENSOR
import mav_sim.parameters.simulation_parameters as SIM
import numpy as np
import numpy.typing as npt
from mav_sim.message_types.msg_sensors import MsgSensors
from mav_sim.message_types.msg_state import MsgState
from mav_sim.tools.rotations import Euler2Rotation
from mav_sim.tools.wrap import wrap
from scipy import stats


class Observer:
    """Defines an oberver for the Mav state
    """
    def __init__(self, ts_control: float) -> None:
        """Initialize the observer

        Args:
            ts_control: time step for the controller
        """
        # store the control time step
        self.ts_control = ts_control
        # initialized estimated state message
        self.ekf = ekfFullState()

    def update(self, measurement: MsgSensors) -> MsgState:
        """Update the state estimates

        Args:
            measurement: The latest measurement received

        Returns:
            estimated_state: The latest estimate of the state
        """
        estimated_state = self.ekf.update(measurement)
        return estimated_state

class ekfFullState:
    """
    implement continous-discrete EKF to estimate full state
    """
    def __init__(self) -> None:
        """Initialize EKF parameters
        """
        self.Q = 1 * np.diag([
                        0.001,  # pn
                        0.001,  # pe
                        0.001,  # pd
                        0.0001,  # u
                        0.0001,  # v
                        0.0001,  # w
                        0.00001,  # phi
                        0.00001,  # theta
                        0.001,  # psi
                        0.000001,  # bx
                        0.000001,  # by
                        0.000001,  # bz
                        0.00001,  # wn
                        0.00001,  # we
                        ])
        self.Q_gyro = SENSOR.gyro_sigma**2 * np.eye(3)
        self.Q_accel = SENSOR.accel_sigma**2 * np.eye(3)
        self.R_analog = np.diag([
            SENSOR.abs_pres_sigma**2,
            SENSOR.diff_pres_sigma**2,
            0.00001**2
        ])
        self.R_gps = np.diag([
            SENSOR.gps_n_sigma**2,
            SENSOR.gps_e_sigma**2,
            SENSOR.gps_Vg_sigma**2,
            SENSOR.gps_course_sigma**2
        ])
        self.N = 2  # number of prediction step per sample
        self.Ts = (SIM.ts_control / self.N)
        self.xhat = np.array([[
                    MAV.north0,  # pn
                    MAV.east0,  # pe
                    MAV.down0,  # pd
                    MAV.Va0,  # u
                    0,  # v
                    0,  # w
                    0,  # phi
                    0,  # theta
                    MAV.psi0,  # psi
                    0,  # bx
                    0,  # by
                    0,  # bz
                    0,  # wn
                    0,  # we
                  ]]).T
        self.P = np.diag([
                    10**2,  # pn
                    10**2,  # pe
                    1**2,  # pd
                    2**2,  # u
                    2**2,  # v
                    2**2,  # w
                    np.radians(5)**2,  # phi
                    np.radians(5)**2,  # theta
                    np.radians(20)**2,  # psi
                    np.radians(10)**2,  # bx
                    np.radians(10)**2,  # by
                    np.radians(10)**2,  # bz
                    5**2,  # wn
                    5**2,  # we
                   ])
        self.analog_threshold = stats.chi2.isf(q=0.01, df=3)
        self.gps_n_old = 9999.
        self.gps_e_old = 9999.
        self.gps_Vg_old = 9999.
        self.gps_course_old = 9999.
        self.estimated_state = MsgState()

    def update(self, measurement: MsgSensors) -> MsgState:
        """Update the state estimates

        Args:
            measurement: The latest measurement received

        Returns:
            estimated_state: The latest estimate of the state
        """
        self.propagate_model(measurement)
        self.measurement_update(measurement)
        # write out estimate state
        self.estimated_state.pn = self.xhat.item(0)
        self.estimated_state.pe = self.xhat.item(1)
        self.estimated_state.h = -self.xhat.item(2)
        self.estimated_state.u = self.xhat.item(3)
        self.estimated_state.v = self.xhat.item(4)
        self.estimated_state.w = self.xhat.item(5)
        self.estimated_state.phi = self.xhat.item(6)
        self.estimated_state.theta = self.xhat.item(7)
        self.estimated_state.psi = self.xhat.item(8)
        self.estimated_state.bx = self.xhat.item(9)
        self.estimated_state.by = self.xhat.item(10)
        self.estimated_state.bz = self.xhat.item(11)
        self.estimated_state.wn = self.xhat.item(12)
        self.estimated_state.we = self.xhat.item(13)
        # estimate needed quantities that are not part of state
        R = Euler2Rotation(self.estimated_state.phi,
                           self.estimated_state.theta,
                           self.estimated_state.psi)
        vel_body = self.xhat[3:6]
        vel_world = R @ vel_body
        wind_world = np.array([[self.estimated_state.wn], [self.estimated_state.we], [0]])
        wind_body = R.T @ wind_world
        va = vel_body - wind_body
        self.estimated_state.Va = np.linalg.norm(va)
        self.estimated_state.alpha = np.arctan(va.item(2) / va.item(0))
        self.estimated_state.beta = np.arcsin(va.item(1) / self.estimated_state.Va)
        self.estimated_state.Vg = np.linalg.norm(vel_world)
        self.estimated_state.chi = np.arctan2(vel_world.item(1), vel_world.item(0))
        self.estimated_state.p = measurement.gyro_x - self.estimated_state.bx
        self.estimated_state.q = measurement.gyro_y - self.estimated_state.by
        self.estimated_state.r = measurement.gyro_z - self.estimated_state.bz
        return self.estimated_state

    def propagate_model(self, measurement: MsgSensors) -> None:
        """Model propagation between measurements
        Propagates the state model between measurements using a model replacement technique

        Args:
            measurement: gyro and accelerometer measurements needed for model propagation
        """
        # model propagation
        for _ in range(0, self.N):
            vel = self.xhat[3:6]
            Theta = self.xhat[6:9]
            S = np.array(
                [[1, np.sin(Theta.item(0)) * np.tan(Theta.item(1)), np.cos(Theta.item(0)) * np.tan(Theta.item(1))],
                 [0, np.cos(Theta.item(0)), -np.sin(Theta.item(0))],
                 [0, (np.sin(Theta.item(0)) / np.cos(Theta.item(1))), (np.cos(Theta.item(0)) / np.cos(Theta.item(1)))]
                 ])
            vel_cross = np.array([[0, -vel.item(2), vel.item(1)],
                                  [vel.item(2), 0, -vel.item(0)],
                                  [-vel.item(1), vel.item(0), 0]])

            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement)
            # convert to discrete time models
            A_d = np.eye(14) + self.Ts * A + (self.Ts ** 2) * A @ A / 2.0
            Gg_d =  np.concatenate((np.zeros((3, 3)), -vel_cross, -S, np.zeros((5, 3))), axis=0)
            Ga_d = np.concatenate((np.zeros((3, 3)), -np.eye(3), np.zeros((8, 3))), axis=0)
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + self.Ts**2 *  ( \
                          self.Q\
                        + Gg_d @ self.Q_gyro @ Gg_d.T\
                        + Ga_d @ self.Q_accel @ Ga_d.T )

    def f(self, x: npt.NDArray[Any] , measurement: MsgSensors) -> npt.NDArray[Any]:
        """
        system dynamics for propagation model: xdot = f(x, u), uses model replacement

        Args:
            x: full state being estimated
            measurement: propagation sensors (gyro and accelerometer)

        Returns:
            f_: state dynamics for propagation
        """
        # pos   = x[0:3]
        vel = x[3:6]
        Theta = x[6:9]
        bias = x[9:12]
        # wind = np.array([[x.item(12), x.item(13), 0]]).T
        y_gyro = np.array([[measurement.gyro_x, measurement.gyro_y, measurement.gyro_z]]).T
        y_accel = np.array([[measurement.accel_x, measurement.accel_y, measurement.accel_z]]).T
        R = Euler2Rotation(Theta.item(0), Theta.item(1), Theta.item(2))
        S = np.array([[1, np.sin(Theta.item(0)) * np.tan(Theta.item(1)), np.cos(Theta.item(0)) * np.tan(Theta.item(1))],
                      [0, np.cos(Theta.item(0)), -np.sin(Theta.item(0))],
                      [0, (np.sin(Theta.item(0))/np.cos(Theta.item(1))), (np.cos(Theta.item(0))/np.cos(Theta.item(1)))]
                      ])
        omega = y_gyro - bias
        vel_cross = np.array([[0, -vel.item(2), vel.item(1)],
                              [vel.item(2), 0, -vel.item(0)],
                              [-vel.item(1), vel.item(0), 0]])
        pos_dot = R @ vel
        vel_dot = vel_cross @ omega + y_accel + R.T @ np.array([[0, 0, CTRL.gravity]]).T
        Theta_dot = S @ omega
        bias_dot = np.array([[0, 0, 0]]).T
        wind_dot = np.array([[0, 0]]).T
        f_ = np.concatenate((pos_dot, vel_dot, Theta_dot, bias_dot, wind_dot), axis=0)
        return cast(npt.NDArray[Any], f_)

    def measurement_update(self, measurement: MsgSensors) -> None:
        """
        Update to state estimate based upon received sensor measurements

        Args:
            measurement: received sensor measurement
        """
        # always update based on sensor measurements
        h = self.h_analog(self.xhat, measurement)
        C = jacobian(self.h_analog, self.xhat, measurement)
        y = np.array([[measurement.abs_pressure,
                      measurement.diff_pressure,
                      0.0
                      ]]).T
        # S_inv = np.linalg.inv(self.R_analog + C @ self.P @ C.T)
        # if False:  #(y - h).T @ S_inv @ (y - h) < self.analog_threshold:
        #     L = self.P @ C.T @ np.linalg.inv(self.R_analog + C @ self.P @ C.T)
        #     tmp = np.eye(14) - L @ C
        #     self.P = tmp @ self.P @ tmp.T +  L @ self.R_analog @ L.T
        #     self.xhat = self.xhat + L @ (y - h)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = self.h_gps(self.xhat, measurement)
            C = jacobian(self.h_gps, self.xhat, measurement)
            y_chi = wrap(measurement.gps_course, h[3, 0])
            y = np.array([[measurement.gps_n, measurement.gps_e, measurement.gps_Vg, y_chi]]).T
            if np.linalg.norm(y-h)<0.5:
                L = self.P @ C.T @ np.linalg.inv(self.R_gps + C @ self.P @ C.T)
                tmp = np.eye(14) - L @ C
                self.P = tmp @ self.P @ tmp.T + L @ self.R_gps @ L.T
                self.xhat = self.xhat + L @ (y - h)

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course

    def h_analog(self, x: npt.NDArray[Any], measurement: MsgSensors) -> npt.NDArray[Any]: # pylint: disable=unused-argument
        """
        analog sensor measurements and pseudo measurements

        Args:
            x: State being estimated
            measurement: Measurement vector

        Returns:
            h_: the sensor measurements as predicted from the state
        """
        pos = x[0:3]
        vel_body = x[3:6]
        Theta = x[6:9]
        wind_world = np.array([[x.item(12), x.item(13), 0]]).T
        R = Euler2Rotation(Theta.item(0), Theta.item(1), Theta.item(2))
        wind_body = R.T @ wind_world
        vel_rel = vel_body - wind_body
        Va = np.linalg.norm(vel_rel)
        abs_pres = -CTRL.rho * CTRL.gravity * pos.item(2)
        diff_pres = CTRL.rho * (Va**2) / 2.0
        sideslip = vel_rel.item(1)
        h = np.array([[abs_pres, diff_pres, sideslip]]).T
        return h

    def h_gps(self, x: npt.NDArray[Any], measurement: MsgSensors) -> npt.NDArray[Any]: # pylint: disable=unused-argument
        """
        measurement model for gps measurements

        Args:
            x: State being estimated
            measurement: Measurement vector

        Returns:
            h: the sensor measurements as predicted from the state
        """
        pos = x[0:3]
        vel_body = x[3:6]
        Theta = x[6:9]
        R = Euler2Rotation(Theta.item(0), Theta.item(1), Theta.item(2))
        vel_world = R @ vel_body
        pn = pos.item(0)
        pe = pos.item(1)
        Vg = np.linalg.norm(vel_world)
        chi = np.arctan2(vel_world.item(1), vel_world.item(0))
        h = np.array([[pn, pe, Vg, chi]]).T
        return h

JacobianFnc = Callable[[npt.NDArray[Any], MsgSensors], npt.NDArray[Any]]
@no_type_check
def jacobian(fun: JacobianFnc, x: npt.NDArray[Any], measurement: MsgSensors) -> npt.NDArray[Any]:
    """
    compute jacobian of fun with respect to x

    Args:
        fun: function about which jacobian is being taken
        x: state about which to compute jacobian
        measurement: current sensor measurements

    Returns:
        J: jacobian of "fun" with respect to the state
    """
    f = fun(x, measurement)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.01  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, measurement)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J
