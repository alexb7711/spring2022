"""
compute_trim
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:
        12/29/2018 - RWB
"""
from typing import Any, cast

import numpy as np
import numpy.typing as npt
from mav_sim.chap3.mav_dynamics import IND, derivatives
from mav_sim.chap4.mav_dynamics import forces_moments, update_velocity_data
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.tools import types
from scipy.optimize import minimize


def compute_trim(state0: types.DynamicState, Va: float, gamma: float) -> tuple[types.DynamicState, MsgDelta]:
    """Compute the trim equilibrium given the airspeed and flight path angle

    Args:
        mav: instance of the mav dynamics class
        Va: air speed
        gamma: flight path angle

    Returns:
        trim_state: The resulting trim trajectory state
        trim_input: The resulting trim trajectory inputs
    """
    # define initial state and input
    delta0 = MsgDelta(elevator=0., aileron=0., rudder=0., throttle=0.5)
    x0 = np.concatenate((state0, delta0.to_array()), axis=0)

    # define equality constraints
    cons = ({'type': 'eq',
             'fun': lambda x: np.array([
                                # magnitude of velocity vector is Va
                                x[IND.U]**2 + x[IND.V]**2 + x[IND.W]**2 - Va**2,
                                # v=0, force side velocity to be zero
                                x[IND.V],
                                # force quaternion to be unit length
                                x[IND.E0]**2 + x[IND.E1]**2 + x[IND.E2]**2 + x[IND.E3]**2 - 1.,
                                # forcing e1=e3=0 ensures zero roll and zero yaw in trim
                                x[IND.E1], # e1=0
                                x[IND.E3], # e3=0
                                # angular rates should all be zero
                                x[IND.P],  # p=0
                                x[IND.Q],  # q=0
                                x[IND.R],  # r=0
                                ]),
             'jac': lambda x: np.array([
                                [0., 0., 0., 2*x[IND.U], 2*x[IND.V], 2*x[IND.W], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 2*x[IND.E0], 2*x[IND.E1], 2*x[IND.E2], \
                                2*x[IND.E3], 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
                                ])
             })
    # solve the minimization problem to find the trim states and inputs

    res = minimize(trim_objective_fun, x0, method='SLSQP', args=(Va, gamma),
                   constraints=cons, options={'ftol': 1e-10, 'disp': False})

    # extract trim state and input and return
    trim_state = types.DynamicState( np.array([res.x[0:13]]).T )
    trim_input = MsgDelta(elevator=res.x.item(13),
                          aileron=res.x.item(14),
                          rudder=res.x.item(15),
                          throttle=res.x.item(16))
    return trim_state, trim_input


def trim_objective_fun(x: npt.NDArray[Any], Va: float, gamma: float) -> float:
    """Calculates the cost on the trim trajectory being optimized

    Objective is the norm of the desired dynamics subtract the actual dynamics (except the x-y position variables)

    Args:
        x: current state and inputs combined into a single vector
        Va: relative wind vector magnitude
        gamma: flight path angle

    Returns:
        J: resulting cost of the current parameters
    """
    # Extract the state and input
    state = x[0:13]
    delta = MsgDelta(elevator=x.item(13),
                     aileron=x.item(14),
                     rudder=x.item(15),
                     throttle=x.item(16))

    # Calculate the desired trim trajectory dynamics
    desired_trim_state_dot = np.array([[0., 0., -Va*np.sin(gamma), 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]]).T

    # Calculate the dynamics based upon the current state and input
    (Va, alpha, beta, _) = update_velocity_data(state)
    forces_moments_vec = forces_moments(state, delta, Va, beta, alpha)
    f = cast(npt.NDArray[Any], derivatives(state, forces_moments_vec) )

    # Calculate the difference between the desired and actual
    tmp: npt.NDArray[Any] = desired_trim_state_dot - f

    # Calculate the square of the difference (neglecting pn and pe)
    J = np.linalg.norm(tmp[2:13])**2.0
    return float( J )
