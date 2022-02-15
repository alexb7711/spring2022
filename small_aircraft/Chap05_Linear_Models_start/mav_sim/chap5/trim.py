"""
compute_trim
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:
        12/29/2018 - RWB
"""
from typing import Any, cast

import mav_sim.parameters.aerosonde_parameters as MAV
import numpy as np
import numpy.typing as npt
from mav_sim.chap3.mav_dynamics import IND, derivatives
from mav_sim.chap4.mav_dynamics import forces_moments, update_velocity_data
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.tools import types
from mav_sim.tools.rotations import Quaternion2Euler, Quaternion2Rotation
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
    x0     = np.concatenate((state0, delta0.to_array()), axis=0)

    # define equality constraints
    cons = ({'type': 'eq',
             ##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
             # Transfer Function
             'fun': lambda x: np.array([
                                # The total airspeed is equal to the sum of its components (No change in Va)
                                x[IND.U]**2 + x[IND.V]**2 + x[IND.W]**2 - Va**2,
                                # Pitch velocity is 0 (v is always 0)
                                x[IND.V],
                                # Enforce attitude change is at constraint rate
                                x[IND.E0]**2 + x[IND.E1]**2 + x[IND.E2]**2 + x[IND.E3]**2 - 1.,
                                # forcing e1=e3=0 ensures zero roll and zero yaw in trim
                                x[IND.E1], # e1=0
                                x[IND.E3], # e3=0
                                # Ensure no change angular velocities in roll, pitch, or yaw
                                x[IND.P],  # p=0
                                x[IND.Q],  # q=0
                                x[IND.R],  # r=0
                                ]),
             ##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
             # Jacobian
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
    state           = x[0:13]
    delta           = MsgDelta(elevator=x.item(13),
                      aileron=x.item(14),
                      rudder=x.item(15),
                      throttle=x.item(16))

    # Calculate h_dot
    h_dot = lambda V,g: V*np.sin(g)

    # Calculate the desired trim trajectory dynamics
    desired_trim_state_dot = np.array([h_dot(Va,gamma), 0, 0, 0, 0, 0, 0, 0, 0, 0])

    # Calculate the actual state dynamics
    ## Va, alpha, and beta for the provided state
    Va_s,alpha,beta,wind_inertial_frame = update_velocity_data(state)

    ## Calculate forces and moments
    fm = forces_moments(state, delta, Va_s, beta, alpha)

    # Calculate the dynamics based upon the current state and input
    x_dot = derivatives(state, fm)

    # Calculate Euler angles
    phi, theta, psi = Quaternion2Euler(x_dot[IND.QUAT])

    ## Actual forces and dynamics
    f = [-x_dot.item(IND.DOWN) ,
         x_dot.item(IND.U)     ,
         x_dot.item(IND.V)     ,
         x_dot.item(IND.W)     ,
         phi                   ,
         theta                 ,
         psi                   ,
         x_dot.item(IND.P)     ,
         x_dot.item(IND.Q)     ,
         x_dot.item(IND.R)]

    # Calculate the difference between the desired and actual
    diff = f - desired_trim_state_dot

    # Calculate the square of the difference (neglecting pn and pe)
    J = np.linalg.norm(diff, 2)
    return float( J )
