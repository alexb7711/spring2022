"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
from typing import Optional

import numpy as np
from mav_sim.message_types.msg_gust_params import MsgGustParams
from mav_sim.tools import types
from mav_sim.tools.transfer_function import TransferFunction


class WindSimulation:
    """
    Class to determine wind velocity at any given moment,
    calculates a steady wind speed and uses a stochastic
    process to represent wind gusts. (Follows section 4.4 in uav book)
    """
    def __init__(self, Ts: float, gust_params: Optional[MsgGustParams] = None, gust_flag: bool = True) -> None:
        """Initialize steady-state and gust parameters
        """
        # steady state wind defined in the inertial frame
        self._steady_state = np.array([[0., 0., 0.]]).T
        #self._steady_state = np.array([[0., 5., 0.]]).T

        # Initialize gust parameters if not passed in
        if gust_params is None:
            gust_params = MsgGustParams()



        #   Dryden gust model parameters (pg 56 UAV book)
        Va = 25 # must set Va to a constant value
        #
        Lu = gust_params.Lu
        Lv = gust_params.Lv
        Lw = gust_params.Lw
        if gust_flag:
            sigma_u = gust_params.sigma_u
            sigma_v = gust_params.sigma_v
            sigma_w = gust_params.sigma_w
        else:
            sigma_u = 0.0
            sigma_v = 0.0
            sigma_w = 0.0

        a1 = sigma_u*np.sqrt(2.*Va/Lu)
        b1 = Va/Lu
        self.u_w = TransferFunction(num=np.array([[a1]]),
                                    den=np.array([[1, b1]]),
                                    Ts=Ts)
        a2 = sigma_v*np.sqrt(3.*Va/Lv)
        b2 = Va/Lv
        self.v_w = TransferFunction(num=np.array([[a2]]),
                                     den=np.array([[1,b2]]),
                                     Ts=Ts)
        a3 = sigma_w*np.sqrt(3.*Va/Lw)
        b3 = Va/Lw
        self.w_w = TransferFunction(num=np.array([[0]]),
                                     den=np.array([[1,1]]),
                                     Ts=Ts)
        self._Ts = Ts

    def update(self) -> types.WindVector:
        """
        returns a six vector.
           The first three elements are the steady state wind in the inertial frame
           The second three elements are the gust in the body frame
        """
        gust = np.array([[self.u_w.update(np.random.randn())],
                         [self.v_w.update(np.random.randn())],
                         [self.w_w.update(np.random.randn())]])
        return types.WindVector( np.concatenate(( self._steady_state, gust )) )
