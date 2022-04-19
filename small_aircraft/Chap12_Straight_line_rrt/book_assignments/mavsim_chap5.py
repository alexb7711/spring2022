"""
mavsim_python
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/2/2019 - RWB
        1/22 - GND
"""

from mav_sim.chap3.mav_dynamics import DynamicState
from mav_sim.chap4.run_sim import run_sim
from mav_sim.chap5.trim import compute_trim
from mav_sim.message_types.msg_sim_params import MsgSimParams


def main() -> None:
    """Provide a test scenario
    """
    # Initialize the simulation parameters
    sim_params = MsgSimParams(end_time=100., video_name="chap5.avi") # Sim ending in 10 seconds
    state = DynamicState()

    # use compute_trim function to compute trim state and trim input
    Va_trim = 35.
    gamma_trim = 0.
    trim_state, trim_input = compute_trim(state.convert_to_numpy(), Va_trim, gamma_trim)

    # Create a function for perturbing the trim input
    delta_fnc = lambda _: trim_input

    # Run the simulation - Note that while not used, the viewer objects
    # need to remain active to keep the windows open
    (mav_view, data_view) = run_sim(sim=sim_params, init_state=DynamicState(trim_state), delta_fnc=delta_fnc) #pylint: disable=unused-variable

    # Wait until user is finished
    print("Press any key to close")
    input()

if __name__ == "__main__":
    main()
