"""ch4_derivatives_tests.py: Implements some basic tests for the chapter 4 martial."""


import numpy as np
from mav_sim.chap3.mav_dynamics import IND
from mav_sim.chap4.mav_dynamics import (
    forces_moments,
    motor_thrust_torque,
    update_velocity_data,
)
from mav_sim.chap4.wind_simulation import WindSimulation
from mav_sim.message_types.msg_delta import MsgDelta


def motor_thrust_torque_test() -> None:
    """Tests the motor_thrust_torque function."""
    print("Starting motor_thrust_torque test\n")
    # Inputs
    inputs = [
        {
            "Va": 1,
            "delta_t": 0,
        },
        {
            "Va": 27.39323489287441,
            "delta_t": 0.5,
        },
    ]
    # Expected outputs
    outputs = [
        (-0.033654137677838994, -0.002823682854958359),
        (-17.91352683604895, -0.7758235361365506),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = motor_thrust_torque(**input_it)  # type: ignore

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-6 < np.abs(np.array(calculated_output) - np.array(output_it))).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def forces_moments_test() -> None:
    """Tests the forces_moments function."""
    print("Starting forces_moments test\n")
    # Inputs
    inputs = [
        {
            "state": np.array(
                [
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                ]
            ),
            "delta": MsgDelta(0, 0, 0, 0),
            "Va": 1,
            "beta": 0,
            "alpha": 0,
        },
        {
            "state": np.array(
                [
                    24.283238643486627,
                    12.605130052025968,
                    1.2957327060769266,
                    3.2299908091423797,
                    0.2308339966235602,
                    8.881532282520418,
                    -0.025995661302161892,
                    -0.011500703223228347,
                    0.05851804333262313,
                    0.10134276693843723,
                    1.8427420637214973,
                    5.2743652738342774,
                    -0.5471458931221012,
                ]
            ),
            "delta": MsgDelta(-0.2, 0.001, 0.005, 0.5),
            "Va": 27.39323489287441,
            "beta": 0.022795289526122853,
            "alpha": 0.05259649205640062,
        },
    ]
    for input_it in inputs:
        input_it["state"][IND.QUAT] = input_it["state"][IND.QUAT] / np.linalg.norm(  # type: ignore
            input_it["state"][IND.QUAT]  # type: ignore
        )
    # Expected outputs
    outputs = [
        np.array(
            [
                [-3.40821628e-02],
                [0],
                [1.07829786e02],
                [2.82368285e-03],
                [8.94274083e-04],
                [0],
            ]
        ),
        np.array(
            [
                [-4.71982679],
                [87.12283471],
                [-113.4856833],
                [-44.44992726],
                [-31.38114459],
                [8.16544191],
            ]
        ),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = forces_moments(**input_it)  # type: ignore

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-6 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def update_velocity_data_test() -> None:
    """Tests the update_velocity_data function."""
    print("Starting update_velocity_data test\n")
    # Inputs
    inputs = [
        {
            "state": np.array(
                [
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                ]
            ),
            "wind": np.zeros([6, 1]),
        },
        {
            "state": np.array(
                [
                    24.283238643486627,
                    12.605130052025968,
                    1.2957327060769266,
                    3.2299908091423797,
                    0.2308339966235602,
                    8.881532282520418,
                    -0.025995661302161892,
                    -0.011500703223228347,
                    0.05851804333262313,
                    0.10134276693843723,
                    1.8427420637214973,
                    5.2743652738342774,
                    -0.5471458931221012,
                ]
            ),
            "wind": np.array([1, -2, 3, -0.004, 0.005, -0.006]),
        },
    ]
    for input_it in inputs:
        input_it["state"][IND.QUAT] = input_it["state"][IND.QUAT] / np.linalg.norm(
            input_it["state"][IND.QUAT]
        )
    # Expected outputs
    outputs = [
        (0, 0, 0, np.zeros([3, 1])),
        (
            10.379686170818202,
            1.252059888018447,
            -0.34301482267104366,
            np.array([1.00712983, -2.00500799, 3.00104193]),
        ),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = update_velocity_data(**input_it)  # type: ignore

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (
            1e-6 < np.abs(np.array(calculated_output[0:2]) - np.array(output_it[0:2]))
        ).any() or (1e-6 < np.abs(calculated_output[3] - output_it[3])).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def wind_simulation_test() -> None:
    """Tests the WindSimulation class."""
    print("Starting WindSimulation test\n")
    # Inputs
    inputs = [
        {"u_w": 1, "v_w": 1, "w_w": 1},
        {"u_w": -0.5, "v_w": 0.35, "w_w": -0.89},
    ]
    # Expected outputs
    outputs = [
        np.array([0.00529669, 0.01058676, 0.01587022]),
        np.array([0.01320205, 0.0150394, 0.01030656]),
    ]

    test_class = WindSimulation(Ts=0.01, gust_flag=True)

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = np.array(
            [
                test_class.u_w.update(input_it["u_w"]),  # type: ignore
                test_class.u_w.update(input_it["v_w"]),  # type: ignore
                test_class.u_w.update(input_it["w_w"]),  # type: ignore
            ]
        )

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-6 < np.abs(np.array(calculated_output) - np.array(output_it))).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def run_all_tests() -> None:
    """Run all tests."""
    motor_thrust_torque_test()
    forces_moments_test()
    update_velocity_data_test()
    wind_simulation_test()


if __name__ == "__main__":
    run_all_tests()
