"""ch5_derivatives_tests.py: Implements some basic tests for the chapter 5 martial."""


from typing import Any, Dict, List

import numpy as np
from mav_sim.chap3.mav_dynamics import IND
from mav_sim.chap5.compute_models import compute_ss_model
from mav_sim.chap5.trim import trim_objective_fun
from mav_sim.message_types.msg_delta import MsgDelta


def trim_objective_fun_test() -> None:
    """Tests the trim_objective_fun function."""
    print("Starting trim_objective_fun test\n")
    # Inputs
    inputs: List[Dict[str, Any]] = [
        {
            "x": np.array(
                [
                    [1],
                    [2],
                    [3],
                    [4],
                    [5],
                    [6],
                    [7],
                    [8],
                    [9],
                    [10],
                    [11],
                    [12],
                    [13],
                    [14],
                    [15],
                    [16],
                    [17],
                ],
                dtype=float,
            ),
            "Va": float(0.0),
            "gamma": float(0.0),
        },
        {
            "x": np.array(
                [
                    [6.19506532e01],
                    [2.22940203e01],
                    [-1.10837551e02],
                    [2.73465947e01],
                    [6.19628233e-01],
                    [1.42257772e00],
                    [9.38688796e-01],
                    [2.47421558e-01],
                    [6.56821468e-02],
                    [2.30936730e-01],
                    [4.98772167e-03],
                    [1.68736005e-01],
                    [1.71797313e-01],
                    [-0.124778],
                    [0.001836],
                    [-0.000303],
                    [0.676752],
                ],
                dtype=float,
            ),
            "Va": float(22.4),
            "gamma": float(10 * np.pi / 180),
        },
    ]
    for input_it in inputs:
        input_it["x"][IND.QUAT] = input_it["x"][IND.QUAT] / np.linalg.norm(
            input_it["x"][IND.QUAT]
        )
    # Expected outputs
    outputs = [float(1437901.491194673), float(29.780760107376643)]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = trim_objective_fun(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-6 < np.abs(np.array(calculated_output) - np.array(output_it))).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def compute_ss_model_test() -> None:
    """Tests the compute_ss_model function."""
    print("Starting compute_ss_model test\n")
    # Inputs
    inputs: List[Dict[str, Any]] = [
        {
            "trim_state": np.array(
                [
                    [0.13],
                    [0.12],
                    [0.11],
                    [0.10],
                    [0.9],
                    [0.8],
                    [0.7],
                    [0.6],
                    [0.5],
                    [0.4],
                    [0.3],
                    [0.2],
                    [0.1],
                ],
                dtype=float,
            ),
            "trim_input": MsgDelta(
                elevator=float(0), aileron=float(0), rudder=float(0), throttle=float(0)
            ),
        },
        {
            "trim_state": np.array(
                [
                    [6.19506532e01],
                    [2.22940203e01],
                    [-1.10837551e02],
                    [2.73465947e01],
                    [6.19628233e-01],
                    [1.42257772e00],
                    [9.38688796e-01],
                    [2.47421558e-01],
                    [6.56821468e-02],
                    [2.30936730e-01],
                    [4.98772167e-03],
                    [1.68736005e-01],
                    [1.71797313e-01],
                ],
                dtype=float,
            ),
            "trim_input": MsgDelta(
                elevator=float(-0.124778),
                aileron=float(0.001836),
                rudder=float(-0.000303),
                throttle=float(0.676752),
            ),
        },
    ]
    for input_it in inputs:
        input_it["trim_state"][IND.QUAT] = input_it["trim_state"][
            IND.QUAT
        ] / np.linalg.norm(input_it["trim_state"][IND.QUAT])
    # Expected outputs
    outputs = [
        (
            np.array(
                [
                    [
                        2.90534196e-02,
                        -2.03329066e-01,
                        -7.71299394e-01,
                        -9.65058225,
                        -0.0,
                    ],
                    [
                        2.87667814e-01,
                        -9.62708070e-02,
                        9.64124243e-02,
                        -5.67809402e-02,
                        -0.0,
                    ],
                    [
                        2.35224406e-01,
                        -4.35155387e-01,
                        -2.55906265e-01,
                        0.0,
                        -0.0,
                    ],
                    [
                        0.0,
                        0.0,
                        3.21948737e-02,
                        -4.58935605e-05,
                        -0.0,
                    ],
                    [
                        1.74603175e-01,
                        -3.17460317e-02,
                        -0.0,
                        2.64492478e-01,
                        0.0,
                    ],
                ],
                dtype=float,
            ),
            np.array(
                [
                    [
                        0.00589363,
                        -0.05824421,
                    ],
                    [
                        -0.00136647,
                        0.0,
                    ],
                    [
                        -0.08435854,
                        0.0,
                    ],
                    [
                        0.0,
                        0.0,
                    ],
                    [
                        -0.0,
                        -0.0,
                    ],
                ],
                dtype=float,
            ),
            np.array(
                [
                    [
                        -7.24623196e-02,
                        8.0e-01,
                        -1.0e-01,
                        2.63152355e-01,
                        0.0,
                    ],
                    [
                        -5.24714851e-01,
                        -1.06940747e00,
                        3.72133535e-01,
                        0.0,
                        0.0,
                    ],
                    [
                        6.87758763e-02,
                        -3.92152562e-02,
                        -8.36295383e-02,
                        0.0,
                        0.0,
                    ],
                    [
                        0.0,
                        9.99933635e-01,
                        5.35388855e-03,
                        -1.67770359e-02,
                        1.98913962e-04,
                    ],
                    [
                        0.0,
                        -1.69444258e-05,
                        3.26513437e-02,
                        -9.58522017e-02,
                        3.73691535e-05,
                    ],
                ],
                dtype=float,
            ),
            np.array(
                [
                    [
                        0.0034717,
                        0.00879497,
                    ],
                    [
                        0.30574427,
                        -0.00419633,
                    ],
                    [
                        0.01170741,
                        -0.05812281,
                    ],
                    [
                        0.0,
                        0.0,
                    ],
                    [
                        0.0,
                        0.0,
                    ],
                ],
                dtype=float,
            ),
        ),
        (
            np.array(
                [
                    [
                        -2.20265873e-01,
                        4.00105112e-01,
                        -1.38851585,
                        -9.80899325,
                        -0.0,
                    ],
                    [
                        -4.58747545e-01,
                        -4.89295597,
                        2.66918142e01,
                        -1.19620371e-01,
                        -0.0,
                    ],
                    [
                        1.74846025e-01,
                        -4.37823559,
                        -5.80103825,
                        0.0,
                        -0.0,
                    ],
                    [
                        0.0,
                        0.0,
                        8.68868663e-01,
                        -1.45951853e-04,
                        -0.0,
                    ],
                    [
                        9.03273952e-03,
                        -8.68936857e-01,
                        -0.0,
                        2.73654375e01,
                        0.0,
                    ],
                ],
                dtype=float,
            ),
            np.array(
                [
                    [
                        -0.16004176,
                        8.03086201,
                    ],
                    [
                        -3.10474937,
                        0.0,
                    ],
                    [
                        -43.34896045,
                        0.0,
                    ],
                    [
                        0.0,
                        0.0,
                    ],
                    [
                        -0.0,
                        -0.0,
                    ],
                ],
                dtype=float,
            ),
            np.array(
                [
                    [
                        -8.51714287e-01,
                        1.42257772,
                        -2.73465947e01,
                        8.49985677,
                        1.77635684e-13,
                    ],
                    [
                        -4.23456493,
                        -2.47721978e01,
                        1.18171038e01,
                        0.0,
                        0.0,
                    ],
                    [
                        8.58685493e-01,
                        -1.54489161e-01,
                        -1.36554366,
                        0.0,
                        0.0,
                    ],
                    [
                        0.0,
                        9.99803542e-01,
                        7.52232694e-03,
                        4.57332277e-04,
                        6.84717318e-06,
                    ],
                    [
                        0.0,
                        -1.81663545e-04,
                        8.68704383e-01,
                        6.04210285e-02,
                        -4.96668697e-05,
                    ],
                ],
                dtype=float,
            ),
            np.array(
                [
                    [
                        1.78398623,
                        4.51943179,
                    ],
                    [
                        157.1114916,
                        -2.15635028,
                    ],
                    [
                        6.01603786,
                        -29.86731962,
                    ],
                    [
                        0.0,
                        0.0,
                    ],
                    [
                        0.0,
                        0.0,
                    ],
                ],
                dtype=float,
            ),
        ),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = compute_ss_model(**input_it)

        print("Testing A_lon")
        print("Calculated output:")
        print(calculated_output[0])
        print("Expected output:")
        print(output_it[0])
        if (
            1e-6 < np.abs(np.array(calculated_output[0]) - np.array(output_it[0]))
        ).any():
            print("Failed test!")
        else:
            print("Passed test")
        print("Testing B_lon")
        print("Calculated output:")
        print(calculated_output[1])
        print("Expected output:")
        print(output_it[1])
        if (
            1e-6 < np.abs(np.array(calculated_output[1]) - np.array(output_it[1]))
        ).any():
            print("Failed test!")
        else:
            print("Passed test")
        print("Testing A_lat")
        print("Calculated output:")
        print(calculated_output[2])
        print("Expected output:")
        print(output_it[2])
        if (
            1e-6 < np.abs(np.array(calculated_output[2]) - np.array(output_it[2]))
        ).any():
            print("Failed test!")
        else:
            print("Passed test")
        print("Testing B_lat")
        print("Calculated output:")
        print(calculated_output[3])
        print("Expected output:")
        print(output_it[3])
        if (
            1e-6 < np.abs(np.array(calculated_output[3]) - np.array(output_it[3]))
        ).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def run_all_tests() -> None:
    """Run all tests."""
    trim_objective_fun_test()
    # compute_ss_model_test() # No need to run as the compute_ss_model function is provided


if __name__ == "__main__":
    run_all_tests()
