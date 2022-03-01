"""ch3_derivatives_tests.py: Implements some basic tests for the dirivatives funciton."""

import mav_sim.parameters.aerosonde_parameters as MAV
import numpy as np
from mav_sim.chap3.mav_dynamics import StateIndices, derivatives

IND = StateIndices()


def derivatives_test() -> None:
    """Tests the derivatives function."""
    print("Starting derivatives test\n")
    # Inputs
    inputs = [
        {
            "state": np.array(
                [
                    MAV.north0,
                    MAV.east0,
                    MAV.down0,
                    MAV.u0,
                    MAV.v0,
                    MAV.w0,
                    MAV.e0,
                    MAV.e1,
                    MAV.e2,
                    MAV.e3,
                    MAV.p0,
                    MAV.q0,
                    MAV.r0,
                ],
                dtype=float,
            ),
            "forces_moments": np.zeros([6, 1], dtype=float),
        },
        {
            "state": np.array(
                [
                    MAV.north0 + 1,
                    MAV.east0 + 2,
                    MAV.down0 + 3,
                    MAV.u0 + 4,
                    MAV.v0 + 5,
                    MAV.w0 + 6,
                    MAV.e0 + 7,
                    MAV.e1 + 8,
                    MAV.e2 + 9,
                    MAV.e3 + 10,
                    MAV.p0 + 11,
                    MAV.q0 + 12,
                    MAV.r0 + 13,
                ],
                dtype=float,
            ),
            "forces_moments": np.array([1, 2, 3, 4, 5, 6], dtype=float),
        },
    ]
    inputs[1]["state"][IND.QUAT] = inputs[1]["state"][IND.QUAT] / np.linalg.norm(
        inputs[1]["state"][IND.QUAT]
    )
    # Expected outputs
    outputs = [
        np.array(
            [[25], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]],
            dtype=float,
        ),
        np.array(
            [
                [0.66990291],
                [29.23300971],
                [6.85436893],
                [-6.90909091],
                [-310.81818182],
                [293.27272727],
                [-9.27274602],
                [2.41774053],
                [2.90128863],
                [2.87284463],
                [-99.40765904],
                [127.24845815],
                [-37.37935313],
            ],
            dtype=float,
        ),
    ]

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = derivatives(**input_it)  # type: ignore

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-6 < np.abs(calculated_output - output_it)).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


if __name__ == "__main__":
    derivatives_test()
