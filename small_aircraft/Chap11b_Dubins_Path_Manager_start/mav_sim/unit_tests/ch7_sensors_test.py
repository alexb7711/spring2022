"""ch7_sensors_tests.py: Implements some basic tests for the chapter 7 martial."""
# pylint: disable=too-many-lines

import itertools
import os
import pickle
from typing import Any, Dict, List

import numpy as np
from mav_sim.chap7 import mav_dynamics
from mav_sim.message_types.msg_sensors import MsgSensors
from mav_sim.tools.rotations import Euler2Quaternion


def gen_accelerometer_vars() -> List[Dict[str, Any]]:
    """Generates test inputs and outputs for the accelerometer function."""
    output: List[Dict[str, Any]] = []
    # Base cases
    output.append(
        {
            "input": {
                "phi": float(0),
                "theta": float(0),
                "forces": np.zeros([3, 1], dtype=float),
                "noise_scale": float(0),
            },
        }
    )
    # Exhaustive cases
    for phi_it in np.arange(-2 * np.pi, 2 * np.pi, 1):
        for theta_it in np.arange(-2 * np.pi, 2 * np.pi, 1):
            for force_it in itertools.product(np.arange(-100, 100, 25), repeat=3):
                output.append(
                    {
                        "input": {
                            "phi": float(phi_it),
                            "theta": float(theta_it),
                            "forces": np.array(force_it, dtype=float).reshape(-1, 1),
                            "noise_scale": float(0),
                        },
                    }
                )
    # Generate output values
    for test_it in output:
        test_it["output"] = mav_dynamics.accelerometer(**test_it["input"])
    return output


def accelerometer_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the accelerometer function."""
    print("Starting accelerometer test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.accelerometer(**test_case_it["input"])

        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(np.array(calculated_output) - np.array(test_case_it["output"]))
            break
    print("End of test\n")


def gen_gyro_vars() -> List[Dict[str, Any]]:
    """Generates test inputs and outputs for the gyro function."""
    output: List[Dict[str, Any]] = []
    # Base cases
    output.append(
        {
            "input": {
                "p": float(0),
                "q": float(0),
                "r": float(0),
                "noise_scale": float(0),
            },
        }
    )
    # Exhaustive cases
    for p_it in np.arange(-10, 10, 2):
        for q_it in np.arange(-10, 10, 2):
            for r_it in np.arange(-10, 10, 2):
                output.append(
                    {
                        "input": {
                            "p": float(p_it),
                            "q": float(q_it),
                            "r": float(r_it),
                            "noise_scale": float(0),
                        },
                    }
                )
    # Generate output values
    for test_it in output:
        test_it["output"] = mav_dynamics.gyro(**test_it["input"])
    return output


def gyro_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the gyro function."""
    print("Starting gyro test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.gyro(**test_case_it["input"])
        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(np.array(calculated_output) - np.array(test_case_it["output"]))
            break
    print("End of test\n")


def gen_pressure_vars() -> List[Dict[str, Any]]:
    """Generates test inputs and outputs for the pressure function."""
    output: List[Dict[str, Any]] = []
    # Base cases
    output.append(
        {
            "input": {
                "down": float(0),
                "Va": float(0),
                "noise_scale": float(0),
            },
        }
    )
    # Exhaustive cases
    for down_it in np.arange(-1000, 1000, 100):
        for Va_it in np.arange(0, 100, 10):
            output.append(
                {
                    "input": {
                        "down": float(down_it),
                        "Va": float(Va_it),
                        "noise_scale": float(0),
                    },
                }
            )
    # Generate output values
    for test_it in output:
        test_it["output"] = mav_dynamics.pressure(**test_it["input"])
    return output


def pressure_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the pressure function."""
    print("Starting pressure test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.pressure(**test_case_it["input"])
        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(np.array(calculated_output) - np.array(test_case_it["output"]))
            break
    print("End of test\n")


def gen_magnometer_vars() -> List[Dict[str, Any]]:
    """Generates test inputs and outputs for the magnometer function."""
    output: List[Dict[str, Any]] = []
    # Base cases
    output.append(
        {
            "input": {
                "quat_b_to_i": np.array([[0], [0], [0], [1]], dtype=float),
                "noise_scale": float(0),
            },
        }
    )
    # Exhaustive cases
    for roll_it in np.arange(0, 2 * np.pi, 1):
        for pitch_it in np.arange(0, 2 * np.pi, 1):
            for yaw_it in np.arange(0, 2 * np.pi, 1):
                output.append(
                    {
                        "input": {
                            "quat_b_to_i": Euler2Quaternion(
                                phi=roll_it, theta=pitch_it, psi=yaw_it
                            ),
                            "noise_scale": float(0),
                        },
                    }
                )
    # Generate output values
    for test_it in output:
        test_it["output"] = mav_dynamics.magnetometer(**test_it["input"])
    return output


def magnometer_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the magnometer function."""
    print("Starting magnometer test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.magnetometer(**test_case_it["input"])
        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(np.array(calculated_output) - np.array(test_case_it["output"]))
            break
    print("End of test\n")


def gen_gps_error_trans_update_vars() -> List[Dict[str, Any]]:
    """Generates test inputs and outputs for the gps_error_trans_update function."""
    output: List[Dict[str, Any]] = []
    # Base cases
    output.append(
        {
            "input": {
                "nu": mav_dynamics.GpsTransient(
                    nu_n=float(0), nu_e=float(0), nu_h=float(0)
                ),
                "noise_scale": float(0),
            },
        }
    )
    # Exhaustive cases
    for north_it in np.arange(-100, 100, 10):
        for east_it in np.arange(-100, 100, 10):
            for down_it in np.arange(-100, 100, 10):
                output.append(
                    {
                        "input": {
                            "nu": mav_dynamics.GpsTransient(
                                nu_n=float(north_it),
                                nu_e=float(east_it),
                                nu_h=float(down_it),
                            ),
                            "noise_scale": float(0),
                        },
                    }
                )
    # Generate output values
    for test_it in output:
        test_it["output"] = mav_dynamics.gps_error_trans_update(**test_it["input"])
    return output


def gps_error_trans_update_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the gps_error_trans_update function."""
    print("Starting gps_error_trans_update test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.gps_error_trans_update(**test_case_it["input"])
        if (
            1e-12
            < np.abs(calculated_output.to_array() - test_case_it["output"].to_array())
        ).any():
            print("Failed test!")
            print("Calculated output:")
            calculated_output.print()
            print("Expected output:")
            test_case_it["output"].print()
            print("Difference:")
            print(calculated_output.to_array() - test_case_it["output"].to_array())
            break
    print("End of test\n")


def gen_gps_vars() -> List[Dict[str, Any]]:
    """Generates test inputs and outputs for the gps function."""
    output: List[Dict[str, Any]] = []
    # Base cases
    output.append(
        {
            "input": {
                "position": np.zeros([3, 1], dtype=float),
                "V_g_b": np.zeros([3, 1], dtype=float),
                "e_quat": np.array([[0], [0], [0], [1]], dtype=float),
                "nu": mav_dynamics.GpsTransient(
                    nu_n=float(0), nu_e=float(0), nu_h=float(0)
                ),
                "noise_scale": float(0),
            },
        }
    )
    # Exhaustive cases
    for position_it in itertools.product(np.arange(-1000, 1000, 500), repeat=3):
        position = np.array(position_it, dtype=float).reshape(-1, 1)
        for V_g_b_it in itertools.product(np.arange(-50, 50, 25), repeat=3):
            V_g_b = np.array(V_g_b_it, dtype=float).reshape(-1, 1)
            for direction_it in itertools.product(np.arange(0, 2 * np.pi, 3), repeat=3):
                e_quat = Euler2Quaternion(
                    direction_it[0], direction_it[1], direction_it[2]
                )
                for nu_it in itertools.product(np.arange(-25, 25, 50), repeat=3):
                    output.append(
                        {
                            "input": {
                                "position": position,
                                "V_g_b": V_g_b,
                                "e_quat": e_quat,
                                "nu": mav_dynamics.GpsTransient(
                                    nu_n=float(nu_it[0]),
                                    nu_e=float(nu_it[1]),
                                    nu_h=float(nu_it[2]),
                                ),
                                "noise_scale": float(0),
                            },
                        }
                    )
    # Generate output values
    for test_it in output:
        test_it["output"] = mav_dynamics.gps(**test_it["input"])
    return output


def gps_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the gps function."""
    print("Starting gps test")
    for test_case_it in test_cases:
        calculated_output = mav_dynamics.gps(**test_case_it["input"])
        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(np.array(calculated_output) - np.array(test_case_it["output"]))
            break
    print("End of test\n")


def gen_calculate_sensor_readings_vars() -> List[Dict[str, Any]]:
    """Generates test inputs and outputs for the calculate_sensor_readings function."""
    output: List[Dict[str, Any]] = []
    # Base cases
    output.append(
        {
            "input": {
                "state": np.array(
                    [
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [1],
                        [0],
                        [0],
                        [0],
                    ],
                    dtype=float,
                ),
                "forces": np.zeros([3, 1], dtype=float),
                "nu": mav_dynamics.GpsTransient(
                    nu_n=float(0), nu_e=float(0), nu_h=float(0)
                ),
                "Va": float(0),
                "sensors_prev": MsgSensors(
                    gyro_x=float(0),
                    gyro_y=float(0),
                    gyro_z=float(0),
                    accel_x=float(0),
                    accel_y=float(0),
                    accel_z=float(0),
                    mag_x=float(0),
                    mag_y=float(0),
                    mag_z=float(0),
                    abs_pressure=float(0),
                    diff_pressure=float(0),
                    gps_n=float(0),
                    gps_e=float(0),
                    gps_h=float(0),
                    gps_Vg=float(0),
                    gps_course=float(0),
                ),
                "update_gps": False,
                "noise_scale": float(0),
            },
        }
    )
    output.append(
        {
            "input": {
                "state": np.array(
                    [
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [1],
                        [0],
                        [0],
                        [0],
                    ],
                    dtype=float,
                ),
                "forces": np.zeros([3, 1], dtype=float),
                "nu": mav_dynamics.GpsTransient(
                    nu_n=float(0), nu_e=float(0), nu_h=float(0)
                ),
                "Va": float(0),
                "sensors_prev": MsgSensors(
                    gyro_x=float(0),
                    gyro_y=float(0),
                    gyro_z=float(0),
                    accel_x=float(0),
                    accel_y=float(0),
                    accel_z=float(0),
                    mag_x=float(0),
                    mag_y=float(0),
                    mag_z=float(0),
                    abs_pressure=float(0),
                    diff_pressure=float(0),
                    gps_n=float(0),
                    gps_e=float(0),
                    gps_h=float(0),
                    gps_Vg=float(0),
                    gps_course=float(0),
                ),
                "update_gps": True,
                "noise_scale": float(0),
            },
        }
    )
    # Exhaustive cases
    # pylint: disable=too-many-nested-blocks
    for position_it in itertools.product(np.arange(-500, 500, 1000), repeat=3):
        position = np.array(position_it, dtype=float).reshape(-1, 1)
        for V_g_b_it in itertools.product(np.arange(-25, 25, 50), repeat=3):
            V_g_b = np.array(V_g_b_it, dtype=float).reshape(-1, 1)
            for direction_it in itertools.product(np.arange(0, 2 * np.pi, 3), repeat=3):
                e_quat = Euler2Quaternion(
                    direction_it[0], direction_it[1], direction_it[2]
                )
                for angle_rate_it in itertools.product(np.arange(-5, 5, 10), repeat=3):
                    angle_rate = np.array(angle_rate_it, dtype=float).reshape(-1, 1)
                    state = np.concatenate(
                        (position, V_g_b, e_quat, angle_rate)
                    ).reshape(-1, 1)
                    for forces_it in itertools.product(np.arange(-5, 5, 10), repeat=3):
                        forces = np.array(forces_it, dtype=float).reshape(-1, 1)
                        for nu_it in itertools.product(
                            np.arange(-25, 25, 50), repeat=3
                        ):
                            for Va_it in np.arange(0, 20, 20):
                                for gyro_accel_mag_it in itertools.product(
                                    np.arange(-5, 5, 10), repeat=9
                                ):
                                    for abs_pressure_it in np.arange(0, 20, 10):
                                        for diff_pressure_it in np.arange(-25, 25, 25):
                                            for gps_position_it in itertools.product(
                                                np.arange(-500, 500, 1000), repeat=3
                                            ):
                                                for gps_Va_it in np.arange(0, 20, 20):
                                                    for gps_course_it in np.arange(
                                                        0, 2 * np.pi, 3
                                                    ):
                                                        for update_gps_it in (
                                                            True,
                                                            False,
                                                        ):
                                                            output.append(
                                                                {
                                                                    "input": {
                                                                        "state": state,
                                                                        "forces": forces,
                                                                        "nu": mav_dynamics.GpsTransient(
                                                                            nu_n=float(
                                                                                nu_it[0]
                                                                            ),
                                                                            nu_e=float(
                                                                                nu_it[1]
                                                                            ),
                                                                            nu_h=float(
                                                                                nu_it[2]
                                                                            ),
                                                                        ),
                                                                        "Va": float(
                                                                            Va_it
                                                                        ),
                                                                        "sensors_prev": MsgSensors(
                                                                            gyro_x=float(
                                                                                gyro_accel_mag_it[
                                                                                    0
                                                                                ]
                                                                            ),
                                                                            gyro_y=float(
                                                                                gyro_accel_mag_it[
                                                                                    1
                                                                                ]
                                                                            ),
                                                                            gyro_z=float(
                                                                                gyro_accel_mag_it[
                                                                                    2
                                                                                ]
                                                                            ),
                                                                            accel_x=float(
                                                                                gyro_accel_mag_it[
                                                                                    3
                                                                                ]
                                                                            ),
                                                                            accel_y=float(
                                                                                gyro_accel_mag_it[
                                                                                    4
                                                                                ]
                                                                            ),
                                                                            accel_z=float(
                                                                                gyro_accel_mag_it[
                                                                                    5
                                                                                ]
                                                                            ),
                                                                            mag_x=float(
                                                                                gyro_accel_mag_it[
                                                                                    6
                                                                                ]
                                                                            ),
                                                                            mag_y=float(
                                                                                gyro_accel_mag_it[
                                                                                    7
                                                                                ]
                                                                            ),
                                                                            mag_z=float(
                                                                                gyro_accel_mag_it[
                                                                                    8
                                                                                ]
                                                                            ),
                                                                            abs_pressure=float(
                                                                                abs_pressure_it
                                                                            ),
                                                                            diff_pressure=float(
                                                                                diff_pressure_it
                                                                            ),
                                                                            gps_n=float(
                                                                                gps_position_it[
                                                                                    0
                                                                                ]
                                                                            ),
                                                                            gps_e=float(
                                                                                gps_position_it[
                                                                                    1
                                                                                ]
                                                                            ),
                                                                            gps_h=float(
                                                                                gps_position_it[
                                                                                    2
                                                                                ]
                                                                            ),
                                                                            gps_Vg=float(
                                                                                gps_Va_it
                                                                            ),
                                                                            gps_course=float(
                                                                                gps_course_it
                                                                            ),
                                                                        ),
                                                                        "update_gps": update_gps_it,
                                                                        "noise_scale": float(
                                                                            0
                                                                        ),
                                                                    },
                                                                }
                                                            )
    # Generate output values
    for test_it in output:
        temp = mav_dynamics.calculate_sensor_readings(**test_it["input"])
        test_it["output"] = np.concatenate((temp[0].to_array(), temp[1].to_array()))
    return output


def calculate_sensor_readings_test(test_cases: List[Dict[str, Any]]) -> None:
    """Tests the calculate_sensor_readings function."""
    print("Starting calculate_sensor_readings test")
    for test_case_it in test_cases:
        temp = mav_dynamics.calculate_sensor_readings(**test_case_it["input"])
        calculated_output = np.concatenate((temp[0].to_array(), temp[1].to_array()))
        if (
            1e-12
            < np.abs(np.array(calculated_output) - np.array(test_case_it["output"]))
        ).any():
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output)
            print("Expected output:")
            print(test_case_it["output"])
            print("Difference:")
            print(calculated_output - test_case_it["output"])
            break
    print("End of test\n")


def gen_test_archive() -> None:
    """Makes the input/output archive of awnsers."""
    archive: Dict[str, List[Dict[str, Any]]] = {}
    # Gather test cases
    archive["accelerometer"] = gen_accelerometer_vars()
    archive["gyro"] = gen_gyro_vars()
    archive["pressure"] = gen_pressure_vars()
    archive["magnometer"] = gen_magnometer_vars()
    archive["gps_error_tran_update"] = gen_gps_error_trans_update_vars()
    archive["gps"] = gen_gps_vars()
    archive["calculate_sensor_readings"] = gen_calculate_sensor_readings_vars()
    # Save to archive
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch7_test_archive.pkl"
        ),
        "wb",
    ) as file:
        pickle.dump(archive, file)


def run_all_tests() -> None:
    """Run all tests."""
    # Open archive
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch7_test_archive.pkl"
        ),
        "rb",
    ) as file:
        tests_archive = pickle.load(file)
    # Run tests
    accelerometer_test(tests_archive["accelerometer"])
    gyro_test(tests_archive["gyro"])
    pressure_test(tests_archive["pressure"])
    magnometer_test(tests_archive["magnometer"])
    gps_error_trans_update_test(tests_archive["gps_error_tran_update"])
    gps_test(tests_archive["gps"])
    calculate_sensor_readings_test(tests_archive["calculate_sensor_readings"])


if __name__ == "__main__":
    # gen_test_archive()
    run_all_tests()
