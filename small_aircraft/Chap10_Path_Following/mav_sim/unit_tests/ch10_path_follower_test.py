"""ch7_sensors_tests.py: Implements some basic tests for the chapter 7 martial."""
# pylint: disable=too-many-lines

import itertools
import os
import pickle
from typing import Any, Dict, List, Literal, Tuple, Union, cast

import numpy as np
from mav_sim.chap10.path_follower import follow_orbit, follow_straight_line
from mav_sim.message_types.msg_path import MsgPath
from mav_sim.message_types.msg_state import MsgState


def follow_straight_line_test(test_cases: List[Tuple[Dict[str, Any], Any]]) -> None:
    """Tests the follow_straight_line function."""
    print("Starting follow_straight_line test")
    for test_case_it in test_cases:
        calculated_output = follow_straight_line(**test_case_it[0])

        if (
            (
                1e-12
                < np.abs(calculated_output.to_array() - test_case_it[1].to_array())
            ).any()
            or np.isnan(calculated_output.to_array()).any()
            or np.isinf(calculated_output.to_array()).any()
        ):
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output.to_array())
            print("Expected output:")
            print(test_case_it[1].to_array())
            print("Difference:")
            print(calculated_output.to_array() - test_case_it[1].to_array())
            break
    print("End of test\n")


def follow_orbit_test(test_cases: List[Tuple[Dict[str, Any], Any]]) -> None:
    """Tests the follow_orbit function."""
    print("Starting follow_orbit test")
    for test_case_it in test_cases:
        calculated_output = follow_orbit(**test_case_it[0])

        if (
            (
                1e-12
                < np.abs(calculated_output.to_array() - test_case_it[1].to_array())
            ).any()
            or np.isnan(calculated_output.to_array()).any()
            or np.isinf(calculated_output.to_array()).any()
        ):
            print("Failed test!")
            print("Calculated output:")
            print(calculated_output.to_array())
            print("Expected output:")
            print(test_case_it[1].to_array())
            print("Difference:")
            print(calculated_output.to_array() - test_case_it[1].to_array())
            break
    print("End of test\n")


def run_all_tests() -> None:
    """Run all tests."""
    # Open archive
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch10_test_archive.pkl"
        ),
        "rb",
    ) as file:
        tests_archive = pickle.load(file)
    # Run tests
    follow_straight_line_test(tests_archive["follow_straight_line"])
    follow_orbit_test(tests_archive["follow_orbit"])


if __name__ == "__main__":
    run_all_tests()
