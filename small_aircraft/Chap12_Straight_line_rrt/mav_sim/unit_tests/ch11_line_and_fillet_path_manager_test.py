"""ch11_line_and_fillet_path_manager_tests.py: Implements some basic tests for the chapter 11 martial."""
# pylint: disable=too-many-lines

import gc
import itertools
import os
import pickle
from typing import Any, Dict, List, Tuple

import numpy as np
from mav_sim.chap11.fillet_manager import construct_fillet_circle, construct_fillet_line
from mav_sim.chap11.line_manager import construct_line
from mav_sim.chap11.path_manager_utilities import (
    HalfSpaceParams,
    WaypointIndices,
    inHalfSpace,
)
from mav_sim.message_types.msg_waypoints import MsgWaypoints

PRECISION = 1e-4

def gen_inHalfSpace_vars() -> List[Tuple[Dict[str, Any], Any]]:
    """Generates test inputs and outputs for the inHlafSpace function."""
    output: List[Tuple[Dict[str, Any], Any]] = []
    # Base cases
    inputs: Dict[str, Any] = {
        "pos": np.zeros([3, 1], dtype=float),
        "hs": HalfSpaceParams(
            normal=np.zeros([3, 1], dtype=float),
            point=np.zeros([3, 1], dtype=float),
        ),
    }
    output.append((inputs, inHalfSpace(**inputs)))
    # Exhaustive cases
    for pos in itertools.product(np.arange(-10, 10.1, 5), repeat=3):
        for normal in itertools.product(np.arange(-10, 10.1, 5), repeat=3):
            for point in itertools.product(np.arange(-10, 10.1, 5), repeat=3):
                inputs = {
                    "pos": np.array(pos, dtype=float).reshape(-1, 1),
                    "hs": HalfSpaceParams(
                        normal=np.array(normal, dtype=float).reshape(-1, 1),
                        point=np.array(point, dtype=float).reshape(-1, 1),
                    ),
                }
                output.append((inputs, inHalfSpace(**inputs)))

    return output


def inHalfSpace_test(test_case: Tuple[Dict[str, Any], Any]) -> bool:
    """Tests the inHalfSpace function."""
    calculated_output = inHalfSpace(**test_case[0])

    if bool(calculated_output) != bool(test_case[1]):
        print("Failed test!")
        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(test_case[1])
        return False
    return True


def gen_construct_line_vars() -> List[Tuple[Dict[str, Any], Any]]:
    """Generates test inputs and outputs for the construct_line function."""
    output: List[Tuple[Any, List[Any]]] = []
    # Exhaustive cases
    for root_pos in itertools.product(np.arange(-100, 100.1, 50), repeat=3):
        for root_airspeed in np.arange(10, 50.1, 20):
            for root_course in np.arange(-2 * np.pi, 2 * np.pi, 3):
                waypoints = MsgWaypoints()
                waypoints.add(
                    ned=np.array(root_pos, dtype=float).reshape(-1, 1),
                    airspeed=float(root_airspeed),
                    course=float(root_course),
                    cost=float(0),
                    parent=int(-1),
                    connect_to_goal=0,
                )

                for diff_pos in itertools.product(np.arange(-8, 19.1, 5), repeat=3):
                    waypoints.add(
                        ned=waypoints.ned[:, -1].reshape(-1, 1)
                        + np.array(diff_pos, dtype=float).reshape(-1, 1),
                        airspeed=float(
                            root_airspeed + (np.array(diff_pos, dtype=float).sum() % 5)
                        ),
                        course=float(
                            root_course + (np.array(diff_pos, dtype=float).sum() % 2)
                        ),
                        cost=float(waypoints.cost[-1] + 1.1),
                        parent=int(waypoints.num_waypoints - 1),
                        connect_to_goal=0,
                    )
                waypoints.connect_to_goal[-1] = 1
                ptr = WaypointIndices()
                lines = []
                for _ in range(waypoints.num_waypoints - 1):
                    lines.append(construct_line(waypoints=waypoints, ptr=ptr))
                    ptr.increment_pointers(num_waypoints=waypoints.num_waypoints)
            output.append((waypoints, lines))
    return output


def construct_line_test(test_case: Tuple[Any, List[Any]]) -> int:
    """Tests the construct_line function."""
    ptr = WaypointIndices()
    for ptr_it in range(test_case[0].num_waypoints - 1):
        calculated_output = construct_line(waypoints=test_case[0], ptr=ptr)
        # pylint: disable=too-many-boolean-expressions
        if (
            (calculated_output[0].type != test_case[1][ptr_it][0].type)
            or (
                np.abs(calculated_output[0].airspeed - test_case[1][ptr_it][0].airspeed)
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[0].line_origin)
                    - np.array(test_case[1][ptr_it][0].line_origin)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].line_direction)
                    - np.array(test_case[1][ptr_it][0].line_direction)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].orbit_center)
                    - np.array(test_case[1][ptr_it][0].orbit_center)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    calculated_output[0].orbit_radius
                    - test_case[1][ptr_it][0].orbit_radius
                )
                > PRECISION
            )
            or (
                calculated_output[0].orbit_direction
                != test_case[1][ptr_it][0].orbit_direction
            )
            or (
                np.abs(
                    (
                        np.array(calculated_output[1].normal).reshape(1, -1)
                        @ np.array(test_case[1][ptr_it][1].normal).reshape(-1, 1)
                    )
                    - 1
                )
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[1].point)
                    - np.array(test_case[1][ptr_it][1].point)
                )
                > PRECISION
            ).any()
        ):
            print("Failed test!")
            print("Calculated output:")
            calculated_output[0].print()
            calculated_output[1].print()
            print("Expected output:")
            test_case[1][ptr_it][0].print()
            test_case[1][ptr_it][1].print()
            return ptr_it
        ptr.increment_pointers(num_waypoints=test_case[0].num_waypoints)
    return -1


def gen_construct_fillet_line_vars() -> List[Tuple[Tuple[Any, Any], List[Any]]]:
    """Generates test inputs and outputs for the construct_fillet_line function."""
    output: List[Tuple[Tuple[Any, Any], List[Any]]] = []
    # Exhaustive cases
    for radius in np.arange(10, 50, 20):
        for root_pos in itertools.product(np.arange(-100, 100.1, 50), repeat=3):
            for root_airspeed in np.arange(10, 50.1, 20):
                for root_course in np.arange(-2 * np.pi+.00001, 2 * np.pi-0.00001, 3):
                    waypoints = MsgWaypoints()
                    waypoints.add(
                        ned=np.array(root_pos, dtype=float).reshape(-1, 1),
                        airspeed=float(root_airspeed),
                        course=float(root_course),
                        cost=float(0),
                        parent=int(-1),
                        connect_to_goal=0,
                    )

                    for diff_pos in itertools.product(np.arange(-8, 19.1, 5), repeat=3):
                        waypoints.add(
                            ned=waypoints.ned[:, -1].reshape(-1, 1)
                            + np.array(diff_pos, dtype=float).reshape(-1, 1),
                            airspeed=float(
                                root_airspeed
                                + (np.array(diff_pos, dtype=float).sum() % 5)
                            ),
                            course=float(
                                root_course
                                + (np.array(diff_pos, dtype=float).sum() % 2)
                            ),
                            cost=float(waypoints.cost[-1] + 1.1),
                            parent=int(waypoints.num_waypoints - 1),
                            connect_to_goal=0,
                        )
                    waypoints.connect_to_goal[-1] = 1
                    ptr = WaypointIndices()
                    lines = []
                    for _ in range(waypoints.num_waypoints - 1):
                        lines.append(
                            construct_fillet_line(
                                waypoints=waypoints, ptr=ptr, radius=radius
                            )
                        )
                        ptr.increment_pointers(num_waypoints=waypoints.num_waypoints)
                output.append(((waypoints, radius), lines))
    return output


def construct_fillet_line_test(test_case: Tuple[Tuple[Any, Any], List[Any]]) -> int:
    """Tests the construct_fillet_line function."""
    ptr = WaypointIndices()
    for ptr_it in range(test_case[0][0].num_waypoints - 1):
        calculated_output = construct_fillet_line(
            waypoints=test_case[0][0], ptr=ptr, radius=test_case[0][1]
        )
        # pylint: disable=too-many-boolean-expressions
        if (
            (calculated_output[0].type != test_case[1][ptr_it][0].type)
            or (
                np.abs(calculated_output[0].airspeed - test_case[1][ptr_it][0].airspeed)
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[0].line_origin)
                    - np.array(test_case[1][ptr_it][0].line_origin)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].line_direction)
                    - np.array(test_case[1][ptr_it][0].line_direction)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].orbit_center)
                    - np.array(test_case[1][ptr_it][0].orbit_center)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    calculated_output[0].orbit_radius
                    - test_case[1][ptr_it][0].orbit_radius
                )
                > PRECISION
            )
            or (
                calculated_output[0].orbit_direction
                != test_case[1][ptr_it][0].orbit_direction
            )
            or (
                np.abs(
                    (
                        np.array(calculated_output[1].normal).reshape(1, -1)
                        @ np.array(test_case[1][ptr_it][1].normal).reshape(-1, 1)
                    )
                    - 1
                )
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[1].point)
                    - np.array(test_case[1][ptr_it][1].point)
                )
                > PRECISION
            ).any()
        ):
            print("Failed test!")
            print("Calculated output:")
            calculated_output[0].print()
            calculated_output[1].print()
            print("Expected output:")
            test_case[1][ptr_it][0].print()
            test_case[1][ptr_it][1].print()
            return ptr_it
        ptr.increment_pointers(num_waypoints=test_case[0][0].num_waypoints)
    return -1


def gen_construct_fillet_circle_vars() -> List[Tuple[Tuple[Any, Any], List[Any]]]:
    """Generates test inputs and outputs for the construct_fillet_circle function."""
    output: List[Tuple[Tuple[Any, Any], List[Any]]] = []
    # Exhaustive cases
    for radius in np.arange(10, 50, 20):
        for root_pos in itertools.product(np.arange(-100, 100.1, 50), repeat=3):
            for root_airspeed in np.arange(10, 50.1, 20):
                for root_course in np.arange(-2 * np.pi, 2 * np.pi, 3):
                    waypoints = MsgWaypoints()
                    waypoints.add(
                        ned=np.array(root_pos, dtype=float).reshape(-1, 1),
                        airspeed=float(root_airspeed),
                        course=float(root_course),
                        cost=float(0),
                        parent=int(-1),
                        connect_to_goal=0,
                    )

                    for diff_pos in itertools.product(np.arange(-8, 19.1, 5), repeat=3):
                        waypoints.add(
                            ned=waypoints.ned[:, -1].reshape(-1, 1)
                            + np.array(diff_pos, dtype=float).reshape(-1, 1),
                            airspeed=float(
                                root_airspeed
                                + (np.array(diff_pos, dtype=float).sum() % 5)
                            ),
                            course=float(
                                root_course
                                + (np.array(diff_pos, dtype=float).sum() % 2)
                            ),
                            cost=float(waypoints.cost[-1] + 1.1),
                            parent=int(waypoints.num_waypoints - 1),
                            connect_to_goal=0,
                        )
                    waypoints.connect_to_goal[-1] = 1
                    ptr = WaypointIndices()
                    lines = []
                    for _ in range(waypoints.num_waypoints - 1):
                        lines.append(
                            construct_fillet_circle(
                                waypoints=waypoints, ptr=ptr, radius=radius
                            )
                        )
                        ptr.increment_pointers(num_waypoints=waypoints.num_waypoints)
                output.append(((waypoints, radius), lines))
    return output


def construct_fillet_circle_test(test_case: Tuple[Tuple[Any, Any], List[Any]]) -> int:
    """Tests the construct_fillet_circle function."""
    ptr = WaypointIndices()
    for ptr_it in range(test_case[0][0].num_waypoints - 1):
        calculated_output = construct_fillet_circle(
            waypoints=test_case[0][0], ptr=ptr, radius=test_case[0][1]
        )
        # pylint: disable=too-many-boolean-expressions
        if (
            (calculated_output[0].type != test_case[1][ptr_it][0].type)
            or (
                np.abs(calculated_output[0].airspeed - test_case[1][ptr_it][0].airspeed)
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[0].line_origin)
                    - np.array(test_case[1][ptr_it][0].line_origin)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].line_direction)
                    - np.array(test_case[1][ptr_it][0].line_direction)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    np.array(calculated_output[0].orbit_center)
                    - np.array(test_case[1][ptr_it][0].orbit_center)
                )
                > PRECISION
            ).any()
            or (
                np.abs(
                    calculated_output[0].orbit_radius
                    - test_case[1][ptr_it][0].orbit_radius
                )
                > PRECISION
            )
            or (
                calculated_output[0].orbit_direction
                != test_case[1][ptr_it][0].orbit_direction
            )
            or (
                np.abs(
                    (
                        np.array(calculated_output[1].normal).reshape(1, -1)
                        @ np.array(test_case[1][ptr_it][1].normal).reshape(-1, 1)
                    )
                    - 1
                )
                > PRECISION
            )
            or (
                np.abs(
                    np.array(calculated_output[1].point)
                    - np.array(test_case[1][ptr_it][1].point)
                )
                > PRECISION
            ).any()
        ):
            print("Failed test!")
            print("Calculated output:")
            calculated_output[0].print()
            calculated_output[1].print()
            print("Expected output:")
            test_case[1][ptr_it][0].print()
            test_case[1][ptr_it][1].print()
            return ptr_it
        ptr.increment_pointers(num_waypoints=test_case[0][0].num_waypoints)
    return -1


def gen_test_archive() -> None:
    """Makes the input/output archive of answers."""
    archive: Dict[str, List[Tuple[Any, Any]]] = {}
    # Gather test cases
    print("Making inHalfSpace archive")
    archive["inHalfSpace"] = gen_inHalfSpace_vars()
    print("Making construct_line archive")
    archive["construct_line"] = gen_construct_line_vars()
    print("Making construct_fillet_line archive")
    archive["construct_fillet_line"] = gen_construct_fillet_line_vars()
    print("Making construct_fillet_circle archive")
    archive["construct_fillet_circle"] = gen_construct_fillet_circle_vars()
    # Save to archive
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch11_test_archive.pkl"
        ),
        "wb",
    ) as file:
        pickle.dump(archive, file)


def run_all_tests() -> None:
    """Run all tests."""
    gc.disable()
    # Open archive
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch11_test_archive.pkl"
        ),
        "rb",
    ) as file:
        tests_archive = pickle.load(file)
    gc.enable()
    # Run tests
    print("Starting inHalfSpace test")
    for test_count in range(len(tests_archive["inHalfSpace"])):
        if not inHalfSpace_test(tests_archive["inHalfSpace"][test_count]):
            print("Failed on test id: " + str(test_count))
            break
    print("End of test\n")
    print("Starting construct_line test")
    for test_count in range(len(tests_archive["construct_line"])):
        fail_case = construct_line_test(tests_archive["construct_line"][test_count])
        if -1 != fail_case:
            print(
                "Failed on test id: "
                + str(test_count)
                + " list index: "
                + str(fail_case)
            )
            break
    print("End of test\n")
    print("Starting construct_fillet_line test")
    for test_count in range(len(tests_archive["construct_fillet_line"])):
        fail_case = construct_fillet_line_test(
            tests_archive["construct_fillet_line"][test_count]
        )
        if -1 != fail_case:
            print(
                "Failed on test id: "
                + str(test_count)
                + " list index: "
                + str(fail_case)
            )
            break
    print("End of test\n")
    print("Starting construct_fillet_circle test")
    for test_count in range(len(tests_archive["construct_fillet_circle"])):
        fail_case = construct_fillet_circle_test(
            tests_archive["construct_fillet_circle"][test_count]
        )
        if -1 != fail_case:
            print(
                "Failed on test id: "
                + str(test_count)
                + " list index: "
                + str(fail_case)
            )
            break
    print("End of test\n")


if __name__ == "__main__":
    gen_test_archive()
    run_all_tests()

    # gc.disable()
    # # Open archive
    # with open(
    #     os.path.join(
    #         os.path.dirname(os.path.realpath(__file__)), "ch11_test_archive.pkl"
    #     ),
    #     "rb",
    # ) as file:
    #     tests_archive = pickle.load(file)
    # gc.enable()

    # construct_line_test(tests_archive["construct_line"][2])
