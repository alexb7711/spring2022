"""ch6_feedback_control_tests.py: Implements some basic tests for the chapter 6 martial."""


from typing import Any, Dict, List

import mav_sim.parameters.simulation_parameters as SIM
import numpy as np
from mav_sim.chap6.autopilot import Autopilot
from mav_sim.chap6.pd_control_with_rate import PDControlWithRate
from mav_sim.chap6.pi_control import PIControl
from mav_sim.chap6.tf_control import TFControl
from mav_sim.message_types.msg_autopilot import MsgAutopilot
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.message_types.msg_state import MsgState
from mav_sim.parameters import control_parameters as CP


def pd_control_with_rate_test() -> None:
    """Tests the PDControlWithRate class."""
    print("Starting pd_control_with_rate test\n")
    # Inputs
    inputs: List[Dict[str, Any]] = [
        {
            "y_ref": float(0),
            "y": float(0),
            "ydot": float(0),
        },
        {
            "y_ref": float(3),
            "y": float(4),
            "ydot": float(5),
        },
    ]
    # Expected outputs
    outputs = [
        float(0),
        float(-0.43974730752905633),
    ]

    test_class = PDControlWithRate(kp=CP.roll_kp, kd=CP.roll_kd, limit=np.radians(45))
    for input_it, output_it in zip(inputs, outputs):
        calculated_output = test_class.update(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-6 < np.abs(np.array(calculated_output) - np.array(output_it))).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def pi_control_test() -> None:
    """Tests the PIControl class."""
    print("Starting pi_control test\n")
    # Inputs
    inputs: List[Dict[str, Any]] = [
        {
            "y_ref": float(0),
            "y": float(0),
        },
        {
            "y_ref": float(3),
            "y": float(4),
        },
    ]
    # Expected outputs
    outputs = [
        float(0),
        float(-0.5235987755982988),
    ]

    test_class = PIControl(
        kp=CP.course_kp, ki=CP.course_ki, Ts=SIM.ts_simulation, limit=np.radians(30)
    )

    for input_it, output_it in zip(inputs, outputs):
        calculated_output = test_class.update(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-6 < np.abs(np.array(calculated_output) - np.array(output_it))).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def tf_control_test() -> None:
    """Tests the TFControl class."""
    print("Starting tf_control test\n")
    # Inputs
    inputs: List[Dict[str, Any]] = [
        {
            "y": float(0),
        },
        {
            "y": float(4),
        },
    ]
    # Expected outputs
    outputs = [
        float(0),
        float(0.15892403727930438),
    ]

    for input_it, output_it in zip(inputs, outputs):
        test_class = TFControl(
            k=CP.yaw_damper_kr,
            n0=0.0,
            n1=1.0,
            d0=CP.yaw_damper_p_wo,
            d1=1,
            Ts=SIM.ts_simulation,
        )
        _ = test_class.update(**input_it)
        calculated_output = test_class.update(**input_it)

        print("Calculated output:")
        print(calculated_output)
        print("Expected output:")
        print(output_it)
        if (1e-6 < np.abs(np.array(calculated_output) - np.array(output_it))).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def autopilot_test() -> None:
    """Tests the AutoPilot class."""
    print("Starting autopilot test\n")
    # Inputs
    inputs: List[Dict[str, Any]] = [
        {
            "cmd": MsgAutopilot(),
            "state": MsgState(),
        },
        {
            "cmd": MsgAutopilot(),
            "state": MsgState(
                north=111,
                east=222,
                altitude=555,
                phi=0.4,
                theta=0.26,
                psi=1.7,
                Va=16,
                alpha=0.7,
                beta=0.32,
                p=0.3,
                q=0.2,
                r=0.1,
                Vg=102,
                gamma=0.123,
                chi=0.234,
                wn=1.1,
                we=1.23,
                bx=0.45,
                by=0.1765,
                bz=0.3465,
            ),
        },
    ]
    inputs[1]["cmd"].airspeed_command = 4
    inputs[1]["cmd"].course_command = 2
    inputs[1]["cmd"].altitude_command = 12
    inputs[1]["cmd"].phi_feedforward = 0.35
    # Expected outputs
    outputs = [
        (
            MsgDelta(elevator=0, aileron=0, rudder=0, throttle=0),
            MsgState(),
        ),
        (
            MsgDelta(
                elevator=0.7853981633974483,
                aileron=0.11389145757219538,
                rudder=0.01995510102269893,
                throttle=0,
            ),
            MsgState(
                north=0,
                east=0,
                altitude=12,
                phi=0.5235987755982988,
                theta=-0.5235987755982988,
                psi=0,
                Va=4,
                alpha=0,
                beta=0,
                p=0,
                q=0,
                r=0,
                Vg=0,
                gamma=0,
                chi=2,
                wn=0,
                we=0,
                bx=0,
                by=0,
                bz=0,
            ),
        ),
    ]

    test_class = Autopilot(ts_control=SIM.ts_simulation)
    for input_it, output_it in zip(inputs, outputs):
        calculated_output = test_class.update(**input_it)

        print("Calculated MsgDelta:")
        calculated_output[0].print()
        print("Expected MsgDelta:")
        output_it[0].print()
        if (
            1e-6 < np.abs(calculated_output[0].to_array() - output_it[0].to_array())
        ).any():
            print("Failed test!")
        else:
            print("Passed test")
        print("Calculated MsgState:")
        calculated_output[1].print()
        print("Expected MsgState:")
        output_it[1].print()
        if (
            1e-6 < np.abs(calculated_output[1].to_array() - output_it[1].to_array())
        ).any():
            print("Failed test!")
        else:
            print("Passed test")
    print("End of test\n")


def run_all_tests() -> None:
    """Run all tests."""
    pd_control_with_rate_test()
    pi_control_test()
    tf_control_test()
    autopilot_test()


if __name__ == "__main__":
    run_all_tests()
