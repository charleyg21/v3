#!/usr/bin/env python

"""Prime the EcoCharger using its serial interface."""

import time
import argparse
import contextlib

import serial  # type: ignore


@contextlib.contextmanager
def task_message(msg, on_success=None, on_error=None):
    """Manage the messaging surrounding the execution of a task."""
    on_error = "Failed with error:" if on_error is None else str(on_error)
    on_success = "Success." if on_success is None else str(on_success)
    try:
        start = ">"
        print(start, msg, "...", "", end="", flush=True)
        yield
    except Exception:
        print(on_error, flush=True)
        raise
    else:
        print(on_success, flush=True)


def main():  # pylint: disable=too-many-statements, too-many-locals
    """Prime the EcoCharger using its serial interface."""
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port to use to communicate with EcoCharger controller.",
    )
    parser.add_argument(
        "--skip_rehome", type=bool, default=False, help="Skip the rehoming process."
    )
    parser.add_argument(
        "--water_steps",
        type=int,
        default=6000,
        help="Water water in units of stepper motor steps.",
    )
    parser.add_argument(
        "--spore_steps",
        type=int,
        default=5000,
        help="Spore volume in units of stepper motor steps.",
    )
    parser.add_argument(
        "--supercharger_steps",
        type=int,
        default=6000,
        help="Supercharger volume in units of stepper motor steps.",
    )
    args = parser.parse_args()

    with serial.Serial(port=args.port, timeout=1.0) as sio:
        for char in b"help\n":
            time.sleep(100e-3)
            sio.write(bytes([char]))

    with serial.Serial(port=args.port, timeout=30.0) as sio:

        ##################
        # Define helpers #
        ##################

        def write_bytes(bytes_):
            line = bytes_.strip() + b"\n"
            for char in line:
                sio.write(bytes([char]))
                time.sleep(50e-3)

        def send_command(command):
            command = bytes(command, "utf-8").strip()
            write_bytes(command)
            reply = sio.readline().strip()
            if reply != b"Success!":
                raise RuntimeError("System responded with an error.")

        ###################################
        # Read any residual serial bytes. #
        ###################################

        try:
            old_timeout = sio.timeout
            sio.timeout = 1.0
            sio.readall()
        except serial.SerialTimeoutException:
            pass
        finally:
            sio.timeout = old_timeout

        #########################
        # Run the prime script. #
        #########################

        with task_message("Rehoming valve"):
            send_command("valve_rehome")

        with task_message("Rehoming syringe"):
            send_command("valve_move_to_water_out")
            send_command("syringe_rehome")
            send_command("valve_move_to_home")

        with task_message("Pulling half water"):
            send_command("valve_move_to_water_in")
            send_command(f"syringe_pull {args.water_steps}")

        with task_message("Pulling spores"):
            send_command("valve_move_to_spores")
            send_command(f"syringe_pull {args.spore_steps}")

        with task_message("Pulling supercharger"):
            send_command("valve_move_to_supercharger")
            send_command(f"syringe_pull {args.supercharger_steps}")

        with task_message("Dispensing"):
            send_command("valve_move_to_water_out")
            send_command("syringe_rehome")


if __name__ == "__main__":
    main()
