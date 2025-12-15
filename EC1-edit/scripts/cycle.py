#!/usr/bin/env python

"""Run a dosing cycle on the EcoCharger using its serial interface."""

import time
import math
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


def generate_dispensing_steps(
    num_steps: int, duration: float, min_sleep_time: float = 5.0
):
    """Generate given total stepper motor steps over given duration."""
    max_num_sleeps = math.ceil(duration / min_sleep_time)
    num_steps_per_sleep = math.ceil(num_steps / max_num_sleeps)
    sleep_time = duration / max_num_sleeps

    while num_steps > num_steps_per_sleep:
        yield num_steps_per_sleep
        num_steps -= num_steps_per_sleep
        time.sleep(sleep_time)

    yield num_steps
    time.sleep(sleep_time)


def main():  # pylint: disable=too-many-statements, too-many-locals
    """Run a dosing cycle on the EcoCharger using its serial interface."""
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port to use to communicate with EcoCharger controller.",
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
    parser.add_argument(
        "--mixing_steps",
        type=int,
        default=1000,
        help="Mixing volume in units of stepper motor steps.",
    )
    parser.add_argument(
        "--target_temperature",
        type=int,
        default=32,
        help="Target mixture temperature in degrees C.",
    )
    parser.add_argument(
        "--activation_time",
        type=int,
        default=30,
        help="Time in seconds to allow mixture to activate before dispensing.",
    )
    parser.add_argument(
        "--dispensing_time",
        type=int,
        default=(5 * 60 * 60),  # Five hours in seconds.
        help="Time in seconds over which to dispense the mixture.",
    )
    parser.add_argument(
        "--rinsing_steps",
        type=int,
        default=15000,
        help="Rising water volume in units of stepper motor steps.",
    )
    parser.add_argument(
        "--rinsing_cycles",
        type=int,
        default=3,
        help="Number of rinsing cycles to run after dispensing.",
    )
    args = parser.parse_args()

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

        def read_tempsensor():
            write_bytes(b"read_tempsensor")

            reply = sio.readline().strip()
            heater, liquid = reply.split(b",")

            _, heater = heater.split(b"=")
            heater = heater.strip()
            heater = int(heater)

            _, liquid = liquid.split(b"=")
            liquid = liquid.strip()
            liquid = int(liquid)

            return heater, liquid

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
        # Run the cycle script. #
        #########################

        with task_message("Rehoming valve"):
            send_command("valve_rehome")

        with task_message("Rehoming syringe"):
            send_command("valve_move_to_water_out")
            send_command("syringe_rehome")
            send_command("valve_move_to_home")

        half_water_steps_round_down: int = args.water_steps // 2
        with task_message("Pulling half water"):
            send_command("valve_move_to_water_in")
            send_command(f"syringe_pull {half_water_steps_round_down}")

        with task_message("Pulling spores"):
            send_command("valve_move_to_spores")
            send_command(f"syringe_pull {args.spore_steps}")

        with task_message("Pulling supercharger"):
            send_command("valve_move_to_supercharger")
            send_command(f"syringe_pull {args.supercharger_steps}")

        try:
            with task_message("Heating"):
                send_command("valve_move_to_water_out")
                send_command("heater_turn_on")

                _, liquid = read_tempsensor()
                output = f"{liquid:3d}C"
                print(output, end="", flush=True)
                time.sleep(1.0)

                while liquid < args.target_temperature:
                    _, liquid = read_tempsensor()

                    erase = len(output) * "\b"
                    output = f"{liquid:3d}C"
                    print(erase, output, end="", sep="", flush=True)

                    send_command(f"syringe_push {args.mixing_steps}")
                    time.sleep(500e-3)

                    send_command(f"syringe_pull {args.mixing_steps}")
                    time.sleep(500e-3)

                erase = len(output) * "\b"
                print(erase, end="", flush=True)

        finally:
            send_command("heater_turn_off")

        with task_message("Waiting activation period"):
            time.sleep(args.activation_period)

        half_water_steps_round_up: int = (args.water_steps + 1) // 2
        with task_message("Pulling half water for cooling"):
            send_command("valve_move_to_water_in")
            send_command(f"syringe_pull {half_water_steps_round_up}")

        num_steps = args.water_steps + args.spore_steps + args.supercharger_steps
        with task_message("Dispensing"):
            send_command("valve_move_to_water_out")

            for steps in generate_dispensing_steps(num_steps, args.dispensing_time):
                send_command(f"syringe_push {steps}")

            # Rehome just in case we calculated dispensing steps incorrectly.
            send_command("syringe_rehome")

        for step in range(1, args.rinsing_cycles + 1):
            with task_message(f"Rinsing step {step}"):
                send_command("valve_move_to_water_in")
                send_command(f"syringe_pull {args.rinsing_steps}")

            with task_message("Dispensing rinsing water"):
                send_command("valve_move_to_water_out")
                send_command("syringe_rehome")


if __name__ == "__main__":
    main()
