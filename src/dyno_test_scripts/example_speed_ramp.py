"""
Example test script: ramp main drive velocity from 0 to target, hold, then ramp back.

Protocol:
    PARAMS  — dict of parameter names and their default values shown in the GUI.
    run()   — called in a background thread when "Run Test" is pressed.
              Check stop_event.is_set() regularly to support Abort.
              commander.set_command() sends to the drive.

Framework contract (as of preamble/epilogue feature):
    Before run() is called the framework has already:
      - zeroed all setpoint commands and disabled drives
      - enabled both drives (main_enable=True, dut_enable=True)
      - set the mode of operation to the value selected in the GUI mode combos

    After run() returns (whether completed or aborted) the framework will:
      - zero all setpoint commands (keep drives enabled briefly)
      - disable both drives and set mode=0 (No Mode)

    Scripts may still call set_command(enable=..., main_mode=...) explicitly —
    these are idempotent with the framework's preamble/epilogue state.
"""

import json
import time

PARAMS = {
    "target_vel_rad_s": 1.0,    # peak velocity (rad/s)
    "ramp_time_s":      2.0,    # time to ramp up / down
    "hold_time_s":      3.0,    # time to hold at peak
    "steps":            20,     # number of ramp steps
}


def run(params: dict, commander, stop_event):
    target = float(params["target_vel_rad_s"])
    ramp_t = float(params["ramp_time_s"])
    hold_t = float(params["hold_time_s"])
    steps  = max(1, int(params["steps"]))
    dt     = ramp_t / steps

    def _send(vel: float):
        commander.set_command(
            numeric     = {"main_velocity": vel},
            main_enable = True,
            dut_enable  = False,
            main_mode   = 9,   # Cyclic Sync Velocity
            dut_mode    = 9,
        )

    # Ramp up
    for i in range(steps + 1):
        if stop_event.is_set():
            break
        _send(target * i / steps)
        time.sleep(dt)

    # Hold
    t0 = time.monotonic()
    while not stop_event.is_set() and time.monotonic() - t0 < hold_t:
        time.sleep(0.05)

    # Ramp down
    for i in range(steps + 1):
        if stop_event.is_set():
            break
        _send(target * (steps - i) / steps)
        time.sleep(dt)

    # Zero and disable — kept for clarity; the framework epilogue does this anyway.
    commander.set_command(
        numeric     = {"main_velocity": 0},
        main_enable = False,
        dut_enable  = False,
        main_mode   = 0,
        dut_mode    = 0,
    )
