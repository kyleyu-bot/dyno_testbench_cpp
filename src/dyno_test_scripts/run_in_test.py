"""
10-minute run-in test.

Puts the selected drive into CSV (Cyclic Synchronous Velocity) mode and steps
through positive speeds [step, 2*step, ..., num_steps*step] rad/s (input side),
holding each for hold_time_s.  After the positive run the drive holds at 0 for
zero_hold_s, then repeats the same speed sequence in the negative direction.

Parameters
----------
drive : str
    Which drive to command: "main" or "dut".
speed_step_rad_s : float
    Velocity increment between steps (rad/s, input side). Default 2.0.
num_steps : int
    Number of speed steps. Default 3  (gives 2, 4, 6 rad/s with step=2).
hold_time_s : float
    Duration to hold at each speed level (s). Default 100.
zero_hold_s : float
    Duration to hold at 0 between positive and negative runs (s). Default 5.

Framework contract (pre/post):
    Before run() — framework enables both drives and applies GUI modes.
    After  run() — framework zeros setpoints, disables drives, sets mode 0.
    This script overrides both explicitly for clarity and safety.
"""

import time

PARAMS = {
    "drive":            ["main", "dut"],
    "speed_step_rad_s": 2.0,
    "num_steps":        3,
    "hold_time_s":      100.0,
    "zero_hold_s":      5.0,
}

CSV = 9   # DS402 Cyclic Synchronous Velocity


def run(params: dict, commander, stop_event):
    is_main   = (params["drive"] == "main")
    step_vel  = float(params["speed_step_rad_s"])
    num_steps = max(1, int(params["num_steps"]))
    hold_t    = float(params["hold_time_s"])
    zero_hold = float(params["zero_hold_s"])
    vel_key   = "main_velocity" if is_main else "dut_velocity"

    def _send(vel: float):
        commander.set_command(
            numeric     = {vel_key: vel},
            main_enable = is_main,
            dut_enable  = not is_main,
            main_mode   = CSV if is_main     else 0,
            dut_mode    = CSV if not is_main else 0,
        )

    def _hold(duration_s: float, vel: float):
        _send(vel)
        t0 = time.monotonic()
        while not stop_event.is_set() and time.monotonic() - t0 < duration_s:
            time.sleep(0.05)

    speeds = [step_vel * (i + 1) for i in range(num_steps)]

    # Positive direction
    for v in speeds:
        if stop_event.is_set():
            break
        _hold(hold_t, v)

    # Zero hold between directions
    if not stop_event.is_set():
        _hold(zero_hold, 0.0)

    # Negative direction
    for v in speeds:
        if stop_event.is_set():
            break
        _hold(hold_t, -v)

    # Zero and disable — framework epilogue also does this.
    commander.set_command(
        numeric     = {vel_key: 0.0},
        main_enable = False,
        dut_enable  = False,
        main_mode   = 0,
        dut_mode    = 0,
    )
