"""
Cogging torque compensation test.

Runs the selected drive at a constant input-side speed in the positive
direction and monitors the input-side encoder (raw counts, 0x204A) to stop
once the shaft has completed the target number of input-side revolutions.

Speed conversion:
    velocity_rad_s = input_speed_rev_s × 2π

Completion condition:
    abs(current_input_enc - start_input_enc) >= target_input_revs × input_enc_cpr

Parameters
----------
drive : str
    Which drive to command: "main" or "dut".
input_speed_rev_s : float
    Constant input-side speed (rev/s). Default 0.5.
target_input_revs : float
    Number of input-side revolutions before stopping. Default 4.0.
input_enc_cpr : int
    Input encoder counts per revolution (check drive datasheet). Default 65536.
timeout_s : float
    Safety timeout (s) — exits early if target is not reached. Default 120.

Framework contract (pre/post):
    Before run() — framework enables both drives and applies GUI modes.
    After  run() — framework zeros setpoints, disables drives, sets mode 0.
    This script overrides both explicitly for clarity and safety.
"""

import math
import time

PARAMS = {
    "drive":              ["main", "dut"],
    "input_speed_rev_s":  0.5,
    "target_input_revs":  4.0,
    "input_enc_cpr":      65536,
    "timeout_s":          120.0,
}

CSV = 9   # DS402 Cyclic Synchronous Velocity


def run(params: dict, commander, stop_event):
    drive       = params["drive"]
    is_main     = (drive == "main")
    speed_rev_s = float(params["input_speed_rev_s"])
    target_revs = float(params["target_input_revs"])
    enc_cpr     = int(params["input_enc_cpr"])
    timeout_s   = float(params["timeout_s"])
    vel_key     = "main_velocity" if is_main else "dut_velocity"

    vel_rad_s      = speed_rev_s * 2.0 * math.pi
    target_counts  = target_revs * enc_cpr

    def _send(vel: float):
        commander.set_command(
            numeric     = {vel_key: vel},
            main_enable = is_main,
            dut_enable  = not is_main,
            main_mode   = CSV if is_main     else 0,
            dut_mode    = CSV if not is_main else 0,
        )

    # Capture starting input encoder position.
    start_enc = commander.get_input_enc_pos_raw(drive)

    _send(vel_rad_s)

    t0 = time.monotonic()
    while not stop_event.is_set():
        if time.monotonic() - t0 >= timeout_s:
            break
        delta = abs(commander.get_input_enc_pos_raw(drive) - start_enc)
        if delta >= target_counts:
            break
        time.sleep(0.02)

    # Zero and disable — framework epilogue also does this.
    commander.set_command(
        numeric     = {vel_key: 0.0},
        main_enable = False,
        dut_enable  = False,
        main_mode   = 0,
        dut_mode    = 0,
    )
