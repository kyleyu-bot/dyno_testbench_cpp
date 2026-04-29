"""
Backdriving torque test.

Sets the main drive to CSV (speed) mode and the DUT to CST (torque) mode.
The DUT applies a constant torque against the main drive's speed-controlled
shaft to characterise backdriving behaviour through the mechanism.

Parameters
----------
main_speed_rad_s : float
    Main drive speed setpoint (rad/s, input side). Default 1.0.
dut_torque_nm : float
    DUT torque command (Nm). May be negative for reverse torque. Default 1.0.
run_time_s : float
    Total test duration (s). Default 60.

Framework contract (pre/post):
    Before run() — framework enables both drives and applies GUI modes.
    After  run() — framework zeros setpoints, disables drives, sets mode 0.
    This script overrides both explicitly for clarity and safety.
"""

import time

PARAMS = {
    "main_speed_rad_s": 1.0,
    "dut_torque_nm":    1.0,
    "run_time_s":       60.0,
}

CSV = 9    # DS402 Cyclic Synchronous Velocity
CST = 10   # DS402 Cyclic Synchronous Torque


def run(params: dict, commander, stop_event):
    main_speed  = float(params["main_speed_rad_s"])
    dut_torque  = float(params["dut_torque_nm"])
    run_time    = float(params["run_time_s"])

    commander.set_command(
        numeric     = {
            "main_velocity": main_speed,
            "dut_torque":    dut_torque,
        },
        main_enable = True,
        dut_enable  = True,
        main_mode   = CSV,
        dut_mode    = CST,
    )

    t0 = time.monotonic()
    while not stop_event.is_set() and time.monotonic() - t0 < run_time:
        time.sleep(0.05)

    # Zero and disable — framework epilogue also does this.
    commander.set_command(
        numeric     = {
            "main_velocity": 0.0,
            "dut_torque":    0.0,
        },
        main_enable = False,
        dut_enable  = False,
        main_mode   = 0,
        dut_mode    = 0,
    )
