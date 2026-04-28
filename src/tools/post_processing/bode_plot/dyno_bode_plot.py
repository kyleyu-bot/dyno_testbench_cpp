#!/usr/bin/env python3
"""Create Bode plots from dyno_testbench_cpp test_data_log CSV runs.

Examples:
  python3 src/tools/post_processing/bode_plot/dyno_bode_plot.py --latest --preset main_torque_ch1
  python3 src/tools/post_processing/bode_plot/dyno_bode_plot.py test_data_log/2026-04-24/173202 --preset main_current
  python3 src/tools/post_processing/bode_plot/dyno_bode_plot.py --latest --ref main_rx_target_velocity --resp main_tx_motor_velocity --f-max 30
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import sys

os.environ.setdefault("MPLCONFIGDIR", "/tmp/dyno_matplotlib")

if os.environ.get("PYTHONNOUSERSITE") != "1" and os.environ.get("DYNO_BODE_ALLOW_USER_SITE") != "1":
    env = os.environ.copy()
    env["PYTHONNOUSERSITE"] = "1"
    os.execvpe(sys.executable, [sys.executable, *sys.argv], env)

from dyno_bode import PRESETS, compute_bode, latest_log, make_bode_figure, resolve_csv_path


def default_output_path(result, preset_name: str | None) -> Path:
    filename = f"bode_{preset_name or result.ref_name + '_to_' + result.resp_name}.png"
    log_dir_path = result.csv_path.with_name(filename)
    if os.access(result.csv_path.parent, os.W_OK):
        return log_dir_path

    out_dir = Path("bode_plots")
    out_dir.mkdir(exist_ok=True)
    run_name = f"{result.csv_path.parent.parent.name}_{result.csv_path.parent.name}"
    return out_dir / f"{run_name}_{filename}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", default="test_data_log",
                        help="dyno_pdo.csv, a run directory, or a log root. Defaults to test_data_log.")
    parser.add_argument("--latest", action="store_true",
                        help="Use the newest dyno_pdo.csv below PATH.")
    parser.add_argument("--preset", choices=sorted(PRESETS),
                        help="Named dyno signal pair. Use --list-presets to print them.")
    parser.add_argument("--list-presets", action="store_true",
                        help="Print available presets and exit.")
    parser.add_argument("--ref", help="Reference/input CSV column.")
    parser.add_argument("--resp", help="Response/output CSV column.")
    parser.add_argument("--title", help="Plot title override.")
    parser.add_argument("--units", help="Time-series axis units override.")
    parser.add_argument("--out", help="Output PNG path. Defaults next to dyno_pdo.csv if writable, otherwise ./bode_plots/.")
    parser.add_argument("--data-out", help="Optional CSV export of frequency, magnitude, phase, and in_band.")
    parser.add_argument("--no-plot", action="store_true",
                        help="Compute/export data without importing matplotlib or rendering a PNG.")
    parser.add_argument("--f-min", type=float, help="Minimum plotted/analysed frequency in Hz.")
    parser.add_argument("--f-max", type=float, help="Maximum plotted/analysed frequency in Hz.")
    parser.add_argument("--chirp-start-hz", type=float, default=0.1,
                        help="Start frequency for generated chirp overlay/range. Default: 0.1.")
    parser.add_argument("--chirp-end-hz", type=float, default=10.0,
                        help="End frequency for generated chirp overlay/range. Default: 10.")
    parser.add_argument("--chirp-duration-s", type=float, default=10.0,
                        help="Chirp duration for generated overlay/range. Default: 10.")
    parser.add_argument("--chirp-kind", choices=("linear", "exponential"), default="linear",
                        help="Generated chirp profile used for overlay/range. Default: linear.")
    parser.add_argument("--no-chirp-overlay", action="store_true",
                        help="Do not generate a chirp frequency overlay; use --f-min/--f-max instead.")
    parser.add_argument("--trim-start-s", type=float, help="Discard samples before this relative time.")
    parser.add_argument("--trim-end-s", type=float, help="Discard samples after this relative time.")
    parser.add_argument("--no-detrend", action="store_true",
                        help="Do not mean-subtract reference and response before spectral estimation.")
    parser.add_argument("--invert-response", action="store_true",
                        help="Negate the response column before analysis.")
    parser.add_argument("--lowpass-hz", type=float,
                        help="Zero-phase Butterworth low-pass cutoff for response before Bode calculation.")
    parser.add_argument("--filter-order", type=int, default=2,
                        help="Butterworth low-pass order. Default: 2.")
    parser.add_argument("--freq-resolution-hz", type=float, default=0.02,
                        help="Target Welch frequency resolution. Default: 0.02 Hz.")
    parser.add_argument("--show-raw", action="store_true",
                        help="Overlay raw response when --lowpass-hz is used.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_presets:
        for name, preset in sorted(PRESETS.items()):
            print(f"{name:20s} ref={preset.reference:28s} resp={preset.response:28s} {preset.description}")
        return

    if args.latest:
        csv_path = latest_log(args.path)
    else:
        csv_path = resolve_csv_path(args.path)

    chirp_start = None if args.no_chirp_overlay else args.chirp_start_hz
    chirp_end = None if args.no_chirp_overlay else args.chirp_end_hz
    chirp_duration = None if args.no_chirp_overlay else args.chirp_duration_s

    result = compute_bode(
        csv_path,
        preset_name=args.preset,
        reference=args.ref,
        response=args.resp,
        title=args.title,
        units=args.units,
        f_min=args.f_min,
        f_max=args.f_max,
        chirp_start_hz=chirp_start,
        chirp_end_hz=chirp_end,
        chirp_duration_s=chirp_duration,
        chirp_kind=args.chirp_kind,
        trim_start_s=args.trim_start_s,
        trim_end_s=args.trim_end_s,
        detrend=not args.no_detrend,
        invert_response=args.invert_response,
        lowpass_hz=args.lowpass_hz,
        filter_order=args.filter_order,
        freq_resolution_hz=args.freq_resolution_hz,
    )

    if args.data_out:
        data_out = Path(args.data_out).expanduser()
        with data_out.open("w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["frequency_hz", "magnitude_db", "phase_deg", "in_band"])
            for freq, mag, phase, in_band in zip(result.frequency, result.magnitude_db, result.phase_deg, result.mask):
                writer.writerow([freq, mag, phase, int(bool(in_band))])
        print(f"Exported Bode data: {data_out}")

    if args.no_plot:
        print(f"CSV: {result.csv_path}")
        print(f"Reference: {result.ref_name}")
        print(f"Response: {result.resp_name}")
        print(f"Sample rate: {result.fs:.2f} Hz")
        print(f"Welch nperseg: {result.nperseg}")
        if result.f_3db is not None:
            print(f"-3 dB frequency: {result.f_3db:.3f} Hz")
        if result.f_90 is not None:
            print(f"-90 deg frequency: {result.f_90:.3f} Hz")
        return

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(
            "Bode data computed, but matplotlib could not be imported to render the PNG.\n"
            "This usually means the Python environment has an incompatible NumPy/matplotlib pair.\n"
            f"Original import error: {exc}"
        ) from exc

    fig = make_bode_figure(result, show_raw=args.show_raw)
    out_path = Path(args.out).expanduser() if args.out else default_output_path(result, args.preset)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)

    print(f"CSV: {result.csv_path}")
    print(f"Reference: {result.ref_name}")
    print(f"Response: {result.resp_name}")
    print(f"Sample rate: {result.fs:.2f} Hz")
    print(f"Welch nperseg: {result.nperseg}")
    if result.f_3db is not None:
        print(f"-3 dB frequency: {result.f_3db:.3f} Hz")
    if result.f_90 is not None:
        print(f"-90 deg frequency: {result.f_90:.3f} Hz")
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
