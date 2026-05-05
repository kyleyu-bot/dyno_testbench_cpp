#!/usr/bin/env python3
"""Kt (torque constant) plot GUI — loads a dyno_pdo.csv from a torque-ramp test,
auto-detects the ramp segment, and fits Iq vs torque to extract Kt."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

os.environ.setdefault("MPLCONFIGDIR", "/tmp/dyno_matplotlib")

if os.environ.get("PYTHONNOUSERSITE") != "1" and os.environ.get("DYNO_KT_ALLOW_USER_SITE") != "1":
    env = os.environ.copy()
    env["PYTHONNOUSERSITE"] = "1"
    os.execvpe(sys.executable, [sys.executable, *sys.argv], env)

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

# Deviation threshold for ramp-start detection (Nm)
_DEVIATION_THRESHOLD = 0.01

_DRIVE_COLS = {
    "main": ("main_rx_torque_command", "main_tx_iq_actual", "main_gear_ratio"),
    "dut":  ("dut_rx_torque_command",  "dut_tx_iq_actual",  "dut_gear_ratio"),
}


def _find_csv(folder: str) -> Path:
    p = Path(folder)
    csv = p / "dyno_pdo.csv"
    if csv.exists():
        return csv
    # also accept if the user selects the CSV file directly
    if p.suffix == ".csv" and p.exists():
        return p
    raise FileNotFoundError(f"No dyno_pdo.csv found in {folder}")


def _latest_log(root: str = "test_data_log") -> Path:
    base = Path(root)
    csvs = sorted(base.glob("*/*/dyno_pdo.csv"))
    if not csvs:
        raise FileNotFoundError(f"No dyno_pdo.csv found under {root}")
    return csvs[-1].parent


def _load_csv(csv_path: Path) -> tuple[np.ndarray, list[str]]:
    with csv_path.open() as f:
        header = f.readline().strip().split(",")
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    return data, header


def _validate_kt_data(data: np.ndarray, header: list[str], drive: str) -> None:
    torque_cmd_col, _, _ = _DRIVE_COLS[drive]
    if torque_cmd_col not in header:
        raise ValueError(
            f"Column '{torque_cmd_col}' not found. Is this a torque-ramp log?"
        )
    idx = header.index(torque_cmd_col)
    torque_cmd = data[:, idx]
    if not np.any(np.abs(torque_cmd - torque_cmd[0]) > _DEVIATION_THRESHOLD):
        raise ValueError(
            "No torque command deviation detected in this log.\n"
            "Select a folder from a torque-ramp test."
        )


def _is_2way_test(csv_path: Path) -> bool:
    return "torque_ramp_2way" in csv_path.parent.name


def _detect_ramp_segment(
    data: np.ndarray, header: list[str], drive: str, torque_sensor_col: str
) -> tuple[np.ndarray, np.ndarray, float]:
    torque_cmd_col, iq_col, gear_col = _DRIVE_COLS[drive]

    torque_cmd = data[:, header.index(torque_cmd_col)]
    iq         = data[:, header.index(iq_col)]
    torque_sns = data[:, header.index(torque_sensor_col)]
    gear_ratio = data[0, header.index(gear_col)] if gear_col in header else 1.0

    initial = torque_cmd[0]
    dev_indices = np.where(np.abs(torque_cmd - initial) > _DEVIATION_THRESHOLD)[0]
    if len(dev_indices) == 0:
        raise ValueError("No torque deviation found.")
    t0_idx = max(0, dev_indices[0] - 1)
    t1_idx = int(np.argmax(np.abs(torque_cmd)))

    if t1_idx <= t0_idx:
        raise ValueError(
            f"Peak torque index ({t1_idx}) is not after ramp start ({t0_idx})."
        )

    seg_iq      = iq[t0_idx : t1_idx + 1]
    seg_torque  = torque_sns[t0_idx : t1_idx + 1]
    return seg_iq, seg_torque, float(gear_ratio)


def _detect_2way_segments(
    data: np.ndarray, header: list[str], drive: str, torque_sensor_col: str
) -> tuple[tuple, tuple, tuple, float]:
    """Return ((iq1,tor1), (iq2,tor2), (iq3,tor3), gear_ratio) for each ramp leg."""
    torque_cmd_col, iq_col, gear_col = _DRIVE_COLS[drive]

    torque_cmd = data[:, header.index(torque_cmd_col)]
    iq         = data[:, header.index(iq_col)]
    torque_sns = data[:, header.index(torque_sensor_col)]
    gear_ratio = data[0, header.index(gear_col)] if gear_col in header else 1.0

    initial = torque_cmd[0]

    # t0: one row before first deviation from initial
    dev = np.where(np.abs(torque_cmd - initial) > _DEVIATION_THRESHOLD)[0]
    if len(dev) == 0:
        raise ValueError("No torque deviation found.")
    t0_idx = max(0, dev[0] - 1)

    # t1: positive peak (end of leg 1 / start of leg 2)
    t1_idx = int(np.argmax(torque_cmd))

    # t2: negative peak after t1 (end of leg 2 / start of leg 3)
    t2_idx = int(np.argmin(torque_cmd[t1_idx:])) + t1_idx

    # t3: first index after t2 where torque_cmd is back within threshold of initial
    after_t2 = torque_cmd[t2_idx:]
    returned = np.where(np.abs(after_t2 - initial) < _DEVIATION_THRESHOLD)[0]
    t3_idx = int(returned[0]) + t2_idx if len(returned) > 0 else len(torque_cmd) - 1

    leg1 = (iq[t0_idx : t1_idx + 1], torque_sns[t0_idx : t1_idx + 1])
    leg2 = (iq[t1_idx : t2_idx + 1], torque_sns[t1_idx : t2_idx + 1])
    leg3 = (iq[t2_idx : t3_idx + 1], torque_sns[t2_idx : t3_idx + 1])
    return leg1, leg2, leg3, float(gear_ratio)


def _linear_fit(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    """Return (slope, intercept, R²)."""
    coeffs = np.polyfit(x, y, 1)
    slope, intercept = float(coeffs[0]), float(coeffs[1])
    y_fit = slope * x + intercept
    ss_res = np.sum((y - y_fit) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return slope, intercept, r2


class DynoKtApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dyno Kt Plot")
        self.geometry("1200x800")
        self._fig = None
        self._canvas = None
        self._toolbar = None
        self._csv_path: Path | None = None
        self._build_controls()
        self._plot_frame = ttk.LabelFrame(self, text="Preview", padding=4)
        self._plot_frame.pack(fill="both", expand=True, padx=10, pady=10)

    def _build_controls(self):
        ctrl = ttk.LabelFrame(self, text="Inputs", padding=10)
        ctrl.pack(fill="x", padx=10, pady=(10, 0))
        ctrl.columnconfigure(1, weight=1)

        self.path_var = tk.StringVar(value="test_data_log")
        ttk.Label(ctrl, text="Run folder:").grid(row=0, column=0, sticky="w")
        ttk.Entry(ctrl, textvariable=self.path_var, width=60).grid(
            row=0, column=1, sticky="ew", padx=6)
        ttk.Button(ctrl, text="Browse", command=self._browse).grid(row=0, column=2)
        ttk.Button(ctrl, text="Latest",  command=self._latest).grid(
            row=0, column=3, padx=(6, 0))

        self.drive_var   = tk.StringVar(value="main")
        self.sensor_var  = tk.StringVar(value="ch1")
        self.invert_var  = tk.BooleanVar(value=False)
        ttk.Label(ctrl, text="Drive:").grid(row=1, column=0, sticky="w", pady=6)
        ttk.Combobox(ctrl, textvariable=self.drive_var,
                     values=["main", "dut"], state="readonly", width=8).grid(
            row=1, column=1, sticky="w", padx=6)
        ttk.Label(ctrl, text="Torque sensor:").grid(row=1, column=2, sticky="w")
        ttk.Combobox(ctrl, textvariable=self.sensor_var,
                     values=["ch1", "ch2"], state="readonly", width=8).grid(
            row=1, column=3, sticky="w", padx=6)
        ttk.Checkbutton(ctrl, text="Invert torque (×−1)",
                        variable=self.invert_var).grid(
            row=1, column=4, sticky="w", padx=(12, 0))

        btn_row = ttk.Frame(ctrl)
        btn_row.grid(row=2, column=0, columnspan=4, sticky="e", pady=(4, 0))
        ttk.Button(btn_row, text="Generate", command=self._generate).pack(side="left")
        self._save_btn = ttk.Button(btn_row, text="Save", command=self._save,
                                    state="disabled")
        self._save_btn.pack(side="left", padx=(6, 0))

        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(ctrl, textvariable=self.status_var, foreground="grey").grid(
            row=3, column=0, columnspan=4, sticky="w")

    def _browse(self):
        path = filedialog.askdirectory(title="Select run folder (contains dyno_pdo.csv)")
        if path:
            self.path_var.set(path)

    def _latest(self):
        try:
            folder = _latest_log(self.path_var.get().strip() or "test_data_log")
            self.path_var.set(str(folder))
            self.status_var.set(f"Loaded: {folder}")
        except Exception as exc:
            messagebox.showerror("Latest log failed", str(exc))

    def _generate(self):
        folder = self.path_var.get().strip() or "test_data_log"
        drive  = self.drive_var.get()
        sensor_col = f"torque_{self.sensor_var.get()}_nm"

        try:
            csv_path = _find_csv(folder)
        except FileNotFoundError as exc:
            messagebox.showerror("Wrong folder", str(exc))
            return

        try:
            data, header = _load_csv(csv_path)
            _validate_kt_data(data, header, drive)
        except Exception as exc:
            messagebox.showerror("Invalid data", str(exc))
            return

        if sensor_col not in header:
            messagebox.showerror("Column missing", f"Column '{sensor_col}' not in CSV.")
            return

        two_way = _is_2way_test(csv_path)

        try:
            if two_way:
                leg1, leg2, leg3, gear_ratio = _detect_2way_segments(
                    data, header, drive, sensor_col)
            else:
                iq, torque, gear_ratio = _detect_ramp_segment(
                    data, header, drive, sensor_col)
                leg1 = (iq, torque)
        except Exception as exc:
            messagebox.showerror("Segment detection failed", str(exc))
            return

        invert = self.invert_var.get()

        def _fits(iq, torque):
            if invert:
                torque = -torque
            return torque, _linear_fit(iq, torque), _linear_fit(iq, torque / gear_ratio)

        tor1, fit1_out, fit1_mot = _fits(*leg1)
        extra_legs = None
        if two_way:
            tor2, fit2_out, fit2_mot = _fits(*leg2)
            tor3, fit3_out, fit3_mot = _fits(*leg3)
            extra_legs = (
                (leg2[0], tor2, fit2_out, fit2_mot),
                (leg3[0], tor3, fit3_out, fit3_mot),
            )

        # Full time-series for the raw data plots
        _, iq_col, _ = _DRIVE_COLS[drive]
        t_s = (data[:, header.index("stamp_ns")] - data[0, header.index("stamp_ns")]) * 1e-9
        full_iq     = data[:, header.index(iq_col)]
        full_torque = data[:, header.index(sensor_col)]
        if invert:
            full_torque = -full_torque

        self._csv_path = csv_path
        self._draw(leg1[0], tor1, gear_ratio,
                   fit1_out, fit1_mot, sensor_col, extra_legs,
                   t_s, full_iq, full_torque)
        self.status_var.set(
            f"Leg 1 — Kt_output = {fit1_out[0]:.4f} Nm/A  |  "
            f"Kt_motor = {fit1_mot[0]:.4f} Nm/A  (gear ratio: {gear_ratio:.4f})"
            + (f"  |  Leg 2 Kt = {fit2_out[0]:.4f}  |  Leg 3 Kt = {fit3_out[0]:.4f}"
               if two_way else "")
        )

    def _draw(self, iq1, tor1, gear_ratio,
              fit1_out, fit1_mot, sensor_col, extra_legs,
              t_s, full_iq, full_torque):
        if self._canvas is not None:
            self._canvas.get_tk_widget().destroy()
        if self._toolbar is not None:
            self._toolbar.destroy()

        n_fit_rows = 1 if extra_legs is None else 3
        n_rows = n_fit_rows + 1          # +1 for the time-series row at the top
        fig = Figure(figsize=(12, 5 * n_rows), tight_layout=True)

        # ── Row 0: raw time-series ────────────────────────────────────────────
        ax_iq = fig.add_subplot(n_rows, 2, 1)
        ax_iq.plot(t_s, full_iq, linewidth=0.8)
        ax_iq.set_xlabel("Time (s)")
        ax_iq.set_ylabel("Iq actual (A)")
        ax_iq.set_title("Iq actual — full run")
        ax_iq.grid(True, alpha=0.3)

        ax_tor = fig.add_subplot(n_rows, 2, 2)
        ax_tor.plot(t_s, full_torque, linewidth=0.8, color="tab:orange")
        ax_tor.set_xlabel("Time (s)")
        ax_tor.set_ylabel(f"{sensor_col} (Nm)")
        ax_tor.set_title(f"{sensor_col} — full run")
        ax_tor.grid(True, alpha=0.3)

        leg_labels = ["Leg 1 (+ ramp)", "Leg 2 (+ → −)", "Leg 3 (− return)"]
        legs = [(iq1, tor1, fit1_out, fit1_mot)]
        if extra_legs is not None:
            legs += list(extra_legs)

        for row, (iq, tor, fit_out, fit_mot) in enumerate(legs):
            kt_out, ic_out, r2_out = fit_out
            kt_mot, ic_mot, r2_mot = fit_mot
            iq_fit = np.linspace(iq.min(), iq.max(), 200)
            label  = leg_labels[row]
            # +1 row offset because row 0 is the time-series plots
            base = (row + 1) * 2 + 1

            ax_out = fig.add_subplot(n_rows, 2, base)
            ax_out.scatter(iq, tor, s=4, alpha=0.5, label="data")
            ax_out.plot(iq_fit, kt_out * iq_fit + ic_out, "r-",
                        label=f"fit: Kt = {kt_out:.4f} Nm/A\nR² = {r2_out:.4f}")
            ax_out.set_xlabel("Iq actual (A)")
            ax_out.set_ylabel(f"{sensor_col} (Nm)")
            ax_out.set_title(f"{label} — Output shaft Kt")
            ax_out.legend(fontsize=9)
            ax_out.grid(True, alpha=0.3)

            ax_mot = fig.add_subplot(n_rows, 2, base + 1)
            ax_mot.scatter(iq, tor / gear_ratio, s=4, alpha=0.5, label="data")
            ax_mot.plot(iq_fit, kt_mot * iq_fit + ic_mot, "r-",
                        label=f"fit: Kt = {kt_mot:.4f} Nm/A\nR² = {r2_mot:.4f}")
            ax_mot.set_xlabel("Iq actual (A)")
            ax_mot.set_ylabel(f"{sensor_col} / gear_ratio (Nm)")
            ax_mot.set_title(f"{label} — Motor shaft Kt  (GR: {gear_ratio:.4f})")
            ax_mot.legend(fontsize=9)
            ax_mot.grid(True, alpha=0.3)

        self._fig = fig
        self._canvas = FigureCanvasTkAgg(fig, master=self._plot_frame)
        self._canvas.draw()
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._toolbar = NavigationToolbar2Tk(self._canvas, self._plot_frame)
        self._toolbar.update()
        self._save_btn.configure(state="normal")

        def _on_scroll(event):
            if event.inaxes is None:
                return
            ax = event.inaxes
            factor = 1.15 if event.button == "down" else (1 / 1.15)
            xdata, ydata = event.xdata, event.ydata
            ax.set_xlim([xdata + (x - xdata) * factor for x in ax.get_xlim()])
            ax.set_ylim([ydata + (y - ydata) * factor for y in ax.get_ylim()])
            self._canvas.draw_idle()

        self._canvas.mpl_connect("scroll_event", _on_scroll)

    def _save(self):
        if self._fig is None:
            return
        default_dir = str(self._csv_path.parent) if self._csv_path else "."
        path = filedialog.asksaveasfilename(
            title="Save Kt Plot",
            initialdir=default_dir,
            initialfile="kt_plot.png",
            defaultextension=".png",
            filetypes=(("PNG", "*.png"), ("All files", "*")),
        )
        if not path:
            return
        try:
            self._fig.savefig(path, dpi=150, bbox_inches="tight")
            self.status_var.set(f"Saved {path}")
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))


if __name__ == "__main__":
    DynoKtApp().mainloop()
