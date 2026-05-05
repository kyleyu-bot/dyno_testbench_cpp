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

        try:
            iq, torque, gear_ratio = _detect_ramp_segment(
                data, header, drive, sensor_col)
        except Exception as exc:
            messagebox.showerror("Segment detection failed", str(exc))
            return

        if self.invert_var.get():
            torque = -torque

        kt_out, ic_out, r2_out = _linear_fit(iq, torque)
        kt_mot, ic_mot, r2_mot = _linear_fit(iq, torque / gear_ratio)

        self._csv_path = csv_path
        self._draw(iq, torque, gear_ratio,
                   kt_out, ic_out, r2_out,
                   kt_mot, ic_mot, r2_mot,
                   sensor_col)
        self.status_var.set(
            f"Kt_output = {kt_out:.4f} Nm/A  |  "
            f"Kt_motor = {kt_mot:.4f} Nm/A  (gear ratio: {gear_ratio:.4f})"
        )

    def _draw(self, iq, torque, gear_ratio,
              kt_out, ic_out, r2_out,
              kt_mot, ic_mot, r2_mot,
              sensor_col):
        if self._canvas is not None:
            self._canvas.get_tk_widget().destroy()
        if self._toolbar is not None:
            self._toolbar.destroy()

        fig = Figure(figsize=(12, 5), tight_layout=True)
        iq_fit = np.linspace(iq.min(), iq.max(), 200)

        ax1 = fig.add_subplot(1, 2, 1)
        ax1.scatter(iq, torque, s=4, alpha=0.5, label="data")
        ax1.plot(iq_fit, kt_out * iq_fit + ic_out, "r-",
                 label=f"fit: Kt = {kt_out:.4f} Nm/A\nR² = {r2_out:.4f}")
        ax1.set_xlabel("Iq actual (A)")
        ax1.set_ylabel(f"{sensor_col} (Nm)")
        ax1.set_title("Output shaft Kt")
        ax1.legend(fontsize=9)
        ax1.grid(True, alpha=0.3)

        ax2 = fig.add_subplot(1, 2, 2)
        ax2.scatter(iq, torque / gear_ratio, s=4, alpha=0.5, label="data")
        ax2.plot(iq_fit, kt_mot * iq_fit + ic_mot, "r-",
                 label=f"fit: Kt = {kt_mot:.4f} Nm/A\nR² = {r2_mot:.4f}")
        ax2.set_xlabel("Iq actual (A)")
        ax2.set_ylabel(f"{sensor_col} / gear_ratio (Nm)")
        ax2.set_title(f"Motor shaft Kt  (gear ratio: {gear_ratio:.4f})")
        ax2.legend(fontsize=9)
        ax2.grid(True, alpha=0.3)

        self._fig = fig
        self._canvas = FigureCanvasTkAgg(fig, master=self._plot_frame)
        self._canvas.draw()
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._toolbar = NavigationToolbar2Tk(self._canvas, self._plot_frame)
        self._toolbar.update()
        self._save_btn.configure(state="normal")

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
