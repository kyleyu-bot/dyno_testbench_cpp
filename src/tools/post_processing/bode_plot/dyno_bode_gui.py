#!/usr/bin/env python3
"""Small Tk GUI for dyno_pdo.csv Bode plots."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

os.environ.setdefault("MPLCONFIGDIR", "/tmp/dyno_matplotlib")

if os.environ.get("PYTHONNOUSERSITE") != "1" and os.environ.get("DYNO_BODE_ALLOW_USER_SITE") != "1":
    env = os.environ.copy()
    env["PYTHONNOUSERSITE"] = "1"
    os.execvpe(sys.executable, [sys.executable, *sys.argv], env)

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

from dyno_bode import PRESETS, compute_bode, latest_log, make_bode_figure, resolve_csv_path


class DynoBodeApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dyno Bode Plot")
        self.geometry("1200x900")
        self._fig = None
        self._canvas = None
        self._toolbar = None
        self._build_controls()
        self._plot_frame = ttk.LabelFrame(self, text="Preview", padding=4)
        self._plot_frame.pack(fill="both", expand=True, padx=10, pady=10)

    def _build_controls(self):
        ctrl = ttk.LabelFrame(self, text="Inputs", padding=10)
        ctrl.pack(fill="x", padx=10, pady=(10, 0))
        ctrl.columnconfigure(1, weight=1)

        self.path_var = tk.StringVar(value="test_data_log")
        ttk.Label(ctrl, text="CSV / Run / Root:").grid(row=0, column=0, sticky="w")
        ttk.Entry(ctrl, textvariable=self.path_var, width=60).grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(ctrl, text="Browse", command=self._browse).grid(row=0, column=2)
        ttk.Button(ctrl, text="Latest", command=self._latest).grid(row=0, column=3, padx=(6, 0))

        self.preset_var = tk.StringVar(value="main_torque_ch1")
        ttk.Label(ctrl, text="Preset:").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Combobox(ctrl, textvariable=self.preset_var, values=sorted(PRESETS), state="readonly", width=28).grid(
            row=1, column=1, sticky="w", padx=6, pady=4)

        self.ref_var = tk.StringVar()
        self.resp_var = tk.StringVar()
        ttk.Label(ctrl, text="Override ref:").grid(row=2, column=0, sticky="w", pady=4)
        ttk.Entry(ctrl, textvariable=self.ref_var, width=30).grid(row=2, column=1, sticky="w", padx=6)
        ttk.Label(ctrl, text="Override resp:").grid(row=2, column=2, sticky="w", pady=4)
        ttk.Entry(ctrl, textvariable=self.resp_var, width=30).grid(row=2, column=3, sticky="w", padx=6)

        self.f0_var = tk.StringVar(value="0.1")
        self.f1_var = tk.StringVar(value="10.0")
        self.dur_var = tk.StringVar(value="10.0")
        self.kind_var = tk.StringVar(value="linear")
        ttk.Label(ctrl, text="Chirp start/end/duration:").grid(row=3, column=0, sticky="w", pady=4)
        chirp = ttk.Frame(ctrl)
        chirp.grid(row=3, column=1, columnspan=3, sticky="w", padx=6)
        ttk.Entry(chirp, textvariable=self.f0_var, width=8).pack(side="left")
        ttk.Entry(chirp, textvariable=self.f1_var, width=8).pack(side="left", padx=4)
        ttk.Entry(chirp, textvariable=self.dur_var, width=8).pack(side="left")
        ttk.Combobox(chirp, textvariable=self.kind_var, values=("linear", "exponential"), state="readonly", width=12).pack(
            side="left", padx=8)

        self.lowpass_var = tk.StringVar()
        self.invert_var = tk.BooleanVar(value=False)
        ttk.Label(ctrl, text="Response LP Hz:").grid(row=4, column=0, sticky="w", pady=4)
        ttk.Entry(ctrl, textvariable=self.lowpass_var, width=10).grid(row=4, column=1, sticky="w", padx=6)
        ttk.Checkbutton(ctrl, text="Invert response", variable=self.invert_var).grid(row=4, column=2, sticky="w")
        ttk.Button(ctrl, text="Generate", command=self._generate).grid(row=4, column=3, sticky="e")

        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(ctrl, textvariable=self.status_var, foreground="grey").grid(row=5, column=0, columnspan=4, sticky="w")

    def _browse(self):
        path = filedialog.askopenfilename(title="Select dyno_pdo.csv", filetypes=(("CSV", "*.csv"), ("All files", "*")))
        if path:
            self.path_var.set(path)

    def _latest(self):
        try:
            self.path_var.set(str(latest_log(self.path_var.get().strip() or "test_data_log")))
        except Exception as exc:
            messagebox.showerror("Latest log failed", str(exc))

    def _float_or_none(self, var: tk.StringVar):
        text = var.get().strip()
        return float(text) if text else None

    def _generate(self):
        try:
            csv_path = resolve_csv_path(self.path_var.get().strip() or "test_data_log")
            lowpass = self._float_or_none(self.lowpass_var)
            result = compute_bode(
                csv_path,
                preset_name=self.preset_var.get(),
                reference=self.ref_var.get().strip() or None,
                response=self.resp_var.get().strip() or None,
                chirp_start_hz=self._float_or_none(self.f0_var),
                chirp_end_hz=self._float_or_none(self.f1_var),
                chirp_duration_s=self._float_or_none(self.dur_var),
                chirp_kind=self.kind_var.get(),
                invert_response=bool(self.invert_var.get()),
                lowpass_hz=lowpass,
            )
            fig = make_bode_figure(result, show_raw=lowpass is not None)
        except Exception as exc:
            messagebox.showerror("Bode plot failed", str(exc))
            return

        if self._canvas is not None:
            self._canvas.get_tk_widget().destroy()
        if self._toolbar is not None:
            self._toolbar.destroy()
        self._fig = fig
        self._canvas = FigureCanvasTkAgg(fig, master=self._plot_frame)
        self._canvas.draw()
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._toolbar = NavigationToolbar2Tk(self._canvas, self._plot_frame)
        self._toolbar.update()

        out_path = Path(result.csv_path).with_name(f"bode_{self.preset_var.get()}.png")
        fig.savefig(out_path, dpi=150, bbox_inches="tight")
        self.status_var.set(f"Saved {out_path}")


if __name__ == "__main__":
    DynoBodeApp().mainloop()
