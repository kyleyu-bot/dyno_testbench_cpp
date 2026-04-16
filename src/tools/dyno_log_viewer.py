#!/usr/bin/env python3
"""
Dyno PDO Log Viewer
===================
Usage:
    python3 src/tools/dyno_log_viewer.py [--log-dir <path>]

    --log-dir   Directory to scan for CSV files (default: test_data_log)

Features:
  - Dropdown to select a log file from test_data_log/
  - Signal list (drag source) — drag onto plot cells to overlay traces
  - Multi-cell plot grid with shared x-axis (time in seconds, derived from stamp_ns)
  - Right-click a curve to remove it; right-click cell to clear all
  - Export: save individual cell as PNG, or all cells as a single PDF
"""

import argparse
import os
import sys

import numpy as np

try:
    from PyQt5.QtCore    import Qt, QTimer, QMimeData, QByteArray, QRectF
    from PyQt5.QtGui     import QDrag, QPainter, QColor
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget,
        QVBoxLayout, QHBoxLayout, QGridLayout, QSplitter,
        QLabel, QComboBox, QPushButton, QSpinBox, QLineEdit,
        QListWidget, QListWidgetItem, QGroupBox,
        QMenu, QAction, QFileDialog, QSizePolicy, QMessageBox,
        QFrame, QScrollArea,
    )
except ImportError:
    print("ERROR: PyQt5 not found.  pip install PyQt5", file=sys.stderr)
    sys.exit(1)

try:
    import pyqtgraph as pg
    from pyqtgraph.exporters import ImageExporter
except ImportError:
    print("ERROR: pyqtgraph not found.  pip install pyqtgraph", file=sys.stderr)
    sys.exit(1)

try:
    import pandas as pd
except ImportError:
    print("ERROR: pandas not found.  pip install pandas", file=sys.stderr)
    sys.exit(1)

# ── Constants ──────────────────────────────────────────────────────────────────

MIME_TYPE   = "application/x-dyno-log-signal"
X_FIELD     = "stamp_ns"          # always the x-axis; converted to seconds

# Resolve default log dir relative to repo root (two levels up from src/tools/)
# so the tool works regardless of which directory it is launched from.
_REPO_ROOT  = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_DIR = os.path.join(_REPO_ROOT, "test_data_log")

CURVE_COLORS = [
    "#e74c3c", "#3498db", "#2ecc71", "#f39c12",
    "#9b59b6", "#1abc9c", "#e67e22", "#ecf0f1",
    "#ff6b9d", "#c0392b", "#2980b9", "#27ae60",
]

# ── Signal list widget (drag source) ──────────────────────────────────────────

class SignalList(QListWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setDragEnabled(True)
        self.setDragDropMode(QListWidget.DragOnly)
        self.setSelectionMode(QListWidget.SingleSelection)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

    def populate(self, columns: list[str]) -> None:
        self.clear()
        for col in columns:
            if col == X_FIELD:
                continue
            item = QListWidgetItem(col)
            item.setData(Qt.UserRole, col)
            self.addItem(item)

    def mimeData(self, items):
        mime = QMimeData()
        if items:
            key = items[0].data(Qt.UserRole) or ""
            mime.setData(MIME_TYPE, QByteArray(key.encode()))
        return mime


# ── Plot cell (drop target) ────────────────────────────────────────────────────

class PlotCell(pg.PlotWidget):
    def __init__(self, parent=None):
        super().__init__(parent, background="#1a1a2e")
        self._curves:    list[dict] = []   # {field, item}
        self._color_idx: int        = 0
        self._df:        "pd.DataFrame | None" = None

        self.setAcceptDrops(True)
        self.addLegend(offset=(5, 5))
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setLabel("bottom", "time (s)")
        self.setMinimumSize(220, 160)

    def set_dataframe(self, df: "pd.DataFrame") -> None:
        self._df = df
        # Refresh existing curves with new data.
        fields = [c["field"] for c in self._curves]
        self._clear()
        for f in fields:
            if f in df.columns:
                self._add_curve(f)

    def _x(self) -> "np.ndarray":
        t = self._df[X_FIELD].to_numpy(dtype=np.float64)
        return (t - t[0]) * 1e-9   # ns → seconds, relative to start

    def _add_curve(self, field: str) -> None:
        if self._df is None or field not in self._df.columns:
            return
        for c in self._curves:
            if c["field"] == field:
                return  # already present
        color = CURVE_COLORS[self._color_idx % len(CURVE_COLORS)]
        self._color_idx += 1
        item = self.plot(
            self._x(),
            self._df[field].to_numpy(dtype=np.float64),
            name=field,
            pen=pg.mkPen(color, width=1.5),
        )
        self._curves.append({"field": field, "item": item})

    def _remove_curve(self, idx: int) -> None:
        c = self._curves.pop(idx)
        self.removeItem(c["item"])

    def _clear(self) -> None:
        for c in self._curves:
            self.removeItem(c["item"])
        self._curves.clear()

    # ── drag / drop ────────────────────────────────────────────────────────────

    def dragEnterEvent(self, ev):
        if ev.mimeData().hasFormat(MIME_TYPE):
            ev.acceptProposedAction()
        else:
            ev.ignore()

    def dragMoveEvent(self, ev):
        ev.acceptProposedAction()

    def dropEvent(self, ev):
        field = bytes(ev.mimeData().data(MIME_TYPE)).decode()
        self._add_curve(field)
        ev.acceptProposedAction()

    def contextMenuEvent(self, ev):
        menu = QMenu(self)
        for i, c in enumerate(self._curves):
            act = QAction(f"Remove: {c['field']}", self)
            act.triggered.connect(lambda _, idx=i: self._remove_curve(idx))
            menu.addAction(act)
        if self._curves:
            menu.addSeparator()
        menu.addAction("Clear all", self._clear)
        menu.exec_(ev.globalPos())


# ── Plot grid ──────────────────────────────────────────────────────────────────

class PlotGrid(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._layout = QGridLayout(self)
        self._layout.setSpacing(4)
        self._cells: list[list[PlotCell]] = []
        self._df: "pd.DataFrame | None" = None
        self._set_dims(2, 2)

    def _set_dims(self, rows: int, cols: int) -> None:
        for row in self._cells:
            for cell in row:
                self._layout.removeWidget(cell)
                cell.deleteLater()
        self._cells = []
        for r in range(rows):
            row_cells = []
            for c in range(cols):
                cell = PlotCell(self)
                if self._df is not None:
                    cell.set_dataframe(self._df)
                self._layout.addWidget(cell, r, c)
                row_cells.append(cell)
            self._cells.append(row_cells)

    def set_dims(self, rows: int, cols: int) -> None:
        self._set_dims(rows, cols)

    def set_dataframe(self, df: "pd.DataFrame") -> None:
        self._df = df
        for row in self._cells:
            for cell in row:
                cell.set_dataframe(df)

    def all_cells(self) -> list[PlotCell]:
        return [cell for row in self._cells for cell in row]


# ── Display box (drop target, shows scalar summary) ───────────────────────────

class DisplayBox(QFrame):
    """Drop-target cell showing a signal's last / min / max value from the loaded CSV."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._field: "str | None"          = None
        self._df:    "pd.DataFrame | None" = None

        self.setAcceptDrops(True)
        self.setFrameShape(QFrame.Box)
        self.setLineWidth(1)
        self.setStyleSheet(
            "DisplayBox { background: #1a1a2e; border: 1px solid #444; }")
        self.setMinimumHeight(80)

        vlay = QVBoxLayout(self)
        vlay.setContentsMargins(6, 4, 6, 4)
        vlay.setSpacing(2)

        self._lbl_name = QLabel("— drop signal —")
        self._lbl_name.setAlignment(Qt.AlignCenter)
        self._lbl_name.setStyleSheet("color: #888; font-size: 13px;")

        self._lbl_value = QLabel("")
        self._lbl_value.setAlignment(Qt.AlignCenter)
        self._lbl_value.setStyleSheet(
            "color: #2ecc71; font-size: 26px; font-family: monospace; font-weight: bold;")

        self._lbl_stats = QLabel("")
        self._lbl_stats.setAlignment(Qt.AlignCenter)
        self._lbl_stats.setStyleSheet("color: #888; font-size: 12px;")

        vlay.addWidget(self._lbl_name)
        vlay.addWidget(self._lbl_value)
        vlay.addWidget(self._lbl_stats)

    # ── data ───────────────────────────────────────────────────────────────────

    def set_dataframe(self, df: "pd.DataFrame") -> None:
        self._df = df
        self._refresh()

    def _refresh(self) -> None:
        if self._df is None or self._field is None or self._field not in self._df.columns:
            self._lbl_value.setText("")
            self._lbl_stats.setText("")
            return
        col  = self._df[self._field].to_numpy(dtype=np.float64)
        last = col[-1] if len(col) else float("nan")
        self._lbl_value.setText(f"{last:.6g}")
        self._lbl_stats.setText(f"min {col.min():.6g}   max {col.max():.6g}")

    def _set_field(self, field: str) -> None:
        self._field = field
        self._lbl_name.setText(field)
        self._lbl_name.setStyleSheet("color: #ecf0f1; font-size: 13px;")
        self._refresh()

    def _clear(self) -> None:
        self._field = None
        self._lbl_name.setText("— drop signal —")
        self._lbl_name.setStyleSheet("color: #888; font-size: 13px;")
        self._lbl_value.setText("")
        self._lbl_stats.setText("")

    # ── drag / drop ────────────────────────────────────────────────────────────

    def dragEnterEvent(self, ev):
        if ev.mimeData().hasFormat(MIME_TYPE):
            ev.acceptProposedAction()
        else:
            ev.ignore()

    def dragMoveEvent(self, ev):
        ev.acceptProposedAction()

    def dropEvent(self, ev):
        field = bytes(ev.mimeData().data(MIME_TYPE)).decode()
        self._set_field(field)
        ev.acceptProposedAction()

    def contextMenuEvent(self, ev):
        menu = QMenu(self)
        if self._field:
            menu.addAction(f"Clear: {self._field}", self._clear)
        else:
            menu.addAction("(empty)", lambda: None).setEnabled(False)
        menu.exec_(ev.globalPos())


# ── Display panel (collection of DisplayBoxes) ─────────────────────────────────

class DisplayPanel(QWidget):
    """Scrollable column of DisplayBox drop-targets."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._df:    "pd.DataFrame | None" = None
        self._boxes: list[DisplayBox]      = []

        self._inner = QWidget()
        self._vlay  = QVBoxLayout(self._inner)
        self._vlay.setSpacing(4)
        self._vlay.setContentsMargins(4, 4, 4, 4)
        self._vlay.addStretch(1)   # sentinel — always at the end

        scroll = QScrollArea()
        scroll.setWidget(self._inner)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        self._set_count(4)

    def set_count(self, n: int) -> None:
        self._set_count(n)

    def set_dataframe(self, df: "pd.DataFrame") -> None:
        self._df = df
        for box in self._boxes:
            box.set_dataframe(df)

    def _set_count(self, n: int) -> None:
        while len(self._boxes) > n:
            box = self._boxes.pop()
            self._vlay.removeWidget(box)
            box.deleteLater()
        while len(self._boxes) < n:
            box = DisplayBox(self._inner)
            if self._df is not None:
                box.set_dataframe(self._df)
            self._vlay.insertWidget(len(self._boxes), box)
            self._boxes.append(box)


# ── Main window ────────────────────────────────────────────────────────────────

class DynoLogViewer(QMainWindow):
    def __init__(self, log_dir: str):
        super().__init__()
        self._log_dir = os.path.abspath(log_dir)
        self._df: "pd.DataFrame | None" = None
        self.setWindowTitle("Dyno PDO Log Viewer")
        self.resize(1400, 800)
        pg.setConfigOptions(antialias=False, useNumba=False)
        self._build_ui()
        self._refresh_file_list()

    # ── UI construction ────────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Directory row ─────────────────────────────────────────────────────
        dir_row  = QWidget()
        dlay     = QHBoxLayout(dir_row)
        dlay.setContentsMargins(6, 2, 6, 2)
        dlay.setSpacing(8)

        dlay.addWidget(QLabel("Log dir:"))
        self._dir_edit = QLineEdit(self._log_dir)
        self._dir_edit.setMinimumWidth(420)
        self._dir_edit.returnPressed.connect(self._on_dir_changed)
        dlay.addWidget(self._dir_edit)

        browse_btn = QPushButton("Browse...")
        browse_btn.setFixedWidth(80)
        browse_btn.clicked.connect(self._on_browse_dir)
        dlay.addWidget(browse_btn)
        dlay.addStretch()

        # ── File / controls toolbar ───────────────────────────────────────────
        toolbar = QWidget()
        tlay    = QHBoxLayout(toolbar)
        tlay.setContentsMargins(6, 4, 6, 4)
        tlay.setSpacing(8)

        tlay.addWidget(QLabel("Log file:"))
        self._file_combo = QComboBox()
        self._file_combo.setMinimumWidth(320)
        self._file_combo.currentIndexChanged.connect(self._on_file_selected)
        tlay.addWidget(self._file_combo)

        refresh_btn = QPushButton("Refresh")
        refresh_btn.setFixedWidth(70)
        refresh_btn.clicked.connect(self._refresh_file_list)
        tlay.addWidget(refresh_btn)

        tlay.addWidget(_vline())

        tlay.addWidget(QLabel("Rows:"))
        self._rows_spin = QSpinBox()
        self._rows_spin.setRange(1, 6)
        self._rows_spin.setValue(2)
        self._rows_spin.setFixedWidth(50)
        self._rows_spin.valueChanged.connect(self._on_dims_changed)
        tlay.addWidget(self._rows_spin)

        tlay.addWidget(QLabel("Cols:"))
        self._cols_spin = QSpinBox()
        self._cols_spin.setRange(1, 6)
        self._cols_spin.setValue(2)
        self._cols_spin.setFixedWidth(50)
        self._cols_spin.valueChanged.connect(self._on_dims_changed)
        tlay.addWidget(self._cols_spin)

        tlay.addWidget(_vline())

        export_png_btn = QPushButton("Export PNG")
        export_png_btn.clicked.connect(self._export_png)
        tlay.addWidget(export_png_btn)

        export_pdf_btn = QPushButton("Export PDF")
        export_pdf_btn.clicked.connect(self._export_pdf)
        tlay.addWidget(export_pdf_btn)

        tlay.addWidget(_vline())

        tlay.addWidget(QLabel("Displays:"))
        self._displays_spin = QSpinBox()
        self._displays_spin.setRange(0, 20)
        self._displays_spin.setValue(4)
        self._displays_spin.setFixedWidth(50)
        self._displays_spin.valueChanged.connect(self._on_display_count_changed)
        tlay.addWidget(self._displays_spin)

        tlay.addStretch()

        self._status = QLabel("No file loaded")
        self._status.setStyleSheet("color: #888;")
        tlay.addWidget(self._status)

        # ── Signal list (left panel) ───────────────────────────────────────────
        sig_group = QGroupBox("Signals")
        sig_vlay  = QVBoxLayout(sig_group)
        sig_vlay.setContentsMargins(4, 6, 4, 4)
        self._sig_list = SignalList()
        sig_vlay.addWidget(self._sig_list)
        sig_group.setMinimumWidth(180)
        sig_group.setMaximumWidth(260)

        # ── Plot grid (centre panel) ───────────────────────────────────────────
        self._grid = PlotGrid()

        # ── Display panel (right panel) ────────────────────────────────────────
        disp_group = QGroupBox("Displays")
        disp_vlay  = QVBoxLayout(disp_group)
        disp_vlay.setContentsMargins(4, 6, 4, 4)
        self._display_panel = DisplayPanel()
        disp_vlay.addWidget(self._display_panel)
        disp_group.setMinimumWidth(160)
        disp_group.setMaximumWidth(280)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(sig_group)
        splitter.addWidget(self._grid)
        splitter.addWidget(disp_group)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        splitter.setSizes([200, 1000, 200])

        central = QWidget()
        vlay    = QVBoxLayout(central)
        vlay.setContentsMargins(4, 4, 4, 4)
        vlay.setSpacing(0)
        vlay.addWidget(dir_row)
        vlay.addWidget(toolbar)
        vlay.addWidget(splitter, 1)
        self.setCentralWidget(central)

    # ── File management ────────────────────────────────────────────────────────

    def _refresh_file_list(self):
        self._file_combo.blockSignals(True)
        prev = self._file_combo.currentText()
        self._file_combo.clear()
        if os.path.isdir(self._log_dir):
            files = sorted(
                [f for f in os.listdir(self._log_dir) if f.endswith(".csv")],
                reverse=True,   # newest first
            )
            for f in files:
                self._file_combo.addItem(f, userData=os.path.join(self._log_dir, f))
        self._file_combo.blockSignals(False)
        # Restore previous selection if still present, else auto-load first.
        idx = self._file_combo.findText(prev)
        if idx >= 0:
            self._file_combo.setCurrentIndex(idx)
        elif self._file_combo.count() > 0:
            self._file_combo.setCurrentIndex(0)
            self._load_file(self._file_combo.itemData(0))

    def _on_file_selected(self, idx: int):
        path = self._file_combo.itemData(idx)
        if path:
            self._load_file(path)

    def _load_file(self, path: str):
        try:
            df = pd.read_csv(path)
        except Exception as e:
            self._status.setText(f"Error: {e}")
            return
        self._df = df
        self._sig_list.populate(list(df.columns))
        self._grid.set_dataframe(df)
        self._display_panel.set_dataframe(df)
        rows = len(df)
        dur  = (df[X_FIELD].iloc[-1] - df[X_FIELD].iloc[0]) * 1e-9 if rows > 1 else 0
        self._status.setText(
            f"{os.path.basename(path)}  |  {rows:,} rows  |  {dur:.1f} s  |  {len(df.columns)} signals"
        )

    # ── Directory selection ────────────────────────────────────────────────────

    def _on_dir_changed(self):
        """Called when the user presses Enter in the directory line-edit."""
        path = self._dir_edit.text().strip()
        if os.path.isdir(path):
            self._log_dir = os.path.abspath(path)
            self._dir_edit.setText(self._log_dir)
            self._refresh_file_list()
        else:
            # Revert to last valid directory.
            self._dir_edit.setText(self._log_dir)

    def _on_browse_dir(self):
        """Open a directory picker and redirect the log source."""
        path = QFileDialog.getExistingDirectory(
            self, "Select Log Directory", self._log_dir)
        if path:
            self._log_dir = path
            self._dir_edit.setText(path)
            self._refresh_file_list()

    # ── Display count ──────────────────────────────────────────────────────────

    def _on_display_count_changed(self):
        self._display_panel.set_count(self._displays_spin.value())

    # ── Grid resize ────────────────────────────────────────────────────────────

    def _on_dims_changed(self):
        self._grid.set_dims(self._rows_spin.value(), self._cols_spin.value())
        if self._df is not None:
            self._grid.set_dataframe(self._df)

    # ── Export ─────────────────────────────────────────────────────────────────

    def _export_png(self):
        """Export each non-empty cell as a separate PNG."""
        cells = [c for c in self._grid.all_cells() if c._curves]
        if not cells:
            QMessageBox.information(self, "Export", "No curves to export.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export PNG", "", "PNG Images (*.png)")
        if not path:
            return
        base, _ = os.path.splitext(path)
        for i, cell in enumerate(cells):
            fname = f"{base}_{i+1}.png" if len(cells) > 1 else f"{base}.png"
            exporter = ImageExporter(cell.plotItem)
            exporter.export(fname)
        QMessageBox.information(self, "Export", f"Saved {len(cells)} PNG(s).")

    def _export_pdf(self):
        """Export all cells stacked vertically into one PDF via matplotlib."""
        cells = [c for c in self._grid.all_cells() if c._curves]
        if not cells:
            QMessageBox.information(self, "Export", "No curves to export.")
            return
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from matplotlib.backends.backend_pdf import PdfPages
        except ImportError:
            QMessageBox.critical(self, "Export", "matplotlib not found.\npip install matplotlib")
            return

        path, _ = QFileDialog.getSaveFileName(
            self, "Export PDF", "", "PDF Files (*.pdf)")
        if not path:
            return

        with PdfPages(path) as pdf:
            for cell in cells:
                fig, ax = plt.subplots(figsize=(12, 4))
                ax.set_xlabel("time (s)")
                ax.grid(True, alpha=0.3)
                if self._df is not None:
                    t = (self._df[X_FIELD].to_numpy(dtype=np.float64) -
                         self._df[X_FIELD].iloc[0]) * 1e-9
                    for curve in cell._curves:
                        field = curve["field"]
                        if field in self._df.columns:
                            ax.plot(t, self._df[field].to_numpy(dtype=np.float64),
                                    label=field, linewidth=0.8)
                ax.legend(fontsize=7, loc="best")
                fig.tight_layout()
                pdf.savefig(fig)
                plt.close(fig)

        QMessageBox.information(self, "Export", f"Saved PDF: {os.path.basename(path)}")


# ── Helpers ────────────────────────────────────────────────────────────────────

def _vline() -> QWidget:
    w = QWidget()
    w.setFixedWidth(1)
    w.setStyleSheet("background: #555;")
    w.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
    return w


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="Dyno PDO Log Viewer")
    p.add_argument("--log-dir", default=DEFAULT_DIR,
                   help=f"Directory to scan for CSV log files (default: {DEFAULT_DIR})")
    args = p.parse_args()

    app    = QApplication(sys.argv)
    window = DynoLogViewer(log_dir=args.log_dir)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
