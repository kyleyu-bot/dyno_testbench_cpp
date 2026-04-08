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
        QLabel, QComboBox, QPushButton, QSpinBox,
        QListWidget, QListWidgetItem, QGroupBox,
        QMenu, QAction, QFileDialog, QSizePolicy, QMessageBox,
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
DEFAULT_DIR = "test_data_log"

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
        # ── Top toolbar ───────────────────────────────────────────────────────
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

        # ── Plot grid (right panel) ────────────────────────────────────────────
        self._grid = PlotGrid()

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(sig_group)
        splitter.addWidget(self._grid)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([200, 1200])

        central = QWidget()
        vlay    = QVBoxLayout(central)
        vlay.setContentsMargins(4, 4, 4, 4)
        vlay.setSpacing(4)
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
        rows = len(df)
        dur  = (df[X_FIELD].iloc[-1] - df[X_FIELD].iloc[0]) * 1e-9 if rows > 1 else 0
        self._status.setText(
            f"{os.path.basename(path)}  |  {rows:,} rows  |  {dur:.1f} s  |  {len(df.columns)} signals"
        )

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
