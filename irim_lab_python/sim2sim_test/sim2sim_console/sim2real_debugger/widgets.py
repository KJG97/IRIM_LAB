"""UI widgets: drag-drop zone and observation plot panel."""

from typing import List, Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from .config import PLOT_COLOR_DEFAULT, PLOT_COLOR_MAP, PLOT_CURVE_WIDTH, PLOT_INDEX_MAX
from .config import ObsDict


class DropZone(QFrame):
    """Drag-and-drop zone: policy (.pt/.pth), trajectory (.npz), env/agent (.yaml/.yml)."""

    files_dropped = Signal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAcceptDrops(True)
        self._set_idle_style()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        lbl = QLabel("Drag&Drop")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("font-size: 32px; font-weight: bold; color: #e0e0e0;")
        layout.addWidget(lbl)
        hint = QLabel(".pt / .pth / .npz / .yaml")
        hint.setAlignment(Qt.AlignCenter)
        hint.setStyleSheet("font-size: 11px; color: #888;")
        layout.addWidget(hint)

    @staticmethod
    def _is_supported(path: str) -> bool:
        p = path.lower()
        return p.endswith((".pt", ".pth", ".npz", ".yaml", ".yml"))

    def _set_idle_style(self) -> None:
        self.setStyleSheet("QFrame { background-color: #333; border: 2px dashed #777; border-radius: 10px; }")

    def _set_ready_style(self) -> None:
        self.setStyleSheet("QFrame { background-color: #2b5c2b; border: 2px dashed #00ff00; border-radius: 10px; }")

    def dragEnterEvent(self, event) -> None:
        if not event.mimeData().hasUrls():
            event.ignore()
            return
        paths = [u.toLocalFile() for u in event.mimeData().urls()]
        if any(self._is_supported(p) for p in paths):
            self._set_ready_style()
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragLeaveEvent(self, event) -> None:
        self._set_idle_style()
        event.accept()

    def dropEvent(self, event) -> None:
        self._set_idle_style()
        if not event.mimeData().hasUrls():
            return
        paths = [u.toLocalFile() for u in event.mimeData().urls()]
        paths = [p for p in paths if self._is_supported(p)]
        if paths:
            self.files_dropped.emit(paths)


class PlotPanel(QWidget):
    """Observation plot panel with ring buffer and trajectory overlay."""

    def __init__(self, buffer_size: int = 400, plot_update_hz: int = 30, parent=None):
        super().__init__(parent)
        self.buffer_size = int(buffer_size)
        self.plot_update_hz = int(plot_update_hz)

        self._selected_key: Optional[str] = None
        self._selected_index: int = -1

        self._buf_y: Optional[np.ndarray] = None
        self._cursor: int = 0
        self._filled: bool = False
        self._curves: List[pg.PlotDataItem] = []

        self._trajectory_data: Optional[np.ndarray] = None
        self._trajectory_curve: Optional[pg.PlotDataItem] = None

        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        ctrl = QFrame()
        ctrl_layout = QHBoxLayout(ctrl)
        ctrl_layout.setContentsMargins(0, 0, 0, 0)

        self.spn_index = QSpinBox()
        self.spn_index.setRange(-1, PLOT_INDEX_MAX)
        self.spn_index.setValue(-1)
        self.spn_index.setToolTip("component index (-1 = ALL)")
        self.spn_index.valueChanged.connect(self._on_index_changed)

        self.btn_clear = QPushButton("Clear")
        self.btn_clear.clicked.connect(self.clear_buffer)

        ctrl_layout.addWidget(QLabel("Index:"))
        ctrl_layout.addWidget(self.spn_index)
        ctrl_layout.addStretch(1)
        ctrl_layout.addWidget(self.btn_clear)
        layout.addWidget(ctrl)

        pg.setConfigOptions(antialias=False)
        pg.setConfigOption("background", (31, 31, 31))
        pg.setConfigOption("foreground", (220, 220, 220))

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.setLabel("bottom", "t (samples)")
        self.plot_widget.setLabel("left", "value")
        self.plot_widget.setTitle("Plot Display")
        for axis_name in ("left", "bottom"):
            axis = self.plot_widget.getAxis(axis_name)
            if axis is not None:
                axis.enableAutoSIPrefix(False)
        layout.addWidget(self.plot_widget, 1)

        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self.redraw)
        self._plot_timer.start(int(1000 / max(1, self.plot_update_hz)))

    @property
    def selected_key(self) -> Optional[str]:
        return self._selected_key

    def set_selected_observation(self, obs_name: Optional[str]) -> None:
        self._selected_key = obs_name
        self.clear_buffer()

        if obs_name:
            title = f"{obs_name} (index={self._selected_index})" if self._selected_index >= 0 else obs_name
            self.plot_widget.setTitle(title)
        else:
            self.plot_widget.setTitle("Plot Display")
            self.set_max_index(-1)

    def set_max_index(self, max_idx: int) -> None:
        if max_idx < 0:
            self.spn_index.setRange(-1, PLOT_INDEX_MAX)
        else:
            self.spn_index.setRange(-1, max(0, max_idx))
            if self._selected_index > max_idx:
                self.spn_index.setValue(-1)
                self._selected_index = -1

    def set_trajectory_data(self, positions: Optional[np.ndarray]) -> None:
        self._trajectory_data = np.asarray(positions) if positions is not None else None

    def plot_trajectory_index0(self) -> None:
        if self._trajectory_data is None:
            return
        pos = np.asarray(self._trajectory_data)
        if pos.ndim != 2 or pos.shape[0] == 0:
            return

        x = np.arange(pos.shape[0], dtype=np.float32)
        y = pos[:, 0].astype(np.float32, copy=False)

        if self._trajectory_curve is None:
            self._trajectory_curve = self.plot_widget.plot(x, y, pen=pg.mkPen(color="#ffff00", width=2))
        else:
            self._trajectory_curve.setData(x, y)

        self.plot_widget.setXRange(0, max(10, pos.shape[0] - 1), padding=0.0)
        self.plot_widget.setTitle(f"Trajectory (index 0) - {pos.shape[0]} steps")

    def ingest_obs(self, obs: ObsDict) -> None:
        if self._selected_key is None or self._selected_key not in obs:
            return

        y = np.asarray(obs[self._selected_key]).reshape(-1)
        if y.size == 0:
            return

        if self._selected_index >= 0:
            idx = min(self._selected_index, y.size - 1)
            y = y[idx : idx + 1]

        self._push_sample(y)

    def _push_sample(self, y: np.ndarray) -> None:
        y = np.asarray(y).reshape(-1)
        d = int(y.size)
        if d <= 0:
            return

        if self._buf_y is None or self._buf_y.shape[1] != d:
            self._buf_y = np.zeros((self.buffer_size, d), dtype=np.float32)
            self._cursor = 0
            self._filled = False
            self._init_curves(d)

        self._buf_y[self._cursor, :] = y
        self._cursor += 1
        if self._cursor >= self.buffer_size:
            self._cursor = 0
            self._filled = True

    def _init_curves(self, d: int) -> None:
        for c in self._curves:
            self.plot_widget.removeItem(c)
        self._curves.clear()

        if self._trajectory_curve is not None:
            self.plot_widget.removeItem(self._trajectory_curve)
            self._trajectory_curve = None

        pen = self._make_pen()
        for _ in range(d):
            self._curves.append(self.plot_widget.plot([], [], pen=pen))

        self.plot_widget.enableAutoRange(y=True, x=False)

    def _make_pen(self) -> pg.mkPen:
        color = PLOT_COLOR_MAP.get(self._selected_key, PLOT_COLOR_DEFAULT)
        return pg.mkPen(color=color, width=PLOT_CURVE_WIDTH)

    def redraw(self) -> None:
        if self._buf_y is None or not self._curves:
            return

        if not self._filled:
            n = self._cursor
            if n <= 2:
                return
            x = np.arange(n, dtype=np.float32)
            yv = self._buf_y[:n, :]
        else:
            n = self.buffer_size
            idx = np.concatenate([np.arange(self._cursor, n), np.arange(0, self._cursor)])
            x = np.arange(n, dtype=np.float32)
            yv = self._buf_y[idx, :]

        for i, curve in enumerate(self._curves):
            curve.setData(x, yv[:, i])

        self.plot_widget.setXRange(0, max(10, n - 1), padding=0.0)
        self.plot_widget.enableAutoRange(y=True, x=False)

    def clear_buffer(self) -> None:
        self._buf_y = None
        self._cursor = 0
        self._filled = False
        for c in self._curves:
            self.plot_widget.removeItem(c)
        self._curves.clear()

    def _on_index_changed(self, v: int) -> None:
        self._selected_index = int(v)
        self.clear_buffer()


__all__ = ["DropZone", "PlotPanel"]
