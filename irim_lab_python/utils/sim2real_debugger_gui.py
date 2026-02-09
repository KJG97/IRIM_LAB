import os
import sys
import threading
import logging
from dataclasses import dataclass
from typing import Callable, Dict, Optional, List, Tuple

import time
import numpy as np
import torch
from PySide6.QtCore import QTimer, Qt, Signal, QSignalBlocker
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QSplitter,
    QGroupBox,
    QPushButton,
    QLineEdit,
    QCheckBox,
    QSpinBox,
)

import pyqtgraph as pg

# --------------------------------------------------------------------------------------
# Logging (minimal by default)
# --------------------------------------------------------------------------------------
LOG = logging.getLogger("sim2real_debugger")
_DEBUG = os.environ.get("SIM2REAL_DEBUG", "0") == "1"
logging.basicConfig(
    level=logging.DEBUG if _DEBUG else logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
)

# --------------------------------------------------------------------------------------
# ROS2 imports (ROS2 Jazzy, Python 3.12)
# --------------------------------------------------------------------------------------
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Float32MultiArray

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    LOG.warning("ROS2 (rclpy) not available. ROS2 features will be disabled.")

# ======================================================================================
# Types / Config
# ======================================================================================
ObsDict = Dict[str, np.ndarray]
ObsProvider = Callable[[], ObsDict]


@dataclass
class DebuggerConfig:
    num_joints: int = 18
    update_hz: int = 30
    buffer_size: int = 200
    plot_update_hz: int = 30


ALLEX_ACTION_JOINT_NAMES = [
    "R_Shoulder_Pitch_Joint",
    "R_Shoulder_Roll_Joint",
    "R_Shoulder_Yaw_Joint",
    "R_Elbow_Joint",
    "R_Wrist_Yaw_Joint",
    "R_Wrist_Roll_Joint",
    "R_Wrist_Pitch_Joint",
    "R_Thumb_Yaw_Joint",
    "R_Thumb_CMC_Joint",
    "R_Thumb_MCP_Joint",
    "R_Index_MCP_Joint",
    "R_Index_PIP_Joint",
    "R_Middle_MCP_Joint",
    "R_Middle_PIP_Joint",
    "R_Ring_MCP_Joint",
    "R_Ring_PIP_Joint",
    "R_Little_MCP_Joint",
    "R_Little_PIP_Joint",
]

# 관측 UI 정의 (인덱스/차원/plot 가능 여부)
OBS_ITEMS = [
    ("actions", 0, 17, 18, True),
    ("hammer_pos", 18, 20, 3, False),
    ("joint_pos", 21, 38, 18, True),
    ("reference_joint_pos_error", 39, 56, 18, True),
    ("right_hand_joint_torque", 57, 71, 15, True),
    ("right_hand_base_pos", 72, 78, 7, True),
    ("target_right_hand_pose", 79, 85, 7, False),
]
EXPECTED_TOTAL_OBS_DIM_FALLBACK = sum(item[3] for item in OBS_ITEMS)  # 86


# ======================================================================================
# ROS2 Node
# ======================================================================================
if ROS2_AVAILABLE:

    class ROS2ObservationSubscriber(Node):
        """ROS2 노드: observation 데이터를 subscribe하고 publish합니다."""

        def __init__(self, obs_data: Dict[str, np.ndarray]):
            super().__init__("sim2real_debugger_obs_subscriber")
            self.obs_data = obs_data
            self._last_update: Dict[str, float] = {}

            self.create_subscription(Float32MultiArray, "/observation/actions", self._cb("actions"), 10)
            self.create_subscription(Float32MultiArray, "/observation/hammer_pos", self._cb("hammer_pos"), 10)
            self.create_subscription(Float32MultiArray, "/observation/joint_pos", self._cb("joint_pos"), 10)
            self.create_subscription(
                Float32MultiArray,
                "/observation/reference_joint_pos_error",
                self._cb("reference_joint_pos_error"),
                10,
            )
            self.create_subscription(
                Float32MultiArray,
                "/observation/right_hand_joint_torque",
                self._cb("right_hand_joint_torque"),
                10,
            )
            self.create_subscription(
                Float32MultiArray,
                "/observation/right_hand_base_pos",
                self._cb("right_hand_base_pos"),
                10,
            )
            self.create_subscription(
                Float32MultiArray,
                "/observation/target_right_hand_pose",
                self._cb("target_right_hand_pose"),
                10,
            )

            # Hammer position publisher (same topic; keep behavior)
            self.pub_hammer_pos = self.create_publisher(Float32MultiArray, "/observation/hammer_pos", 10)
            
            # Target right hand pose publisher
            self.pub_target_pose = self.create_publisher(Float32MultiArray, "/observation/target_right_hand_pose", 10)
            
            # Policy action publisher (18 DOF joint position commands)
            self.pub_policy_action = self.create_publisher(Float32MultiArray, "/policy/action", 10)

            self.get_logger().info("ROS2 Observation Subscriber started")

        def _cb(self, key: str):
            def _handler(msg: Float32MultiArray):
                self.obs_data[key] = np.asarray(msg.data, dtype=np.float32)
                self._last_update[key] = time.time()

            return _handler

        def get_last_update_age(self, key: str) -> Optional[float]:
            """마지막 수신 이후 경과 시간(sec) 반환. 기록이 없으면 None."""
            ts = self._last_update.get(key)
            if ts is None:
                return None
            return time.time() - ts

        def publish_hammer_pos(self, x: float, y: float, z: float) -> None:
            msg = Float32MultiArray()
            msg.data = [float(x), float(y), float(z)]
            self.pub_hammer_pos.publish(msg)
            # publish는 빈번할 수 있어 debug로 낮춤
            self.get_logger().debug(f"Published hammer_pos: [{x:.4f}, {y:.4f}, {z:.4f}]")
        
        def publish_target_pose(self, x: float, y: float, z: float, qw: float, qx: float, qy: float, qz: float) -> None:
            """Target right hand pose를 ROS2 topic으로 publish합니다."""
            msg = Float32MultiArray()
            msg.data = [float(x), float(y), float(z), float(qw), float(qx), float(qy), float(qz)]
            self.pub_target_pose.publish(msg)
            self.get_logger().debug(f"Published target_right_hand_pose: pos=[{x:.4f}, {y:.4f}, {z:.4f}], quat=[{qw:.4f}, {qx:.4f}, {qy:.4f}, {qz:.4f}]")
        
        def publish_policy_action(self, actions: np.ndarray) -> None:
            """Policy action (18 DOF joint positions)을 ROS2 topic으로 publish합니다.
            
            Args:
                actions: (18,) shape의 numpy 배열, ALLEX_ACTION_JOINT_NAMES 순서대로
            """
            if actions is None or actions.size != 18:
                self.get_logger().warning(f"Invalid actions shape for publish: {actions.shape if actions is not None else None}")
                return
            
            msg = Float32MultiArray()
            msg.data = [float(x) for x in actions.reshape(-1)]
            self.pub_policy_action.publish(msg)
            self.get_logger().debug(f"Published policy action: {msg.data[:3]}... (18 DOF)")


# ======================================================================================
# UI Widgets
# ======================================================================================
class DropZone(QFrame):
    files_dropped = Signal(list)  # list[str]

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

    @staticmethod
    def _is_supported(path: str) -> bool:
        p = path.lower()
        return p.endswith((".pt", ".pth", ".npz"))

    def _set_idle_style(self):
        self.setStyleSheet("QFrame { background-color: #333; border: 2px dashed #777; border-radius: 10px; }")

    def _set_ready_style(self):
        self.setStyleSheet("QFrame { background-color: #2b5c2b; border: 2px dashed #00ff00; border-radius: 10px; }")

    def dragEnterEvent(self, event):
        if not event.mimeData().hasUrls():
            event.ignore()
            return
        paths = [u.toLocalFile() for u in event.mimeData().urls()]
        if any(self._is_supported(p) for p in paths):
            self._set_ready_style()
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragLeaveEvent(self, event):
        self._set_idle_style()
        event.accept()

    def dropEvent(self, event):
        self._set_idle_style()
        if not event.mimeData().hasUrls():
            return
        paths = [u.toLocalFile() for u in event.mimeData().urls()]
        paths = [p for p in paths if self._is_supported(p)]
        if paths:
            self.files_dropped.emit(paths)


class PlotPanel(QWidget):
    def __init__(self, buffer_size: int = 400, plot_update_hz: int = 30, parent=None):
        super().__init__(parent)
        self.buffer_size = int(buffer_size)
        self.plot_update_hz = int(plot_update_hz)

        self._trajectory_key_name: str = "Trajectory (index 0)"

        self._selected_key: Optional[str] = None
        self._selected_index: int = -1
        self._freeze: bool = False
        self._autoscale: bool = True

        self._buf_y: Optional[np.ndarray] = None  # (T, D)
        self._cursor: int = 0
        self._filled: bool = False
        self._curves: List[pg.PlotDataItem] = []

        self._trajectory_data: Optional[np.ndarray] = None

        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        ctrl = QFrame()
        ctrl_layout = QHBoxLayout(ctrl)
        ctrl_layout.setContentsMargins(0, 0, 0, 0)

        self.spn_index = QSpinBox()
        self.spn_index.setRange(-1, 4096)
        self.spn_index.setValue(-1)
        self.spn_index.setToolTip("component index (-1 = ALL)")
        self.spn_index.valueChanged.connect(self._on_index_changed)

        self.chk_freeze = QCheckBox("Freeze")
        self.chk_freeze.stateChanged.connect(self._on_freeze_changed)

        self.chk_autoscale = QCheckBox("Auto-scale")
        self.chk_autoscale.setChecked(True)
        self.chk_autoscale.stateChanged.connect(self._on_autoscale_changed)

        self.btn_clear = QPushButton("Clear")
        self.btn_clear.clicked.connect(self.clear_buffer)

        ctrl_layout.addWidget(QLabel("Index:"))
        ctrl_layout.addWidget(self.spn_index)
        ctrl_layout.addWidget(self.chk_freeze)
        ctrl_layout.addWidget(self.chk_autoscale)
        ctrl_layout.addStretch(1)
        ctrl_layout.addWidget(self.btn_clear)
        layout.addWidget(ctrl)

        # pyqtgraph global config
        pg.setConfigOptions(antialias=False)
        pg.setConfigOption("background", (31, 31, 31))
        pg.setConfigOption("foreground", (220, 220, 220))

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.setLabel("bottom", "t (samples)")
        self.plot_widget.setLabel("left", "value")
        self.plot_widget.setTitle("Plot Display")
        layout.addWidget(self.plot_widget, 1)

        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self.redraw)
        self._plot_timer.start(int(1000 / max(1, self.plot_update_hz)))

    @property
    def selected_key(self) -> Optional[str]:
        return self._selected_key

    def set_selected_observation(self, obs_name: Optional[str], actions_trajectory: Optional[np.ndarray] = None) -> None:
        self._selected_key = obs_name
        self.clear_buffer()

        if obs_name == self._trajectory_key_name:
            if self._trajectory_data is not None:
                self.plot_trajectory(self._trajectory_data)
            return

        if obs_name:
            title = f"{obs_name} (index={self._selected_index})" if self._selected_index >= 0 else obs_name
            self.plot_widget.setTitle(title)
        else:
            self.plot_widget.setTitle("Plot Display")
    
    def plot_actions_trajectory(self, actions_trajectory: np.ndarray) -> None:
        """Actions trajectory를 한 번에 plot합니다. 실시간 버퍼를 사용하지 않습니다.
        
        Args:
            actions_trajectory: (T, 18) shape의 actions trajectory 배열
        """
        if actions_trajectory is None:
            return
        actions_trajectory = np.asarray(actions_trajectory)
        if actions_trajectory.ndim != 2 or actions_trajectory.shape[0] == 0 or actions_trajectory.shape[1] == 0:
            return
        
        num_frames, num_joints = actions_trajectory.shape
        
        # 기존 curves 제거
        for curve in self._curves:
            self.plot_widget.removeItem(curve)
        self._curves.clear()
        
        # x축은 고정 (0부터 num_frames-1까지)
        x = np.arange(num_frames, dtype=np.float32)
        
        # selected_index가 지정된 경우 해당 joint만 plot, 아니면 모든 joint plot
        if self._selected_index >= 0:
            if self._selected_index < num_joints:
                y = actions_trajectory[:, self._selected_index]  # (T,)
                curve = self.plot_widget.plot(
                    x, y,
                    pen=pg.mkPen(color="#ffff00", width=2),
                    name=f"Joint {self._selected_index}"
                )
                self._curves.append(curve)
        else:
            # 모든 joint plot
            for joint_idx in range(num_joints):
                y = actions_trajectory[:, joint_idx]  # (T,)
                curve = self.plot_widget.plot(
                    x, y,
                    pen=pg.mkPen(color="#ffff00", width=2),
                    name=f"Joint {joint_idx}"
                )
                self._curves.append(curve)
        
        # x축 범위 설정
        self.plot_widget.setXRange(0, max(10, num_frames - 1), padding=0.0)
        
        # 제목 업데이트
        title = f"actions (trajectory: {num_frames} frames, {num_joints} joints)"
        if self._selected_index >= 0:
            title += f" [index={self._selected_index}]"
        self.plot_widget.setTitle(title)
        
        # 버퍼는 사용하지 않음 (정적 plot)
        self._buf_y = None
        self._cursor = 0
        self._filled = False

    def ingest_obs(self, obs: ObsDict, trajectory_actions: Optional[np.ndarray] = None) -> None:
        if self._freeze or self._selected_key is None:
            return

        # Trajectory 전용 Key
        if self._selected_key == self._trajectory_key_name:
            if self._trajectory_data is not None:
                self.plot_trajectory(self._trajectory_data)
            return

        if self._selected_key not in obs:
            return

        y = np.asarray(obs[self._selected_key]).reshape(-1)
        if y.size == 0:
            return

        if self._selected_index >= 0:
            if self._selected_index >= y.size:
                return
            y = y[self._selected_index : self._selected_index + 1]

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
        for curve in self._curves:
            self.plot_widget.removeItem(curve)
        self._curves.clear()

        title = f"{self._selected_key} (index={self._selected_index})" if self._selected_key else "Plot Display"
        self.plot_widget.setTitle(title)

        # joint_pos는 초록색, 굵기 2로 표시
        # right_hand_joint_torque는 밝은 보라색, 굵기 2로 표시
        # right_hand_base_pos는 밝은 분홍색, 굵기 2로 표시
        if self._selected_key == "joint_pos":
            pen = pg.mkPen(color="#00ff00", width=2)
        elif self._selected_key == "right_hand_joint_torque":
            pen = pg.mkPen(color="#CC66FF", width=2)
        elif self._selected_key == "right_hand_base_pos":
            pen = pg.mkPen(color="#FF66CC", width=2)
        else:
            pen = pg.mkPen(width=1)

        for _ in range(d):
            self._curves.append(self.plot_widget.plot([], [], pen=pen))

        self.plot_widget.enableAutoRange(y=self._autoscale, x=False)

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
        if self._autoscale:
            self.plot_widget.enableAutoRange(y=True, x=False)

    def clear_buffer(self) -> None:
        self._buf_y = None
        self._cursor = 0
        self._filled = False
        for curve in self._curves:
            self.plot_widget.removeItem(curve)
        self._curves.clear()

    def plot_trajectory(self, positions: np.ndarray) -> None:
        if positions is None:
            return
        positions = np.asarray(positions)
        if positions.ndim != 2 or positions.shape[0] == 0 or positions.shape[1] == 0:
            return

        traj_0 = positions[:, 0]
        x = np.arange(positions.shape[0], dtype=np.float32)

        self.plot_widget.setXRange(0, max(10, positions.shape[0] - 1), padding=0.0)
        self.plot_widget.setTitle(f"Trajectory (index 0) - {positions.shape[0]} steps")
        self._trajectory_data = positions

    def clear_trajectory(self) -> None:
        self._trajectory_data = None

    def set_trajectory_data(self, positions: Optional[np.ndarray]) -> None:
        self._trajectory_data = np.asarray(positions) if positions is not None else None

    def _on_index_changed(self, v: int) -> None:
        self._selected_index = int(v)
        # index 변경 시, 단순히 버퍼를 초기화하여 새 shape로 다시 그리도록 함
        self.clear_buffer()

    def _on_freeze_changed(self, state: int) -> None:
        self._freeze = state == Qt.Checked

        # ✅ 완전 정지: PlotPanel의 redraw 타이머까지 멈춤
        if self._freeze:
            if self._plot_timer.isActive():
                self._plot_timer.stop()
        else:
            if not self._plot_timer.isActive():
                self._plot_timer.start(int(1000 / max(1, self.plot_update_hz)))
            self.redraw()  # (선택) 해제 직후 즉시 화면 갱신


    def _on_autoscale_changed(self, state: int) -> None:
        self._autoscale = state == Qt.Checked
        if self._autoscale:
            self.plot_widget.enableAutoRange(y=True, x=False)
            self.plot_widget.autoRange()   # ✅ 즉시 재계산/반영
        else:
            self.plot_widget.disableAutoRange(axis='y')  # ✅ y 고정


# ======================================================================================
# Main Window
# ======================================================================================
class Sim2RealDebugger(QMainWindow):
    _STYLE_BOX = (
        "QGroupBox { font-weight: bold; border: 2px solid #555; border-radius: 5px; "
        "margin-top: 10px; padding-top: 10px; } "
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }"
    )
    _STYLE_STATUS_OK = "background-color: #004400; color: #00ff00; padding: 4px 8px; font-weight: bold; font-size: 11px; border-radius: 5px;"
    _STYLE_STATUS_ERR = "background-color: #550000; color: #ffaaaa; padding: 4px 8px; font-weight: bold; font-size: 11px; border-radius: 5px;"
    _STYLE_STATUS_IDLE = "background-color: #444; padding: 4px 8px; font-weight: bold; font-size: 11px; border-radius: 5px;"

    def __init__(self, obs_provider: Optional[ObsProvider] = None, cfg: Optional[DebuggerConfig] = None):
        super().__init__()
        self.cfg = cfg or DebuggerConfig()

        self.setWindowTitle("Sim2Real Proprioception Debugger (PyQtGraph + DropZone)")
        self.resize(1618, 1000)
        self.setStyleSheet("background-color: #2b2b2b; color: #ffffff;")

        self.loaded_policy = None
        self.loaded_trajectory: Optional[dict] = None
        self.policy_expected_obs_dim: Optional[int] = None
        self.policy_expected_action_dim: Optional[int] = None

        # Inference / trajectory 재생 상태
        self._inference_running: bool = False
        # Trajectory는 기본적으로 50Hz, 약 2초(100 step) 기준이라고 가정
        self._base_traj_hz: float = 50.0
        self._traj_playback_time: float = 0.0  # [sec]
        self._traj_speed: float = 1.0         # 1.0 = 원래 속도(약 2초), 2.0 = 1초에 끝나도록 2배 빠르게
        # Residual RL 스케일 (IsaacLab 설정과 동일: residual_scale=0.2)
        self._residual_scale: float = 0.1

        # ROS2 state
        self.ros_node: Optional["Node"] = None
        self.ros_thread: Optional[threading.Thread] = None
        self.ros_running = False
        self.ros_obs_data: Dict[str, np.ndarray] = {}
        # 각 observation 별 마지막 ROS2 수신 시각 (sec)
        self.ros_obs_last_update: Dict[str, float] = {}

        if obs_provider is None:
            self.obs_provider = lambda: self.ros_obs_data if self.ros_running else {}
        else:
            self.obs_provider = obs_provider

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(10, 10, 10, 10)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        root_layout.addWidget(splitter, 1)

        # LEFT
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setSpacing(10)

        self.drop_zone = DropZone()
        self.drop_zone.setMinimumHeight(90)
        self.drop_zone.files_dropped.connect(self._on_files_dropped)
        left_layout.addWidget(self.drop_zone)

        # ROS2 Connection (한 줄로 배치, 높이 최소화)
        ros_group = QGroupBox("ROS2 Connection")
        ros_layout = QHBoxLayout(ros_group)
        ros_layout.setSpacing(6)
        ros_layout.setContentsMargins(8, 4, 8, 4)  # 상하 마진 최소화

        ros_layout.addWidget(QLabel("Domain ID:"))
        self.ros_domain_input = QLineEdit("0")
        self.ros_domain_input.setPlaceholderText("0")
        self.ros_domain_input.setMaximumWidth(80)
        ros_layout.addWidget(self.ros_domain_input)

        self.lbl_ros_status = QLabel("● Disconnected")
        self.lbl_ros_status.setStyleSheet("font-size: 11px; color: #888; font-weight: bold;")
        ros_layout.addWidget(self.lbl_ros_status)

        self.btn_ros_connect = QPushButton("Connect")
        self.btn_ros_connect.clicked.connect(self._on_ros_connect)
        if not ROS2_AVAILABLE:
            self.btn_ros_connect.setEnabled(False)
            self.btn_ros_connect.setToolTip("ROS2 (rclpy) not available")
        ros_layout.addWidget(self.btn_ros_connect)

        ros_layout.addStretch()

        ros_group.setStyleSheet(self._STYLE_BOX)
        left_layout.addWidget(ros_group)

        # Policy / Trajectory status
        status_row = QHBoxLayout()
        status_row.setSpacing(8)

        self.lbl_model_status = QLabel("Policy: None (drop .pt/.pth)")
        self.lbl_model_status.setStyleSheet(self._STYLE_STATUS_IDLE)
        self.lbl_model_status.setAlignment(Qt.AlignCenter)
        self.lbl_model_status.setMaximumHeight(40)
        self.lbl_model_status.setWordWrap(True)
        status_row.addWidget(self.lbl_model_status, 1)

        self.lbl_traj_status = QLabel("Trajectory: None (drop .npz)")
        self.lbl_traj_status.setStyleSheet(self._STYLE_STATUS_IDLE)
        self.lbl_traj_status.setAlignment(Qt.AlignCenter)
        self.lbl_traj_status.setMaximumHeight(40)
        self.lbl_traj_status.setWordWrap(True)
        status_row.addWidget(self.lbl_traj_status, 1)

        left_layout.addLayout(status_row)

        btn_row = QHBoxLayout()
        self.btn_unload_policy = QPushButton("Unload Policy")
        self.btn_unload_traj = QPushButton("Unload Trajectory")
        self.btn_unload_policy.clicked.connect(self.unload_policy)
        self.btn_unload_traj.clicked.connect(self.unload_trajectory)
        btn_row.addWidget(self.btn_unload_policy)
        btn_row.addWidget(self.btn_unload_traj)
        left_layout.addLayout(btn_row)

        # Control Section
        control_group = QGroupBox("Control Section")
        control_layout = QVBoxLayout(control_group)
        control_layout.setSpacing(8)

        hammer_label = QLabel("Hammer Position (ALLEX Chest 기준, m)")
        hammer_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        control_layout.addWidget(hammer_label)

        hammer_input_row = QHBoxLayout()
        hammer_input_row.addWidget(QLabel("X:"))
        self.hammer_x_input = QLineEdit("0.5408")
        hammer_input_row.addWidget(self.hammer_x_input)

        hammer_input_row.addWidget(QLabel("Y:"))
        self.hammer_y_input = QLineEdit("-0.1004")
        hammer_input_row.addWidget(self.hammer_y_input)

        hammer_input_row.addWidget(QLabel("Z:"))
        self.hammer_z_input = QLineEdit("-0.4279")
        hammer_input_row.addWidget(self.hammer_z_input)
        control_layout.addLayout(hammer_input_row)

        hammer_btn_row = QHBoxLayout()
        self.btn_apply_hammer = QPushButton("Apply")
        self.btn_reset_hammer = QPushButton("Reset")
        self.btn_apply_hammer.clicked.connect(self._on_apply_hammer)
        self.btn_reset_hammer.clicked.connect(self._on_reset_hammer)
        hammer_btn_row.addWidget(self.btn_apply_hammer)
        hammer_btn_row.addWidget(self.btn_reset_hammer)
        control_layout.addLayout(hammer_btn_row)

        # Target Right Hand Pose 입력 섹션
        target_pose_label = QLabel("Target Right Hand Pose")
        target_pose_label.setStyleSheet("font-weight: bold; font-size: 12px; margin-top: 8px;")
        control_layout.addWidget(target_pose_label)

        # Position (x, y, z) 입력
        target_pos_row = QHBoxLayout()
        target_pos_row.addWidget(QLabel("Pos X:"))
        self.target_x_input = QLineEdit("0.55")
        target_pos_row.addWidget(self.target_x_input)
        
        target_pos_row.addWidget(QLabel("Y:"))
        self.target_y_input = QLineEdit("-0.2")
        target_pos_row.addWidget(self.target_y_input)
        
        target_pos_row.addWidget(QLabel("Z:"))
        self.target_z_input = QLineEdit("0.4")
        target_pos_row.addWidget(self.target_z_input)
        control_layout.addLayout(target_pos_row)

        # Quaternion (qw, qx, qy, qz) 입력
        target_quat_row = QHBoxLayout()
        target_quat_row.addWidget(QLabel("Quat W:"))
        self.target_qw_input = QLineEdit("0.5")
        target_quat_row.addWidget(self.target_qw_input)
        
        target_quat_row.addWidget(QLabel("X:"))
        self.target_qx_input = QLineEdit("-0.5")
        target_quat_row.addWidget(self.target_qx_input)
        
        target_quat_row.addWidget(QLabel("Y:"))
        self.target_qy_input = QLineEdit("-0.5")
        target_quat_row.addWidget(self.target_qy_input)
        
        target_quat_row.addWidget(QLabel("Z:"))
        self.target_qz_input = QLineEdit("0.5")
        target_quat_row.addWidget(self.target_qz_input)
        control_layout.addLayout(target_quat_row)

        # 적용 & 해제 버튼
        target_btn_row = QHBoxLayout()
        self.btn_apply_target = QPushButton("Apply")
        self.btn_reset_target = QPushButton("Reset")
        self.btn_apply_target.clicked.connect(self._on_apply_target_pose)
        self.btn_reset_target.clicked.connect(self._on_reset_target_pose)
        target_btn_row.addWidget(self.btn_apply_target)
        target_btn_row.addWidget(self.btn_reset_target)
        control_layout.addLayout(target_btn_row)

        # Inference Control (Residual RL 테스트용)
        infer_row = QHBoxLayout()
        # Inference Speed 입력
        infer_row.addWidget(QLabel("Inference Speed:"))
        self.infer_speed_input = QLineEdit("1.0")
        self.infer_speed_input.setMaximumWidth(80)
        self.infer_speed_input.setToolTip(
            "1.0 = 원래 속도(50Hz, 약 2초)\n"
        )
        infer_row.addWidget(self.infer_speed_input)
        # Residual Scale 입력
        infer_row.addWidget(QLabel("Residual Scale:"))
        self.residual_scale_input = QLineEdit(f"{self._residual_scale:.2f}")
        self.residual_scale_input.setMaximumWidth(80)
        self.residual_scale_input.setToolTip(
            "Residual RL 스케일 (예: 0.1, 0.2 등)\n"
            "최종 행동 = reference_trajectory + residual * residual_scale"
        )
        infer_row.addWidget(self.residual_scale_input)

        self.btn_infer_start = QPushButton("Start Inference")
        self.btn_infer_start.setToolTip("Trajectory + Observations를 사용해 Policy Inference를 시작합니다.")
        # Inference 버튼을 눈에 잘 띄는 초록색으로 표시
        self.btn_infer_start.setStyleSheet(
            "QPushButton { background-color: #008800; color: white; font-weight: bold; padding: 4px 10px; }"
        )
        self.btn_infer_start.clicked.connect(self._on_inference_start)
        infer_row.addWidget(self.btn_infer_start)

        self.btn_infer_stop = QPushButton("Stop Inference")
        self.btn_infer_stop.setToolTip("실행 중인 Policy Inference를 중지합니다.")
        # Stop 버튼을 눈에 잘 띄는 빨간색으로 표시
        self.btn_infer_stop.setStyleSheet(
            "QPushButton { background-color: #880000; color: white; font-weight: bold; padding: 4px 10px; }"
        )
        self.btn_infer_stop.clicked.connect(self._on_inference_stop)
        self.btn_infer_stop.setEnabled(False)  # 초기에는 비활성화
        infer_row.addWidget(self.btn_infer_stop)

        infer_row.addStretch(1)
        control_layout.addLayout(infer_row)

        control_group.setStyleSheet(self._STYLE_BOX)
        left_layout.addWidget(control_group)

        # Observation Debug
        obs_debug_group = QGroupBox("Observation Debug")
        obs_debug_layout = QVBoxLayout(obs_debug_group)
        obs_debug_layout.setSpacing(4)

        self.lbl_total_obs_dim = QLabel("Total Obs Dim: - / -")
        self.lbl_total_obs_dim.setStyleSheet("font-size: 12px; font-weight: bold; color: #888; padding: 4px 0px;")
        obs_debug_layout.addWidget(self.lbl_total_obs_dim)

        self.obs_status_labels: Dict[str, QLabel] = {}
        self.obs_plot_checkboxes: Dict[str, QCheckBox] = {}

        for name, start_idx, end_idx, _dim, plot_enabled in OBS_ITEMS:
            row = QHBoxLayout()

            idx_label = QLabel(f"[{start_idx:3d}-{end_idx:3d}]")
            idx_label.setMinimumWidth(80)
            idx_label.setStyleSheet("font-family: monospace; font-size: 10px; color: #aaa;")
            row.addWidget(idx_label)

            name_label = QLabel(name)
            name_label.setMinimumWidth(200)
            name_label.setStyleSheet("font-size: 11px;")
            row.addWidget(name_label)

            status_label = QLabel("● Waiting")
            status_label.setStyleSheet("font-size: 10px; color: #888; font-weight: bold;")
            status_label.setMinimumWidth(100)
            self.obs_status_labels[name] = status_label
            row.addWidget(status_label)

            if plot_enabled:
                cb = QCheckBox("Plot")
                cb.setStyleSheet("font-size: 10px;")
                cb.stateChanged.connect(lambda _state, obs=name: self._on_plot_checkbox_changed(obs))
                self.obs_plot_checkboxes[name] = cb
                row.addWidget(cb)

            row.addStretch()
            obs_debug_layout.addLayout(row)

        obs_debug_group.setStyleSheet(self._STYLE_BOX)
        left_layout.addWidget(obs_debug_group)

        splitter.addWidget(left)

        # RIGHT
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setSpacing(10)

        disp_title_row = QHBoxLayout()
        disp_title = QLabel("Display")
        disp_title.setStyleSheet("font-size: 16px; font-weight: bold; padding: 4px;")
        disp_title_row.addWidget(disp_title)

        self.lbl_current_plot = QLabel("(No data selected)")
        self.lbl_current_plot.setStyleSheet("font-size: 12px; color: #888; padding: 4px; font-style: italic;")
        disp_title_row.addWidget(self.lbl_current_plot)
        disp_title_row.addStretch()
        right_layout.addLayout(disp_title_row)

        self.plot_panel = PlotPanel(buffer_size=self.cfg.buffer_size, plot_update_hz=self.cfg.plot_update_hz)
        right_layout.addWidget(self.plot_panel, 1)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 10)
        splitter.setStretchFactor(1, 16)

        # Main timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(int(1000 / max(1, self.cfg.update_hz)))

    # ----------------------------------------------------------------------------------
    # DropZone
    # ----------------------------------------------------------------------------------
    def _on_files_dropped(self, paths: List[str]) -> None:
        last_policy = None
        last_traj = None
        for p in paths:
            pl = p.lower()
            if pl.endswith((".pt", ".pth")):
                last_policy = p
            elif pl.endswith(".npz"):
                last_traj = p
        if last_policy:
            self.load_policy_model(last_policy)
        if last_traj:
            self.load_trajectory_file(last_traj)

    def unload_policy(self) -> None:
        self.loaded_policy = None
        self.policy_expected_obs_dim = None
        self.policy_expected_action_dim = None
        self.lbl_model_status.setText("Policy: None (drop .pt/.pth)")
        self.lbl_model_status.setStyleSheet(self._STYLE_STATUS_IDLE)

    def unload_trajectory(self) -> None:
        self.loaded_trajectory = None
        self.plot_panel.clear_trajectory()
        self.lbl_traj_status.setText("Trajectory: None (drop .npz)")
        self.lbl_traj_status.setStyleSheet(self._STYLE_STATUS_IDLE)

    # ----------------------------------------------------------------------------------
    # Loaders
    # ----------------------------------------------------------------------------------
    def load_policy_model(self, file_path: str) -> None:
        try:
            self.lbl_model_status.setText(f"Loading Policy: {os.path.basename(file_path)}...")
            QApplication.processEvents()

            try:
                self.loaded_policy = torch.jit.load(file_path, map_location="cpu")
                model_type = "JIT Script"
            except Exception:
                loaded_obj = torch.load(file_path, map_location="cpu")
                # torch.load()가 dict를 반환할 수 있음 (체크포인트 파일 등)
                if isinstance(loaded_obj, dict):
                    # 일반적인 키 이름들 확인
                    if "model" in loaded_obj:
                        self.loaded_policy = loaded_obj["model"]
                        model_type = "PyTorch Model (from dict['model'])"
                    elif "policy" in loaded_obj:
                        self.loaded_policy = loaded_obj["policy"]
                        model_type = "PyTorch Model (from dict['policy'])"
                    elif "actor" in loaded_obj:
                        self.loaded_policy = loaded_obj["actor"]
                        model_type = "PyTorch Model (from dict['actor'])"
                    elif "model_state_dict" in loaded_obj:
                        # model_state_dict만 있는 경우: 모델 아키텍처가 필요함
                        available_keys = list(loaded_obj.keys())
                        raise ValueError(
                            f"체크포인트 파일에는 'model_state_dict'만 포함되어 있습니다. "
                            f"모델 객체가 필요합니다.\n"
                            f"가능한 해결 방법:\n"
                            f"1. 모델 객체가 포함된 파일을 사용하세요 (예: torch.jit.save()로 저장된 .pt 파일)\n"
                            f"2. 또는 모델 아키텍처를 별도로 로드한 후 load_state_dict()를 사용하세요.\n"
                            f"체크포인트 파일의 키: {available_keys}"
                        )
                    else:
                        # dict지만 모델 키를 찾을 수 없음
                        available_keys = list(loaded_obj.keys())
                        raise ValueError(
                            f"체크포인트 파일에서 모델 객체를 찾을 수 없습니다. "
                            f"필요한 키: 'model', 'policy', 'actor' 중 하나\n"
                            f"체크포인트 파일의 키: {available_keys}\n"
                            f"참고: 'model_state_dict'만 있는 경우 모델 아키텍처가 별도로 필요합니다."
                        )
                else:
                    # dict가 아니면 직접 모델 객체로 가정
                    self.loaded_policy = loaded_obj
                    model_type = "PyTorch Model"
                
                # 최종적으로 callable한지 확인
                if not callable(self.loaded_policy):
                    raise ValueError(
                        f"로드된 정책이 호출 가능한 객체가 아닙니다. 타입: {type(self.loaded_policy)}\n"
                        f"예상: torch.nn.Module 또는 호출 가능한 객체"
                    )

            obs_dim, action_dim = self.infer_policy_io_dims(self.loaded_policy)
            self.policy_expected_obs_dim = obs_dim
            self.policy_expected_action_dim = action_dim

            extra = []
            if obs_dim is not None:
                extra.append(f"expected obs_dim={obs_dim}")
            if action_dim is not None:
                extra.append(f"expected action_dim={action_dim}")
            if obs_dim is not None and action_dim is not None and obs_dim >= action_dim:
                extra.append(f"expected proprio_dim={obs_dim - action_dim}")
            extra_text = ("\n- " + ", ".join(extra)) if extra else "\n- dim inference: Uncertain"

            self.lbl_model_status.setText(f"✅ Loaded Policy: {os.path.basename(file_path)}")
            self.lbl_model_status.setStyleSheet(self._STYLE_STATUS_OK)
            LOG.info("Policy loaded: %s", file_path)
        except Exception as e:
            self.loaded_policy = None
            self.policy_expected_obs_dim = None
            self.policy_expected_action_dim = None
            self.lbl_model_status.setText(f"❌ Policy Load Failed: {str(e)}")
            self.lbl_model_status.setStyleSheet(self._STYLE_STATUS_ERR)
            LOG.exception("Policy load failed: %s", file_path)

    @staticmethod
    def _state_dict_from_any(obj) -> Optional[Dict[str, torch.Tensor]]:
        if isinstance(obj, dict):
            if "model_state_dict" in obj and isinstance(obj["model_state_dict"], dict):
                sd = obj["model_state_dict"]
                if sd and isinstance(next(iter(sd.values())), torch.Tensor):
                    return sd
            if obj and isinstance(next(iter(obj.values())), torch.Tensor):
                return obj

        if hasattr(obj, "state_dict"):
            try:
                sd = obj.state_dict()
                if sd and isinstance(next(iter(sd.values())), torch.Tensor):
                    return sd
            except Exception:
                return None
        return None

    @staticmethod
    def _find_actor_linear_weight(sd: Dict[str, torch.Tensor], first: bool) -> Optional[torch.Tensor]:
        items: List[Tuple[int, torch.Tensor]] = []
        for k, v in sd.items():
            if not isinstance(v, torch.Tensor) or v.ndim != 2:
                continue
            if not (k.startswith("actor.") and k.endswith(".weight")):
                continue
            mid = k[len("actor.") : -len(".weight")]
            if mid.isdigit():
                items.append((int(mid), v))
        if not items:
            return None
        items.sort(key=lambda t: t[0])
        return items[0][1] if first else items[-1][1]

    def infer_policy_io_dims(self, policy_obj) -> Tuple[Optional[int], Optional[int]]:
        sd = self._state_dict_from_any(policy_obj)
        if sd is None:
            return None, None

        obs_dim: Optional[int] = None
        for k in (
            "actor_obs_normalizer._mean",
            "obs_normalizer._mean",
            "actor_obs_normalizer.mean",
            "obs_normalizer.mean",
        ):
            if k in sd and isinstance(sd[k], torch.Tensor):
                obs_dim = int(sd[k].numel())
                break
        if obs_dim is None:
            w0 = self._find_actor_linear_weight(sd, first=True)
            if w0 is not None:
                obs_dim = int(w0.shape[1])

        action_dim: Optional[int] = None
        if "std" in sd and isinstance(sd["std"], torch.Tensor):
            action_dim = int(sd["std"].numel())
        else:
            w_last = self._find_actor_linear_weight(sd, first=False)
            if w_last is not None:
                action_dim = int(w_last.shape[0])

        return obs_dim, action_dim

    def load_trajectory_file(self, file_path: str) -> None:
        try:
            self.lbl_traj_status.setText(f"Loading Trajectory: {os.path.basename(file_path)}...")
            QApplication.processEvents()

            data = np.load(file_path, allow_pickle=True)
            if "positions" not in data:
                raise ValueError("NPZ file does not contain 'positions' array.")

            positions = np.asarray(data["positions"])
            if positions.ndim != 2:
                raise ValueError(f"'positions' must be 2D (T, D), got shape {positions.shape}.")

            num_frames, num_joints = positions.shape

            joint_names = None
            if "joint_names" in data:
                jn_raw = data["joint_names"]
                joint_names = jn_raw.tolist() if isinstance(jn_raw, np.ndarray) else list(jn_raw)

            action_indices = None
            action_ok = False
            order_ok = False
            missing: List[str] = []

            if joint_names is not None:
                name_to_idx = {n: i for i, n in enumerate(joint_names)}
                idxs = []
                for name in ALLEX_ACTION_JOINT_NAMES:
                    if name not in name_to_idx:
                        missing.append(name)
                    else:
                        idxs.append(name_to_idx[name])
                if not missing:
                    action_ok = True
                    action_indices = idxs
                    subset_names = [joint_names[i] for i in action_indices]
                    order_ok = subset_names == ALLEX_ACTION_JOINT_NAMES

            # 전체 actions trajectory 추출 (T, 18)
            actions_trajectory = None
            if positions is not None:
                if action_indices is not None:
                    actions_trajectory = positions[:, action_indices]  # (T, 18)
                else:
                    actions_trajectory = positions[:, : self.cfg.num_joints]  # (T, 18)
            
            self.loaded_trajectory = {
                "file_path": file_path,
                "num_frames": int(num_frames),
                "num_joints": int(num_joints),
                "joint_names": joint_names,
                "action_indices": action_indices,
                "positions": positions,
                "actions_trajectory": actions_trajectory,  # (T, 18) 전체 trajectory
            }

            self.plot_panel.set_trajectory_data(positions)

            status_lines = [
                f"✅ Trajectory Loaded: {os.path.basename(file_path)}",
            ]

            ok = (joint_names is not None) and action_ok and order_ok
            self.lbl_traj_status.setText("\n".join(status_lines))
            self.lbl_traj_status.setStyleSheet(self._STYLE_STATUS_OK if ok else self._STYLE_STATUS_ERR)

            LOG.info("Trajectory loaded: %s", file_path)
        except Exception as e:
            self.loaded_trajectory = None
            self.lbl_traj_status.setText(f"❌ Trajectory Load Failed: {str(e)}")
            self.lbl_traj_status.setStyleSheet(self._STYLE_STATUS_ERR)
            LOG.exception("Trajectory load failed: %s", file_path)

    # ----------------------------------------------------------------------------------
    # Control Section
    # ----------------------------------------------------------------------------------
    def get_hammer_position(self) -> Tuple[float, float, float]:
        default_x, default_y, default_z = 0.5408, -0.1004, -0.4279

        def _to_float(text: str, default: float) -> float:
            try:
                return float(text) if text else default
            except ValueError:
                return default

        x = _to_float(self.hammer_x_input.text(), default_x)
        y = _to_float(self.hammer_y_input.text(), default_y)
        z = _to_float(self.hammer_z_input.text(), default_z)
        return x, y, z

    def _on_apply_hammer(self) -> None:
        x, y, z = self.get_hammer_position()
        LOG.info("Hammer Position Applied: x=%.4f y=%.4f z=%.4f", x, y, z)

        if ROS2_AVAILABLE and self.ros_running and self.ros_node is not None:
            self.ros_node.publish_hammer_pos(x, y, z)
        else:
            LOG.warning("ROS2 not connected. Cannot publish hammer_pos.")

    def _on_reset_hammer(self) -> None:
        self.hammer_x_input.setText("0.5408")
        self.hammer_y_input.setText("-0.1004")
        self.hammer_z_input.setText("-0.4279")
        LOG.info("Hammer Position Reset to default values")

        if ROS2_AVAILABLE and self.ros_running:
            self.ros_obs_data.pop("hammer_pos", None)

    def get_target_pose(self) -> Tuple[float, float, float, float, float, float, float]:
        """Target right hand pose (x, y, z, qw, qx, qy, qz)를 반환합니다."""
        # 기본 자세: [0.55, -0.2, 0.4, 0.5, -0.5, -0.5, 0.5]
        default_x, default_y, default_z = 0.55, -0.2, 0.4
        default_qw, default_qx, default_qy, default_qz = 0.5, -0.5, -0.5, 0.5

        def _to_float(text: str, default: float) -> float:
            try:
                return float(text) if text else default
            except ValueError:
                return default

        x = _to_float(self.target_x_input.text(), default_x)
        y = _to_float(self.target_y_input.text(), default_y)
        z = _to_float(self.target_z_input.text(), default_z)
        qw = _to_float(self.target_qw_input.text(), default_qw)
        qx = _to_float(self.target_qx_input.text(), default_qx)
        qy = _to_float(self.target_qy_input.text(), default_qy)
        qz = _to_float(self.target_qz_input.text(), default_qz)
        return x, y, z, qw, qx, qy, qz

    def _on_apply_target_pose(self) -> None:
        x, y, z, qw, qx, qy, qz = self.get_target_pose()
        LOG.info("Target Right Hand Pose Applied: pos=[%.4f, %.4f, %.4f], quat=[%.4f, %.4f, %.4f, %.4f]", 
                 x, y, z, qw, qx, qy, qz)

        if ROS2_AVAILABLE and self.ros_running and self.ros_node is not None:
            self.ros_node.publish_target_pose(x, y, z, qw, qx, qy, qz)
        else:
            LOG.warning("ROS2 not connected. Cannot publish target_right_hand_pose.")

    def _on_reset_target_pose(self) -> None:
        # 기본 자세로 리셋: [0.55, -0.2, 0.4, 0.5, -0.5, -0.5, 0.5]
        self.target_x_input.setText("0.55")
        self.target_y_input.setText("-0.2")
        self.target_z_input.setText("0.4")
        self.target_qw_input.setText("0.5")
        self.target_qx_input.setText("-0.5")
        self.target_qy_input.setText("-0.5")
        self.target_qz_input.setText("0.5")
        LOG.info("Target Right Hand Pose Reset to default values")

        if ROS2_AVAILABLE and self.ros_running:
            self.ros_obs_data.pop("target_right_hand_pose", None)

    # ----------------------------------------------------------------------------------
    # Inference Control (Residual RL 테스트용)
    # ----------------------------------------------------------------------------------
    def _on_inference_start(self) -> None:
        """Inference 시작 버튼 콜백.

        - Trajectory 재생 속도(infer_speed_input)를 읽어서 self._traj_speed에 반영
        - 재생 시간/상태 리셋
        - Policy obs dim과 구성 Obs dim이 맞는지 1차 체크
        """
        # 재생 속도 파싱
        try:
            speed_text = (self.infer_speed_input.text() or "1.0").strip()
            speed = float(speed_text)
            if speed <= 0.0:
                raise ValueError("speed must be > 0")
        except Exception:
            speed = 1.0
            self.infer_speed_input.setText("1.0")

        self._traj_speed = speed

        # Residual Scale 파싱
        try:
            rs_text = (self.residual_scale_input.text() or f"{self._residual_scale:.2f}").strip()
            rs = float(rs_text)
            # 너무 큰 값은 위험하니 간단히 클램프 (예: 0.0 ~ 1.0)
            if rs < 0.0:
                rs = 0.0
            elif rs > 1.0:
                rs = 1.0
        except Exception:
            rs = self._residual_scale
            # 파싱 실패 시 현재 값을 입력 필드에 다시 써줌
            self.residual_scale_input.setText(f"{rs:.2f}")

        self._residual_scale = rs
        self._traj_playback_time = 0.0

        # Obs dim 검증 (정적 구성 기준: EXPECTED_TOTAL_OBS_DIM_FALLBACK)
        obs_cfg_dim = EXPECTED_TOTAL_OBS_DIM_FALLBACK
        if self.policy_expected_obs_dim is None:
            LOG.warning("Policy expected obs_dim is unknown; cannot verify against OBS_ITEMS (cfg=%d).", obs_cfg_dim)
        else:
            if self.policy_expected_obs_dim != obs_cfg_dim:
                LOG.warning(
                    "Policy obs_dim (%d) != configured OBS dim (%d). Inference obs 구성이 다를 수 있습니다.",
                    self.policy_expected_obs_dim,
                    obs_cfg_dim,
                )

        self._inference_running = True
        # 버튼 상태 업데이트: Start 비활성화, Stop 활성화
        self.btn_infer_start.setEnabled(False)
        self.btn_infer_stop.setEnabled(True)
        LOG.info("Inference started with trajectory speed=%.3f", self._traj_speed)

    def _on_inference_stop(self) -> None:
        """Inference 중지 버튼 콜백.
        
        - Inference 실행 상태를 False로 설정
        - 버튼 상태 업데이트
        """
        self._inference_running = False
        # 버튼 상태 업데이트: Start 활성화, Stop 비활성화
        self.btn_infer_start.setEnabled(True)
        self.btn_infer_stop.setEnabled(False)
        LOG.info("Inference stopped")

    def _build_policy_obs_vector(self, data: ObsDict, key_map: Dict[str, str], streaming_keys: set) -> Optional[np.ndarray]:
        """OBS_ITEMS 순서대로 observation 벡터를 패킹합니다.
        
        Args:
            data: observation 데이터 딕셔너리
            key_map: obs_name -> data key 매핑
            streaming_keys: ROS2 타임아웃 체크가 필요한 스트리밍 키 집합
            
        Returns:
            (obs_dim,) shape의 numpy 배열. 패킹 실패 시 None.
        """
        obs_parts = []
        
        for name, start_idx, end_idx, expected_dim, _ in OBS_ITEMS:
            key = key_map.get(name)
            
            # actions: trajectory 로드 여부로 판단
            if name == "actions":
                if self.loaded_trajectory is not None and key and key in data:
                    arr = np.asarray(data[key])
                    if arr.size == expected_dim:
                        obs_parts.append(arr.reshape(-1))
                    else:
                        LOG.warning("actions dim mismatch: expected=%d, got=%d", expected_dim, arr.size)
                        return None
                else:
                    LOG.warning("actions not available for obs packing")
                    return None
                continue
            
            # reference_joint_pos_error: joint_pos와 actions가 모두 Active일 때만
            if name == "reference_joint_pos_error":
                actions_ok = self.loaded_trajectory is not None
                joint_pos_key = key_map.get("joint_pos")
                joint_pos_ok = False
                if joint_pos_key and joint_pos_key in data:
                    arr = np.asarray(data[joint_pos_key])
                    if arr.size > 0:
                        if (
                            "joint_pos" in streaming_keys
                            and self.ros_running
                            and self.ros_node is not None
                            and hasattr(self.ros_node, "get_last_update_age")
                        ):
                            timeout_sec = 1.0
                            age = self.ros_node.get_last_update_age(joint_pos_key)
                            if age is None or age <= timeout_sec:
                                joint_pos_ok = True
                        else:
                            joint_pos_ok = True
                
                if actions_ok and joint_pos_ok and key and key in data:
                    arr = np.asarray(data[key])
                    if arr.size == expected_dim:
                        obs_parts.append(arr.reshape(-1))
                    else:
                        LOG.warning("reference_joint_pos_error dim mismatch: expected=%d, got=%d", expected_dim, arr.size)
                        return None
                else:
                    LOG.warning("reference_joint_pos_error not available (actions_ok=%s, joint_pos_ok=%s)", actions_ok, joint_pos_ok)
                    return None
                continue
            
            # 나머지 observation들
            if not (key and key in data):
                LOG.warning("Observation '%s' not found in data", name)
                return None
            
            try:
                arr = np.asarray(data[key])
                if arr.size <= 0:
                    LOG.warning("Observation '%s' is empty", name)
                    return None
                
                # 스트리밍 토픽은 ROS2 타임아웃 체크
                if (
                    name in streaming_keys
                    and self.ros_running
                    and self.ros_node is not None
                    and hasattr(self.ros_node, "get_last_update_age")
                ):
                    timeout_sec = 1.0
                    age = self.ros_node.get_last_update_age(key)
                    if age is not None and age > timeout_sec:
                        LOG.warning("Observation '%s' timeout (age=%.3f > %.3f)", name, age, timeout_sec)
                        return None
                
                # 차원 검증
                if arr.size != expected_dim:
                    LOG.warning("Observation '%s' dim mismatch: expected=%d, got=%d", name, expected_dim, arr.size)
                    return None
                
                obs_parts.append(arr.reshape(-1))
            except Exception as e:
                LOG.exception("Failed to process observation '%s': %s", name, e)
                return None
        
        if not obs_parts:
            LOG.warning("No observations to pack")
            return None
        
        obs_vec = np.concatenate(obs_parts, axis=0).astype(np.float32)
        expected_total = EXPECTED_TOTAL_OBS_DIM_FALLBACK
        
        if obs_vec.size != expected_total:
            LOG.warning("Total obs dim mismatch: expected=%d, got=%d", expected_total, obs_vec.size)
            return None
        
        return obs_vec

    # ----------------------------------------------------------------------------------
    # Plot checkbox (single selection)
    # ----------------------------------------------------------------------------------
    def _on_plot_checkbox_changed(self, obs_name: str) -> None:
        cb = self.obs_plot_checkboxes.get(obs_name)
        if cb is None:
            return

        is_checked = cb.isChecked()

        if is_checked:
            # 단일 선택: 다른 체크박스는 모두 해제
            for name, other in self.obs_plot_checkboxes.items():
                if name == obs_name:
                    continue
                if other.isChecked():
                    with QSignalBlocker(other):
                        other.setChecked(False)

            # 모든 obs는 동일하게 실시간 버퍼 기반 Plot
            self.plot_panel.set_selected_observation(obs_name)
            self.lbl_current_plot.setText(f"Plotting: {obs_name}")
            self.lbl_current_plot.setStyleSheet("font-size: 12px; color: #00ff00; padding: 4px; font-weight: bold;")
            LOG.debug("Plot selected: %s", obs_name)
        else:
            # 현재 선택이 해제된 것과 동일할 때만 Plot 해제
            if self.plot_panel.selected_key == obs_name:
                self.plot_panel.set_selected_observation(None)
                self.lbl_current_plot.setText("(No data selected)")
                self.lbl_current_plot.setStyleSheet("font-size: 12px; color: #888; padding: 4px; font-style: italic;")
                LOG.debug("Plot deselected: %s", obs_name)

    # ----------------------------------------------------------------------------------
    # ROS2 Connection
    # ----------------------------------------------------------------------------------
    def _on_ros_connect(self) -> None:
        if not ROS2_AVAILABLE:
            self.lbl_ros_status.setText("● Error: ROS2 not available")
            self.lbl_ros_status.setStyleSheet("font-size: 11px; color: #ff0000; font-weight: bold;")
            return

        if self.ros_running:
            self._ros_disconnect()
            return

        try:
            domain_id = int(self.ros_domain_input.text() or "0")
        except ValueError:
            self.lbl_ros_status.setText("● Error: Invalid Domain ID")
            self.lbl_ros_status.setStyleSheet("font-size: 11px; color: #ff0000; font-weight: bold;")
            return

        self._ros_connect(domain_id)

    def _ros_connect(self, domain_id: int) -> None:
        if self.ros_running:
            return

        try:
            # Domain ID는 init 전에 설정 (원 코드의 실제 동작 문제 가능성 개선)
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)

            if not rclpy.ok():
                rclpy.init()

            self.ros_node = ROS2ObservationSubscriber(self.ros_obs_data)
            self.ros_running = True

            self.ros_thread = threading.Thread(target=self._ros_spin_thread, daemon=True)
            self.ros_thread.start()

            self.btn_ros_connect.setText("Disconnect")
            self.lbl_ros_status.setText("● Connected")
            self.lbl_ros_status.setStyleSheet("font-size: 11px; color: #00ff00; font-weight: bold;")
            self.ros_domain_input.setEnabled(False)

            self.obs_provider = lambda: self.ros_obs_data if self.ros_running else {}

            LOG.info("ROS2 Connected (Domain ID=%d)", domain_id)
        except Exception:
            self.ros_running = False
            self.lbl_ros_status.setText("● Error: Connection failed")
            self.lbl_ros_status.setStyleSheet("font-size: 11px; color: #ff0000; font-weight: bold;")
            LOG.exception("ROS2 connection failed")

    def _ros_disconnect(self) -> None:
        if not self.ros_running:
            return

        self.ros_running = False
        self.btn_ros_connect.setText("Connect")
        self.lbl_ros_status.setText("● Disconnected")
        self.lbl_ros_status.setStyleSheet("font-size: 11px; color: #888; font-weight: bold;")
        self.ros_domain_input.setEnabled(True)
        self.ros_obs_data.clear()
        self.obs_provider = lambda: {}

        LOG.info("ROS2 Disconnected")

    def _ros_spin_thread(self) -> None:
        try:
            executor = SingleThreadedExecutor()
            executor.add_node(self.ros_node)

            while self.ros_running and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        except Exception:
            LOG.exception("ROS2 spin thread error")
        finally:
            try:
                if self.ros_node is not None:
                    self.ros_node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()

    # ----------------------------------------------------------------------------------
    # Update loop
    # ----------------------------------------------------------------------------------
    def update_loop(self) -> None:
        try:
            data = self.obs_provider() or {}
        except Exception:
            return

        # alias 허용(기존 기능 유지)
        def pick(*names: str) -> Optional[str]:
            for n in names:
                if n in data:
                    return n
            return None

        key_map = {
            "actions": pick("actions"),
            "hammer_pos": pick("hammer_pos"),
            "joint_pos": pick("joint_pos"),
            "reference_joint_pos_error": pick("reference_joint_pos_error", "ref_error"),
            "right_hand_joint_torque": pick("right_hand_joint_torque", "torque"),
            "right_hand_base_pos": pick("right_hand_base_pos", "hand_base_pos"),
            "target_right_hand_pose": pick("target_right_hand_pose", "target_hand_pose"),
        }

        # 스트리밍 관측 토픽 (ROS2 타임아웃 체크 대상)
        streaming_keys = {
            "joint_pos",
            "right_hand_joint_torque",
            "right_hand_base_pos",
        }

        # ----------------------------------------------------------------------------------
        # actions (18차원) 주입: Inference Start 전에는 제로 패딩, Start 후에는 ref_trajectory + policy_output
        # reference_joint_pos_error = joint_pos - actions 계산하여 data에 주입
        # ----------------------------------------------------------------------------------
        if self.loaded_trajectory is not None:
            if not self._inference_running:
                # Inference Start 전: actions를 제로 패딩으로 주입
                zero_actions = np.zeros(self.cfg.num_joints, dtype=np.float32)
                data["actions"] = zero_actions
                key_map["actions"] = "actions"
            else:
                # Inference Start 후: Trajectory 재생 + Policy forward (추후 구현) + actions 계산
                try:
                    actions_traj = self.loaded_trajectory.get("actions_trajectory")
                    if actions_traj is not None:
                        actions_traj = np.asarray(actions_traj)
                    if (
                        actions_traj is not None
                        and actions_traj.ndim == 2
                        and actions_traj.shape[1] == self.cfg.num_joints
                    ):
                        # Trajectory 재생 시간 업데이트 (update_hz를 기반으로 추정 dt)
                        dt = 1.0 / float(max(1, self.cfg.update_hz))
                        self._traj_playback_time += dt * float(self._traj_speed)

                        # base_traj_hz(예: 50Hz)를 기준으로 trajectory 인덱스 계산
                        num_frames = actions_traj.shape[0]
                        if num_frames > 0 and self._base_traj_hz > 0.0:
                            phase = (self._traj_playback_time * self._base_traj_hz) % float(num_frames)
                            idx = int(phase)
                            ref_action = actions_traj[idx]  # (num_joints,)

                            # Step 1: 현재 프레임의 joint_pos와 ref_action으로 초기 reference_joint_pos_error 계산
                            # (Policy forward 전에 Obs 벡터를 만들기 위해 필요)
                            joint_pos_key = key_map.get("joint_pos")
                            if joint_pos_key and joint_pos_key in data:
                                cur_joint = np.asarray(data[joint_pos_key])
                                cur_joint_flat = cur_joint.reshape(-1)
                                ref_action_flat = ref_action.reshape(-1)

                                if cur_joint_flat.shape[0] == ref_action_flat.shape[0]:
                                    # 초기 reference_joint_pos_error = joint_pos - ref_action (residual=0 가정)
                                    initial_ref_error = cur_joint_flat - ref_action_flat
                                    data["reference_joint_pos_error"] = initial_ref_error.astype(np.float32)
                                    key_map["reference_joint_pos_error"] = "reference_joint_pos_error"
                                    
                                    # Step 2: Obs 벡터 패킹
                                    obs_vec = self._build_policy_obs_vector(data, key_map, streaming_keys)
                                    
                                    if obs_vec is not None:
                                        # Step 3: Policy forward
                                        residual_output = None
                                        if self.loaded_policy is not None:
                                            try:
                                                # torch.Tensor로 변환 (batch dimension 추가)
                                                obs_tensor = torch.from_numpy(obs_vec).float().unsqueeze(0)  # (1, obs_dim)
                                                
                                                # Policy forward (no_grad로 gradient 계산 비활성화)
                                                with torch.no_grad():
                                                    policy_output = self.loaded_policy(obs_tensor)  # (1, action_dim) 또는 다른 shape
                                                
                                                # numpy로 변환
                                                if isinstance(policy_output, torch.Tensor):
                                                    residual_output = policy_output.squeeze(0).cpu().numpy()  # (action_dim,)
                                                else:
                                                    # Policy가 dict나 다른 형태를 반환할 수 있음
                                                    LOG.warning("Policy output is not a tensor: %s", type(policy_output))
                                                    residual_output = np.zeros(self.cfg.num_joints, dtype=np.float32)
                                                
                                                # 차원 검증
                                                if residual_output.size != self.cfg.num_joints:
                                                    LOG.warning(
                                                        "Policy output dim mismatch: expected=%d, got=%d. Using zeros.",
                                                        self.cfg.num_joints,
                                                        residual_output.size,
                                                    )
                                                    residual_output = np.zeros(self.cfg.num_joints, dtype=np.float32)
                                                
                                                residual_output = residual_output.astype(np.float32)
                                                
                                            except Exception as e:
                                                LOG.exception("Policy forward failed: %s", e)
                                                residual_output = np.zeros(self.cfg.num_joints, dtype=np.float32)
                                        else:
                                            # Policy가 로드되지 않았으면 residual = 0
                                            residual_output = np.zeros(self.cfg.num_joints, dtype=np.float32)

                                        # Step 4: actions = ref_action + residual * residual_scale 계산
                                        actions = ref_action + residual_output * self._residual_scale  # (num_joints,)

                                        # actions를 data에 주입
                                        data["actions"] = actions.astype(np.float32)
                                        key_map["actions"] = "actions"
                                        
                                        # Step 4.5: Policy action을 ROS2로 publish
                                        if ROS2_AVAILABLE and self.ros_running and self.ros_node is not None:
                                            self.ros_node.publish_policy_action(actions)

                                        # Step 5: 최종 reference_joint_pos_error = joint_pos - actions 계산
                                        if joint_pos_key and joint_pos_key in data:
                                            cur_joint = np.asarray(data[joint_pos_key])
                                            cur_joint_flat = cur_joint.reshape(-1)
                                            actions_flat = actions.reshape(-1)

                                            if cur_joint_flat.shape[0] == actions_flat.shape[0]:
                                                ref_error = cur_joint_flat - actions_flat
                                                # numpy float32로 맞추고 data에 주입
                                                data["reference_joint_pos_error"] = ref_error.astype(np.float32)
                                                key_map["reference_joint_pos_error"] = "reference_joint_pos_error"
                                            else:
                                                LOG.warning(
                                                    "reference_joint_pos_error calc skipped: joint_pos dim=%d, actions dim=%d",
                                                    cur_joint_flat.shape[0],
                                                    actions_flat.shape[0],
                                                )
                                    else:
                                        # Obs 벡터 패킹 실패 시, ref_action만 사용
                                        LOG.warning("Failed to build obs vector, using ref_action only")
                                        residual_output = np.zeros(self.cfg.num_joints, dtype=np.float32)
                                        actions = ref_action + residual_output * self._residual_scale
                                        data["actions"] = actions.astype(np.float32)
                                        key_map["actions"] = "actions"
                                        
                                        # Policy action을 ROS2로 publish
                                        if ROS2_AVAILABLE and self.ros_running and self.ros_node is not None:
                                            self.ros_node.publish_policy_action(actions)
                                else:
                                    LOG.warning(
                                        "reference_joint_pos_error calc skipped: joint_pos dim=%d, ref_action dim=%d",
                                        cur_joint_flat.shape[0],
                                        ref_action_flat.shape[0],
                                    )
                                    # Policy forward를 건너뛰고 ref_action만 사용
                                    residual_output = np.zeros(self.cfg.num_joints, dtype=np.float32)
                                    actions = ref_action + residual_output * self._residual_scale
                                    data["actions"] = actions.astype(np.float32)
                                    key_map["actions"] = "actions"
                                    
                                    # Policy action을 ROS2로 publish
                                    if ROS2_AVAILABLE and self.ros_running and self.ros_node is not None:
                                        self.ros_node.publish_policy_action(actions)
                except Exception as e:
                    LOG.exception("Failed to compute actions and reference_joint_pos_error: %s", e)

        # Total Obs Dim (UI 표기 로직은 기존 그대로 유지: actions는 trajectory 로드 여부로 Active 처리)
        current_active_dim = 0
        actions_active = self.loaded_trajectory is not None
        if actions_active:
            current_active_dim += self.cfg.num_joints  # actions dim

        # joint_pos Active 여부 확인
        joint_pos_active = False
        joint_pos_key = key_map.get("joint_pos")
        if joint_pos_key and joint_pos_key in data:
            try:
                arr = np.asarray(data[joint_pos_key])
                if arr.size > 0:
                    if (
                        "joint_pos" in streaming_keys
                        and self.ros_running
                        and self.ros_node is not None
                        and hasattr(self.ros_node, "get_last_update_age")
                    ):
                        timeout_sec = 1.0
                        age = self.ros_node.get_last_update_age(joint_pos_key)
                        if age is None or age <= timeout_sec:
                            joint_pos_active = True
                            current_active_dim += int(arr.size)
                    else:
                        joint_pos_active = True
                        current_active_dim += int(arr.size)
            except Exception:
                pass

        # reference_joint_pos_error는 joint_pos와 actions가 모두 Active일 때만 Active
        ref_error_active = actions_active and joint_pos_active
        if ref_error_active:
            current_active_dim += self.cfg.num_joints  # reference_joint_pos_error dim (18개)

        # current_active_dim 에는 실제로 "Active" 인 관측만 포함
        for name, key in key_map.items():
            if name in ("actions", "reference_joint_pos_error", "joint_pos"):
                continue
            if not (key and key in data):
                continue
            try:
                arr = np.asarray(data[key])
                if arr.size <= 0:
                    continue

                # 스트리밍 토픽은 ROS2 타임아웃 기준을 적용
                if (
                    name in streaming_keys
                    and self.ros_running
                    and self.ros_node is not None
                    and hasattr(self.ros_node, "get_last_update_age")
                ):
                    timeout_sec = 1.0
                    age = self.ros_node.get_last_update_age(key)
                    # age 가 None 이거나 timeout 이하일 때만 Active 로 카운트
                    if age is None or age <= timeout_sec:
                        current_active_dim += int(arr.size)
                else:
                    # 비-스트리밍 토픽은 값만 있으면 Active
                    current_active_dim += int(arr.size)
            except Exception:
                pass

        total_obs_dim = self.policy_expected_obs_dim
        if total_obs_dim is not None:
            self.lbl_total_obs_dim.setText(f"Total Obs Dim: {current_active_dim} / {total_obs_dim}")
            if current_active_dim == total_obs_dim:
                self.lbl_total_obs_dim.setStyleSheet("font-size: 12px; font-weight: bold; color: #00ff00; padding: 4px 0px;")
            else:
                self.lbl_total_obs_dim.setStyleSheet("font-size: 12px; font-weight: bold; color: #ffff00; padding: 4px 0px;")
        else:
            self.lbl_total_obs_dim.setText("Total Obs Dim: - / -")
            self.lbl_total_obs_dim.setStyleSheet("font-size: 12px; font-weight: bold; color: #888; padding: 4px 0px;")

        # Status labels
        # joint_pos Active 여부 미리 계산 (reference_joint_pos_error에서 사용)
        joint_pos_status_active = False
        joint_pos_key = key_map.get("joint_pos")
        if joint_pos_key and joint_pos_key in data:
            try:
                arr = np.asarray(data[joint_pos_key])
                if arr.size > 0:
                    if (
                        "joint_pos" in streaming_keys
                        and self.ros_running
                        and self.ros_node is not None
                        and hasattr(self.ros_node, "get_last_update_age")
                    ):
                        timeout_sec = 1.0
                        age = self.ros_node.get_last_update_age(joint_pos_key)
                        if age is None or age <= timeout_sec:
                            joint_pos_status_active = True
                    else:
                        joint_pos_status_active = True
            except Exception:
                pass

        actions_status_active = self.loaded_trajectory is not None

        for obs_name, label in self.obs_status_labels.items():
            if obs_name == "actions":
                if actions_status_active:
                    label.setText("● Active")
                    label.setStyleSheet("font-size: 10px; color: #00ff00; font-weight: bold;")
                else:
                    label.setText("● Missing")
                    label.setStyleSheet("font-size: 10px; color: #ffff00; font-weight: bold;")
                continue

            # reference_joint_pos_error는 joint_pos와 actions가 모두 Active일 때만 Active
            if obs_name == "reference_joint_pos_error":
                if actions_status_active and joint_pos_status_active:
                    label.setText("● Active")
                    label.setStyleSheet("font-size: 10px; color: #00ff00; font-weight: bold;")
                else:
                    label.setText("● Missing")
                    label.setStyleSheet("font-size: 10px; color: #ffff00; font-weight: bold;")
                continue

            key = key_map.get(obs_name)
            if key and key in data:
                try:
                    arr = np.asarray(data[key])
                    if arr.size > 0:
                        # 1) 스트리밍 관측 토픽: 타임아웃 적용
                        if (
                            obs_name in streaming_keys
                            and self.ros_running
                            and self.ros_node is not None
                            and hasattr(self.ros_node, "get_last_update_age")
                        ):
                            timeout_sec = 1.0
                            age = self.ros_node.get_last_update_age(key)
                            if age is not None and age > timeout_sec:
                                # 오래 안 들어왔으면 Missing
                                label.setText("● Missing")
                                label.setStyleSheet("font-size: 10px; color: #ffff00; font-weight: bold;")
                            else:
                                # 최근에 들어온 스트리밍 데이터
                                label.setText("● Active")
                                label.setStyleSheet("font-size: 10px; color: #00ff00; font-weight: bold;")
                        else:
                            # 2) Hammer Position / Target Right Hand Pose 등:
                            #    값만 있으면 계속 Active 유지 (타임아웃 X)
                            label.setText("● Active")
                            label.setStyleSheet("font-size: 10px; color: #00ff00; font-weight: bold;")
                    else:
                        label.setText("● Missing")
                        label.setStyleSheet("font-size: 10px; color: #ffff00; font-weight: bold;")
                except Exception:
                    label.setText("● Error")
                    label.setStyleSheet("font-size: 10px; color: #ff0000; font-weight: bold;")
            else:
                label.setText("● Missing")
                label.setStyleSheet("font-size: 10px; color: #ffff00; font-weight: bold;")

        # 모든 observation들( actions 포함 )을 실시간 버퍼로 처리
        self.plot_panel.ingest_obs(data)


def main() -> None:
    app = QApplication(sys.argv)
    window = Sim2RealDebugger()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
