import os
import threading
import time
import logging
from typing import Dict, List, Optional

import numpy as np
from PySide6.QtCore import QSignalBlocker, Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
    QMainWindow,
    QSizePolicy,
)

from .config import (
    DebuggerConfig,
    EXPECTED_TOTAL_OBS_DIM,
    OBS_SPECS,
    OBS_TIMEOUT_SEC,
    ObsDict,
    ObsProvider,
    STREAMING_NAMES,
    STYLE_GROUP_BOX,
    STYLE_STATUS_ERR,
    STYLE_STATUS_IDLE,
    STYLE_STATUS_OK,
    WINDOW_HEIGHT,
    WINDOW_TITLE,
    WINDOW_WIDTH,
)
from .inference import InferenceEngine, TrajectoryLoader
from .observation import ObsNormalizer, ObservationStore
from .policy import PolicyWrapper
from .ros2_io import ROS2_AVAILABLE, ROS2ObservationSubscriber, spin_while
from .widgets import DropZone, PlotPanel

LOG = logging.getLogger(__name__)


class Sim2RealDebugger(QMainWindow):
    def __init__(self, obs_provider: Optional[ObsProvider] = None, cfg: Optional[DebuggerConfig] = None):
        super().__init__()
        self.cfg = cfg or DebuggerConfig()

        self.setWindowTitle(WINDOW_TITLE)
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)
        self.setStyleSheet("background-color: #2b2b2b; color: #ffffff;")

        # core components
        self.store = ObservationStore()
        self.normalizer = ObsNormalizer()
        self.policy = PolicyWrapper()
        self.infer = InferenceEngine(self.cfg, self.policy)

        # ROS2 state
        self.ros_node: Optional["ROS2ObservationSubscriber"] = None
        self.ros_thread: Optional[threading.Thread] = None
        self.ros_running = False

        # external obs provider (optional)
        self.external_provider = obs_provider
        self._obs_provider: ObsProvider = self._make_obs_provider()

        # UI references
        self.obs_status_labels: Dict[str, QLabel] = {}
        self.obs_plot_checkboxes: Dict[str, QCheckBox] = {}
        self.obs_rate_labels: Dict[str, QLabel] = {}
        self._obs_rate_hz: Dict[str, float] = {}
        self._obs_last_seen: Dict[str, float] = {}
        self._obs_last_ros_ts: Dict[str, float] = {}
        self._obs_rate_lock = threading.Lock()

        # status
        self.loaded_trajectory: Optional[dict] = None

        # build UI
        self._build_ui()

        # wire inference callbacks
        self.infer.set_freshness_callback(self._is_stream_fresh)
        self.infer.set_publish_callback(self._publish_policy_action)
        self.infer.set_last_action_publish_callback(self._publish_last_actions)

        # control loop in background thread (액션 계산/퍼블리시: 50Hz 기본)
        self._control_running = True
        self._last_data: Optional[ObsDict] = None
        self._last_data_lock = threading.Lock()
        self.control_thread = threading.Thread(target=self._control_loop_thread, daemon=True)
        self.control_thread.start()

        # UI timer (상태/플롯 업데이트: 30Hz 기본)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.update_loop)
        self.ui_timer.start(int(1000 / max(1, self.cfg.update_hz)))

    # ----------------------------------------------------------------------------------
    # UI builder helpers
    # ----------------------------------------------------------------------------------
    @staticmethod
    def _float_edit(text: str, max_width: Optional[int] = None) -> QLineEdit:
        le = QLineEdit(text)
        if max_width is not None:
            le.setMaximumWidth(max_width)
        return le

    @staticmethod
    def _to_float(text: str, default: float) -> float:
        try:
            return float(text) if text else default
        except ValueError:
            return default

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        root.addWidget(splitter, 1)

        # left
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setSpacing(10)

        self.drop_zone = DropZone()
        self.drop_zone.setMinimumHeight(90)
        self.drop_zone.files_dropped.connect(self._on_files_dropped)
        left_layout.addWidget(self.drop_zone)

        left_layout.addWidget(self._build_ros_group())
        left_layout.addLayout(self._build_model_traj_status_row())
        left_layout.addLayout(self._build_unload_buttons())
        left_layout.addWidget(self._build_control_group())
        left_layout.addWidget(self._build_obs_debug_group())

        splitter.addWidget(left)

        # right
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

    def _build_ros_group(self) -> QGroupBox:
        ros_group = QGroupBox("ROS2 Connection")
        ros_group.setMaximumHeight(80)  
        ros_layout = QHBoxLayout(ros_group)
        ros_layout.setSpacing(6)
        ros_layout.setContentsMargins(8, 4, 8, 4)

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
        ros_group.setStyleSheet(STYLE_GROUP_BOX)
        return ros_group

    def _build_model_traj_status_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(8)

        self.lbl_model_status = QLabel("Policy: None (drop .pt/.pth)")
        self.lbl_model_status.setStyleSheet(STYLE_STATUS_IDLE)
        self.lbl_model_status.setAlignment(Qt.AlignCenter)
        self.lbl_model_status.setMaximumHeight(40)
        self.lbl_model_status.setWordWrap(True)
        row.addWidget(self.lbl_model_status, 1)

        self.lbl_traj_status = QLabel("Trajectory: None (drop .npz)")
        self.lbl_traj_status.setStyleSheet(STYLE_STATUS_IDLE)
        self.lbl_traj_status.setAlignment(Qt.AlignCenter)
        self.lbl_traj_status.setMaximumHeight(40)
        self.lbl_traj_status.setWordWrap(True)
        row.addWidget(self.lbl_traj_status, 1)

        return row

    def _build_unload_buttons(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self.btn_unload_policy = QPushButton("Unload Policy")
        self.btn_unload_traj = QPushButton("Unload Trajectory")
        self.btn_unload_policy.clicked.connect(self.unload_policy)
        self.btn_unload_traj.clicked.connect(self.unload_trajectory)
        row.addWidget(self.btn_unload_policy)
        row.addWidget(self.btn_unload_traj)
        return row

    def _build_control_group(self) -> QGroupBox:
        g = QGroupBox("Control Section")
        g.setStyleSheet(STYLE_GROUP_BOX)
        layout = QVBoxLayout(g)
        # Control Section은 세로로 최소 크기만 차지하도록 마진/간격을 줄이고,
        # Observation Debug가 남는 공간을 더 많이 쓰도록 사이즈 정책을 제한
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(4)
        g.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)

        self.btn_infer_start = QPushButton("Start Inference")
        self.btn_infer_start.setStyleSheet(
            "QPushButton { background-color: #008800; color: white; font-weight: bold; padding: 4px 10px; }"
        )
        self.btn_infer_start.clicked.connect(self._on_inference_start)
        btn_row.addWidget(self.btn_infer_start)

        self.btn_infer_stop = QPushButton("Stop Inference")
        self.btn_infer_stop.setStyleSheet(
            "QPushButton { background-color: #880000; color: white; font-weight: bold; padding: 4px 10px; }"
        )
        self.btn_infer_stop.clicked.connect(self._on_inference_stop)
        self.btn_infer_stop.setEnabled(False)
        btn_row.addWidget(self.btn_infer_stop)

        self.chk_infer_loop = QCheckBox("Loop")
        self.chk_infer_loop.setChecked(self.infer.loop)
        self.chk_infer_loop.setStyleSheet("font-size: 11px;")
        self.chk_infer_loop.stateChanged.connect(self._on_inference_loop_changed)
        btn_row.addWidget(self.chk_infer_loop)

        self.btn_reset_pose = QPushButton("Reset to Init Pose")
        self.btn_reset_pose.setStyleSheet(
            "QPushButton { background-color: #444488; color: white; font-weight: bold; padding: 4px 10px; }"
        )
        self.btn_reset_pose.clicked.connect(self._on_reset_pose)
        btn_row.addWidget(self.btn_reset_pose)

        btn_row.addStretch(1)
        layout.addLayout(btn_row)

        ir = QHBoxLayout()
        ir.setSpacing(6)

        inputs_col = QVBoxLayout()
        inputs_col.setSpacing(2)

        row_scale = QHBoxLayout()
        row_scale.setSpacing(4)
        row_scale.addWidget(QLabel("Scale:"))
        self.residual_scale_input = QDoubleSpinBox()
        self.residual_scale_input.setRange(0.0, 0.2)
        self.residual_scale_input.setSingleStep(0.01)
        self.residual_scale_input.setValue(0.1)
        self.residual_scale_input.setDecimals(2)
        self.residual_scale_input.setMaximumWidth(80)
        self.residual_scale_input.setToolTip("최종 행동 = reference_trajectory + residual * scale (0.0~0.2)")
        row_scale.addWidget(self.residual_scale_input)

        row_scale.addWidget(QLabel("Speed:"))
        self.speed_input = QDoubleSpinBox()
        self.speed_input.setRange(0.1, 2.0)
        self.speed_input.setSingleStep(0.1)
        self.speed_input.setValue(0.8)
        self.speed_input.setDecimals(2)
        self.speed_input.setMaximumWidth(80)
        self.speed_input.setToolTip("궤적 재생 속도 (기본 0.8)")
        row_scale.addWidget(self.speed_input)
        inputs_col.addLayout(row_scale)

        row_bottom = QHBoxLayout()
        row_bottom.setSpacing(4)
        row_bottom.addWidget(QLabel("Duration:"))
        self.infer_duration_input = self._float_edit(f"{self.cfg.infer_duration_s:.1f}", max_width=80)
        self.infer_duration_input.setToolTip("인퍼런스를 유지할 최대 시간 (초). 0이면 무제한.")
        row_bottom.addWidget(self.infer_duration_input)

        inputs_col.addLayout(row_bottom)
        ir.addLayout(inputs_col)
        ir.addStretch(1)
        layout.addLayout(ir)

        return g

    def _build_obs_debug_group(self) -> QGroupBox:
        g = QGroupBox("Observation Debug")
        g.setStyleSheet(STYLE_GROUP_BOX)
        layout = QVBoxLayout(g)
        layout.setSpacing(4)

        self.lbl_total_obs_dim = QLabel("Total Obs Dim: - / -")
        self.lbl_total_obs_dim.setStyleSheet("font-size: 12px; font-weight: bold; color: #888; padding: 4px 0px;")
        layout.addWidget(self.lbl_total_obs_dim)

        for spec in OBS_SPECS:
            row = QHBoxLayout()

            idx_label = QLabel(f"[{spec.start:3d}-{spec.end:3d}]")
            idx_label.setMinimumWidth(80)
            idx_label.setStyleSheet("font-family: monospace; font-size: 10px; color: #aaa;")
            row.addWidget(idx_label)

            name_label = QLabel(spec.name)
            name_label.setMinimumWidth(200)
            name_label.setStyleSheet("font-size: 11px;")
            row.addWidget(name_label)

            status_label = QLabel("● Waiting")
            status_label.setStyleSheet("font-size: 10px; color: #888; font-weight: bold;")
            status_label.setMinimumWidth(100)
            self.obs_status_labels[spec.name] = status_label
            row.addWidget(status_label)

            if spec.plot_enabled:
                cb = QCheckBox("Plot")
                cb.setStyleSheet("font-size: 10px;")
                cb.stateChanged.connect(lambda _state, obs=spec.name: self._on_plot_checkbox_changed(obs))
                self.obs_plot_checkboxes[spec.name] = cb
                row.addWidget(cb)

            rate_label = QLabel("- Hz")
            rate_label.setMinimumWidth(70)
            rate_label.setStyleSheet("font-size: 10px; color: #888; font-family: monospace;")
            self.obs_rate_labels[spec.name] = rate_label
            row.addWidget(rate_label)

            row.addStretch()
            layout.addLayout(row)

        return g

    # ----------------------------------------------------------------------------------
    # Providers / Normalization
    # ----------------------------------------------------------------------------------
    def _make_obs_provider(self) -> ObsProvider:
        if self.external_provider is not None:
            return self.external_provider
        return lambda: self.store.snapshot() if self.ros_running else {}

    def _snapshot_raw(self) -> ObsDict:
        try:
            return dict(self._obs_provider() or {})
        except Exception:
            return {}

    def _normalized_data(self) -> ObsDict:
        raw = self._snapshot_raw()
        return self.normalizer.normalize(raw)

    # ----------------------------------------------------------------------------------
    # ROS2 freshness
    # ----------------------------------------------------------------------------------
    def _is_stream_fresh(self, obs_name: str) -> bool:
        if obs_name not in STREAMING_NAMES:
            return True
        if not self.ros_running or self.ros_node is None or not hasattr(self.ros_node, "get_last_update_age"):
            return True
        age = self.ros_node.get_last_update_age(obs_name)
        return (age is None) or (age <= OBS_TIMEOUT_SEC)

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
        self.policy.unload()
        self.lbl_model_status.setText("Policy: None (drop .pt/.pth)")
        self.lbl_model_status.setStyleSheet(STYLE_STATUS_IDLE)

    def unload_trajectory(self) -> None:
        self.loaded_trajectory = None
        self.infer.set_trajectory(None)
        self.plot_panel.set_trajectory_data(None)
        self.lbl_traj_status.setText("Trajectory: None (drop .npz)")
        self.lbl_traj_status.setStyleSheet(STYLE_STATUS_IDLE)

    # ----------------------------------------------------------------------------------
    # Loaders
    # ----------------------------------------------------------------------------------
    def load_policy_model(self, file_path: str) -> None:
        try:
            self.lbl_model_status.setText(f"Loading Policy: {os.path.basename(file_path)}...")
            QApplication.processEvents()

            model_type, obs_dim, action_dim = self.policy.load(file_path)

            msg = f"✅ Loaded Policy: {os.path.basename(file_path)} ({model_type})"

            self.lbl_model_status.setText(msg)
            self.lbl_model_status.setStyleSheet(STYLE_STATUS_OK)
            LOG.info("Policy loaded: %s", file_path)
        except Exception as e:
            self.policy.unload()
            self.lbl_model_status.setText(f"❌ Policy Load Failed: {str(e)}")
            self.lbl_model_status.setStyleSheet(STYLE_STATUS_ERR)
            LOG.exception("Policy load failed: %s", file_path)

    def load_trajectory_file(self, file_path: str) -> None:
        try:
            self.lbl_traj_status.setText(f"Loading Trajectory: {os.path.basename(file_path)}...")
            QApplication.processEvents()

            traj = TrajectoryLoader.load_npz(file_path, self.cfg.num_joints)
            self.loaded_trajectory = traj
            self.infer.set_trajectory(traj)
            self.plot_panel.set_trajectory_data(traj["positions"])

            ok = bool(traj.get("action_ok")) and bool(traj.get("order_ok"))
            self.lbl_traj_status.setText(f"✅ Trajectory Loaded: {os.path.basename(file_path)}")
            self.lbl_traj_status.setStyleSheet(STYLE_STATUS_OK if ok else STYLE_STATUS_ERR)

            LOG.info("Trajectory loaded: %s", file_path)
        except Exception as e:
            self.loaded_trajectory = None
            self.infer.set_trajectory(None)
            self.lbl_traj_status.setText(f"❌ Trajectory Load Failed: {str(e)}")
            self.lbl_traj_status.setStyleSheet(STYLE_STATUS_ERR)
            LOG.exception("Trajectory load failed: %s", file_path)

    # ----------------------------------------------------------------------------------
    # Control callbacks
    # ----------------------------------------------------------------------------------

    def _on_inference_start(self) -> None:
        if hasattr(self, "chk_infer_loop"):
            self.infer.loop = self.chk_infer_loop.isChecked()

        rs = float(self.residual_scale_input.value())
        speed = float(self.speed_input.value())
        self.infer.speed = speed

        duration_s = self._to_float(
            (self.infer_duration_input.text() or f"{self.cfg.infer_duration_s:.1f}").strip(),
            self.cfg.infer_duration_s,
        )
        if duration_s < 0.0:
            duration_s = 0.0
        self.infer_duration_input.setText(f"{duration_s:.1f}")

        if self.policy.expected_obs_dim is not None and self.policy.expected_obs_dim != EXPECTED_TOTAL_OBS_DIM:
            LOG.warning("Policy obs_dim=%d != expected=%d", self.policy.expected_obs_dim, EXPECTED_TOTAL_OBS_DIM)

        self.infer.start(
            speed=speed,
            residual_scale=rs,
            max_duration_s=duration_s,
        )
        self.infer.speed = speed
        self.btn_infer_start.setEnabled(False)
        self.btn_infer_stop.setEnabled(True)
        LOG.info("Inference started (residual_scale=%.3f)", rs)

    def _on_inference_loop_changed(self, state: int) -> None:
        self.infer.loop = bool(state == Qt.Checked)

    def _on_inference_stop(self) -> None:
        self.infer.stop()
        self.btn_infer_start.setEnabled(True)
        self.btn_infer_stop.setEnabled(False)
        LOG.info("Inference stopped")

    def _on_reset_pose(self) -> None:
        self.infer.reset_to_init_pose()
        LOG.info("Reset to initial pose requested")

    # ----------------------------------------------------------------------------------
    # Plot checkbox (single selection)
    # ----------------------------------------------------------------------------------
    def _on_plot_checkbox_changed(self, obs_name: str) -> None:
        cb = self.obs_plot_checkboxes.get(obs_name)
        if cb is None:
            return

        if cb.isChecked():
            for name, other in self.obs_plot_checkboxes.items():
                if name != obs_name and other.isChecked():
                    with QSignalBlocker(other):
                        other.setChecked(False)

            self.plot_panel.set_selected_observation(obs_name)
            spec = next((s for s in OBS_SPECS if s.name == obs_name), None)
            if spec is not None:
                self.plot_panel.set_max_index(spec.dim - 1)
            self.lbl_current_plot.setText(f"Plotting: {obs_name}")
            self.lbl_current_plot.setStyleSheet("font-size: 12px; color: #00ff00; padding: 4px; font-weight: bold;")
        else:
            if self.plot_panel.selected_key == obs_name:
                self.plot_panel.set_selected_observation(None)
                self.lbl_current_plot.setText("(No data selected)")
                self.lbl_current_plot.setStyleSheet("font-size: 12px; color: #888; padding: 4px; font-style: italic;")

    # ----------------------------------------------------------------------------------
    # ROS2 connect/disconnect
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
        try:
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)
            os.environ.setdefault("RCUTILS_LOGGING_SEVERITY", "ERROR")
            import rclpy  # type: ignore

            if not rclpy.ok():
                rclpy.init()

            self.ros_node = ROS2ObservationSubscriber(self.store, domain_id=domain_id)
            self.ros_running = True
            self._obs_provider = self._make_obs_provider()

            self.ros_thread = threading.Thread(target=self._ros_spin_thread, daemon=True)
            self.ros_thread.start()

            self.btn_ros_connect.setText("Disconnect")
            self.lbl_ros_status.setText("● Connected")
            self.lbl_ros_status.setStyleSheet("font-size: 11px; color: #00ff00; font-weight: bold;")
            self.ros_domain_input.setEnabled(False)

            LOG.info("ROS2 Connected (Domain ID=%d)", domain_id)
        except Exception:
            self.ros_running = False
            self.ros_node = None
            self.lbl_ros_status.setText("● Error: Connection failed")
            self.lbl_ros_status.setStyleSheet("font-size: 11px; color: #ff0000; font-weight: bold;")
            LOG.exception("ROS2 connection failed")

    def _ros_disconnect(self) -> None:
        self.ros_running = False
        self.btn_ros_connect.setText("Connect")
        self.lbl_ros_status.setText("● Disconnected")
        self.lbl_ros_status.setStyleSheet("font-size: 11px; color: #888; font-weight: bold;")
        self.ros_domain_input.setEnabled(True)

        self.store.clear()
        self._obs_provider = lambda: {}
        with self._obs_rate_lock:
            self._obs_rate_hz.clear()
            self._obs_last_seen.clear()
            self._obs_last_ros_ts.clear()
        LOG.info("ROS2 Disconnected")

    def _ros_spin_thread(self) -> None:
        try:
            spin_while(self.ros_node, lambda: self.ros_running)
        except Exception:
            LOG.exception("ROS2 spin thread error")
        finally:
            try:
                if self.ros_node is not None:
                    self.ros_node.destroy_node()
            finally:
                try:
                    import rclpy  # type: ignore
                except Exception:
                    return
                if rclpy.ok():  # type: ignore[attr-defined]
                    rclpy.shutdown()

    # ----------------------------------------------------------------------------------
    # ROS publish callbacks used by inference engine
    # ----------------------------------------------------------------------------------
    def _publish_policy_action(self, actions: np.ndarray) -> None:
        if ROS2_AVAILABLE and self.ros_running and self.ros_node is not None:
            try:
                self.ros_node.publish_policy_action(actions)
            except Exception:
                LOG.exception("publish_policy_action failed")

    def _publish_last_actions(self, actions: np.ndarray) -> None:
        if ROS2_AVAILABLE and self.ros_running and self.ros_node is not None:
            try:
                self.ros_node.publish_last_actions(actions)
            except Exception:
                LOG.exception("publish_last_actions failed")

    # ----------------------------------------------------------------------------------
    # UI status update helpers
    # ----------------------------------------------------------------------------------
    def _spec_active(self, data: ObsDict, spec, joint_pos_active: bool, actions_active: bool) -> bool:
        if spec.name == "last_actions":
            return actions_active
        if spec.name == "reference_joint_pos":
            return actions_active  # 추론 시 궤적 ref로 채움
        if spec.name not in data:
            return False
        arr = np.asarray(data[spec.name]).reshape(-1)
        if arr.size != spec.dim:
            return False
        if spec.streaming and not self._is_stream_fresh(spec.name):
            return False
        return True

    def _update_obs_ui(self, data: ObsDict) -> None:
        actions_active = self.loaded_trajectory is not None
        joint_pos_active = ("joint_pos" in data) and self._is_stream_fresh("joint_pos") and (np.asarray(data["joint_pos"]).size == self.cfg.num_joints)

        active_dim = 0
        for spec in OBS_SPECS:
            if self._spec_active(data, spec, joint_pos_active, actions_active):
                active_dim += spec.dim

        if self.policy.expected_obs_dim is not None:
            self.lbl_total_obs_dim.setText(f"Total Obs Dim: {active_dim} / {self.policy.expected_obs_dim}")
            if active_dim == self.policy.expected_obs_dim:
                self.lbl_total_obs_dim.setStyleSheet("font-size: 12px; font-weight: bold; color: #00ff00; padding: 4px 0px;")
            else:
                self.lbl_total_obs_dim.setStyleSheet("font-size: 12px; font-weight: bold; color: #ffff00; padding: 4px 0px;")
        else:
            self.lbl_total_obs_dim.setText("Total Obs Dim: - / -")
            self.lbl_total_obs_dim.setStyleSheet("font-size: 12px; font-weight: bold; color: #888; padding: 4px 0px;")

        for spec in OBS_SPECS:
            lbl = self.obs_status_labels.get(spec.name)
            rate_lbl = self.obs_rate_labels.get(spec.name)
            rate_val = None
            with self._obs_rate_lock:
                rate_val = self._obs_rate_hz.get(spec.name)
            if lbl is None:
                continue
            if self._spec_active(data, spec, joint_pos_active, actions_active):
                lbl.setText("● Active")
                lbl.setStyleSheet("font-size: 10px; color: #00ff00; font-weight: bold;")
                if rate_lbl is not None:
                    hz_text = f"{rate_val:5.1f} Hz" if rate_val is not None else "- Hz"
                    rate_lbl.setText(hz_text)
                    rate_lbl.setStyleSheet("font-size: 10px; color: #00ff00; font-family: monospace;")
            else:
                lbl.setText("● Missing")
                lbl.setStyleSheet("font-size: 10px; color: #ffff00; font-weight: bold;")
                if rate_lbl is not None:
                    rate_lbl.setText("- Hz")
                    rate_lbl.setStyleSheet("font-size: 10px; color: #888; font-family: monospace;")

    def _update_obs_rate(self, data: ObsDict) -> None:
        """관측 업데이트 주파수(Hz)를 계산해 UI에 표시하기 위한 내부 스토어."""
        now = time.perf_counter()
        actions_active = self.loaded_trajectory is not None
        joint_pos_active = ("joint_pos" in data) and self._is_stream_fresh("joint_pos") and (np.asarray(data["joint_pos"]).size == self.cfg.num_joints)

        with self._obs_rate_lock:
            for spec in OBS_SPECS:
                if not self._spec_active(data, spec, joint_pos_active, actions_active):
                    continue

                # 1) 스트리밍 obs: ROS 수신 주기로 계산 (실제 네트워크 입력 갱신 주기)
                if spec.streaming:
                    ros_ts = None
                    if self.ros_running and self.ros_node is not None and hasattr(self.ros_node, "get_last_update_time"):
                        try:
                            ros_ts = self.ros_node.get_last_update_time(spec.name)  # type: ignore[attr-defined]
                        except Exception:
                            ros_ts = None

                    if ros_ts is not None:
                        last_ts = self._obs_last_ros_ts.get(spec.name)
                        if last_ts is not None and ros_ts != last_ts:
                            dt = ros_ts - last_ts
                            if dt > 0:
                                self._obs_rate_hz[spec.name] = 1.0 / dt
                        self._obs_last_ros_ts[spec.name] = ros_ts
                        # 스트리밍은 ROS 수신 시각 기반만 사용
                        continue

                # 2) 비스트리밍 obs (hammer_pos/target_right_hand_pose 등) 또는 ROS 정보 없음:
                #    실제 네트워크 입력 직전에 제어 루프에서 주입한 주기로 계산 (ZOH일 경우 control_hz)
                last = self._obs_last_seen.get(spec.name)
                if last is not None:
                    dt = now - last
                    if dt > 0:
                        self._obs_rate_hz[spec.name] = 1.0 / dt
                self._obs_last_seen[spec.name] = now

    # ----------------------------------------------------------------------------------
    # Control / update loops
    # ----------------------------------------------------------------------------------
    def _control_loop_thread(self) -> None:
        """액션 계산·퍼블리시 루프 (50Hz 기본) - 백그라운드 스레드."""
        interval = 1.0 / float(max(1, self.cfg.control_hz))
        while self._control_running:
            t0 = time.perf_counter()
            data = self._normalized_data()
            self._update_obs_rate(data)
            self.infer.step_and_inject(data, interval)
            with self._last_data_lock:
                self._last_data = data
            elapsed = time.perf_counter() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def update_loop(self) -> None:
        """UI/플롯 업데이트 루프 (30Hz 기본)."""
        with self._last_data_lock:
            data = self._last_data if self._last_data is not None else self._normalized_data()
        self._update_obs_ui(data)
        self.plot_panel.ingest_obs(data)
        self._sync_inference_buttons()

        if hasattr(self, "speed_input"):
            self.infer.speed = float(self.speed_input.value())

    def _sync_inference_buttons(self) -> None:
        running = bool(self.infer.running)
        self.btn_infer_start.setEnabled(not running)
        self.btn_infer_stop.setEnabled(running)


def run_app() -> None:
    app = QApplication([])
    window = Sim2RealDebugger()
    window.show()
    app.exec()


__all__ = ["Sim2RealDebugger", "run_app"]
