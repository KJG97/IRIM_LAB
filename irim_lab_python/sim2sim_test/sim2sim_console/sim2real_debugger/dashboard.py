import os
import threading
import time
import logging
from typing import Callable, Dict, List, Optional

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
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from .config import (
    DebuggerConfig,
    DEPLOY_MODE_SIM2REAL,
    DEPLOY_MODE_SIM2SIM,
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
from .env_agent_check import (
    EnvAgentCheckResult,
    compare_with_debugger,
    load_agent_yaml,
    load_env_agent_from_policy_path,
    load_env_yaml,
)
from .inference import InferenceEngine, TrajectoryLoader
from .observation import ObsNormalizer, ObservationStore
from .policy import PolicyWrapper
from .ros2_io import ROS2_AVAILABLE, ROS2ObservationSubscriber, spin_while
from .widgets import DropZone, PlotPanel

LOG = logging.getLogger(__name__)

# O(1) spec lookup (OBS_SPECS는 config에서 단일 소스)
OBS_SPECS_BY_NAME = {s.name: s for s in OBS_SPECS}

# UI 스타일 상수 (매직 스트링 제거)
_STYLE_ROS_OK = "font-size: 11px; color: #00ff00; font-weight: bold;"
_STYLE_ROS_ERR = "font-size: 11px; color: #ff0000; font-weight: bold;"
_STYLE_ROS_IDLE = "font-size: 11px; color: #888; font-weight: bold;"
_STYLE_LBL_OBS_DIM_BASE = "font-size: 12px; font-weight: bold; padding: 4px 0px;"
_STYLE_LBL_PLOT_ACTIVE = "font-size: 12px; color: #00ff00; padding: 4px; font-weight: bold;"
_STYLE_LBL_PLOT_IDLE = "font-size: 12px; color: #888; padding: 4px; font-style: italic;"
_STYLE_OBS_ACTIVE = "font-size: 10px; color: #00ff00; font-weight: bold;"
_STYLE_OBS_MISSING = "font-size: 10px; color: #ffff00; font-weight: bold;"
_STYLE_OBS_RATE_ACTIVE = "font-size: 10px; color: #00ff00; font-family: monospace;"
_STYLE_OBS_WAITING = "font-size: 10px; color: #888; font-weight: bold;"
_STYLE_OBS_RATE_IDLE = "font-size: 10px; color: #888; font-family: monospace;"
_STYLE_BTN_START = "QPushButton { background-color: #008800; color: white; font-weight: bold; padding: 4px 10px; }"
_STYLE_BTN_STOP = "QPushButton { background-color: #880000; color: white; font-weight: bold; padding: 4px 10px; }"
_STYLE_BTN_RESET = "QPushButton { background-color: #444488; color: white; font-weight: bold; padding: 4px 10px; }"


class Sim2RealDebugger(QMainWindow):
    def __init__(
        self,
        obs_provider: Optional[ObsProvider] = None,
        cfg: Optional[DebuggerConfig] = None,
        deploy_mode: str = DEPLOY_MODE_SIM2REAL,
        physics_dt_provider: Optional[Callable[[], float]] = None,
        policy_action_applier: Optional[Callable[[np.ndarray], None]] = None,
    ):
        super().__init__()
        self.cfg = cfg or DebuggerConfig()
        self.deploy_mode = deploy_mode if deploy_mode in (DEPLOY_MODE_SIM2REAL, DEPLOY_MODE_SIM2SIM) else DEPLOY_MODE_SIM2REAL
        self._sim2sim_mode = self.deploy_mode == DEPLOY_MODE_SIM2SIM
        self.policy_action_applier = policy_action_applier

        title_suffix = " (Sim2Sim)" if self._sim2sim_mode else " (Sim2Real / ROS2)"
        self.setWindowTitle(WINDOW_TITLE + title_suffix)
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)
        self.setStyleSheet("background-color: #2b2b2b; color: #ffffff;")

        # core components
        self.store = ObservationStore()
        self.normalizer = ObsNormalizer()
        self.policy = PolicyWrapper()
        self.infer = InferenceEngine(self.cfg, self.policy)

        # ROS2 state (Sim2Sim 모드에서는 사용하지 않음)
        self.ros_node: Optional["ROS2ObservationSubscriber"] = None
        self.ros_thread: Optional[threading.Thread] = None
        self.ros_running = False

        # Sim2Sim: Isaac Sim physics_dt + decimation으로 policy 입력 주기 (학습 시 decimation=4 → 200Hz/4=50Hz)
        self.physics_dt_provider = physics_dt_provider
        self._sim2sim_physics_dt: float = 0.0
        self._sim2sim_physics_dt_lock = threading.Lock()
        self._decimation: int = 4  # control 스레드에서 읽음, UI에서 갱신

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
        self._loaded_policy_path: Optional[str] = None
        self._env_agent_check_result: Optional[EnvAgentCheckResult] = None

        # build UI
        self._build_ui()

        # wire inference callbacks
        self.infer.set_freshness_callback(self._is_stream_fresh)
        self.infer.set_publish_callback(self._publish_policy_action)
        self.infer.set_last_action_publish_callback(self._publish_last_actions)

        # control loop in background thread
        self._control_running = True
        self._last_data: Optional[ObsDict] = None
        self._last_data_lock = threading.Lock()
        # 정책 출력 주기·raw_actions 터미널 로그 (inference 실행 중일 때만)
        self._policy_rate_step_count = 0
        self._policy_rate_interval_start: Optional[float] = None
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

        if self._sim2sim_mode:
            left_layout.addWidget(self._build_sim2sim_mode_group())
        else:
            left_layout.addWidget(self._build_ros_group())
        left_layout.addLayout(self._build_model_traj_status_row())
        left_layout.addLayout(self._build_unload_buttons())
        left_layout.addWidget(self._build_control_group())
        left_layout.addWidget(self._build_obs_debug_group())
        left_layout.addWidget(self._build_env_agent_check_group())

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
        self.lbl_current_plot.setStyleSheet(_STYLE_LBL_PLOT_IDLE)
        disp_title_row.addWidget(self.lbl_current_plot)

        disp_title_row.addStretch()
        right_layout.addLayout(disp_title_row)

        self.plot_panel = PlotPanel(buffer_size=self.cfg.buffer_size, plot_update_hz=self.cfg.plot_update_hz)
        right_layout.addWidget(self.plot_panel, 1)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 10)
        splitter.setStretchFactor(1, 16)

    def _build_sim2sim_mode_group(self) -> QGroupBox:
        """Sim2Sim 모드: ROS2 비활성화, physics_dt × decimation으로 policy 입력 주기 (학습 시 200Hz/4=50Hz)."""
        g = QGroupBox("Deploy Mode")
        g.setMinimumHeight(80)
        layout = QVBoxLayout(g)
        layout.setContentsMargins(8, 4, 8, 4)
        row1 = QHBoxLayout()
        lbl = QLabel("Sim2Sim — ROS2 disabled. Obs/Action via Isaac Sim API.")
        lbl.setStyleSheet("font-size: 11px; color: #88ff88;")
        lbl.setWordWrap(True)
        row1.addWidget(lbl)
        layout.addLayout(row1)
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Decimation:"))
        self.decimation_input = QSpinBox()
        self.decimation_input.setRange(1, 32)
        self.decimation_input.setValue(4)
        self.decimation_input.setToolTip("Policy 입력 주기 = Physics Hz / Decimation (학습 시 200Hz/4=50Hz)")
        self.decimation_input.setMaximumWidth(60)
        self.decimation_input.setStyleSheet("font-size: 11px;")
        row2.addWidget(self.decimation_input)
        row2.addWidget(QLabel("→ Policy:"))
        self.lbl_sim2sim_input_period = QLabel("— Hz")
        self.lbl_sim2sim_input_period.setStyleSheet("font-size: 11px; color: #aaffaa; font-family: monospace;")
        row2.addWidget(self.lbl_sim2sim_input_period)
        row2.addStretch()
        layout.addLayout(row2)
        g.setStyleSheet(STYLE_GROUP_BOX)
        return g

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
        self.lbl_ros_status.setStyleSheet(_STYLE_ROS_IDLE)
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
        self.btn_infer_start.setStyleSheet(_STYLE_BTN_START)
        self.btn_infer_start.clicked.connect(self._on_inference_start)
        btn_row.addWidget(self.btn_infer_start)

        self.btn_infer_stop = QPushButton("Stop Inference")
        self.btn_infer_stop.setStyleSheet(_STYLE_BTN_STOP)
        self.btn_infer_stop.clicked.connect(self._on_inference_stop)
        self.btn_infer_stop.setEnabled(False)
        btn_row.addWidget(self.btn_infer_stop)

        self.chk_infer_loop = QCheckBox("Loop")
        self.chk_infer_loop.setChecked(self.infer.loop)
        self.chk_infer_loop.setStyleSheet("font-size: 11px;")
        self.chk_infer_loop.stateChanged.connect(self._on_inference_loop_changed)
        btn_row.addWidget(self.chk_infer_loop)

        self.btn_reset_pose = QPushButton("Reset to Init Pose")
        self.btn_reset_pose.setStyleSheet(_STYLE_BTN_RESET)
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
        self.lbl_total_obs_dim.setStyleSheet(f"{_STYLE_LBL_OBS_DIM_BASE} color: #888;")
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
            status_label.setStyleSheet(_STYLE_OBS_WAITING)
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
            rate_label.setStyleSheet(_STYLE_OBS_RATE_IDLE)
            self.obs_rate_labels[spec.name] = rate_label
            row.addWidget(rate_label)

            row.addStretch()
            layout.addLayout(row)

        return g

    def _build_env_agent_check_group(self) -> QGroupBox:
        """Env/Agent YAML과 디버거·정책 비교 요약 패널."""
        g = QGroupBox("Env / Policy Check")
        g.setStyleSheet(STYLE_GROUP_BOX)
        layout = QVBoxLayout(g)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(4)
        g.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)

        self.lbl_env_agent_summary = QLabel("정책 로드 후 env.yaml/agent.yaml과 비교 결과가 표시됩니다.")
        self.lbl_env_agent_summary.setWordWrap(True)
        self.lbl_env_agent_summary.setStyleSheet("font-size: 10px; color: #aaa; padding: 4px;")
        self.lbl_env_agent_summary.setMinimumHeight(60)
        layout.addWidget(self.lbl_env_agent_summary)
        return g

    def _update_env_agent_check_ui(self) -> None:
        """_env_agent_check_result에 따라 Env/Policy Check 패널 텍스트 갱신."""
        lbl = getattr(self, "lbl_env_agent_summary", None)
        if lbl is None:
            return
        r = self._env_agent_check_result
        if r is None:
            if self._loaded_policy_path:
                lbl.setText("정책 로드됨. YAML을 찾지 못했거나 비교 미실행.")
                lbl.setStyleSheet("font-size: 10px; color: #888; padding: 4px;")
            else:
                lbl.setText("정책 로드 후 env.yaml/agent.yaml과 비교 결과가 표시됩니다.")
                lbl.setStyleSheet("font-size: 10px; color: #aaa; padding: 4px;")
            return
        lines: List[str] = []
        if r.env_summary:
            e = r.env_summary
            parts = [f"env: joints={e.num_joints}" if e.num_joints is not None else "env: joints=—"]
            if e.expected_obs_dim is not None:
                parts.append(f"obs_dim={e.expected_obs_dim}")
            if e.decimation is not None:
                parts.append(f"decimation={e.decimation}")
            if e.residual_scale is not None:
                parts.append(f"residual_scale={e.residual_scale}")
            lines.append(", ".join(parts))
        if r.agent_summary and r.agent_summary.experiment_name:
            lines.append(f"agent: {r.agent_summary.experiment_name}")
        if r.mismatches:
            lines.append("")
            lines.append("⚠ 불일치:")
            for m in r.mismatches:
                lines.append(f"  • {m}")
            lbl.setStyleSheet("font-size: 10px; color: #ffaa00; padding: 4px; font-weight: bold;")
        elif r.warnings:
            lines.append("")
            lines.append("참고:")
            for w in r.warnings:
                lines.append(f"  • {w}")
            lbl.setStyleSheet("font-size: 10px; color: #aaccff; padding: 4px;")
        else:
            lbl.setStyleSheet("font-size: 10px; color: #88ff88; padding: 4px;")
        if not lines:
            lines.append("YAML 로드됨. 비교 항목 없음.")
        lbl.setText("\n".join(lines))

    # ----------------------------------------------------------------------------------
    # Providers / Normalization
    # ----------------------------------------------------------------------------------
    def _make_obs_provider(self) -> ObsProvider:
        if self.external_provider is not None:
            return self.external_provider
        if self._sim2sim_mode:
            # Sim2Sim: external_provider가 주입되지 않았으면 빈 관측 (연동 시 obs_provider 전달)
            return lambda: {}
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
        yaml_paths: List[str] = []
        for p in paths:
            pl = p.lower()
            if pl.endswith((".pt", ".pth")):
                last_policy = p
            elif pl.endswith(".npz"):
                last_traj = p
            elif pl.endswith((".yaml", ".yml")):
                yaml_paths.append(p)
        if last_policy:
            self.load_policy_model(last_policy)
        if last_traj:
            self.load_trajectory_file(last_traj)
        for ypath in yaml_paths:
            self.load_yaml_and_compare(ypath)

    def load_yaml_and_compare(self, yaml_path: str) -> None:
        """드롭된 YAML 파일(또는 그 파일이 있는 디렉터리)에서 env.yaml/agent.yaml 로드 후 비교."""
        if not os.path.isfile(yaml_path):
            return
        yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
        env_path = os.path.join(yaml_dir, "env.yaml")
        agent_path = os.path.join(yaml_dir, "agent.yaml")

        env_summary = load_env_yaml(env_path) if os.path.isfile(env_path) else None
        agent_summary = load_agent_yaml(agent_path) if os.path.isfile(agent_path) else None
        # 드롭한 파일이 env.yaml/agent.yaml이면 해당 경로로도 시도 (같은 경로면 이미 로드됨)
        if env_summary is None and os.path.basename(yaml_path).lower() == "env.yaml":
            env_summary = load_env_yaml(yaml_path)
        if agent_summary is None and os.path.basename(yaml_path).lower() == "agent.yaml":
            agent_summary = load_agent_yaml(yaml_path)

        rs_ui = float(self.residual_scale_input.value()) if hasattr(self, "residual_scale_input") else 0.1
        self._env_agent_check_result = compare_with_debugger(
            env_summary,
            agent_summary,
            num_joints=self.cfg.num_joints,
            expected_total_obs_dim=EXPECTED_TOTAL_OBS_DIM,
            policy_obs_dim=self.policy.expected_obs_dim,
            policy_action_dim=self.policy.expected_action_dim,
            residual_scale_ui=rs_ui,
        )
        self._update_env_agent_check_ui()
        if env_summary is not None or agent_summary is not None:
            LOG.info("YAML loaded from drop: %s (env=%s, agent=%s)", yaml_dir, env_summary is not None, agent_summary is not None)

    def unload_policy(self) -> None:
        self.policy.unload()
        self._loaded_policy_path = None
        self._env_agent_check_result = None
        self.lbl_model_status.setText("Policy: None (drop .pt/.pth)")
        self.lbl_model_status.setStyleSheet(STYLE_STATUS_IDLE)
        self._update_env_agent_check_ui()

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

            model_type, *_ = self.policy.load(file_path)
            msg = f"✅ Loaded Policy: {os.path.basename(file_path)} ({model_type})"

            self.lbl_model_status.setText(msg)
            self.lbl_model_status.setStyleSheet(STYLE_STATUS_OK)
            self._loaded_policy_path = file_path

            # env/agent YAML 찾아서 비교
            env_summary, agent_summary, any_loaded = load_env_agent_from_policy_path(file_path)
            rs_ui = float(self.residual_scale_input.value()) if hasattr(self, "residual_scale_input") else 0.1
            self._env_agent_check_result = compare_with_debugger(
                env_summary,
                agent_summary,
                num_joints=self.cfg.num_joints,
                expected_total_obs_dim=EXPECTED_TOTAL_OBS_DIM,
                policy_obs_dim=self.policy.expected_obs_dim,
                policy_action_dim=self.policy.expected_action_dim,
                residual_scale_ui=rs_ui,
            )
            self._update_env_agent_check_ui()

            LOG.info("Policy loaded: %s", file_path)
        except Exception as e:
            self.policy.unload()
            self._loaded_policy_path = None
            self._env_agent_check_result = None
            self._update_env_agent_check_ui()
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
        self.infer.loop = self.chk_infer_loop.isChecked()
        rs = float(self.residual_scale_input.value())
        speed = float(self.speed_input.value())
        duration_s = self._to_float(
            (self.infer_duration_input.text() or f"{self.cfg.infer_duration_s:.1f}").strip(),
            self.cfg.infer_duration_s,
        )
        duration_s = max(0.0, duration_s)
        self.infer_duration_input.setText(f"{duration_s:.1f}")

        if self.policy.expected_obs_dim is not None and self.policy.expected_obs_dim != EXPECTED_TOTAL_OBS_DIM:
            LOG.warning("Policy obs_dim=%d != expected=%d", self.policy.expected_obs_dim, EXPECTED_TOTAL_OBS_DIM)

        self.infer.start(speed=speed, residual_scale=rs, max_duration_s=duration_s)
        self.infer.speed = speed
        self._policy_rate_step_count = 0
        self._policy_rate_interval_start = None
        self.btn_infer_start.setEnabled(False)
        self.btn_infer_stop.setEnabled(True)
        LOG.info("Inference started (residual_scale=%.3f)", rs)

    def _on_inference_loop_changed(self, state: int) -> None:
        self.infer.loop = bool(state == Qt.Checked)

    def _on_inference_stop(self) -> None:
        self.infer.stop()
        self._policy_rate_step_count = 0
        self._policy_rate_interval_start = None
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
            spec = OBS_SPECS_BY_NAME.get(obs_name)
            if spec is not None:
                self.plot_panel.set_max_index(spec.dim - 1)
            self.lbl_current_plot.setText(f"Plotting: {obs_name}")
            self.lbl_current_plot.setStyleSheet(_STYLE_LBL_PLOT_ACTIVE)
        else:
            if self.plot_panel.selected_key == obs_name:
                self.plot_panel.set_selected_observation(None)
                self.lbl_current_plot.setText("(No data selected)")
                self.lbl_current_plot.setStyleSheet(_STYLE_LBL_PLOT_IDLE)

    # ----------------------------------------------------------------------------------
    # ROS2 connect/disconnect
    # ----------------------------------------------------------------------------------
    def _on_ros_connect(self) -> None:
        if not ROS2_AVAILABLE:
            self.lbl_ros_status.setText("● Error: ROS2 not available")
            self.lbl_ros_status.setStyleSheet(_STYLE_ROS_ERR)
            return

        if self.ros_running:
            self._ros_disconnect()
            return

        try:
            domain_id = int(self.ros_domain_input.text() or "0")
        except ValueError:
            self.lbl_ros_status.setText("● Error: Invalid Domain ID")
            self.lbl_ros_status.setStyleSheet(_STYLE_ROS_ERR)
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
            self.lbl_ros_status.setStyleSheet(_STYLE_ROS_OK)
            self.ros_domain_input.setEnabled(False)

            LOG.info("ROS2 Connected (Domain ID=%d)", domain_id)
        except Exception:
            self.ros_running = False
            self.ros_node = None
            self.lbl_ros_status.setText("● Error: Connection failed")
            self.lbl_ros_status.setStyleSheet(_STYLE_ROS_ERR)
            LOG.exception("ROS2 connection failed")

    def _ros_disconnect(self) -> None:
        self.ros_running = False
        self.btn_ros_connect.setText("Connect")
        self.lbl_ros_status.setText("● Disconnected")
        self.lbl_ros_status.setStyleSheet(_STYLE_ROS_IDLE)
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
        if self._sim2sim_mode and self.policy_action_applier is not None:
            try:
                self.policy_action_applier(actions)
            except Exception:
                LOG.exception("policy_action_applier (Sim2Sim) failed")
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

    def _joint_pos_active(self, data: ObsDict) -> bool:
        return (
            "joint_pos" in data
            and self._is_stream_fresh("joint_pos")
            and np.asarray(data["joint_pos"]).size == self.cfg.num_joints
        )

    def _update_obs_ui(self, data: ObsDict, joint_pos_active: bool, actions_active: bool) -> None:
        with self._obs_rate_lock:
            rate_hz_copy = dict(self._obs_rate_hz)

        active_dim = 0
        for spec in OBS_SPECS:
            active = self._spec_active(data, spec, joint_pos_active, actions_active)
            if active:
                active_dim += spec.dim
            lbl = self.obs_status_labels.get(spec.name)
            rate_lbl = self.obs_rate_labels.get(spec.name)
            if lbl is None:
                continue
            if active:
                lbl.setText("● Active")
                lbl.setStyleSheet(_STYLE_OBS_ACTIVE)
                if rate_lbl is not None:
                    rate_val = rate_hz_copy.get(spec.name)
                    rate_lbl.setText(f"{rate_val:5.1f} Hz" if rate_val is not None else "- Hz")
                    rate_lbl.setStyleSheet(_STYLE_OBS_RATE_ACTIVE)
            else:
                lbl.setText("● Missing")
                lbl.setStyleSheet(_STYLE_OBS_MISSING)
                if rate_lbl is not None:
                    rate_lbl.setText("- Hz")
                    rate_lbl.setStyleSheet(_STYLE_OBS_RATE_IDLE)

        base_style = _STYLE_LBL_OBS_DIM_BASE
        if self.policy.expected_obs_dim is not None:
            self.lbl_total_obs_dim.setText(f"Total Obs Dim: {active_dim} / {self.policy.expected_obs_dim}")
            self.lbl_total_obs_dim.setStyleSheet(
                f"{base_style} color: #00ff00;" if active_dim == self.policy.expected_obs_dim else f"{base_style} color: #ffff00;"
            )
        else:
            self.lbl_total_obs_dim.setText("Total Obs Dim: - / -")
            self.lbl_total_obs_dim.setStyleSheet(f"{base_style} color: #888;")

    def _update_obs_rate(self, data: ObsDict) -> None:
        """관측 업데이트 주파수(Hz)를 계산해 UI에 표시하기 위한 내부 스토어."""
        now = time.perf_counter()
        actions_active = self.loaded_trajectory is not None
        joint_pos_active = self._joint_pos_active(data)

        # Sim2Sim: policy 입력 주기(physics_dt × decimation)로 스트리밍 obs Hz 표시
        sim2sim_dt = 0.0
        if self._sim2sim_mode and self.physics_dt_provider is not None:
            with self._sim2sim_physics_dt_lock:
                sim2sim_dt = self._sim2sim_physics_dt
        decimation = max(1, self._decimation)

        ros_get_ts = (
            getattr(self.ros_node, "get_last_update_time", None)
            if (self.ros_running and self.ros_node is not None)
            else None
        )
        with self._obs_rate_lock:
            for spec in OBS_SPECS:
                if not self._spec_active(data, spec, joint_pos_active, actions_active):
                    continue

                # Sim2Sim + physics_dt 있음: 스트리밍 obs는 policy 입력 주기(Hz) = 1/(physics_dt×decimation)
                if self._sim2sim_mode and sim2sim_dt > 0 and spec.streaming:
                    self._obs_rate_hz[spec.name] = 1.0 / (sim2sim_dt * decimation)
                    continue

                # 1) 스트리밍 obs: ROS 수신 주기로 계산 (실제 네트워크 입력 갱신 주기)
                if spec.streaming and ros_get_ts is not None:
                    try:
                        ros_ts = ros_get_ts(spec.name)
                    except Exception:
                        ros_ts = None
                    if ros_ts is not None:
                        last_ts = self._obs_last_ros_ts.get(spec.name)
                        if last_ts is not None and ros_ts != last_ts:
                            dt = ros_ts - last_ts
                            if dt > 0:
                                self._obs_rate_hz[spec.name] = 1.0 / dt
                        self._obs_last_ros_ts[spec.name] = ros_ts
                        continue

                # 2) 비스트리밍 obs: 제어 루프 주입 주기로 계산
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
        """액션 계산·퍼블리시 루프. Sim2Sim은 physics_dt×decimation 주기만 사용(physics_dt 미제공 시 정책 스텝 미실행)."""
        while self._control_running:
            if self._sim2sim_mode:
                with self._sim2sim_physics_dt_lock:
                    dt = self._sim2sim_physics_dt
                if dt <= 0:
                    time.sleep(0.02)
                    continue
                interval = dt * max(1, self._decimation)
            else:
                interval = 1.0 / 50.0  # Sim2Real: 고정 50Hz
            t0 = time.perf_counter()
            data = self._normalized_data()
            self._update_obs_rate(data)
            self.infer.step_and_inject(data, interval)
            with self._last_data_lock:
                self._last_data = data
            # Inference 실행 중일 때만: 정책 출력 주기·액션 터미널 로그
            if self.infer.running:
                if self._policy_rate_interval_start is None:
                    self._policy_rate_interval_start = t0
                self._policy_rate_step_count += 1
                if self._policy_rate_step_count >= 50:
                    elapsed = time.perf_counter() - self._policy_rate_interval_start
                    if elapsed > 0:
                        actual_hz = self._policy_rate_step_count / elapsed
                        msg = f"[Policy] output rate: {actual_hz:.1f} Hz ({self._policy_rate_step_count:.0f} steps in {elapsed:.3f} s)"
                        raw_for_log = getattr(self.infer, "_last_raw_actions_for_log", None)
                        if raw_for_log is not None and raw_for_log.size > 0:
                            arr = np.asarray(raw_for_log).ravel()
                            short = np.array2string(arr[: min(8, arr.size)], precision=3, separator=", ", max_line_width=120)
                            if arr.size > 8:
                                short = short.rstrip("]") + ", ...]"
                            msg += f"  raw_actions: {short}"
                        print(msg, flush=True)
                    self._policy_rate_step_count = 0
                    self._policy_rate_interval_start = time.perf_counter()
            elapsed = time.perf_counter() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def update_loop(self) -> None:
        """UI/플롯 업데이트 루프 (30Hz 기본)."""
        # Sim2Sim: physics_dt 조회 및 decimation 반영 (Policy Hz = Physics Hz / Decimation)
        lbl_period = getattr(self, "lbl_sim2sim_input_period", None)
        decimation_input = getattr(self, "decimation_input", None)
        if self._sim2sim_mode:
            if decimation_input is not None:
                self._decimation = max(1, decimation_input.value())
            if self.physics_dt_provider is not None:
                try:
                    dt = float(self.physics_dt_provider())
                    if dt > 0:
                        with self._sim2sim_physics_dt_lock:
                            self._sim2sim_physics_dt = dt
                        if lbl_period is not None:
                            policy_hz = 1.0 / (dt * self._decimation)
                            lbl_period.setText(f"{policy_hz:.1f} Hz")
                except Exception:
                    with self._sim2sim_physics_dt_lock:
                        self._sim2sim_physics_dt = 0.0
                    if lbl_period is not None:
                        lbl_period.setText("— Hz")
            elif lbl_period is not None:
                lbl_period.setText("— Hz (no physics_dt)")

        with self._last_data_lock:
            data = self._last_data if self._last_data is not None else self._normalized_data()
        joint_pos_active = self._joint_pos_active(data)
        actions_active = self.loaded_trajectory is not None
        self._update_obs_ui(data, joint_pos_active, actions_active)
        self.plot_panel.ingest_obs(data)
        self._sync_inference_buttons()
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
