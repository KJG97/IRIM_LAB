from dataclasses import dataclass
from typing import Callable, Dict, List

import numpy as np

# -----------------------------------------------------------------------------
# 공통 타입 (DDVC: 타입과 스펙을 한 곳에서 관리)
# -----------------------------------------------------------------------------
ObsDict = Dict[str, np.ndarray]
ObsProvider = Callable[[], ObsDict]

# observation freshness (ROS streaming)
OBS_TIMEOUT_SEC = 1.0

# -----------------------------------------------------------------------------
# UI / Display (매직값 제거 - DDVC 101)
# -----------------------------------------------------------------------------
WINDOW_TITLE = "Sim2Real Proprioception Debugger (PyQtGraph + DropZone)"
WINDOW_WIDTH = 1618
WINDOW_HEIGHT = 1000

STYLE_GROUP_BOX = (
    "QGroupBox { font-weight: bold; border: 2px solid #555; border-radius: 5px; "
    "margin-top: 10px; padding-top: 10px; } "
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }"
)
STYLE_STATUS_OK = "background-color: #004400; color: #00ff00; padding: 4px 8px; font-weight: bold; font-size: 11px; border-radius: 5px;"
STYLE_STATUS_ERR = "background-color: #550000; color: #ffaaaa; padding: 4px 8px; font-weight: bold; font-size: 11px; border-radius: 5px;"
STYLE_STATUS_IDLE = "background-color: #444; padding: 4px 8px; font-weight: bold; font-size: 11px; border-radius: 5px;"

# PlotPanel: 관측별 플롯 색상, 인덱스 스핀박스 최대값
PLOT_INDEX_MAX = 4096
PLOT_COLOR_MAP: Dict[str, str] = {
    "joint_pos": "#00ff00",
    "right_hand_joint_torque": "#CC66FF",
    "right_hand_base_pos": "#FF66CC",
    "last_actions": "#66CCFF",
    "reference_joint_pos": "#FF4444",
    "hammer_pos": "#FFAA00",
    "target_right_hand_pose": "#66FFCC",
}
PLOT_COLOR_DEFAULT = "#AAAAAA"
PLOT_CURVE_WIDTH = 4

# -----------------------------------------------------------------------------
# Deploy 모드 (Sim2Real = ROS2, Sim2Sim = Isaac Sim API)
# -----------------------------------------------------------------------------
DEPLOY_MODE_SIM2REAL = "sim2real"
DEPLOY_MODE_SIM2SIM = "sim2sim"


@dataclass
class DebuggerConfig:
    num_joints: int = 18
    action_dim: int = 19  # 상한: 18 joint + 1 playback_speed. 실제는 로드된 모델 출력 차원(보통 18) 사용
    # UI 갱신 주기(Hz). 액션 계산 주기와 분리.
    update_hz: int = 30
    buffer_size: int = 200
    plot_update_hz: int = 30
    # 인퍼런스 최대 수행 시간 (초) - 0이면 무제한
    infer_duration_s: float = 5.0


ALLEX_ACTION_JOINT_NAMES: List[str] = [
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


@dataclass(frozen=True)
class ObsSpec:
    name: str
    start: int
    dim: int
    plot_enabled: bool
    streaming: bool = False

    @property
    def end(self) -> int:
        return self.start + self.dim - 1


# 학습과 동일: last_action = action_manager.action = 처리된 관절 위치 18차원만 (playback_speed 제외)
# PolicyCfg.history_length=3 → last_actions = 18*3 = 54차원
# ProprioObsCfg: joint_pos(18) + reference_joint_pos(18) + right_hand_joint_torque(19) + right_hand_base_pos(7)
# 총 차원: 54 + 18 + 18 + 19 + 7 = 116
OBS_SPECS: List[ObsSpec] = [
    ObsSpec("last_actions", 0, 54, True, False),        # 18*3 (policy group, history_length=3)
    ObsSpec("joint_pos", 54, 18, True, True),           # 54~71
    ObsSpec("reference_joint_pos", 72, 18, True, False),  # 72~89 (학습과 동일, ref 궤적 목표 위치)
    ObsSpec("right_hand_joint_torque", 90, 19, True, True),     # 90~108
    ObsSpec("right_hand_base_pos", 109, 7, True, True),         # 109~115
]

EXPECTED_TOTAL_OBS_DIM = max(spec.start + spec.dim for spec in OBS_SPECS)  # 116
STREAMING_NAMES = {spec.name for spec in OBS_SPECS if spec.streaming}

# ALLEX 로봇의 초기 자세 (init state) - inference가 안 돌아갈 때 이 값을 publish
# ALLEX_ACTION_JOINT_NAMES 순서와 동일하게 정의됨
ALLEX_INIT_POSE: List[float] = [
    0.0,        # R_Shoulder_Pitch_Joint
    0.0,        # R_Shoulder_Roll_Joint
    0.0,        # R_Shoulder_Yaw_Joint
    -1.5708,    # R_Elbow_Joint
    3.49066,    # R_Wrist_Yaw_Joint
    0.0,        # R_Wrist_Roll_Joint
    0.0,        # R_Wrist_Pitch_Joint
    0.0,        # R_Thumb_Yaw_Joint
    0.872665,   # R_Thumb_CMC_Joint
    0.349066,   # R_Thumb_MCP_Joint
    0.0,        # R_Index_MCP_Joint
    0.0,        # R_Index_PIP_Joint
    0.0,        # R_Middle_MCP_Joint
    0.0,        # R_Middle_PIP_Joint
    0.0,        # R_Ring_MCP_Joint
    0.0,        # R_Ring_PIP_Joint
    0.0,        # R_Little_MCP_Joint
    0.0,        # R_Little_PIP_Joint
]

__all__ = [
    "ObsDict",
    "ObsProvider",
    "DebuggerConfig",
    "ObsSpec",
    "OBS_SPECS",
    "EXPECTED_TOTAL_OBS_DIM",
    "STREAMING_NAMES",
    "ALLEX_ACTION_JOINT_NAMES",
    "ALLEX_INIT_POSE",
    "OBS_TIMEOUT_SEC",
    "WINDOW_TITLE",
    "WINDOW_WIDTH",
    "WINDOW_HEIGHT",
    "STYLE_GROUP_BOX",
    "STYLE_STATUS_OK",
    "STYLE_STATUS_ERR",
    "STYLE_STATUS_IDLE",
    "PLOT_INDEX_MAX",
    "PLOT_COLOR_MAP",
    "PLOT_COLOR_DEFAULT",
    "PLOT_CURVE_WIDTH",
]
