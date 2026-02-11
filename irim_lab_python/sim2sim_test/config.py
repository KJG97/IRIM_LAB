"""
Sim2Sim 설정 통합
- 카메라/시뮬레이션/UI/시나리오 표시/오버레이/ROS2 를 클래스별로 분리하여 관리.
"""

# -----------------------------------------------------------------------------
# CameraConfig — 뷰포트·캐릭터 기본값
# -----------------------------------------------------------------------------
class CameraConfig:
    """캐릭·총 관절 수 등 시나리오 공통 상수."""
    TOTAL_JOINTS = 59
    DEFAULT_CAMERA_EYE = [1.6, 0.3, 1.6]
    DEFAULT_CAMERA_TARGET = [0, -0.1, 1.1]
    DEFAULT_CAMERA_PRIM_PATH = "/OmniverseKit_Persp"


# -----------------------------------------------------------------------------
# JointConfig — 관절 제어
# -----------------------------------------------------------------------------
class JointConfig:
    """관절 이름·핸드 인덱스·초기화 제한."""
    JOINT_NAME_TEMPLATE = "joint{}"
    MAX_INITIALIZATION_ATTEMPTS = 5
    HAND_JOINT_START, HAND_JOINT_END = 19, 49
    HAND_JOINT_INDICES = tuple(range(HAND_JOINT_START, HAND_JOINT_END))


# -----------------------------------------------------------------------------
# SimulationConfig — 물리·렌더링 (UIConfig에서 분리)
# -----------------------------------------------------------------------------
class SimulationConfig:
    """시뮬레이션 타임스텝·중력 등 월드 설정."""
    SIMULATION_HZ = 50
    PHYSICS_DT = 1.0 / 200
    RENDERING_DT = 1.0 / SIMULATION_HZ
    GRAVITY = [0, 0, -9.81]


# -----------------------------------------------------------------------------
# UIColors — 색상 기준 명명 (0xAABBGGRR)
# -----------------------------------------------------------------------------
class UIColors:
    """UI 색상. 변수명으로 색을 구분할 수 있도록 정의."""
    BACKGROUND = 0xFF23211F      # 어두운 배경
    TEXT_PRIMARY = 0xFFFFFFFF    # 흰색 텍스트

    # 초록 (Green)
    GREEN_BG = 0xFF0D4F1C
    GREEN_BORDER = 0xFF2ECC40
    GREEN_HOVER = 0xFF1A6B2A

    # 파랑 (Blue)
    BLUE_BG = 0xFF0D0D4F
    BLUE_BORDER = 0xFF2E2ECC
    BLUE_HOVER = 0xFF1A1A6B

    # 주황 (Orange)
    ORANGE_BG = 0xFF007ACC
    ORANGE_BORDER = 0xFF00CCFF
    ORANGE_HOVER = 0xFF0099E6

    # 호박색 (Amber)
    AMBER_BG = 0xFF7E231E
    AMBER_BORDER = 0xFFD27619
    AMBER_HOVER = 0xFFC06515

    # 자주/마젠타 (Magenta)
    MAGENTA_BG = 0xFFCC007A
    MAGENTA_BORDER = 0xFFE60099
    MAGENTA_HOVER = 0xFFB30066

    # 회색 (Gray)
    GRAY_BG = 0xFF808080
    GRAY_BORDER = 0xFF606060
    GRAY_HOVER = 0xFF909090

    # 사이드바 (색상 재사용)
    SIDEBAR_GREEN = GREEN_BORDER
    SIDEBAR_BLUE = BLUE_BORDER


# -----------------------------------------------------------------------------
# UILayout — 간격·크기·테두리
# -----------------------------------------------------------------------------
class UILayout:
    """UI 요소 크기·간격·테두리."""
    BUTTON_HEIGHT = 30
    BUTTON_HEIGHT_SMALL = 25
    BUTTON_HEIGHT_LARGE = 35
    LABEL_HEIGHT = 25
    SEPARATOR_HEIGHT = 5
    SPACING_SMALL = 5
    SPACING_MEDIUM = 8
    SIDEBAR_WIDTH = 4

    LABEL_WIDTH_MEDIUM = 80
    LABEL_WIDTH_LARGE = 150

    BUTTON_BORDER_WIDTH = 1
    BUTTON_BORDER_WIDTH_THICK = 2
    BUTTON_BORDER_RADIUS = 2
    BUTTON_BORDER_RADIUS_LARGE = 3
    BUTTON_BORDER_RADIUS_XLARGE = 4
    BUTTON_MARGIN = 0
    BUTTON_PADDING = 5


# -----------------------------------------------------------------------------
# UIConfig — UI 초기 상태 (접힘 등)
# -----------------------------------------------------------------------------
class UIConfig:
    """UI 패널 초기 접힘·표시 상태."""
    WORLD_CONTROLS_COLLAPSED = True
    JOINT_CONTROL_COLLAPSED = True


# -----------------------------------------------------------------------------
# ScenarioDisplayConfig — 시나리오·오버레이 텍스트 표시
# -----------------------------------------------------------------------------
class ScenarioDisplayConfig:
    """오버레이 갱신 주기, 관절 라벨, 구분선 길이 등."""
    TEXT_UPDATE_INTERVAL = 15
    POLICY_ACTION_JOINT_COUNT = 18
    HAND_JOINT_DISPLAY_LIMIT = 15
    OVERLAY_LINE_WIDTH = 20
    OVERLAY_SEP_WIDTH = 18

    BODY_JOINT_LABELS = {"waist": ["WY", "WP", "CP"], "neck": ["NP", "NY"]}
    ARM_JOINT_LABELS = {
        "left_arm": ["LSP", "LSR", "LSY", "LEP", "LWY", "LWR", "LWP"],
        "right_arm": ["RSP", "RSR", "RSY", "REP", "RWY", "RWR", "RWP"],
    }
    FINGER_GROUPS = [
        ("Thumb", "TH", [0, 5, 10]),
        ("Index", "IN", [1, 6, 11]),
        ("Middle", "MI", [2, 7, 12]),
        ("Ring", "RI", [3, 8, 13]),
        ("Pinky", "PI", [4, 9, 14]),
    ]


# -----------------------------------------------------------------------------
# OverlayConfig — 오버레이 윈도우·체크박스 UI
# -----------------------------------------------------------------------------
class OverlayConfig:
    """관절/손 오버레이 창 크기·위치·스타일, 체크박스 스타일."""
    # 체크박스
    CHECKBOX_BORDER_RADIUS = 3
    CHECKBOX_WIDTH = 20
    CHECKBOX_LABEL_FONT_SIZE = 14
    CHECKBOX_LABEL_WIDTH_OFFSET = 25
    CHECKBOX_GROUP_ITEM_WIDTH = 100

    # 오버레이 공통
    BORDER_RADIUS = 8
    BORDER_WIDTH = 1
    VSTACK_SPACING = 3
    SPACER_HEIGHT = 5
    FONT_SIZE = 16
    LABEL_MARGIN = 8

    # 관절(본체) 오버레이
    JOINT_OVERLAY_WIDTH = 175
    JOINT_OVERLAY_HEIGHT = 525
    JOINT_OVERLAY_POSITION_X = 15
    JOINT_OVERLAY_POSITION_Y = 75
    JOINT_OVERLAY_BORDER_COLOR = 0xFF00AAFF

    # 손 관절 오버레이
    HAND_OVERLAY_WIDTH = 175
    HAND_OVERLAY_HEIGHT = 810
    HAND_OVERLAY_POSITION_X = 190
    HAND_OVERLAY_POSITION_Y = 75
    HAND_OVERLAY_BORDER_COLOR = 0xFFFF6600


# -----------------------------------------------------------------------------
# ROS2Topics — 토픽 이름
# -----------------------------------------------------------------------------
class ROS2Topics:
    """ROS2 토픽 경로."""
    JOINT_COMMAND = "/joint_command"
    JOINT_OBS = "/observation/joint_pos"
    RIGHT_HAND_JOINT_TORQUE = "/observation/right_hand_joint_torque"
    RIGHT_HAND_BASE_POS = "/observation/right_hand_base_pos"


# -----------------------------------------------------------------------------
# ROS2QoS — QoS
# -----------------------------------------------------------------------------
class ROS2QoS:
    """ROS2 QoS 프로파일."""
    HISTORY_DEPTH = 10


# -----------------------------------------------------------------------------
# ROS2Config — 노드·토픽 모드·조인트 매핑
# -----------------------------------------------------------------------------
class ROS2Config:
    """ROS2 노드, 브리지, 토픽 모드, 조인트 이름 매핑."""
    NODE_NAME = "isaac_sim_integrated_ros2"
    BRIDGE_EXTENSION = "isaacsim.ros2.bridge"
    THREAD_DAEMON = True

    TOPIC_MODE_CURRENT = "joint_positions_deg"
    TOPIC_MODE_DESIRED = "joint_ang_target_deg"
    TOPIC_MODE_TORQUE = "joint_torque"
    DEFAULT_TOPIC_MODE = TOPIC_MODE_CURRENT
    TOPIC_SUFFIXES = {
        TOPIC_MODE_CURRENT: TOPIC_MODE_CURRENT,
        TOPIC_MODE_DESIRED: TOPIC_MODE_DESIRED,
        TOPIC_MODE_TORQUE: TOPIC_MODE_TORQUE,
    }
    TOPIC_MODE_DISPLAY_NAMES = {
        TOPIC_MODE_CURRENT: "Current",
        TOPIC_MODE_DESIRED: "Desired",
    }

    OUTBOUND_TOPIC_TO_JOINTS = {
        "Arm_R_theOne": ["RSP", "RSR", "RSY", "REP", "RWY", "RWR", "RWP"],
        "Hand_R_thumb_wir": ["R11", "R12", "R13"],
        "Hand_R_index_wir": ["R21", "R22", "R23"],
        "Hand_R_middle_wir": ["R31", "R32", "R33"],
        "Hand_R_ring_wir": ["R41", "R42", "R43"],
        "Hand_R_little_wir": ["R51", "R52", "R53"],
        "Arm_L_theOne": ["LSP", "LSR", "LSY", "LEP", "LWY", "LWR", "LWP"],
        "Hand_L_thumb_wir": ["L11", "L12", "L13"],
        "Hand_L_index_wir": ["L21", "L22", "L23"],
        "Hand_L_middle_wir": ["L31", "L32", "L33"],
        "Hand_L_ring_wir": ["L41", "L42", "L43"],
        "Hand_L_little_wir": ["L51", "L52", "L53"],
        "theOne_waist": ["WY", "WP"],
        "theOne_neck": ["NP", "NY"],
    }

    ALLEX_ACTION_JOINT_FULL_NAMES = [
        "R_Shoulder_Pitch_Joint", "R_Shoulder_Roll_Joint", "R_Shoulder_Yaw_Joint",
        "R_Elbow_Joint", "R_Wrist_Yaw_Joint", "R_Wrist_Roll_Joint", "R_Wrist_Pitch_Joint",
        "R_Thumb_Yaw_Joint", "R_Thumb_CMC_Joint", "R_Thumb_MCP_Joint",
        "R_Index_MCP_Joint", "R_Index_PIP_Joint", "R_Middle_MCP_Joint", "R_Middle_PIP_Joint",
        "R_Ring_MCP_Joint", "R_Ring_PIP_Joint", "R_Little_MCP_Joint", "R_Little_PIP_Joint",
    ]
    RIGHT_HAND_TORQUE_JOINT_NAMES = [
        "R_Shoulder_Pitch_Joint", "R_Shoulder_Roll_Joint", "R_Shoulder_Yaw_Joint",
        "R_Elbow_Joint",
        "R_Thumb_Yaw_Joint", "R_Thumb_CMC_Joint", "R_Thumb_MCP_Joint",
        "R_Index_Roll_Joint", "R_Index_MCP_Joint", "R_Index_PIP_Joint",
        "R_Middle_Roll_Joint", "R_Middle_MCP_Joint", "R_Middle_PIP_Joint",
        "R_Ring_Roll_Joint", "R_Ring_MCP_Joint", "R_Ring_PIP_Joint",
        "R_Little_Roll_Joint", "R_Little_MCP_Joint", "R_Little_PIP_Joint",
    ]

    @classmethod
    def get_outbound_topics_by_mode(cls, topic_mode):
        if topic_mode not in cls.TOPIC_SUFFIXES:
            raise ValueError(f"Invalid topic mode: {topic_mode}")
        suffix = cls.TOPIC_SUFFIXES[topic_mode]
        return {
            f"/robot_outbound_data/{name}/{suffix}": {"joint_names": joints, "group_name": name}
            for name, joints in cls.OUTBOUND_TOPIC_TO_JOINTS.items()
        }

    @classmethod
    def get_available_topic_modes(cls):
        return list(cls.TOPIC_SUFFIXES.keys())

    @classmethod
    def is_valid_topic_mode(cls, topic_mode):
        return topic_mode in cls.TOPIC_SUFFIXES

    @classmethod
    def get_topic_mode_display_name(cls, topic_mode):
        return cls.TOPIC_MODE_DISPLAY_NAMES.get(topic_mode, topic_mode)

