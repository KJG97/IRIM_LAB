"""
ALLEX Digital Twin 상수 정의
"""

# 관절 관련 상수
TOTAL_JOINTS = 59  # 전체 관절 수
EFFECTIVE_JOINTS = 49  # 실제 제어할 관절 수 (0~48)

# 기본 관절 위치
DEFAULT_JOINT_POSITIONS = [0.0] * TOTAL_JOINTS

# 카메라 기본 설정
DEFAULT_CAMERA_EYE = [1.6, 0.3, 1.6]
DEFAULT_CAMERA_TARGET = [0, -0.1, 1.1]
DEFAULT_CAMERA_PRIM_PATH = "/OmniverseKit_Persp"

# 센서 랩용 카메라 설정
SENSOR_LAB_CAMERA_EYE = [0.65, -0.57279, 1.02]  # 센서 테스트에 적합한 시야각
SENSOR_LAB_CAMERA_TARGET = [1.0, 10, 1.0]  # 센서 테스트 에셋 중심
SENSOR_LAB_CAMERA_PRIM_PATH = "/OmniverseKit_Persp"

# 로봇 설정
ROBOT_PRIM_PATH = "/ALLEX" 


# 🆕 Force 벡터 시각화 관련 상수
DEFAULT_FORCE_BASE_SCALE = 0.1  # Force 벡터 기본 스케일
DEFAULT_FORCE_SCALE_FACTOR = 0.1  # Force 크기당 스케일 증가량
DEFAULT_FORCE_THRESHOLD = 0.001  # Force 벡터 임계값

# 🆕 Hand Force 프림 매핑
HAND_FORCE_PRIMS = {
    # Hand Force Prim 경로들
    "L_TCP": "/ALLEX/left_hand/L_TCP_force",
    "L14": "/ALLEX/left_thumb_14/L14_force",
    "L24": "/ALLEX/left_finger_24/L24_force",
    "L34": "/ALLEX/left_finger_34/L34_force",
    "L44": "/ALLEX/left_finger_44/L44_force",
    "L54": "/ALLEX/left_finger_54/L54_force",
    "R_TCP": "/ALLEX/right_hand/R_TCP_force",
    "R14": "/ALLEX/right_thumb_14/R14_force",
    "R24": "/ALLEX/right_finger_24/R24_force",
    "R34": "/ALLEX/right_finger_34/R34_force",
    "R44": "/ALLEX/right_finger_44/R44_force",
    "R54": "/ALLEX/right_finger_54/R54_force"
}