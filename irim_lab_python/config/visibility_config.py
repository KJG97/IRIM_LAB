# ========================================
# 👁️ Visibility 설정
# ========================================
class VisibilityConfig:
    """Visibility 제어 관련 설정"""
    # Hand Force Visualization Prim 경로들
    HAND_FORCE_PRIMS = [
        "/ALLEX/left_hand/L_TCP_force",
        "/ALLEX/left_thumb_14/L14_force", 
        "/ALLEX/left_finger_24/L24_force",
        "/ALLEX/left_finger_34/L34_force",
        "/ALLEX/left_finger_44/L44_force",
        "/ALLEX/left_finger_54/L54_force",
        "/ALLEX/right_hand/R_TCP_force",
        "/ALLEX/right_thumb_14/R14_force",
        "/ALLEX/right_finger_24/R24_force", 
        "/ALLEX/right_finger_34/R34_force",
        "/ALLEX/right_finger_44/R44_force",
        "/ALLEX/right_finger_54/R54_force"
    ]

    # 🆕 다중 Joint Torque 프림 매핑
    JOINT_TORQUE_PRIMS = [
    "/ALLEX/waist_yaw/WY_torque",
    "/ALLEX/waist_pitch/WP_torque",  
    "/ALLEX/left_shoulder_pitch/LSP_torque",  
    "/ALLEX/right_shoulder_pitch/RSP_torque",
    "/ALLEX/left_shoulder_roll/LSR_torque",
    "/ALLEX/neck_pitch/NP_torque",
    "/ALLEX/right_shoulder_roll/RSR_torque",
    "/ALLEX/left_shoulder_yaw/LSY_torque",
    "/ALLEX/head/NY_torque",
    "/ALLEX/right_shoulder_yaw/RSY_torque",
    "/ALLEX/left_elbow/LEP_torque",
    "/ALLEX/right_elbow/REP_torque",
    "/ALLEX/left_forearm/LWY_torque",
    "/ALLEX/right_forearm/RWY_torque",
    "/ALLEX/left_wrist_roll/LWR_torque",
    "/ALLEX/right_wrist_roll/RWR_torque",
    "/ALLEX/left_hand/LWP_torque",
    "/ALLEX/right_hand/RWP_torque"
    ]

    TABLE_CAN_PRIMS = [
        "/ALLEX/Table",
        "/ALLEX/Can"
    ]

    TABLE_CAN_COLLISION_PRIMS = [
        "/ALLEX/Table/collisions/Cube",
        "/ALLEX/Can/collision"
    ]
    
    # 기본 상태
    HAND_FORCE_DEFAULT_VISIBLE = True