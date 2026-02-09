"""
ROS2 통신 관련 설정
"""

# ========================================
# 🎯 ROS2 토픽 설정
# ========================================
class ROS2Topics:
    """ROS2 토픽 이름들"""
    JOINT_COMMAND = '/joint_command'
    ROBOT_STATE = '/robot_state'
    # Isaac Sim → Sim2Real Debugger 관절 관측 토픽
    JOINT_OBS = '/observation/joint_pos'
    # Isaac Sim → Sim2Real Debugger 오른손 손가락 토크 관측 토픽
    RIGHT_HAND_JOINT_TORQUE = '/observation/right_hand_joint_torque'
    # Isaac Sim → Sim2Real Debugger Right_Hand_base pose (Origin_Body 기준) 토픽
    RIGHT_HAND_BASE_POS = '/observation/right_hand_base_pos'
    

# ========================================
# 🔧 ROS2 QoS 설정  
# ========================================
class ROS2QoS:
    """ROS2 QoS 설정"""
    HISTORY_DEPTH = 10
    # QoS 정책들은 런타임에 import해서 사용


# ========================================
# ⚙️ ROS2 전반적인 설정
# ========================================
class ROS2Config:
    """ROS2 통신 전반적인 설정"""
    # 노드 이름
    NODE_NAME = 'isaac_sim_integrated_ros2'
    
    # Extension 이름
    BRIDGE_EXTENSION = "isaacsim.ros2.bridge"
    
    # 타임아웃 설정
    INIT_TIMEOUT = 5.0
    SHUTDOWN_TIMEOUT = 1.0
    
    # 스레드 설정
    THREAD_DAEMON = True
    EXECUTOR_TIMEOUT = 0.1

    # 🆕 토픽 모드 설정
    TOPIC_MODE_CURRENT = 'joint_positions_deg'
    TOPIC_MODE_DESIRED = 'joint_ang_target_deg'
    TOPIC_MODE_TORQUE = 'joint_torque'
    
    # 🆕 기본 토픽 모드 (기본적으로 positions 모드로 시작)
    DEFAULT_TOPIC_MODE = TOPIC_MODE_CURRENT
    
    # 🆕 토픽 suffix 매핑
    TOPIC_SUFFIXES = {
        TOPIC_MODE_CURRENT: 'joint_positions_deg',
        TOPIC_MODE_DESIRED: 'joint_ang_target_deg',
        TOPIC_MODE_TORQUE: 'joint_torque'
    }
    
    # 🆕 토픽 모드 표시명 (UI에서 사용)
    TOPIC_MODE_DISPLAY_NAMES = {
        TOPIC_MODE_CURRENT: 'Current',
        TOPIC_MODE_DESIRED: 'Desired'
    }
    # 통합 데이터 크기
    UNIFIED_DATA_MIN_SIZE = 55  # 19개 torque + 36개 force
    TORQUE_DATA_SIZE = 19       # 토크 데이터 개수
    FORCE_DATA_SIZE = 12        # Force 벡터 개수 (12개 * 3축 = 36개)
    FORCE_VECTOR_SIZE = 3       # 각 Force 벡터 크기 (x, y, z)

    # 오른손(R-Hand) 관절 인덱스 (joint_config.json 참고)
    R_HAND_JOINT_INDICES = [24, 25, 26, 27, 28, 34, 35, 36, 37, 38, 44, 45, 46, 47, 48]

    # 🆕 왼손(L-Hand) 관절 인덱스 (joint_config.json 참고)
    L_HAND_JOINT_INDICES = [19, 20, 21, 22, 23, 29, 30, 31, 32, 33, 39, 40, 41, 42, 43]

    # 🆕 오른팔(R-Arm) 관절 인덱스 (joint_config.json 참고)
    R_ARM_JOINT_INDICES = [4, 7, 10, 12, 14, 16, 18]  # RSP, RSR, RSY, REP, RWY, RWR, RWP

    # 🆕 왼팔(L-Arm) 관절 인덱스 (joint_config.json 참고)  
    L_ARM_JOINT_INDICES = [3, 5, 8, 11, 13, 15, 17]   # LSP, LSR, LSY, LEP, LWY, LWR, LWP

    # 🆕 허리(Waist) 관절 인덱스 (joint_config.json 참고)
    WAIST_JOINT_INDICES = [0, 1]  # WY, WP

    # 🆕 목(Neck) 관절 인덱스 (joint_config.json 참고)
    NECK_JOINT_INDICES = [6, 9]  # NP, NY

    # 🆕 관절 그룹별 이름 매핑
    R_HAND_JOINT_NAMES = [
        "R11", "R21", "R31", "R41", "R51",  # 첫 번째 마디 (엄지, 검지, 중지, 약지, 새끼)
        "R12", "R22", "R32", "R42", "R52",  # 두 번째 마디
        "R13", "R23", "R33", "R43", "R53"   # 세 번째 마디
    ]
    
    L_HAND_JOINT_NAMES = [
        "L11", "L21", "L31", "L41", "L51",  # 첫 번째 마디 (엄지, 검지, 중지, 약지, 새끼)
        "L12", "L22", "L32", "L42", "L52",  # 두 번째 마디
        "L13", "L23", "L33", "L43", "L53"   # 세 번째 마디
    ]
    
    R_ARM_JOINT_NAMES = ["RSP", "RSR", "RSY", "REP", "RWY", "RWR", "RWP"]
    L_ARM_JOINT_NAMES = ["LSP", "LSR", "LSY", "LEP", "LWY", "LWR", "LWP"]
    WAIST_JOINT_NAMES = ["WY", "WP"]
    NECK_JOINT_NAMES = ["NP", "NY"]    

    # ==================================================================================
    # Sim2Real Debugger와 동일한 순서의 18개 ALLEX ACTION 조인트 이름 (USD joint name)
    # ==================================================================================
    ALLEX_ACTION_JOINT_FULL_NAMES = [
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

    # 오른팔+오른손 토크 관측용 USD 조인트 이름 (19개: 어깨3 + 팔꿈치1 + 손가락15)
    # dexblind_allex_env_cfg.py의 right_arm_hand_joint_names와 동일한 순서
    RIGHT_HAND_TORQUE_JOINT_NAMES = [
        # 어깨 관절 (3)
        "R_Shoulder_Pitch_Joint",
        "R_Shoulder_Roll_Joint",
        "R_Shoulder_Yaw_Joint",
        # 팔꿈치 관절 (1)
        "R_Elbow_Joint",
        # 엄지 관절 (3)
        "R_Thumb_Yaw_Joint",
        "R_Thumb_CMC_Joint",
        "R_Thumb_MCP_Joint",
        # 검지 관절 (3) - Roll 포함
        "R_Index_Roll_Joint",
        "R_Index_MCP_Joint",
        "R_Index_PIP_Joint",
        # 중지 관절 (3) - Roll 포함
        "R_Middle_Roll_Joint",
        "R_Middle_MCP_Joint",
        "R_Middle_PIP_Joint",
        # 약지 관절 (3) - Roll 포함
        "R_Ring_Roll_Joint",
        "R_Ring_MCP_Joint",
        "R_Ring_PIP_Joint",
        # 소지 관절 (3) - Roll 포함
        "R_Little_Roll_Joint",
        "R_Little_MCP_Joint",
        "R_Little_PIP_Joint",
    ]

    # 🆕 전체 관절 그룹 인덱스 매핑
    ALL_JOINT_INDICES = {
        'right_hand': R_HAND_JOINT_INDICES,
        'left_hand': L_HAND_JOINT_INDICES,
        'right_arm': R_ARM_JOINT_INDICES,
        'left_arm': L_ARM_JOINT_INDICES,
        'waist': WAIST_JOINT_INDICES,
        'neck': NECK_JOINT_INDICES
    }

    # 🆕 전체 관절 그룹 이름 매핑
    ALL_JOINT_NAMES = {
        'right_hand': R_HAND_JOINT_NAMES,
        'left_hand': L_HAND_JOINT_NAMES,
        'right_arm': R_ARM_JOINT_NAMES,
        'left_arm': L_ARM_JOINT_NAMES,
        'waist': WAIST_JOINT_NAMES,
        'neck': NECK_JOINT_NAMES
    }

    # 🆕 새로운 14개 토픽 관절 매핑
    OUTBOUND_TOPIC_TO_JOINTS = {
        # 오른팔 및 오른손
        "Arm_R_theOne": ["RSP", "RSR", "RSY", "REP", "RWY", "RWR", "RWP"],  # 7개 관절
        "Hand_R_thumb_wir": ["R11", "R12", "R13"],
        "Hand_R_index_wir": ["R21", "R22", "R23"],
        "Hand_R_middle_wir": ["R31", "R32", "R33"],
        "Hand_R_ring_wir": ["R41", "R42", "R43"],
        "Hand_R_little_wir": ["R51", "R52", "R53"],
        
        # 왼팔 및 왼손
        "Arm_L_theOne": ["LSP", "LSR", "LSY", "LEP", "LWY", "LWR", "LWP"],  # 7개 관절
        "Hand_L_thumb_wir": ["L11", "L12", "L13"],
        "Hand_L_index_wir": ["L21", "L22", "L23"],
        "Hand_L_middle_wir": ["L31", "L32", "L33"],
        "Hand_L_ring_wir": ["L41", "L42", "L43"],
        "Hand_L_little_wir": ["L51", "L52", "L53"],
        
        # 허리 및 목
        "theOne_waist": ["WY", "WP"],  
        "theOne_neck": ["NP", "NY"]  # 2개 관절
    }

    # 🆕 토크 시각화 가능한 관절 그룹들
    TORQUE_ENABLED_GROUPS = [
        "Arm_R_theOne",    # 오른팔 (7개 관절: RSP, RSR, RSY, REP, RWY, RWR, RWP)
        "Arm_L_theOne",    # 왼팔 (7개 관절: LSP, LSR, LSY, LEP, LWY, LWR, LWP)  
        "theOne_waist",    # 허리 (2개 관절: WY, WP)
        "theOne_neck"      # 목 (2개 관절: NP, NY)
    ]

    # 🆕 토픽 생성을 위한 헬퍼 메서드들
    @classmethod
    def get_outbound_topics_by_mode(cls, topic_mode):
        """특정 모드에 맞는 14개 outbound 토픽 리스트 반환
        
        Args:
            topic_mode (str): 토픽 모드 ('joint_positions_deg' 또는 'joint_ang_target_deg')
            
        Returns:
            dict: {topic_name: {'joint_names': [...], 'group_name': '...'}} 형태
        """
        if topic_mode not in cls.TOPIC_SUFFIXES:
            raise ValueError(f"Invalid topic mode: {topic_mode}")
        
        suffix = cls.TOPIC_SUFFIXES[topic_mode]
        outbound_topics = {}
        
        for group_name, joint_names in cls.OUTBOUND_TOPIC_TO_JOINTS.items():
            topic = f"/robot_outbound_data/{group_name}/{suffix}"
            outbound_topics[topic] = {
                'joint_names': joint_names,
                'group_name': group_name
            }
        
        return outbound_topics
    
    @classmethod
    def get_available_topic_modes(cls):
        """사용 가능한 토픽 모드 리스트 반환"""
        return list(cls.TOPIC_SUFFIXES.keys())
    
    @classmethod
    def is_valid_topic_mode(cls, topic_mode):
        """토픽 모드 유효성 확인"""
        return topic_mode in cls.TOPIC_SUFFIXES
    
    @classmethod
    def get_topic_mode_display_name(cls, topic_mode):
        """토픽 모드의 표시명 반환 (UI용)"""
        return cls.TOPIC_MODE_DISPLAY_NAMES.get(topic_mode, topic_mode)

    @classmethod  
    def get_torque_topics(cls):
        """🔧 토크 시각화가 가능한 4개 토픽만 반환 (손가락 제외)
        
        Returns:
            dict: {topic_name: {'joint_names': [...], 'group_name': '...'}} 형태
        """
        # 전체 토크 토픽에서 토크 시각화 가능한 그룹들만 필터링
        all_torque_topics = cls.get_outbound_topics_by_mode(cls.TOPIC_MODE_TORQUE)
        filtered_topics = {}
        
        for topic_name, topic_info in all_torque_topics.items():
            group_name = topic_info['group_name']
            if group_name in cls.TORQUE_ENABLED_GROUPS:
                filtered_topics[topic_name] = topic_info
        
        return filtered_topics

    @classmethod
    def get_torque_topic_list(cls):
        """🔧 토크 시각화가 가능한 4개 토픽명만 리스트로 반환 (구독용)
        
        Returns:
            list: 토크 토픽명 리스트 (손가락 제외)
        """
        torque_topics = cls.get_torque_topics()
        return list(torque_topics.keys())