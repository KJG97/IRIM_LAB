# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.timeline
from omni.usd import StageEventType

from .config import ROS2Config, JointConfig
from .ros2 import ROS2IntegratedManager
from .ui import WorldControlsBuilder, JointControlsBuilder
from .event_handlers import JointEventHandler, ROS2EventHandler, ScenarioEventHandler
from .scenario import ALLEXDigitalTwin
HAND_JOINT_INDICES = [
    19, 20, 21, 22, 23,   # L11~L51
    24, 25, 26, 27, 28,   # R11~R51
    29, 30, 31, 32, 33,   # L12~L52
    34, 35, 36, 37, 38,   # R12~R52
    39, 40, 41, 42, 43,   # L13~L53
    44, 45, 46, 47, 48    # R13~R53
]



class UIBuilder:

    # ========================================
    # 🏗️ 초기화 및 설정
    # ========================================
    def __init__(self):
        # UI 요소들 - UIElementWrapper 인스턴스로 생성된 UI 구성요소들
        self.wrapped_ui_elements = []

        # 타임라인 접근 - 정지/일시정지/재생을 프로그램으로 제어하기 위함
        self._timeline = omni.timeline.get_timeline_interface()


        self._hand_joint_indices = HAND_JOINT_INDICES
        self._effective_joints = len(self._hand_joint_indices)  # 30
        self._joint_values = [0.0] * self._effective_joints
        self._joint_names = [JointConfig.JOINT_NAME_TEMPLATE.format(i+1) for i in self._hand_joint_indices]

        # 🎯 ROS2 관리 변수
        self._publisher_enabled = False
        self._subscriber_enabled = False
        self._unified_subscriber_enabled = False
        self._latest_unified_data = {}
        self._topic_mode_status_label = None

        # 🆕 Articulation 초기화 추적 변수
        self._articulation_initialized = False
        self._physics_step_count = 0
        self._initialization_attempts = 0
        self._max_initialization_attempts = JointConfig.MAX_INITIALIZATION_ATTEMPTS

        # 예제 초기화 실행 - 제공된 예제의 초기 설정 수행
        self._on_init()

    def _on_init(self):
        self._articulation = None
        self._cuboid = None
        self._scenario = ALLEXDigitalTwin()

        # �� 시나리오에 UIBuilder 참조 설정
        self._scenario.set_ui_builder_ref(self)

        # 🆕 ROS2 관리자에 scenario 참조 설정
        self._ros2_manager = ROS2IntegratedManager(scenario_ref=self._scenario)

        self._scenario.set_ros2_manager(self._ros2_manager)

        self._ros2_manager.set_joint_control_reference(
            self._effective_joints, 
            self._joint_values
        )

        # 🆕 이벤트 핸들러들 초기화
        self._joint_handler = JointEventHandler(self)
        self._ros2_handler = ROS2EventHandler(self)
        self._scenario_handler = ScenarioEventHandler(self)

    def _try_delayed_initialization(self):
        """🆕 지연된 Articulation 초기화 시도"""
        self._initialization_attempts += 1        
        success = self._scenario.delayed_initialization()
        
        if success:
            self._articulation_initialized = True            
        else:
            print(f"⚠️ Articulation 초기화 실패 (시도 {self._initialization_attempts})")


    # ========================================
    # 🎯 ROS2 통신 관리
    # ========================================
    def _setup_ros2_manager(self):
        """🆕 ROS2 관리자 초기화 (이벤트 핸들러로 위임)"""

        if self._ros2_manager is None:
            self._ros2_manager = ROS2IntegratedManager(scenario_ref=self._scenario)
            self._scenario.set_ros2_manager(self._ros2_manager)
            
            # Joint control reference 재설정
            self._ros2_manager.set_joint_control_reference(
                self._effective_joints, 
                self._joint_values
            )

        success = self._ros2_handler.setup_ros2_manager()
        
        # 관절 제어 참조 재설정
        if success and self._ros2_manager:
            self._ros2_manager.set_joint_control_reference(
                self._effective_joints, 
                self._joint_values
            )
            print(f"🔧 Joint Position Initialized")
            
            # 🆕 초기 상태는 Subscriber 비활성화로 설정
            if self._scenario and self._scenario._joint_controller:
                self._scenario._joint_controller.set_ros2_subscriber_status(False)
            
            # 🆕 전체 상태 동기화
            self._ros2_handler._sync_all_ros2_status()
        
        return success

    def _toggle_topic_mode(self):
        """토픽 모드 토글"""
        if not self._ros2_manager or not self._ros2_manager.is_initialized():
            print("❌ ROS2 Manager not initialized!")
            return
        
        try:
            success, status_message = self._ros2_manager.toggle_topic_mode()
            
            if success:
                # 기존 업데이트
                current_mode = self._ros2_manager.get_current_topic_mode()
                mode_display = ROS2Config.get_topic_mode_display_name(current_mode)
                self._topic_mode_status_label.text = f"Control Mode: {mode_display}"
                
                # Scenario 캐시 업데이트
                if hasattr(self, '_scenario') and self._scenario:
                    self._scenario._update_cached_topic_mode()
                    
                    # 🆕 즉시 텍스트 색상 업데이트
                    if self._scenario._joint_data_label is not None:
                        self._scenario.update_joint_display_text()
                    if self._scenario._hand_data_label is not None:
                        self._scenario.update_hand_display_text()
                
                print(f"✅ {status_message}")
            else:
                print(f"❌ {status_message}")
                
        except Exception as e:
            print(f"❌ Topic mode toggle error: {e}")

    def _toggle_publisher(self):
        """🎛️ Publisher 토글 (이벤트 핸들러로 위임)"""
        return self._ros2_handler.toggle_publisher()

    def _toggle_subscriber(self):
        """🎛️ Subscriber 토글 (이벤트 핸들러로 위임)"""
        return self._ros2_handler.toggle_subscriber()

    def _toggle_joint_torque_subscriber(self):
        """🚀 Torque Subscriber 토글 (이벤트 핸들러로 위임)"""
        return self._ros2_handler.toggle_joint_torque_subscriber()

    def _cleanup_ros2(self):
        """🆕 ROS2 정리 (이벤트 핸들러로 위임)"""
        self._ros2_handler.cleanup_ros2()

    def _on_unified_robot_state_command(self, msg):
        """🚀 통합 Robot State 명령 처리 (Torque + Force 동시 처리)"""
        try:
            if len(msg.effort) >= ROS2Config.UNIFIED_DATA_MIN_SIZE:  # 19개 torque + 36개 force = 55개
                
                # 🔥 1. Joint Torque 데이터 추출 (처음 19개)
                torque_data = {}
                for i in range(ROS2Config.TORQUE_DATA_SIZE):
                    torque_data[i] = msg.effort[i]
                
                self._latest_torque_data = torque_data
                
                # 🎨 Scenario에 외부 토크 데이터 전달
                if self._scenario:
                    self._scenario.set_external_torque_data(torque_data)
                    # Torque 모드를 external로 전환
                    self._scenario.set_torque_source_mode('external')
                
                # 🌊 2. EE Force 데이터 추출 (20~55번째, 12개 * 3축)
                force_data = {}
                for i in range(ROS2Config.FORCE_DATA_SIZE):
                    start_idx = ROS2Config.TORQUE_DATA_SIZE + (i * ROS2Config.FORCE_VECTOR_SIZE)  # 19부터 시작
                    vector_x = msg.effort[start_idx + 0]
                    vector_y = msg.effort[start_idx + 1] 
                    vector_z = msg.effort[start_idx + 2]
                    
                    force_data[i] = {
                        'x': vector_x,
                        'y': vector_y,
                        'z': vector_z
                    }
                
                self._latest_force_data = force_data
                
                # 🎨 Scenario에 force 데이터 전달
                if self._scenario:
                    self._scenario.set_external_force_data(force_data)
          
                
            else:
                print(f"⚠️ Insufficient unified data: expected 55, got {len(msg.effort)}")
                
        except Exception as e:
            print(f"❌ Unified robot state processing error: {e}")
            import traceback
            traceback.print_exc()

    def _update_topic_mode_status(self):
        """토픽 모드 상태 라벨 업데이트 (외부 호출용)"""
        if hasattr(self, '_ros2_handler') and self._ros2_handler:
            self._ros2_handler._update_topic_mode_status()

    #  ========================================
    # 🎮 Isaac Sim 이벤트 콜백 (자동 호출)
    # ========================================
    def on_menu_callback(self):
        """
        툴바에서 UI가 열릴 때 호출되는 콜백 함수.
        build_ui() 함수 실행 후 직접 호출됨.
        """
        pass

    def on_timeline_event(self, event):
        """
        타임라인 이벤트 (재생, 일시정지, 정지)에 대한 콜백 함수.
        """
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            if hasattr(self, "_scenario_state_btn") and self._scenario_state_btn:
                self._scenario_state_btn.reset()
                self._scenario_state_btn.enabled = False

    def on_physics_step(self, step: float):
        """
        물리 시뮬레이션 단계에 대한 콜백 함수.
        Physics Step은 타임라인이 재생 중일 때만 발생함.

        Args:
            step (float): Size of physics step
        """
        pass

    def on_stage_event(self, event):
        """
        스테이지 이벤트에 대한 콜백 함수.

        Args:
            event (omni.usd.StageEventType): 이벤트 타입
        """
        if event.type == int(StageEventType.OPENED):
            # If the user opens a new stage, the extension should completely reset
            self._reset_extension()

    def cleanup(self):
        # UI 요소 정리
        for ui_elem in self.wrapped_ui_elements:
            try:
                ui_elem.cleanup()
            except Exception:
                pass

        # 리스트 비우기 및 강한 참조 제거 (누수 차단)
        self.wrapped_ui_elements.clear()
        if hasattr(self, "_load_btn"):
            self._load_btn = None
        if hasattr(self, "_reset_btn"):
            self._reset_btn = None
        if hasattr(self, "_scenario_state_btn"):
            self._scenario_state_btn = None
        if hasattr(self, "_pub_status_label"):
            self._pub_status_label = None
        if hasattr(self, "_sub_status_label"):
            self._sub_status_label = None
        if hasattr(self, "_joint_torque_status_label"):
            self._joint_torque_status_label = None
        if hasattr(self, "_topic_mode_status_label"):
            self._topic_mode_status_label = None
        if hasattr(self, "_joint_group_checkboxes"):
            self._joint_group_checkboxes = None

        # ROS2 정리
        self._cleanup_ros2()

    def build_ui(self):
        """
        확장 기능 실행을 위한 맞춤형 UI 도구 구성.
        UI 창이 닫혔다가 다시 열릴 때마다 이 함수가 호출됨.
        """
        
        # 🟢 World Controls 섹션
        world_controls_builder = WorldControlsBuilder(self)
        world_controls_builder.build()
        
        # 🦾 Joint Controls 섹션
        joint_controls_builder = JointControlsBuilder(self)
        joint_controls_builder.build()
            
    # ========================================
    # 🦾 관절 제어
    # ======================================== 
    def _on_joint_slider_changed(self, joint_index, model, label):
        """🔄 슬라이더 값 변경 시 호출 (이벤트 핸들러로 위임)"""
        self._joint_handler.on_joint_slider_changed(joint_index, model, label)


    # ========================================
    # 🎬 시나리오 관리
    # ========================================
    def _add_light_to_stage(self):
        """새 스테이지에 구형 조명 생성 (이벤트 핸들러로 위임)"""
        self._scenario_handler.add_light_to_stage()

    def _setup_scene(self):
        """🔄 Scene 설정 - LOAD 버튼 콜백 (이벤트 핸들러로 위임)"""
        self._scenario_handler.setup_scene()

    def _setup_scenario(self):
        """🔄 Scenario 설정 (이벤트 핸들러로 위임)"""
        self._scenario_handler.setup_scenario()

    def _on_post_reset_btn(self):
        """🔄 Reset 버튼 콜백 (이벤트 핸들러로 위임)"""
        self._scenario_handler.on_post_reset_btn()

    def _update_scenario(self, step: float):
        """시나리오 업데이트 (이벤트 핸들러로 위임)"""
        self._scenario_handler.update_scenario(step)

    def _on_run_scenario_a_text(self):
        """🎬 시뮬레이션 시작 (이벤트 핸들러로 위임)"""
        self._scenario_handler.on_run_scenario_start()

    def _on_run_scenario_b_text(self):
        """⏸️ 시뮬레이션 일시정지 (이벤트 핸들러로 위임)"""
        self._scenario_handler.on_run_scenario_stop()

    def _reset_extension(self):
        """
        사용자가 self.on_stage_event()에서 새 스테이지를 열 때 호출됨.
        모든 상태가 리셋되어야 함.  
        """
        self._on_init()
        self._reset_ui()

    def _reset_ui(self):
        if hasattr(self, "_scenario_state_btn") and self._scenario_state_btn:
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False
        if hasattr(self, "_reset_btn") and self._reset_btn:
            self._reset_btn.enabled = False

