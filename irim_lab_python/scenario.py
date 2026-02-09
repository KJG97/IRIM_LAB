# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""
ALLEX Digital Twin 메인 시나리오 클래스
"""

from .core import (
    ALLEXInitializer,
    ALLEXAssetManager, 
    ALLEXJointController,
    ALLEXSensorManager,
    ALLEXVisualization,
    ALLEXSimulationLoop
)

import threading

import omni.ui as ui
import numpy as np
from isaacsim.core.utils.xforms import get_world_pose
from isaacsim.core.utils.types import ArticulationAction


class ALLEXDigitalTwin:
    """ALLEX 디지털 트윈 메인 클래스 - 모든 모듈을 통합 관리"""

    def __init__(self):
        """클래스 초기화 - 모든 모듈 인스턴스 생성"""
        # 🏗️ 모듈 인스턴스 생성
        self._initializer = ALLEXInitializer()
        self._asset_manager = ALLEXAssetManager()
        self._joint_controller = ALLEXJointController()
        self._sensor_manager = ALLEXSensorManager()
        self._visualization = ALLEXVisualization()
        self._simulation_loop = ALLEXSimulationLoop()
        # PhysX API 호출을 직렬화하기 위한 락 (ROS2 콜백 스레드와 메인 스레드 충돌 방지)
        self._physx_lock = threading.Lock()
        # ROS/외부 스레드에서 받은 policy action을 메인 물리 루프에서만 적용하기 위한 버퍼
        self._pending_policy_action: np.ndarray | None = None
        

        # 🔗 참조 변수들
        self._articulation = None
        self._joint_controller.set_scenario_reference(self)



        # 🆕 Force 시각화 소스 관리 추가
        self._external_force_data = {}  # 외부에서 받은 Force 데이터 저장
        self._last_external_force_update = 0  # Force 데이터 마지막 업데이트 시간
        self._force_visualization_pending = False  # ⭐ 누락된 플래그 추가!

        self._rmpflow_controller = None
        self._follow_target_cube = None
        self._physics_update_callback = None
        self._ui_builder_ref = None  # UIBuilder 참조 추가

        # �� Follow Cube 관련 변수들 추가
        self._follow_cube_active = False
        self._right_target_cube = None   # 오른손용
        self._left_target_cube  = None   # 왼손용
        self._right_ik_controller = None
        self._left_ik_controller = None
        self._follow_cube_callback_registered = False
        self.right_hand_initial_position = None
        self.right_hand_initial_orientation = None

        self._text_overlay_window = None
        self._joint_data_label = None
        self._text_update_counter = 0

        # 🆕 손 관절 전용 오버레이 변수들 추가
        self._hand_overlay_window = None
        self._hand_data_label = None

        self._joint_group_display_enabled = {
            'body': False,        # Body (허리, 목)
            'left_arm': False,    # Left Arm
            'right_arm': False,   # Right Arm  
            'hand': False,       # Hand
        }
        
        self._group_checkboxes = {}
        self._cached_topic_mode = None 
        # ROS2 관측 퍼블리셔용 캐시
        self._obs_joint_indices = None
        self._obs_publish_acc_time = 0.0
        # 오른손 손가락 토크용 DOF 인덱스 캐시 (get_dof_projected_joint_forces 사용)
        self._right_hand_torque_dof_indices = None
        # 🆕 Policy action용 DOF 인덱스 캐시 (18개 조인트)
        self._policy_action_dof_indices = None

    def setup(self):
        """시나리오 초기 설정"""        
        # 카메라 뷰 설정
        self._initializer.setup_camera_view()
        
        # Coupled Joint 설정 로드
        # 확장 루트 경로를 동적으로 찾기
        import os
        current_file = os.path.abspath(__file__)
        extension_root = os.path.dirname(current_file)
        joint_config_path = os.path.join(extension_root, "joint_config.json")
        self._joint_controller.load_coupled_joint_config(joint_config_path)

        # 🆕 right_hand 경로 확인 (USD 구조 변경 대응)
        try:
            if self.right_hand_initial_position is None:
                # 여러 가능한 경로 시도
                possible_paths = [
                    "/ALLEX/right_hand",
                    "/ALLEX/Right_Hand_base",
                    "/ALLEX/Right_Hand"
                ]
                for path in possible_paths:
                    try:
                        self.right_hand_initial_position, self.right_hand_initial_orientation = get_world_pose(path)
                        print(f"✅ right_hand 경로 확인: {path}")
                        break
                    except:
                        continue
                else:
                    print("⚠️ right_hand 경로를 찾을 수 없습니다. 기본값 사용")
        except Exception as e:
            print(f"⚠️ right_hand 초기화 실패: {e}")
        
        self._target_joint_position_overlay()
        self._create_hand_joint_overlay()

        print("✅ ALLEX Digital Twin 설정 완료")

    def load_example_assets(self):
        """에셋 로딩 및 초기화"""        
        # 로봇 에셋 로딩
        self._articulation = self._asset_manager.load_robot_asset()
        
        if self._articulation is not None:
            # 🆕 Articulation 초기화 시도
            init_success = self._asset_manager.initialize_articulation()
            
            if init_success:
                # 관절 위치 초기화
                self._initializer.initialize_joint_positions(self._articulation)
                # 🆕 시뮬레이션 시작 전에 Policy action용 DOF 인덱스 캐싱
                self._cache_policy_action_dof_indices()

                def get_target_positions():
                    """ROS2 데이터 우선, 없으면 기본값 사용"""
                    try:
                        ros2_positions = self._joint_controller.get_unified_target_positions()
                        if ros2_positions and any(pos != 0.0 for pos in ros2_positions):
                            return ros2_positions
                    except Exception as e:
                        print(f"⚠️ ROS2 positions access failed: {e}")
                    return self._initializer.target_joint_positions
                    
                generator = self._joint_controller.create_joint_control_generator(
                    articulation=self._articulation,
                    get_target_positions_func=get_target_positions
                )
                self._simulation_loop.set_script_generator(generator)

            else:
                print("🔄 'RUN' 버튼을 누르면 Articulation이 초기화됩니다")
        
        return self._articulation

    def delayed_initialization(self):
        """🆕 지연된 초기화 - 시뮬레이션 시작 후 호출"""
        if self._articulation is None:
            print("⚠️ Articulation이 로딩되지 않았습니다")
            return False
            
        # Articulation 재초기화 시도
        init_success = self._asset_manager.initialize_articulation()
        
        
        if init_success:
            # 관절 위치 초기화
            self._initializer.initialize_joint_positions(self._articulation)
            # 시뮬레이션 루프에 제너레이터 설정
            self._initial_joint_positions = self._articulation.get_joint_positions()
            print(f"✅ 전체 관절 위치 저장 완료: {len(self._initial_joint_positions)}개 관절")

            def get_target_positions():
                """ROS2 데이터 우선, 없으면 기본값 사용"""
                try:
                    ros2_positions = self._joint_controller.get_unified_target_positions()
                    if ros2_positions and any(pos != 0.0 for pos in ros2_positions):
                        return ros2_positions
                except Exception as e:
                    print(f"⚠️ ROS2 positions access failed: {e}")
                return self._initializer.target_joint_positions

            generator = self._joint_controller.create_joint_control_generator(
                articulation=self._articulation,
                get_target_positions_func=get_target_positions
            )
            self._simulation_loop.set_script_generator(generator)
            print("✅ Articulation 초기화 완료")
            return True
        else:
            print("❌ Articulation 초기화 실패")
            return False

    def reset(self):
        """시스템 리셋"""        
        # 각 모듈 리셋
        self._initializer.reset(self._articulation)
        self._simulation_loop.reset()
        
        # 시뮬레이션 루프 재설정
        if self._articulation is not None:
            
            def get_target_positions():
                """ROS2 데이터 우선, 없으면 기본값 사용"""
                try:
                    ros2_positions = self._joint_controller.get_unified_target_positions()
                    if ros2_positions and any(pos != 0.0 for pos in ros2_positions):
                        return ros2_positions
                except Exception as e:
                    print(f"⚠️ ROS2 positions access failed: {e}")
                return self._initializer.target_joint_positions

            generator = self._joint_controller.create_joint_control_generator(
                articulation=self._articulation,
                get_target_positions_func=get_target_positions
            )
            self._simulation_loop.set_script_generator(generator)
        
        print("✅ 시스템 리셋 완료")

    def set_ui_builder_ref(self, ui_builder_ref):
        """UIBuilder 참조를 설정합니다."""
        self._ui_builder_ref = ui_builder_ref

    def set_external_force_data(self, force_data):
        """외부 Force 데이터 설정 (비동기 처리)"""
        import time
        
        self._external_force_data = force_data.copy()
        self._last_external_force_update = time.time()
        
        # 🚨 직접 시각화 호출하지 않고 플래그만 설정
        self._force_visualization_pending = True

    def _update_force_visualizations(self):
        """🎨 Force 시각화 업데이트 (리맵핑은 visualization에서 처리)"""
        try:
            
            from .utils.constants import HAND_FORCE_PRIMS
            
            # 🎯 Prim 매핑
            prim_mapping = {
                "L_TCP": 0, "L14": 1, "L24": 2, "L34": 3, "L44": 4, "L54": 5,
                "R_TCP": 6, "R14": 7, "R24": 8, "R34": 9, "R44": 10, "R54": 11
            }
            
            success_count = 0
            failed_count = 0
            
            for prim_name, force_index in prim_mapping.items():
                if force_index in self._external_force_data:
                    try:
                        prim_path = HAND_FORCE_PRIMS[prim_name]
                        
                        # 🆕 원본 force 벡터 그대로 사용 (리맵핑은 visualization에서)
                        fx = float(self._external_force_data[force_index]['x'])
                        fy = float(self._external_force_data[force_index]['y']) 
                        fz = float(self._external_force_data[force_index]['z'])
                        
                        
                        # 🎨 시각화 업데이트 (원본 데이터로, 리맵핑은 visualization에서 처리)
                        success = self._visualization.update_force_vector_visualization(
                            prim_path, fx, fy, fz
                        )
                        
                        if success:
                            success_count += 1
                        else:
                            failed_count += 1
                            print(f"   ❌ {prim_name} visualization failed")
                            
                    except Exception as e:
                        failed_count += 1
                        print(f"   ❌ Error processing {prim_name}: {e}")
                        
                else:
                    print(f"   ⚠️ No data for {prim_name} (index {force_index})")
                        
        except Exception as e:
            print(f"❌ Force visualization error: {e}")
            import traceback
            traceback.print_exc()


    def update(self, step: float):
        """시뮬레이션 스텝 업데이트 (메인 스레드에서 Force와 Torque 모두 처리)"""
        # 🎨 펜딩된 Force 시각화 처리 (메인 스레드에서)
        if hasattr(self, '_force_visualization_pending') and self._force_visualization_pending:
            try:
                self._update_force_visualizations()
                self._force_visualization_pending = False
            except Exception as e:
                print(f"❌ Force visualization in main thread failed: {e}")
        
        # 🆕 관절 데이터 텍스트 업데이트 (15프레임마다 = 약 4Hz)
        self._text_update_counter += 1
        if self._text_update_counter % 15 == 0:
            # 🔘 1️⃣ 기존 관절 윈도우 업데이트 (Body + Arm)
            if self._joint_data_label is not None:
                self.update_joint_display_text()
            
            # 🔘 2️⃣ 손 관절 윈도우 업데이트 (Hand)
            if self._hand_data_label is not None:
                self.update_hand_display_text()

        # 🆕 ROS2 관절 관측 (/observation/joint_pos) 및 오른손 베이스 pose 발행 - 메인 물리 루프 내부에서만 수행
        self._publish_joint_observation_if_needed(step)

        # 🆕 ROS/외부 스레드에서 받은 policy action을 메인 루프에서만 적용
        if self._pending_policy_action is not None:
            # 시뮬레이션이 running 상태일 때만 적용
            if self._simulation_loop.is_running():
                try:
                    with self._physx_lock:
                        self._articulation.apply_action(
                            ArticulationAction(
                                joint_positions=self._pending_policy_action,
                                joint_indices=np.array(self._policy_action_dof_indices, dtype=np.int32)
                            )
                        )
                except Exception as e:
                    print(f"❌ Failed to apply pending policy action in update: {e}")
                finally:
                    # 한 번 적용 후에는 비움 (latest-only 동작)
                    self._pending_policy_action = None

        return self._simulation_loop.update(step)

    def _publish_joint_observation_if_needed(self, step: float) -> None:
        """ROS2 Publisher가 켜져 있을 때, 50Hz로 관절 관측값, 오른손 토크, Right_Hand_base pose를 발행
        
        Note: 물리 시뮬레이션이 100Hz (sim.dt = 0.01s)로 돌아가지만, 관측은 50Hz (0.02s 주기)로 발행합니다.
        시간 누적 방식으로 구현되어 있어, step이 0.01s일 때 2번 호출되면 1번 발행됩니다.
        """
        # ROS2 Manager / Node 상태 확인
        ros2_manager = getattr(self, "_ros2_manager", None)
        if (
            ros2_manager is None
            or not ros2_manager.is_initialized()
            or not ros2_manager.is_publisher_enabled()
        ):
            return

        if self._articulation is None:
            return

        # 매 physics step(50Hz)마다 관측 발행

        try:
            from .config.ros2_config import ROS2Config
            from isaacsim.core.utils.xforms import get_world_pose
            import numpy as np

            # ======================================================
            # 1) 관절 위치 관측 (/observation/joint_pos, 18개)
            # ======================================================
            if self._obs_joint_indices is None:
                indices: list[int] = []
                for name in ROS2Config.ALLEX_ACTION_JOINT_FULL_NAMES:
                    idx = self._articulation.get_dof_index(name)
                    if idx is None:
                        print(f"⚠️ Joint not found for observation publishing: {name}")
                        indices = []
                        break
                    indices.append(idx)

                if not indices:
                    # 더 이상 시도하지 않음
                    self._obs_joint_indices = []
                    print("❌ Failed to resolve ALLEX action joint indices; joint observation disabled.")
                else:
                    self._obs_joint_indices = indices

            if self._obs_joint_indices:
                with self._physx_lock:
                    positions = self._articulation.get_joint_positions(
                        joint_indices=self._obs_joint_indices
                    )
                ros2_manager.publish_joint_observation(positions)

            # ======================================================
            # 2) 오른팔+오른손 토크 관측 (/observation/right_hand_joint_torque, 19개)
            # ======================================================
            # DOF 인덱스 캐시는 한 번만 계산
            if self._right_hand_torque_dof_indices is None:
                # 오른팔+오른손 토크 관측용 19개 조인트 이름 (USD)
                finger_names = ROS2Config.RIGHT_HAND_TORQUE_JOINT_NAMES
                torque_indices: list[int] = []
                for name in finger_names:
                    idx = self._articulation.get_dof_index(name)
                    if idx is None:
                        print(f"⚠️ Joint not found for right hand torque publishing: {name}")
                        torque_indices = []
                        break
                    torque_indices.append(idx)

                if not torque_indices:
                    self._right_hand_torque_dof_indices = []
                    print("❌ Failed to resolve right hand torque DOF indices; torque publishing disabled.")
                else:
                    self._right_hand_torque_dof_indices = torque_indices

            if self._right_hand_torque_dof_indices:
                try:
                    with self._physx_lock:
                        torques = self._articulation.get_measured_joint_efforts(
                            joint_indices=self._right_hand_torque_dof_indices
                        )
                    if torques is not None:
                        right_hand_torques = [float(v) for v in list(torques)]
                        ros2_manager.publish_right_hand_joint_torque(right_hand_torques)
                except Exception as te:
                    print(f"⚠️ Right hand joint torque publish error in scenario.update: {te}")

            # ======================================================
            # 3) Right_Hand_base pose (Origin_Body 기준) 관측
            #    토픽: /observation/right_hand_base_pos
            #    형식: [x, y, z, qw, qx, qy, qz]
            #    구현: numpy 기반 쿼터니언 연산 (IsaacLab 예제 로직 차용)
            # ======================================================
            try:
                # 월드 기준 pose 읽기
                # NOTE: 기존 "/ALLEX/Right_Hand_base" 대신 "/ALLEX/R_Hand_Pose" 사용
                right_pos_w, right_quat_w = get_world_pose("/ALLEX/R_Hand_Pose")
                origin_pos_w, origin_quat_w = get_world_pose("/ALLEX/Origin_Body")

                right_pos_w = np.asarray(right_pos_w, dtype=np.float32)
                right_quat_w = np.asarray(right_quat_w, dtype=np.float32)
                origin_pos_w = np.asarray(origin_pos_w, dtype=np.float32)
                origin_quat_w = np.asarray(origin_quat_w, dtype=np.float32)

                # IsaacSim 쿼터니언 포맷: [w, x, y, z]
                def quat_conjugate_np(q: np.ndarray) -> np.ndarray:
                    """단일 쿼터니언 [w,x,y,z]의 켤레 반환"""
                    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=q.dtype)

                def quat_mul_np(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
                    """두 단일 쿼터니언 [w,x,y,z]의 곱"""
                    w1, x1, y1, z1 = q1
                    w2, x2, y2, z2 = q2
                    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
                    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
                    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
                    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
                    return np.array([w, x, y, z], dtype=q1.dtype)

                def quat_apply_np(q: np.ndarray, v: np.ndarray) -> np.ndarray:
                    """쿼터니언 q로 3D 벡터 v 회전"""
                    # v를 순수 쿼터니언으로 만들고 q * v * q_conj 계산
                    v_q = np.array([0.0, v[0], v[1], v[2]], dtype=q.dtype)
                    q_conj = quat_conjugate_np(q)
                    return quat_mul_np(quat_mul_np(q, v_q), q_conj)[1:]

                # Origin_Body 기준 상대 위치 / 회전 계산
                origin_quat_conj = quat_conjugate_np(origin_quat_w)
                rel_pos = quat_apply_np(origin_quat_conj, right_pos_w - origin_pos_w)
                rel_quat = quat_mul_np(origin_quat_conj, right_quat_w)

                pose_7d = np.concatenate([rel_pos, rel_quat], axis=-1)
                ros2_manager.publish_right_hand_base_pos(pose_7d)
            except Exception as pe:
                print(f"⚠️ Right_Hand_base pose publish error in scenario.update: {pe}")

        except Exception as e:
            print(f"⚠️ Joint observation publish error in scenario.update: {e}")



    # ========================================
    # 🔗 Public API Methods
    # ========================================
    
    def get_current_joint_positions(self):
        """현재 관절 위치 반환"""
        return self._sensor_manager.get_joint_positions(self._articulation)


    def get_robot_info(self):
        """로봇 정보 반환"""
        return self._asset_manager.get_joint_info()

    def get_coupled_joints_info(self):
        """Coupled Joint 정보 반환"""
        return self._joint_controller.get_coupled_joints_info()

    def get_visualization_info(self):
        """시각화 설정 정보 반환"""
        return self._visualization.get_visualization_info()

    def is_simulation_running(self):
        """시뮬레이션 실행 상태 확인"""
        return self._simulation_loop.is_running()

    def stop_simulation(self):
        """시뮬레이션 중지"""
        self._simulation_loop.stop()




    # ========================================
    # 🛠️ Configuration Methods
    # ========================================
    
    def _target_joint_position_overlay(self):
        """Hello World 텍스트 오버레이 생성"""
        try:
            # 🎯 오버레이 윈도우 생성 
            self._text_overlay_window = ui.Window(
                "Text Overlay",
                width=175,
                height=525,
                flags=(
                    ui.WINDOW_FLAGS_NO_TITLE_BAR |      # 타이틀 바 제거
                    ui.WINDOW_FLAGS_NO_RESIZE |         # 크기 조절 비활성화
                    ui.WINDOW_FLAGS_NO_SCROLLBAR |      # 스크롤바 제거
                    ui.WINDOW_FLAGS_NO_BACKGROUND |     # 윈도우 배경 투명
                    ui.WINDOW_FLAGS_NO_MOVE             # 이동 비활성화
                ),
                # 화면 상단 중앙에 고정
                position_x=15,
                position_y=75
            )
            # 🆕 초기에는 숨김 (hand_overlay_window와 동일하게)
            self._text_overlay_window.visible = False
            
            # 윈도우 내용 구성
            with self._text_overlay_window.frame:
                with ui.ZStack():
                    # 🎨 배경 스타일 개선
                    ui.Rectangle(
                        style={
                            "border_radius": 8,
                            "border_width": 1,
                            "border_color": 0xFF00AAFF       # 🟠 주황색 테두리 (손 관절 구분용)
                        }
                    )
                    with ui.VStack(spacing=3):
                        ui.Spacer(height=5)
                        self._joint_data_label = ui.Label(
                            "Loading Joint Data...",
                            alignment=ui.Alignment.LEFT_TOP,  # 왼쪽 상단 정렬
                            style={
                                "color": 0xFFFFFFFF,        # 흰색 텍스트
                                "font_size": 16,            # 더 작은 폰트 (많은 데이터용)
                                "margin": 8,                # 여백
                                "word_wrap": False,         # 자동 줄바꿈
                            }
                        )
                        ui.Spacer(height=5)
            
            print("✅ Joint Data 텍스트 오버레이 생성 완료")
            
        except Exception as e:
            print(f"❌ 텍스트 오버레이 생성 실패: {e}")
            import traceback
            traceback.print_exc()
    
    def _create_hand_joint_overlay(self):
        """손 관절 전용 텍스트 오버레이 생성 (기존 윈도우 오른쪽에 배치)"""
        try:
            # 🎯 손 관절 전용 오버레이 윈도우 생성 
            self._hand_overlay_window = ui.Window(
                "Hand Joint Overlay",
                width=175,  # 손 관절 데이터용으로 조금 더 넓게
                height=810,
                flags=(
                    ui.WINDOW_FLAGS_NO_TITLE_BAR |      # 타이틀 바 제거
                    ui.WINDOW_FLAGS_NO_RESIZE |         # 크기 조절 비활성화
                    ui.WINDOW_FLAGS_NO_SCROLLBAR |      # 스크롤바 제거
                    ui.WINDOW_FLAGS_NO_BACKGROUND |     # 윈도우 배경 투명
                    ui.WINDOW_FLAGS_NO_MOVE             # 이동 비활성화
                ),
                # 🎯 기존 윈도우 오른쪽에 배치 (기존이 15이므로 220 위치)
                position_x=190,
                position_y=75
            )
            self._hand_overlay_window.visible = False

            # 윈도우 내용 구성
            with self._hand_overlay_window.frame:
                with ui.ZStack():
                    # 🎨 배경 스타일 (손 관절용으로 다른 색상)
                    ui.Rectangle(
                        style={
                            "border_radius": 8,
                            "border_width": 1,
                            "border_color": 0xFFFF6600      # 파란색 테두리 
                        }
                    )
                    with ui.VStack(spacing=3):
                        ui.Spacer(height=5)
                        self._hand_data_label = ui.Label(
                            "Loading Hand Joint Data...",
                            alignment=ui.Alignment.LEFT_TOP,  # 왼쪽 상단 정렬
                            style={
                                "color": 0xFFFFFFFF,        # 흰색 텍스트
                                "font_size": 16,            # 손 관절용 폰트 크기
                                "margin": 8,                # 여백
                                "word_wrap": False,         # 자동 줄바꿈
                            }
                        )
                        ui.Spacer(height=5)
                        
        except Exception as e:
            print(f"❌ 손 관절 오버레이 생성 실패: {e}")
            import traceback
            traceback.print_exc()

    def update_joint_display_text(self):
        """관절 데이터 텍스트 업데이트 - 모든 관절 정보 표시"""
        if self._joint_data_label is None:
            return
        
        try:
            # 🎯 모든 관절 정보를 한 번에 생성
            new_text = self.format_all_joint_text()
            
            # 🎨 텍스트 업데이트
            self._joint_data_label.text = new_text
            
        except Exception as e:
            error_text = f"❌ Display Error:\n{str(e)}"
            self._joint_data_label.text = error_text
            print(f"❌ Joint display update failed: {e}")

    def update_hand_display_text(self):
        """손 관절 데이터 텍스트 업데이트 - 손 관절 전용 윈도우"""
        if self._hand_data_label is None:
            return
        
        try:
            # 🎯 손 관절 정보를 한 번에 생성
            new_text = self.format_hand_joint_text()
            
            # 🎨 텍스트 업데이트
            self._hand_data_label.text = new_text
            
        except Exception as e:
            error_text = f"❌ Hand Data Error:\n{str(e)}"
            self._hand_data_label.text = error_text
            print(f"❌ Hand display update failed: {e}")


    def load_joint_config(self, config_path):
        """관절 설정 파일 로드"""
        self._joint_controller.load_coupled_joint_config(config_path)

    def get_joint_data_both_modes(self):
        """텍스트 오버레이용 관절 데이터 수집 (current + desired 동시 반환)"""
        try:
            # 🔍 ROS2 구독 상태 확인
            ros2_active = self._joint_controller._ros2_subscriber_active
            
            if not ros2_active:
                return {
                    'status': 'disconnected',
                    'message': 'ROS2 Inactive',
                    'current_data': {},
                    'desired_data': {}
                }
            
            # 🎯 각 그룹별 current/desired 데이터 가져오기
            current_joint_groups = {
                'right_arm': getattr(self._joint_controller, 'Arm_R_current', []),
                'left_arm': getattr(self._joint_controller, 'Arm_L_current', []),
                'waist': getattr(self._joint_controller, 'Waist_current', []),
                'neck': getattr(self._joint_controller, 'Neck_current', [])
            }
            
            desired_joint_groups = {
                'right_arm': getattr(self._joint_controller, 'Arm_R_desired', []),
                'left_arm': getattr(self._joint_controller, 'Arm_L_desired', []),
                'waist': getattr(self._joint_controller, 'Waist_desired', []),
                'neck': getattr(self._joint_controller, 'Neck_desired', [])
            }
            
            return {
                'status': 'active',
                'current_data': {
                    'joint_groups': current_joint_groups
                },
                'desired_data': {
                    'joint_groups': desired_joint_groups
                }
            }
            
        except Exception as e:
            print(f"❌ Failed to get both modes joint data: {e}")
            return {
                'status': 'error',
                'message': f'Error: {str(e)}',
                'current_data': {},
                'desired_data': {}
            }

    def get_hand_joint_data_both_modes(self):
        """손 관절 전용 데이터 수집 (current + desired 동시 반환)"""
        try:
            # 🔍 ROS2 구독 상태 확인
            ros2_active = self._joint_controller._ros2_subscriber_active
            
            if not ros2_active:
                return {
                    'status': 'disconnected',
                    'message': 'ROS2 Inactive',
                    'current_data': {'left_hand': [], 'right_hand': []},
                    'desired_data': {'left_hand': [], 'right_hand': []}
                }
            
            # 🎯 current 손 관절 데이터 추출
            current_left_hand = getattr(self._joint_controller, 'Hand_L_current', []) or []
            current_right_hand = getattr(self._joint_controller, 'Hand_R_current', []) or []
            
            # 🎯 desired 손 관절 데이터 추출
            desired_left_hand = getattr(self._joint_controller, 'Hand_L_desired', []) or []
            desired_right_hand = getattr(self._joint_controller, 'Hand_R_desired', []) or []
            
            # 🎯 15개 관절로 제한 (혹시 더 많은 데이터가 있을 경우)
            current_left_filtered = current_left_hand[:15] if current_left_hand else []
            current_right_filtered = current_right_hand[:15] if current_right_hand else []
            desired_left_filtered = desired_left_hand[:15] if desired_left_hand else []
            desired_right_filtered = desired_right_hand[:15] if desired_right_hand else []
            
            return {
                'status': 'active',
                'current_data': {
                    'left_hand': current_left_filtered,
                    'right_hand': current_right_filtered
                },
                'desired_data': {
                    'left_hand': desired_left_filtered,
                    'right_hand': desired_right_filtered
                }
            }
            
        except Exception as e:
            print(f"❌ Failed to get both modes hand joint data: {e}")
            return {
                'status': 'error',
                'message': f'Hand Data Error: {str(e)}',
                'current_data': {'left_hand': [], 'right_hand': []},
                'desired_data': {'left_hand': [], 'right_hand': []}
            }

    def set_ros2_manager(self, ros2_manager):
        """ROS2 Manager 설정 및 모드 초기화"""
        self._ros2_manager = ros2_manager
        self._update_cached_topic_mode()  # 🆕 초기 모드 캐시
    
    def _update_cached_topic_mode(self):
        """토픽 모드 캐시 업데이트"""
        if hasattr(self, '_ros2_manager') and self._ros2_manager:
            try:
                self._cached_topic_mode = self._ros2_manager.get_current_topic_mode()
            except Exception as e:
                print(f"⚠️ ROS2 Manager 모드 조회 실패: {e}")
        else:
            print("❌ ROS2 Manager not initialized!")

    def format_all_joint_text(self):
        """모든 관절 데이터를 통합 포맷으로 표시 (current | desired 동시 표시)"""
        # 🆕 both_modes 함수 사용으로 변경
        joint_data = self.get_joint_data_both_modes()
        
        if joint_data['status'] != 'active':
            return f"{joint_data.get('message', 'ROS2 Disconnected')}"

        # 🆕 현재 토픽 모드 확인
        from .config.ros2_config import ROS2Config
        current_mode = self._cached_topic_mode
        
        text_lines = ["Current | Desired"]
        text_lines.append("=" * 20)
        
        # 🆕 current와 desired 데이터 분리
        current_groups = joint_data['current_data']['joint_groups']
        desired_groups = joint_data['desired_data']['joint_groups']
        
        # 🔘 1️⃣ 허리 & 목 관절들 (Body 그룹 체크박스 확인) - 수직 배치 + 구분
        if self._joint_group_display_enabled.get('body', True):
            current_waist = current_groups.get('waist', [])
            current_neck = current_groups.get('neck', [])
            desired_waist = desired_groups.get('waist', [])
            desired_neck = desired_groups.get('neck', [])
            
            if current_waist or current_neck or desired_waist or desired_neck:
                text_lines.append(" Body Joints:")
                
                def format_joint_line_both_marked(name, current_val, desired_val, current_topic_mode):
                    """current | desired 형식으로 모드 구분 포맷팅"""
                    from .config.ros2_config import ROS2Config
                    
                    prefix = "     "
                    name_part = f"{name:2s}"
                    colon = ": "
                    
                    # 🎯 모드에 따른 강조 표시
                    if current_topic_mode == ROS2Config.TOPIC_MODE_CURRENT:
                        current_part = f"[{current_val:6.1f}]"  # Current 강조
                        desired_part = f" {desired_val:6.1f}"
                    else:  # DESIRED 모드
                        current_part = f" {current_val:6.1f}"
                        desired_part = f"[{desired_val:6.1f}]"  # Desired 강조
                    
                    separator = " | "
                    full_line = prefix + name_part + colon + current_part + separator + desired_part
                    return full_line
                
                # ✅ 허리 관절들 (current | desired 마커 적용)
                if current_waist or desired_waist:
                    text_lines.append("   Waist:")
                    # 표시용 짧은 이름 (실제 joint name: Waist_Yaw_Joint, Waist_Pitch_Lower_Joint, Waist_Pitch_Upper_Joint)
                    waist_names = ["WY", "WP", "CP"]
                    max_len = max(len(current_waist), len(desired_waist))
                    for i in range(max_len):
                        if i < len(waist_names):
                            waist_name = waist_names[i]
                            current_val = current_waist[i] if i < len(current_waist) else 0.0
                            desired_val = desired_waist[i] if i < len(desired_waist) else 0.0
                            text_lines.append(format_joint_line_both_marked(waist_name, current_val, desired_val, current_mode))
                
                # ✅ 목 관절들 (current | desired 마커 적용)
                if current_neck or desired_neck:
                    text_lines.append("   Neck:")
                    # 표시용 짧은 이름 (실제 joint name: Neck_Pitch_Joint, Neck_Yaw_Joint)
                    neck_names = ["NP", "NY"]
                    max_len = max(len(current_neck), len(desired_neck))
                    for i in range(max_len):
                        if i < len(neck_names):
                            neck_name = neck_names[i]
                            current_val = current_neck[i] if i < len(current_neck) else 0.0
                            desired_val = desired_neck[i] if i < len(desired_neck) else 0.0
                            text_lines.append(format_joint_line_both_marked(neck_name, current_val, desired_val, current_mode))
        
        # 🔘 2️⃣ 팔 관절들 (Left Arm & Right Arm 체크박스 확인) - 수직 배치
        left_arm_enabled = self._joint_group_display_enabled.get('left_arm', True)
        right_arm_enabled = self._joint_group_display_enabled.get('right_arm', True)
        
        if left_arm_enabled or right_arm_enabled:
            current_left_arm = current_groups.get('left_arm', []) if left_arm_enabled else []
            current_right_arm = current_groups.get('right_arm', []) if right_arm_enabled else []
            desired_left_arm = desired_groups.get('left_arm', []) if left_arm_enabled else []
            desired_right_arm = desired_groups.get('right_arm', []) if right_arm_enabled else []
            
            if current_left_arm or current_right_arm or desired_left_arm or desired_right_arm:
                text_lines.append("=" * 20)
                text_lines.append("Arm Joints:")
                
                # 🎯 팔 관절용 포맷팅 함수 (3칸 이름, 텍스트 마커 적용)
                def format_joint_line_both_arm_marked(name, current_val, desired_val, current_topic_mode):
                    """팔 관절용 current | desired 형식으로 텍스트 마커 포맷팅"""
                    from .config.ros2_config import ROS2Config
                    
                    prefix = "     "  # 5칸
                    name_part = f"{name:3s}"  # 이름 3칸 고정 (LSP, RSP 등)
                    colon = ": "  # 2칸
                    
                    # 🎯 모드에 따른 강조 표시
                    if current_topic_mode == ROS2Config.TOPIC_MODE_CURRENT:
                        current_part = f"[{current_val:6.1f}]"  # Current 강조
                        desired_part = f" {desired_val:6.1f}"
                    else:  # DESIRED 모드
                        current_part = f" {current_val:6.1f}"
                        desired_part = f"[{desired_val:6.1f}]"  # Desired 강조
                    
                    separator = " | "  # 구분자 3칸
                    full_line = prefix + name_part + colon + current_part + separator + desired_part
                    return full_line
                
                # ✅ 왼팔 관절들 (current | desired 마커 적용)
                if current_left_arm or desired_left_arm:
                    text_lines.append("   Left Arm:")
                    # 표시용 짧은 이름 (실제 joint name: L_Shoulder_Pitch_Joint, L_Shoulder_Roll_Joint, ...)
                    left_arm_names = ["LSP", "LSR", "LSY", "LEP", "LWY", "LWR", "LWP"]
                    max_len = max(len(current_left_arm), len(desired_left_arm))
                    for i in range(max_len):
                        if i < len(left_arm_names):
                            left_name = left_arm_names[i]
                            current_val = current_left_arm[i] if i < len(current_left_arm) else 0.0
                            desired_val = desired_left_arm[i] if i < len(desired_left_arm) else 0.0
                            text_lines.append(format_joint_line_both_arm_marked(left_name, current_val, desired_val, current_mode))
                
                # ✅ 오른팔 관절들 (current | desired 마커 적용)
                if current_right_arm or desired_right_arm:
                    text_lines.append("   Right Arm:")
                    # 표시용 짧은 이름 (실제 joint name: R_Shoulder_Pitch_Joint, R_Shoulder_Roll_Joint, ...)
                    right_arm_names = ["RSP", "RSR", "RSY", "REP", "RWY", "RWR", "RWP"]
                    max_len = max(len(current_right_arm), len(desired_right_arm))
                    for i in range(max_len):
                        if i < len(right_arm_names):
                            right_name = right_arm_names[i]
                            current_val = current_right_arm[i] if i < len(current_right_arm) else 0.0
                            desired_val = desired_right_arm[i] if i < len(desired_right_arm) else 0.0
                            text_lines.append(format_joint_line_both_arm_marked(right_name, current_val, desired_val, current_mode))
        
        return "\n".join(text_lines)

    def format_hand_joint_text(self):
        """손 관절 전용 텍스트 포맷팅 (current | desired 동시 표시)"""
        # 🆕 both_modes 함수 사용으로 변경
        hand_data = self.get_hand_joint_data_both_modes()
        
        if hand_data['status'] != 'active':
            return f"{hand_data.get('message', 'ROS2 Disconnected')}"

        # 🆕 현재 토픽 모드 확인
        from .config.ros2_config import ROS2Config
        current_mode = self._cached_topic_mode
        


        text_lines = ["Current | Desired"]
        text_lines.append("=" * 20)
        
        # 🆕 current와 desired 데이터 분리
        current_left_hand = hand_data['current_data']['left_hand']
        current_right_hand = hand_data['current_data']['right_hand']
        desired_left_hand = hand_data['desired_data']['left_hand']
        desired_right_hand = hand_data['desired_data']['right_hand']
                
        # 🎯 손 관절용 current | desired 포맷팅 함수 (수정됨)
        def format_joint_line_both_hand_marked(name, current_val, desired_val, current_topic_mode):
            """손 관절용 current | desired 형식으로 포맷팅"""
            from .config.ros2_config import ROS2Config
            
            prefix = "  "  # 2칸 들여쓰기
            name_part = f"{name:3s}"  # 이름 3칸 고정 (TH1, IN2 등)
            colon = ": "  # 2칸
            
            # 🎯 모드에 따른 강조 표시
            if current_topic_mode == ROS2Config.TOPIC_MODE_CURRENT:
                current_part = f"[{current_val:6.1f}]"  # Current 강조
                desired_part = f" {desired_val:6.1f}"
            else:  # DESIRED 모드
                current_part = f" {current_val:6.1f}"
                desired_part = f"[{desired_val:6.1f}]"  # Desired 강조
            
            separator = " | "  # 구분자 3칸
            full_line = prefix + name_part + colon + current_part + separator + desired_part
            return full_line
        
        # 🎯 손가락별 그룹 정의
        finger_groups = [
            ("Thumb", "TH", [0, 5, 10]),    # 엄지: 1마디(0), 2마디(5), 3마디(10)
            ("Index", "IN", [1, 6, 11]),    # 검지: 1마디(1), 2마디(6), 3마디(11)  
            ("Middle", "MI", [2, 7, 12]),   # 중지: 1마디(2), 2마디(7), 3마디(12)
            ("Ring", "RI", [3, 8, 13]),     # 약지: 1마디(3), 2마디(8), 3마디(13)
            ("Pinky", "PI", [4, 9, 14])     # 소지: 1마디(4), 2마디(9), 3마디(14)
        ]
        
        # 🔘 1️⃣ 왼손 관절들 (손가락별 그룹화, current | desired 마커 적용)
        if current_left_hand or desired_left_hand:
            text_lines.append("Left Hand:")
            
            for finger_name, finger_code, indices in finger_groups:                
                for joint_num, data_index in enumerate(indices, 1):
                    joint_name = f"L_{finger_code}{joint_num}"
                    current_val = current_left_hand[data_index] if data_index < len(current_left_hand) else 0.0
                    desired_val = desired_left_hand[data_index] if data_index < len(desired_left_hand) else 0.0
                    text_lines.append(format_joint_line_both_hand_marked(joint_name, current_val, desired_val, current_mode))
                
                # 🔘 손가락 간 구분선 (마지막 손가락 제외)
                if finger_name != "Pinky":
                    text_lines.append("-" * 18)
        else:
            text_lines.append("Left Hand: No Data")
        
        # 🔘 양손 구분선
        text_lines.append("=" * 20)
        
        # 🔘 2️⃣ 오른손 관절들 (손가락별 그룹화, current | desired 마커 적용)
        if current_right_hand or desired_right_hand:
            text_lines.append("Right Hand:")
            
            for finger_name, finger_code, indices in finger_groups:                
                for joint_num, data_index in enumerate(indices, 1):
                    joint_name = f"R_{finger_code}{joint_num}"
                    current_val = current_right_hand[data_index] if data_index < len(current_right_hand) else 0.0
                    desired_val = desired_right_hand[data_index] if data_index < len(desired_right_hand) else 0.0
                    text_lines.append(format_joint_line_both_hand_marked(joint_name, current_val, desired_val, current_mode))
                
                # 🔘 손가락 간 구분선 (마지막 손가락 제외)
                if finger_name != "Pinky":
                    text_lines.append("-" * 18)
        else:
            text_lines.append("Right Hand: No Data")
        
        return "\n".join(text_lines)


    # ========================================
    # 🔘 관절 그룹 체크박스 상태 관리 메서드들
    # ========================================
    
    def set_joint_group_enabled(self, group_name: str, enabled: bool):
        """특정 관절 그룹 표시 활성화/비활성화 (손 관절은 윈도우 표시/숨김 제어)"""
        if group_name in self._joint_group_display_enabled:
            self._joint_group_display_enabled[group_name] = enabled
            
            # 🔘 손 관절 그룹인 경우 손 윈도우 표시/숨김 제어
            if group_name == 'hand':  # 🆕 hand 하나로 통합
                self._toggle_hand_window_visibility(enabled)
            else:
                # 🔄 관절 윈도우 표시/숨김 제어 (Body + Arm)
                self._toggle_joint_window_visibility()
        else:
            print(f"❌ 알 수 없는 관절 그룹: {group_name}")
    
    def get_all_group_states(self) -> dict:
        """모든 관절 그룹 상태 반환"""
        return self._joint_group_display_enabled.copy()
    
    def set_all_groups_enabled(self, enabled: bool):
        """모든 관절 그룹 활성화/비활성화 (손 관절은 윈도우 표시/숨김 포함)"""
        for group_name in self._joint_group_display_enabled:
            self._joint_group_display_enabled[group_name] = enabled
        
        print(f"✅ 모든 그룹 표시: {'ON' if enabled else 'OFF'}")
        
        # 🔄 관절 윈도우 표시/숨김 제어 (Body + Arm)
        self._toggle_joint_window_visibility()
        
        # 🔘 손 관절 윈도우 표시/숨김 제어
        self._toggle_hand_window_visibility(enabled)
    
    def toggle_joint_group(self, group_name: str):
        """특정 관절 그룹 토글 (ON ↔ OFF)"""
        if group_name in self._joint_group_display_enabled:
            current_state = self._joint_group_display_enabled[group_name]
            self.set_joint_group_enabled(group_name, not current_state)
        else:
            print(f"❌ 알 수 없는 관절 그룹: {group_name}")

    def _toggle_hand_window_visibility(self, show: bool):
        """손 관절 윈도우 표시/숨김 제어"""
        try:
            if self._hand_overlay_window is not None:
                # 🎯 윈도우 가시성 제어
                self._hand_overlay_window.visible = show
                
                if show:
                    print("✅ 손 관절 윈도우 표시")
                    # 🔄 윈도우를 보여줄 때 즉시 데이터 업데이트
                    if self._hand_data_label is not None:
                        self.update_hand_display_text()
                else:
                    print("✅ 손 관절 윈도우 숨김")
            else:
                print("⚠️ 손 관절 윈도우가 초기화되지 않았습니다")
                
        except Exception as e:
            print(f"❌ 손 윈도우 가시성 제어 실패: {e}")

    def _toggle_joint_window_visibility(self):
        """관절 윈도우 표시/숨김 제어 (body, left_arm, right_arm 중 하나라도 ON이면 표시)"""
        try:
            if self._text_overlay_window is not None:
                # 🎯 body, left_arm, right_arm 중 하나라도 활성화되어 있으면 표시
                should_show = (
                    self._joint_group_display_enabled.get('body', False) or
                    self._joint_group_display_enabled.get('left_arm', False) or
                    self._joint_group_display_enabled.get('right_arm', False)
                )
                
                self._text_overlay_window.visible = should_show
                
                if should_show:
                    print("✅ 관절 윈도우 표시")
                    # 🔄 윈도우를 보여줄 때 즉시 데이터 업데이트
                    if self._joint_data_label is not None:
                        self.update_joint_display_text()
                else:
                    print("✅ 관절 윈도우 숨김")
            else:
                print("⚠️ 관절 윈도우가 초기화되지 않았습니다")
                
        except Exception as e:
            print(f"❌ 관절 윈도우 가시성 제어 실패: {e}")


    # ========================================
    # 🔗 Follow Cube 관련 메서드들
    # ========================================

    def start_follow_cube(self):
        """'Follow Cube' 루틴을 시작합니다."""
        try:
            if self._follow_cube_active:
                print("⚠️ Follow Cube mode is already active")
                return
            
            if self._articulation is None:
                print("❌ Articulation이 초기화되지 않았습니다")
                return
                        
            self._initialize_ik_controllers()          # ← 이후 양손 IK 로 교체 예정
            self._create_follow_target_cubes()        # 👉 양손 큐브 동시 생성
            self._register_follow_cube_callback()
            self._follow_cube_active = True
                        
        except Exception as e:
            print(f"❌ Failed to start Follow Cube routine: {e}")
            import traceback
            traceback.print_exc()

    def stop_follow_cube(self):
        """'Follow Cube' 루틴을 중지하고 리셋합니다."""
        try:
            if not self._follow_cube_active:
                print("⚠️ Follow Cube mode is not active")
                return
            
            self._unregister_follow_cube_callback()
            self._remove_follow_target_cubes()     
            self._cleanup_ik_controllers()
            self._follow_cube_active = False
                        
        except Exception as e:
            print(f"❌ Failed to stop Follow Cube routine: {e}")
            import traceback
            traceback.print_exc()

    def _initialize_ik_controllers(self):
        """양손 IK 컨트롤러 초기화"""
        if self._right_ik_controller is None:
            from .controllers.right_ik_controller import ALLEXRightIKController
            self._right_ik_controller = ALLEXRightIKController()
        if self._left_ik_controller is None:
            from .controllers.left_ik_controller import ALLEXLeftIKController
            self._left_ik_controller = ALLEXLeftIKController()

    def _create_right_target_cube(self):
        """추적할 큐브를 월드에 생성합니다. (right_hand 초기 위치 기준)"""

        try:
            import omni.usd
            from pxr import UsdGeom, Gf, Sdf
            from isaacsim.core.utils.xforms import get_world_pose
            import numpy as np

            # 스테이지 가져오기
            stage = omni.usd.get_context().get_stage()
            
            # 🎯 큐브 경로를 right_target_cube로 변경
            cube_path = "/World/right_target_cube"
            
            # 기존 큐브가 있다면 제거
            if stage.GetPrimAtPath(cube_path):
                stage.RemovePrim(cube_path)
            
            # 🎯 right_hand의 현재 위치 가져오기
            try:
                right_hand_position, right_hand_orientation = get_world_pose("/ALLEX/right_hand")
                # 🔧 numpy array를 Gf.Vec3f로 변환 (위치)
                initial_position = Gf.Vec3f(
                    float(right_hand_position[0]),
                    float(right_hand_position[1]), 
                    float(right_hand_position[2])
                )
                
                # 🔧 numpy array를 Gf.Quatf로 변환 (방향)
                # Isaac Sim quaternion 형식: [w, x, y, z]
                initial_orientation = Gf.Quatf(
                    float(right_hand_orientation[0]),  # w
                    float(right_hand_orientation[1]),  # x
                    float(right_hand_orientation[2]),  # y
                    float(right_hand_orientation[3])   # z
                )

            except Exception as e:
                print(f"⚠️ right_hand 위치를 가져올 수 없어 기본 위치 사용: {e}")

            
            # 큐브 생성
            cube_prim = UsdGeom.Cube.Define(stage, cube_path)
            
            # 큐브 속성 설정
            cube_prim.CreateSizeAttr(0.08)  # 8cm 크기 (약간 작게)
            cube_prim.CreateDisplayColorAttr([(0.0, 1.0, 0.0)])  # 🟢 초록색 (right_hand용)
            
            # 🎯 right_hand 위치 기준으로 초기 위치 설정
            cube_prim.AddTranslateOp().Set(initial_position)
            cube_prim.AddOrientOp().Set(initial_orientation)     

            # 큐브 참조 저장
            self._follow_target_cube = cube_prim
                
        except Exception as e:
            print(f"❌ Failed to create right target cube: {e}")
            raise
            
    def _remove_right_target_cube(self):
        """추적 큐브를 월드에서 제거합니다."""
        try:
            if self._follow_target_cube is not None:
                import omni.usd
                stage = omni.usd.get_context().get_stage()
                
                # 큐브 제거
                stage.RemovePrim(self._follow_target_cube.GetPath())
                self._follow_target_cube = None
                
        except Exception as e:
            print(f"❌ Failed to remove right target cube: {e}")

    def _create_left_target_cube(self):
        """왼손용 큐브 생성 (/World/left_target_cube, 🔵 파란색)"""
        import omni.usd
        from pxr import UsdGeom, Gf
        from isaacsim.core.utils.xforms import get_world_pose

        stage = omni.usd.get_context().get_stage()
        cube_path = "/World/left_target_cube"

        if stage.GetPrimAtPath(cube_path):
            stage.RemovePrim(cube_path)

        try:
            left_pos, left_ori = get_world_pose("/ALLEX/left_hand")
            init_pos = Gf.Vec3f(float(left_pos[0]), float(left_pos[1]), float(left_pos[2]))
            init_ori = Gf.Quatf(float(left_ori[0]), float(left_ori[1]), float(left_ori[2]), float(left_ori[3]))
        except Exception as e:
            print(f"⚠️ left_hand 위치를 얻지 못해 기본값 사용: {e}")
            init_pos, init_ori = Gf.Vec3f(0,0,0), Gf.Quatf(1,0,0,0)

        cube_prim = UsdGeom.Cube.Define(stage, cube_path)
        cube_prim.CreateSizeAttr(0.08)
        cube_prim.CreateDisplayColorAttr([(0.0, 0.0, 1.0)])  # 🔵
        cube_prim.AddTranslateOp().Set(init_pos)
        cube_prim.AddOrientOp().Set(init_ori)

        self._left_target_cube = cube_prim

    def _remove_left_target_cube(self):
        """왼손용 큐브 제거"""
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        if self._left_target_cube:
            stage.RemovePrim(self._left_target_cube.GetPath())
            self._left_target_cube = None

    def _create_follow_target_cubes(self):
        """오른손·왼손 큐브 모두 생성"""
        self._create_right_target_cube()
        self._create_left_target_cube()

    def _remove_follow_target_cubes(self):
        """오른손·왼손 큐브 모두 제거"""
        self._remove_right_target_cube()
        self._remove_left_target_cube()

    def _register_follow_cube_callback(self):
        """Follow Cube 콜백을 시뮬레이션 루프에 등록합니다."""
        try:
            if not self._follow_cube_callback_registered:
                # 시뮬레이션 루프에 콜백 추가
                self._simulation_loop.add_callback(self._update_follow_cube_callback)
                self._follow_cube_callback_registered = True
                                
        except Exception as e:
            print(f"❌ Failed to register Follow Cube callback: {e}")
            raise

    def _unregister_follow_cube_callback(self):
        """Follow Cube 콜백을 시뮬레이션 루프에서 해제합니다."""
        try:
            if self._follow_cube_callback_registered:
                # 시뮬레이션 루프에서 콜백 제거
                self._simulation_loop.remove_callback(self._update_follow_cube_callback)
                self._follow_cube_callback_registered = False
                                
        except Exception as e:
            print(f"❌ Failed to unregister Follow Cube callback: {e}")

    def _cleanup_ik_controllers(self):
        """IK 컨트롤러를 정리합니다."""
        try:
            if self._right_ik_controller is not None:
                self._right_ik_controller = None
            if self._left_ik_controller is not None:
                self._left_ik_controller = None
                print("✅ IK controller cleaned up")
                
        except Exception as e:
            print(f"❌ Failed to cleanup IK controller: {e}")
    
    def _update_follow_cube_callback(self, step_size: float):
        """매 스텝마다 양손 IK 계산"""
        if self._articulation is None:
            print("⚠️ Articulation 미초기화")
            return

        # IK 컨트롤러 초기화
        if self._right_ik_controller is None or self._left_ik_controller is None:
            self._initialize_ik_controllers()

        # 프레임 카운터 변수 준비
        if not hasattr(self, '_apply_action_counter'):
            self._apply_action_counter = 0
        self._apply_action_counter += 1


        right_pos, right_ori = get_world_pose("/World/right_target_cube")

        right_action, right_ok = self._right_ik_controller.compute_ik(right_pos, right_ori)

        # 2️⃣ 왼손 타겟
        left_pos , left_ori  = get_world_pose("/World/left_target_cube")
        left_action , left_ok = self._left_ik_controller.compute_ik(left_pos, left_ori)

        #print(f"right_pos: {np.round(right_pos,3)}| {np.round(euler,3)}" , end='\r', flush=True)

        # 3️⃣ 결과 적용 (3프레임에 한 번만 실행)
        if self._apply_action_counter % 10 == 0:
            if right_ok and right_action is not None:
                with self._physx_lock:
                    self._articulation.apply_action(right_action)
            if left_ok and left_action is not None:
                with self._physx_lock:
                    self._articulation.apply_action(left_action)

        if (not right_ok) or (not left_ok):
            pass
            #print(f"❌ IK 실패 (R:{right_ok}, L:{left_ok}) | right_pos: {np.round(right_pos,3)}| {np.round(euler,3)}" , end='\r', flush=True)
    
    def _cache_policy_action_dof_indices(self) -> bool:
        """Policy action용 DOF 인덱스를 시뮬레이션 시작 전에 캐싱합니다."""
        if self._policy_action_dof_indices is not None:
            return True
        # 시뮬레이션 실행 중이라면 경고 후에도 일단 시도 (반복 경고를 피하기 위해 단발성)
        if self._simulation_loop.is_running():
            print("⚠️ Policy action DOF indices not cached yet; attempting to cache during simulation (may be unsafe).")

        try:
            from .config.ros2_config import ROS2Config
            indices: list[int] = []
            for name in ROS2Config.ALLEX_ACTION_JOINT_FULL_NAMES:
                idx = self._articulation.get_dof_index(name)
                if idx is None:
                    print(f"⚠️ Joint not found for policy action: {name}")
                    return False
                indices.append(idx)

            if len(indices) != 18:
                print(f"❌ Failed to resolve all 18 policy action joint indices (got {len(indices)})")
                return False

            self._policy_action_dof_indices = indices
            print(f"✅ Policy action DOF indices cached: {self._policy_action_dof_indices}")
            return True
        except Exception as e:
            print(f"❌ Failed to cache policy action DOF indices: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def apply_policy_action(self, joint_values: list) -> bool:
        """Policy action (18개 조인트 위치)을 articulation에 적용
        
        Args:
            joint_values: 18개 조인트 위치 값 (ALLEX_ACTION_JOINT_FULL_NAMES 순서)
            
        Returns:
            bool: 적용 성공 여부
        """
        if self._articulation is None:
            print("⚠️ Articulation not initialized, cannot apply policy action")
            return False
        
        if len(joint_values) != 18:
            print(f"⚠️ Policy action expects 18 joints, got {len(joint_values)}")
            return False
        
        try:
            # DOF 인덱스 캐시 확인 및 초기화 (시뮬레이션 시작 전에만)
            if self._policy_action_dof_indices is None:
                if not self._cache_policy_action_dof_indices():
                    return False
            
            # ArticulationAction은 메인 루프에서만 적용하도록 버퍼에 저장
            joint_positions = np.array(joint_values, dtype=np.float32)
            self._pending_policy_action = joint_positions
            return True
            
        except Exception as e:
            print(f"❌ Failed to apply policy action: {e}")
            import traceback
            traceback.print_exc()
            return False