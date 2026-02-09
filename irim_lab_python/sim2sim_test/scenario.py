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
    ALLEXSimulationLoop,
)
from .config.ros2_config import ROS2Config
from .ui.joint_overlay_ui import JointOverlayUI
import os
import threading
import traceback

import numpy as np
from isaacsim.core.utils.xforms import get_world_pose
from isaacsim.core.utils.types import ArticulationAction

# ---------------------------------------------------------------------------
# 상수 (매직 넘버 제거, DDVC 101_Refactoring)
# ---------------------------------------------------------------------------
TEXT_UPDATE_INTERVAL = 15
POLICY_ACTION_JOINT_COUNT = 18
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


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    """단일 쿼터니언 [w,x,y,z]의 켤레 반환"""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=q.dtype)


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """두 단일 쿼터니언 [w,x,y,z]의 곱"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=q1.dtype)


def _quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """쿼터니언 q로 3D 벡터 v 회전"""
    v_q = np.array([0.0, v[0], v[1], v[2]], dtype=q.dtype)
    return _quat_mul(_quat_mul(q, v_q), _quat_conjugate(q))[1:]


def _resolve_dof_indices(articulation, joint_names: list) -> list:
    """조인트 이름 리스트로 DOF 인덱스 리스트 반환. 실패 시 빈 리스트."""
    indices = []
    for name in joint_names:
        idx = articulation.get_dof_index(name)
        if idx is None:
            return []
        indices.append(idx)
    return indices


def _format_joint_line(name: str, current_val: float, desired_val: float,
                       topic_mode: str, name_width: int = 3, prefix: str = "     ") -> str:
    """current | desired 한 줄 포맷 (모드에 따라 강조)."""
    current_part = f"[{current_val:6.1f}]" if topic_mode == ROS2Config.TOPIC_MODE_CURRENT else f" {current_val:6.1f}"
    desired_part = f" {desired_val:6.1f}" if topic_mode == ROS2Config.TOPIC_MODE_CURRENT else f"[{desired_val:6.1f}]"
    return f"{prefix}{name:{name_width}s}: {current_part} | {desired_part}"


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
        self._physx_lock = threading.Lock()
        self._pending_policy_action: np.ndarray | None = None
        self._articulation = None
        self._joint_controller.set_scenario_reference(self)
        self._ui_builder_ref = None
        self._overlay_ui = JointOverlayUI()
        self._text_update_counter = 0
        self._joint_group_display_enabled = {
            "body": False,
            "left_arm": False,
            "right_arm": False,
            "hand": False,
        }
        self._cached_topic_mode = None
        self._obs_joint_indices = None
        self._right_hand_torque_dof_indices = None
        self._policy_action_dof_indices = None

    def setup(self):
        """시나리오 초기 설정"""        
        # 카메라 뷰 설정
        self._initializer.setup_camera_view()
        
        extension_root = os.path.dirname(os.path.abspath(__file__))
        joint_config_path = os.path.join(extension_root, "joint_config.json")
        self._joint_controller.load_coupled_joint_config(joint_config_path)

        self._overlay_ui.create_joint_overlay_window()
        self._overlay_ui.create_hand_overlay_window()

        print("✅ ALLEX Digital Twin 설정 완료")

    def _get_target_positions(self):
        """ROS2 데이터 우선, 없으면 기본값 사용 (joint control generator용)."""
        try:
            ros2_positions = self._joint_controller.get_unified_target_positions()
            if ros2_positions and any(pos != 0.0 for pos in ros2_positions):
                return ros2_positions
        except Exception as e:
            print(f"⚠️ ROS2 positions access failed: {e}")
        return self._initializer.target_joint_positions

    def _setup_joint_control_generator(self):
        """현재 target positions 소스로 joint control generator를 만들어 시뮬레이션 루프에 설정."""
        generator = self._joint_controller.create_joint_control_generator(
            articulation=self._articulation,
            get_target_positions_func=self._get_target_positions,
        )
        self._simulation_loop.set_script_generator(generator)

    def load_example_assets(self):
        """에셋 로딩 및 초기화"""
        self._articulation = self._asset_manager.load_robot_asset()
        if self._articulation is None:
            return None
        init_success = self._asset_manager.initialize_articulation()
        if not init_success:
            print("🔄 'RUN' 버튼을 누르면 Articulation이 초기화됩니다")
            return self._articulation
        self._initializer.initialize_joint_positions(self._articulation)
        self._cache_policy_action_dof_indices()
        self._setup_joint_control_generator()
        return self._articulation

    def delayed_initialization(self):
        """지연된 초기화 - 시뮬레이션 시작 후 호출"""
        if self._articulation is None:
            print("⚠️ Articulation이 로딩되지 않았습니다")
            return False
        init_success = self._asset_manager.initialize_articulation()
        if not init_success:
            print("❌ Articulation 초기화 실패")
            return False
        self._initializer.initialize_joint_positions(self._articulation)
        self._initial_joint_positions = self._articulation.get_joint_positions()
        print(f"✅ 전체 관절 위치 저장 완료: {len(self._initial_joint_positions)}개 관절")
        self._setup_joint_control_generator()
        print("✅ Articulation 초기화 완료")
        return True

    def reset(self):
        """시스템 리셋"""
        self._initializer.reset(self._articulation)
        self._simulation_loop.reset()
        if self._articulation is not None:
            self._setup_joint_control_generator()
        print("✅ 시스템 리셋 완료")

    def set_ui_builder_ref(self, ui_builder_ref):
        """UIBuilder 참조를 설정합니다."""
        self._ui_builder_ref = ui_builder_ref

    def update(self, step: float):
        """시뮬레이션 스텝 업데이트 (메인 스레드에서 policy action 처리)."""
        self._text_update_counter += 1
        if self._text_update_counter % TEXT_UPDATE_INTERVAL == 0:
            self.update_joint_display_text()
            self.update_hand_display_text()
        self._publish_joint_observation_if_needed(step)
        if self._pending_policy_action is not None and self._simulation_loop.is_running():
            try:
                with self._physx_lock:
                    self._articulation.apply_action(
                        ArticulationAction(
                            joint_positions=self._pending_policy_action,
                            joint_indices=np.array(self._policy_action_dof_indices, dtype=np.int32),
                        )
                    )
            except Exception as e:
                print(f"❌ Failed to apply pending policy action in update: {e}")
            finally:
                self._pending_policy_action = None
        return self._simulation_loop.update(step)

    def _publish_joint_observation_if_needed(self, step: float) -> None:
        """ROS2 Publisher 활성 시 관절 관측, 오른손 토크, Right_Hand_base pose 발행."""
        ros2_manager = getattr(self, "_ros2_manager", None)
        if not ros2_manager or not ros2_manager.is_initialized() or not ros2_manager.is_publisher_enabled():
            return
        if self._articulation is None:
            return
        try:
            if self._obs_joint_indices is None:
                self._obs_joint_indices = _resolve_dof_indices(
                    self._articulation, ROS2Config.ALLEX_ACTION_JOINT_FULL_NAMES
                )
                if not self._obs_joint_indices:
                    print("❌ Failed to resolve ALLEX action joint indices; joint observation disabled.")
            if self._obs_joint_indices:
                with self._physx_lock:
                    positions = self._articulation.get_joint_positions(joint_indices=self._obs_joint_indices)
                ros2_manager.publish_joint_observation(positions)

            if self._right_hand_torque_dof_indices is None:
                self._right_hand_torque_dof_indices = _resolve_dof_indices(
                    self._articulation, ROS2Config.RIGHT_HAND_TORQUE_JOINT_NAMES
                )
                if not self._right_hand_torque_dof_indices:
                    print("❌ Failed to resolve right hand torque DOF indices; torque publishing disabled.")
            if self._right_hand_torque_dof_indices:
                try:
                    with self._physx_lock:
                        torques = self._articulation.get_measured_joint_efforts(
                            joint_indices=self._right_hand_torque_dof_indices
                        )
                    if torques is not None:
                        ros2_manager.publish_right_hand_joint_torque([float(v) for v in torques])
                except Exception as te:
                    print(f"⚠️ Right hand joint torque publish error in scenario.update: {te}")

            try:
                right_pos_w, right_quat_w = get_world_pose("/ALLEX/R_Hand_Pose")
                origin_pos_w, origin_quat_w = get_world_pose("/ALLEX/Origin_Body")
                right_pos_w = np.asarray(right_pos_w, dtype=np.float32)
                right_quat_w = np.asarray(right_quat_w, dtype=np.float32)
                origin_pos_w = np.asarray(origin_pos_w, dtype=np.float32)
                origin_quat_w = np.asarray(origin_quat_w, dtype=np.float32)
                origin_quat_conj = _quat_conjugate(origin_quat_w)
                rel_pos = _quat_apply(origin_quat_conj, right_pos_w - origin_pos_w)
                rel_quat = _quat_mul(origin_quat_conj, right_quat_w)
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
    
    def update_joint_display_text(self):
        """관절 오버레이 텍스트 갱신 (JointOverlayUI에 위임)."""
        self._overlay_ui.update_joint_display_text(self)

    def update_hand_display_text(self):
        """손 관절 오버레이 텍스트 갱신 (JointOverlayUI에 위임)."""
        self._overlay_ui.update_hand_display_text(self)


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
        """모든 관절 데이터를 통합 포맷으로 표시 (current | desired 동시 표시)."""
        joint_data = self.get_joint_data_both_modes()
        if joint_data["status"] != "active":
            return joint_data.get("message", "ROS2 Disconnected")
        mode = self._cached_topic_mode
        current_groups = joint_data["current_data"]["joint_groups"]
        desired_groups = joint_data["desired_data"]["joint_groups"]
        lines = ["Current | Desired", "=" * 20]

        if self._joint_group_display_enabled.get("body", True):
            current_waist = current_groups.get("waist", [])
            current_neck = current_groups.get("neck", [])
            desired_waist = desired_groups.get("waist", [])
            desired_neck = desired_groups.get("neck", [])
            if current_waist or current_neck or desired_waist or desired_neck:
                lines.append(" Body Joints:")
                for label, cur, des in [("Waist:", current_waist, desired_waist), ("Neck:", current_neck, desired_neck)]:
                    names = BODY_JOINT_LABELS["waist"] if label == "Waist:" else BODY_JOINT_LABELS["neck"]
                    if not (cur or des):
                        continue
                    lines.append(f"   {label}")
                    for i, n in enumerate(names):
                        if i >= max(len(cur), len(des)):
                            break
                        lines.append(_format_joint_line(
                            n, cur[i] if i < len(cur) else 0.0, des[i] if i < len(des) else 0.0,
                            mode, name_width=2,
                        ))

        left_arm_on = self._joint_group_display_enabled.get("left_arm", True)
        right_arm_on = self._joint_group_display_enabled.get("right_arm", True)
        if left_arm_on or right_arm_on:
            cur_l = current_groups.get("left_arm", []) if left_arm_on else []
            cur_r = current_groups.get("right_arm", []) if right_arm_on else []
            des_l = desired_groups.get("left_arm", []) if left_arm_on else []
            des_r = desired_groups.get("right_arm", []) if right_arm_on else []
            if cur_l or cur_r or des_l or des_r:
                lines.extend(["=" * 20, "Arm Joints:"])
                for arm_label, names, cur, des in [
                    ("Left Arm:", ARM_JOINT_LABELS["left_arm"], cur_l, des_l),
                    ("Right Arm:", ARM_JOINT_LABELS["right_arm"], cur_r, des_r),
                ]:
                    if not (cur or des):
                        continue
                    lines.append(f"   {arm_label}")
                    for i, n in enumerate(names):
                        if i >= max(len(cur), len(des)):
                            break
                        lines.append(_format_joint_line(
                            n, cur[i] if i < len(cur) else 0.0, des[i] if i < len(des) else 0.0, mode, name_width=3
                        ))
        return "\n".join(lines)

    def format_hand_joint_text(self):
        """손 관절 전용 텍스트 포맷팅 (current | desired 동시 표시)."""
        hand_data = self.get_hand_joint_data_both_modes()
        if hand_data["status"] != "active":
            return hand_data.get("message", "ROS2 Disconnected")
        mode = self._cached_topic_mode
        cur_l = hand_data["current_data"]["left_hand"]
        cur_r = hand_data["current_data"]["right_hand"]
        des_l = hand_data["desired_data"]["left_hand"]
        des_r = hand_data["desired_data"]["right_hand"]
        lines = ["Current | Desired", "=" * 20]
        hand_prefix = "  "

        def append_hand_blocks(hand_label: str, cur: list, des: list):
            if not (cur or des):
                lines.append(f"{hand_label}: No Data")
                return
            lines.append(f"{hand_label}:")
            for finger_name, finger_code, indices in FINGER_GROUPS:
                for joint_num, data_index in enumerate(indices, 1):
                    name = f"L_{finger_code}{joint_num}" if "Left" in hand_label else f"R_{finger_code}{joint_num}"
                    c = cur[data_index] if data_index < len(cur) else 0.0
                    d = des[data_index] if data_index < len(des) else 0.0
                    lines.append(_format_joint_line(name, c, d, mode, name_width=3, prefix=hand_prefix))
                if finger_name != "Pinky":
                    lines.append("-" * 18)

        append_hand_blocks("Left Hand", cur_l, des_l)
        lines.append("=" * 20)
        append_hand_blocks("Right Hand", cur_r, des_r)
        return "\n".join(lines)


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
        """손 관절 윈도우 표시/숨김 (JointOverlayUI에 위임). 표시 시 내용 갱신."""
        self._overlay_ui.set_hand_window_visibility(show)
        if show:
            self._overlay_ui.update_hand_display_text(self)

    def _toggle_joint_window_visibility(self):
        """관절 윈도우 표시/숨김 (body, left_arm, right_arm 중 하나라도 ON이면 표시)."""
        should_show = (
            self._joint_group_display_enabled.get("body", False)
            or self._joint_group_display_enabled.get("left_arm", False)
            or self._joint_group_display_enabled.get("right_arm", False)
        )
        self._overlay_ui.set_joint_window_visibility(should_show)
        if should_show:
            self._overlay_ui.update_joint_display_text(self)

    def _cache_policy_action_dof_indices(self) -> bool:
        """Policy action용 DOF 인덱스를 시뮬레이션 시작 전에 캐싱."""
        if self._policy_action_dof_indices is not None:
            return True
        if self._simulation_loop.is_running():
            print("⚠️ Policy action DOF indices not cached yet; attempting to cache during simulation (may be unsafe).")
        try:
            indices = _resolve_dof_indices(self._articulation, ROS2Config.ALLEX_ACTION_JOINT_FULL_NAMES)
            if len(indices) != POLICY_ACTION_JOINT_COUNT:
                print(f"❌ Failed to resolve all {POLICY_ACTION_JOINT_COUNT} policy action joint indices (got {len(indices)})")
                return False
            self._policy_action_dof_indices = indices
            print(f"✅ Policy action DOF indices cached: {self._policy_action_dof_indices}")
            return True
        except Exception as e:
            print(f"❌ Failed to cache policy action DOF indices: {e}")
            traceback.print_exc()
            return False

    def apply_policy_action(self, joint_values: list) -> bool:
        """Policy action (18개 조인트 위치)을 articulation에 적용. 메인 루프에서만 적용되도록 버퍼에 저장."""
        if self._articulation is None:
            print("⚠️ Articulation not initialized, cannot apply policy action")
            return False
        if len(joint_values) != POLICY_ACTION_JOINT_COUNT:
            print(f"⚠️ Policy action expects {POLICY_ACTION_JOINT_COUNT} joints, got {len(joint_values)}")
            return False
        try:
            if self._policy_action_dof_indices is None and not self._cache_policy_action_dof_indices():
                return False
            self._pending_policy_action = np.array(joint_values, dtype=np.float32)
            return True
        except Exception as e:
            print(f"❌ Failed to apply policy action: {e}")
            traceback.print_exc()
            return False