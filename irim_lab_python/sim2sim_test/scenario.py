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

import threading
import traceback
from typing import Any, Dict, Iterable, List

import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.xforms import get_world_pose

from .asset_manager import ALLEXAssetManager
from .config import (
    CameraConfig,
    ROS2Config,
    ScenarioDisplayConfig,
)
from .utils import (
    format_joint_line,
    quat_apply,
    quat_conjugate,
    quat_mul,
    resolve_dof_indices,
)


class ALLEXDigitalTwin:
    """ALLEX 디지털 트윈 메인 클래스 - 모든 모듈을 통합 관리"""

    def __init__(self):
        self._asset_manager = ALLEXAssetManager()
        self._overlay_ui = None  # UIBuilder에서 set_overlay_ui()로 주입
        self._physx_lock = threading.Lock()
        self._pending_policy_action: np.ndarray | None = None
        self._articulation = None
        self._script_generator = None
        self._simulation_running = False
        self._simulation_callbacks = []
        self._target_joint_positions = [0.0] * CameraConfig.TOTAL_JOINTS
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
        # Sim2Sim 디버거용: physics 콜백에서만 채움, 디버거는 이 캐시만 참조 (physics step 중 직접 읽기 방지)
        self._sim2sim_obs_cache = {}
        # 관절 제어 상태 (기존 ALLEXJointController 흡수)
        self._coupled_joints: Dict[int, Dict[str, Any]] = {}
        self._ros2_subscriber_active = False
        self._topic_mode: str | None = None
        self.Hand_R_current: List[float] = []
        self.Hand_R_desired: List[float] = []
        self.Hand_L_current: List[float] = []
        self.Hand_L_desired: List[float] = []
        self.Arm_R_current: List[float] = []
        self.Arm_R_desired: List[float] = []
        self.Arm_L_current: List[float] = []
        self.Arm_L_desired: List[float] = []
        self.Waist_current: List[float] = []
        self.Waist_desired: List[float] = []
        self.Neck_current: List[float] = []
        self.Neck_desired: List[float] = []

    def setup(self):
        set_camera_view(
            eye=CameraConfig.DEFAULT_CAMERA_EYE,
            target=CameraConfig.DEFAULT_CAMERA_TARGET,
            camera_prim_path=CameraConfig.DEFAULT_CAMERA_PRIM_PATH,
        )
        self.load_coupled_joint_config(None)
        if self._overlay_ui is not None:
            self._overlay_ui.create_joint_overlay_window()
            self._overlay_ui.create_hand_overlay_window()

    def _get_target_positions(self):
        """ROS2 데이터 우선, 없으면 기본값 사용 (joint control generator용)."""
        try:
            ros2_positions = self.get_unified_target_positions()
            if ros2_positions and any(pos != 0.0 for pos in ros2_positions):
                return ros2_positions
        except Exception:
            pass
        return self._target_joint_positions

    def _setup_joint_control_generator(self):
        """현재 target positions 소스로 joint control generator를 만들어 시뮬레이션 루프에 설정."""
        generator = self.create_joint_control_generator(
            articulation=self._articulation,
            get_target_positions_func=self._get_target_positions,
        )
        self._script_generator = generator
        self._simulation_running = True

    def load_example_assets(self):
        """에셋 로딩 및 초기화"""
        self._articulation = self._asset_manager.load_robot_asset()
        if self._articulation is None:
            return None
        init_success = self._asset_manager.initialize_articulation()
        if not init_success:
            return self._articulation
        self._initialize_joint_positions()
        self._cache_policy_action_dof_indices()
        self._setup_joint_control_generator()
        return self._articulation

    def _initialize_joint_positions(self):
        """현재 articulation 관절 위치를 _target_joint_positions에 반영."""
        if self._articulation is None:
            return
        try:
            current = self._articulation.get_joint_positions()
            if current is None or len(current) == 0:
                self._target_joint_positions = [0.0] * CameraConfig.TOTAL_JOINTS
                return
            self._target_joint_positions = list(current)
            while len(self._target_joint_positions) < CameraConfig.TOTAL_JOINTS:
                self._target_joint_positions.append(0.0)
        except Exception:
            self._target_joint_positions = [0.0] * CameraConfig.TOTAL_JOINTS

    def delayed_initialization(self):
        if self._articulation is None:
            return False
        if not self._asset_manager.initialize_articulation():
            return False
        self._initialize_joint_positions()
        self._setup_joint_control_generator()
        return True

    def reset(self):
        if self._articulation is not None:
            try:
                current = self._articulation.get_joint_positions()
                if current is not None and len(current) > 0:
                    self._target_joint_positions = list(current)
            except Exception:
                pass
        self._script_generator = None
        self._simulation_running = False
        self._simulation_callbacks.clear()
        if self._articulation is not None:
            self._setup_joint_control_generator()

    def update(self, step: float):
        """시뮬레이션 스텝 업데이트 (메인 스레드에서 policy action 처리)."""
        self._text_update_counter += 1
        if self._text_update_counter % ScenarioDisplayConfig.TEXT_UPDATE_INTERVAL == 0:
            self.update_joint_display_text()
            self.update_hand_display_text()
        self._publish_joint_observation_if_needed(step)
        if self._pending_policy_action is not None and self._simulation_running:
            try:
                with self._physx_lock:
                    self._articulation.apply_action(
                        ArticulationAction(
                            joint_positions=self._pending_policy_action,
                            joint_indices=np.array(self._policy_action_dof_indices, dtype=np.int32),
                        )
                    )
            except Exception:
                pass
            finally:
                self._pending_policy_action = None
        for cb in self._simulation_callbacks:
            try:
                cb(step)
            except Exception:
                pass
        if self._script_generator is None:
            return True
        try:
            next(self._script_generator)
            return False
        except StopIteration:
            self._simulation_running = False
            return True

    def _publish_joint_observation_if_needed(self, step: float) -> None:
        """관절 관측·토크·손 pose 수집(Sim2Sim 캐시 채움). ROS2 Publisher 활성 시에만 발행."""
        if self._articulation is None:
            return
        view = getattr(self._articulation, "_articulation_view", None)
        if view is None or not view.is_physics_handle_valid():
            return
        ros2_manager = getattr(self, "_ros2_manager", None)
        do_publish = bool(ros2_manager and ros2_manager.is_initialized() and ros2_manager.is_publisher_enabled())
        try:
            if self._obs_joint_indices is None:
                self._obs_joint_indices = resolve_dof_indices(
                    self._articulation, ROS2Config.ALLEX_ACTION_JOINT_FULL_NAMES
                )
                if not self._obs_joint_indices:
                    pass  # joint observation disabled
            if self._obs_joint_indices:
                with self._physx_lock:
                    positions = self._articulation.get_joint_positions(joint_indices=self._obs_joint_indices)
                if positions is not None:
                    self._sim2sim_obs_cache["joint_pos"] = np.asarray(positions, dtype=np.float32).reshape(-1)
                if do_publish:
                    ros2_manager.publish_joint_observation(positions)

            if self._right_hand_torque_dof_indices is None:
                self._right_hand_torque_dof_indices = resolve_dof_indices(
                    self._articulation, ROS2Config.RIGHT_HAND_TORQUE_JOINT_NAMES
                )
                if not self._right_hand_torque_dof_indices:
                    pass  # torque publishing disabled
            if self._right_hand_torque_dof_indices:
                try:
                    with self._physx_lock:
                        torques = self._articulation.get_measured_joint_efforts(
                            joint_indices=self._right_hand_torque_dof_indices
                        )
                    if torques is not None:
                        self._sim2sim_obs_cache["right_hand_joint_torque"] = np.asarray(torques, dtype=np.float32).reshape(-1)
                        if do_publish:
                            ros2_manager.publish_right_hand_joint_torque([float(v) for v in torques])
                except Exception:
                    pass

            try:
                right_pos_w, right_quat_w = get_world_pose("/ALLEX/R_Hand_Pose")
                origin_pos_w, origin_quat_w = get_world_pose("/ALLEX/Origin_Body")
                right_pos_w = np.asarray(right_pos_w, dtype=np.float32)
                right_quat_w = np.asarray(right_quat_w, dtype=np.float32)
                origin_pos_w = np.asarray(origin_pos_w, dtype=np.float32)
                origin_quat_w = np.asarray(origin_quat_w, dtype=np.float32)
                origin_quat_conj = quat_conjugate(origin_quat_w)
                rel_pos = quat_apply(origin_quat_conj, right_pos_w - origin_pos_w)
                rel_quat = quat_mul(origin_quat_conj, right_quat_w)
                pose_7d = np.concatenate([rel_pos, rel_quat], axis=-1)
                self._sim2sim_obs_cache["right_hand_base_pos"] = np.asarray(pose_7d, dtype=np.float32)
                if do_publish:
                    ros2_manager.publish_right_hand_base_pos(pose_7d)
            except Exception:
                pass
        except Exception:
            pass

    # ----- Configuration -----

    def set_overlay_ui(self, overlay_ui):
        """UIBuilder에서 관절/손 오버레이 UI 인스턴스 주입."""
        self._overlay_ui = overlay_ui

    def update_joint_display_text(self):
        """관절 오버레이 텍스트 갱신 (JointOverlayUI에 위임)."""
        if self._overlay_ui is not None:
            self._overlay_ui.update_joint_display_text(self)

    def update_hand_display_text(self):
        """손 관절 오버레이 텍스트 갱신 (JointOverlayUI에 위임)."""
        if self._overlay_ui is not None:
            self._overlay_ui.update_hand_display_text(self)

    # ------------------------------------------------------------------
    # 관절 제어 (기존 ALLEXJointController 흡수)
    # ------------------------------------------------------------------
    def load_coupled_joint_config(self, config_path: str | None = None) -> None:
        """Coupled Joint 설정 로드 (스텁)."""
        pass

    def set_ros2_subscriber_status(self, is_active: bool) -> None:
        """ROS2 Subscriber 활성/비활성 플래그."""
        self._ros2_subscriber_active = bool(is_active)

    def set_topic_mode(self, topic_mode: str) -> None:
        """토픽 모드 캐시."""
        self._topic_mode = topic_mode

    def create_joint_control_generator(self, articulation: Any, get_target_positions_func: Any):
        """관절 제어 제너레이터 스텁 (yield만)."""
        while True:
            yield

    def get_unified_target_positions(self) -> List[float]:
        """통합 목표 관절 위치 (스텁은 빈 리스트)."""
        return []

    def update_hand_joint_group(
        self,
        hand_side: str,
        finger_name: str,
        joint_values: Iterable[float],
        mode: str = "current",
    ) -> None:
        """손가락별 관절 버퍼 업데이트."""
        values = list(joint_values)
        if hand_side == "right":
            buf = self.Hand_R_current if mode == "current" else self.Hand_R_desired
        elif hand_side == "left":
            buf = self.Hand_L_current if mode == "current" else self.Hand_L_desired
        else:
            return
        for i in range(min(len(buf), len(values))):
            buf[i] = float(values[i])

    def update_joint_group(
        self,
        group_name: str,
        joint_values: Iterable[float],
        mode: str = "current",
    ) -> None:
        """팔/허리/목 관절 그룹 버퍼 업데이트."""
        values = [float(v) for v in joint_values]
        buf = None
        if group_name == "right_arm":
            buf = self.Arm_R_current if mode == "current" else self.Arm_R_desired
        elif group_name == "left_arm":
            buf = self.Arm_L_current if mode == "current" else self.Arm_L_desired
        elif group_name == "waist":
            buf = self.Waist_current if mode == "current" else self.Waist_desired
        elif group_name == "neck":
            buf = self.Neck_current if mode == "current" else self.Neck_desired
        elif group_name.startswith("hand_"):
            parts = group_name.split("_")
            if len(parts) >= 3:
                self.update_hand_joint_group(parts[1], parts[2], values, mode)
            return
        else:
            return
        if buf is not None:
            if len(buf) < len(values):
                buf.extend([0.0] * (len(values) - len(buf)))
            for i in range(len(values)):
                buf[i] = values[i]

    def _data_response(self, status: str, message: str = "", current_data: dict = None, desired_data: dict = None) -> dict:
        """오버레이용 공통 응답 형식."""
        return {
            "status": status,
            "message": message,
            "current_data": current_data if current_data is not None else {},
            "desired_data": desired_data if desired_data is not None else {},
        }

    def get_joint_data_both_modes(self):
        """텍스트 오버레이용 관절 데이터 수집 (current + desired 동시 반환)."""
        if not self._ros2_subscriber_active:
            return self._data_response("disconnected", "ROS2 Inactive")
        try:
            current_joint_groups = {
                "right_arm": getattr(self, "Arm_R_current", []),
                "left_arm": getattr(self, "Arm_L_current", []),
                "waist": getattr(self, "Waist_current", []),
                "neck": getattr(self, "Neck_current", []),
            }
            desired_joint_groups = {
                "right_arm": getattr(self, "Arm_R_desired", []),
                "left_arm": getattr(self, "Arm_L_desired", []),
                "waist": getattr(self, "Waist_desired", []),
                "neck": getattr(self, "Neck_desired", []),
            }
            return self._data_response(
                "active",
                current_data={"joint_groups": current_joint_groups},
                desired_data={"joint_groups": desired_joint_groups},
            )
        except Exception as e:
            return self._data_response("error", str(e))

    def get_hand_joint_data_both_modes(self):
        """손 관절 전용 데이터 수집 (current + desired 동시 반환)."""
        if not self._ros2_subscriber_active:
            empty_hand = {"left_hand": [], "right_hand": []}
            return self._data_response("disconnected", "ROS2 Inactive", empty_hand, empty_hand)
        try:
            limit = ScenarioDisplayConfig.HAND_JOINT_DISPLAY_LIMIT
            cur_l = (getattr(self, "Hand_L_current", []) or [])[:limit]
            cur_r = (getattr(self, "Hand_R_current", []) or [])[:limit]
            des_l = (getattr(self, "Hand_L_desired", []) or [])[:limit]
            des_r = (getattr(self, "Hand_R_desired", []) or [])[:limit]
            return self._data_response(
                "active",
                current_data={"left_hand": cur_l, "right_hand": cur_r},
                desired_data={"left_hand": des_l, "right_hand": des_r},
            )
        except Exception as e:
            empty_hand = {"left_hand": [], "right_hand": []}
            return self._data_response("error", f"Hand Data Error: {str(e)}", empty_hand, empty_hand)

    def set_ros2_manager(self, ros2_manager):
        """ROS2 Manager 설정 및 초기 토픽 모드 캐시."""
        self._ros2_manager = ros2_manager
        self._update_cached_topic_mode()

    def _update_cached_topic_mode(self):
        """토픽 모드 캐시 업데이트"""
        if hasattr(self, '_ros2_manager') and self._ros2_manager:
            try:
                self._cached_topic_mode = self._ros2_manager.get_current_topic_mode()
            except Exception:
                pass

    def _append_joint_lines(self, lines: List[str], names: List[str], cur: List[float], des: List[float], mode: str, name_width: int = 3) -> None:
        """이름 리스트와 current/desired로 format_joint_line 한 블록을 lines에 추가."""
        for i, n in enumerate(names):
            if i >= max(len(cur), len(des)):
                break
            lines.append(format_joint_line(
                n,
                cur[i] if i < len(cur) else 0.0,
                des[i] if i < len(des) else 0.0,
                mode,
                name_width=name_width,
            ))

    def format_all_joint_text(self):
        """모든 관절 데이터를 통합 포맷으로 표시 (current | desired 동시 표시)."""
        joint_data = self.get_joint_data_both_modes()
        if joint_data["status"] != "active":
            return joint_data.get("message", "ROS2 Disconnected")
        mode = self._cached_topic_mode
        current_groups = joint_data["current_data"]["joint_groups"]
        desired_groups = joint_data["desired_data"]["joint_groups"]
        line_width = ScenarioDisplayConfig.OVERLAY_LINE_WIDTH
        lines = ["Current | Desired", "=" * line_width]

        if self._joint_group_display_enabled.get("body", True):
            cur_waist = current_groups.get("waist", [])
            cur_neck = current_groups.get("neck", [])
            des_waist = desired_groups.get("waist", [])
            des_neck = desired_groups.get("neck", [])
            if cur_waist or cur_neck or des_waist or des_neck:
                lines.append(" Body Joints:")
                for label, cur, des in [("Waist:", cur_waist, des_waist), ("Neck:", cur_neck, des_neck)]:
                    if not (cur or des):
                        continue
                    names = ScenarioDisplayConfig.BODY_JOINT_LABELS["waist"] if label == "Waist:" else ScenarioDisplayConfig.BODY_JOINT_LABELS["neck"]
                    lines.append(f"   {label}")
                    self._append_joint_lines(lines, names, cur, des, mode, name_width=2)

        left_arm_on = self._joint_group_display_enabled.get("left_arm", True)
        right_arm_on = self._joint_group_display_enabled.get("right_arm", True)
        if left_arm_on or right_arm_on:
            cur_l = current_groups.get("left_arm", []) if left_arm_on else []
            cur_r = current_groups.get("right_arm", []) if right_arm_on else []
            des_l = desired_groups.get("left_arm", []) if left_arm_on else []
            des_r = desired_groups.get("right_arm", []) if right_arm_on else []
            if cur_l or cur_r or des_l or des_r:
                lines.extend(["=" * line_width, "Arm Joints:"])
                for arm_label, names, cur, des in [
                    ("Left Arm:", ScenarioDisplayConfig.ARM_JOINT_LABELS["left_arm"], cur_l, des_l),
                    ("Right Arm:", ScenarioDisplayConfig.ARM_JOINT_LABELS["right_arm"], cur_r, des_r),
                ]:
                    if not (cur or des):
                        continue
                    lines.append(f"   {arm_label}")
                    self._append_joint_lines(lines, names, cur, des, mode, name_width=3)
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
        lines = ["Current | Desired", "=" * ScenarioDisplayConfig.OVERLAY_LINE_WIDTH]
        hand_prefix = "  "

        def append_hand_blocks(hand_label: str, cur: list, des: list):
            if not (cur or des):
                lines.append(f"{hand_label}: No Data")
                return
            lines.append(f"{hand_label}:")
            for finger_name, finger_code, indices in ScenarioDisplayConfig.FINGER_GROUPS:
                for joint_num, data_index in enumerate(indices, 1):
                    name = f"L_{finger_code}{joint_num}" if "Left" in hand_label else f"R_{finger_code}{joint_num}"
                    c = cur[data_index] if data_index < len(cur) else 0.0
                    d = des[data_index] if data_index < len(des) else 0.0
                    lines.append(format_joint_line(name, c, d, mode, name_width=3, prefix=hand_prefix))
                if finger_name != "Pinky":
                    lines.append("-" * ScenarioDisplayConfig.OVERLAY_SEP_WIDTH)

        append_hand_blocks("Left Hand", cur_l, des_l)
        lines.append("=" * ScenarioDisplayConfig.OVERLAY_LINE_WIDTH)
        append_hand_blocks("Right Hand", cur_r, des_r)
        return "\n".join(lines)


    # ----- 관절 그룹 표시 (체크박스) -----

    def set_joint_group_enabled(self, group_name: str, enabled: bool):
        """특정 관절 그룹 표시 활성화/비활성화 (손 관절은 윈도우 표시/숨김 제어)."""
        if group_name not in self._joint_group_display_enabled:
            return
        self._joint_group_display_enabled[group_name] = enabled
        if group_name == "hand":
            self._toggle_hand_window_visibility(enabled)
        else:
            self._toggle_joint_window_visibility()
    
    def get_all_group_states(self) -> dict:
        """모든 관절 그룹 상태 반환"""
        return self._joint_group_display_enabled.copy()
    
    def set_all_groups_enabled(self, enabled: bool):
        """모든 관절 그룹 활성화/비활성화 (손 관절 윈도우 표시/숨김 포함)."""
        for group_name in self._joint_group_display_enabled:
            self._joint_group_display_enabled[group_name] = enabled
        self._toggle_joint_window_visibility()
        self._toggle_hand_window_visibility(enabled)

    def _toggle_hand_window_visibility(self, show: bool):
        """손 관절 윈도우 표시/숨김 (JointOverlayUI에 위임). 표시 시 내용 갱신."""
        if self._overlay_ui is None:
            return
        self._overlay_ui.set_hand_window_visibility(show)
        if show:
            self._overlay_ui.update_hand_display_text(self)

    def _toggle_joint_window_visibility(self):
        """관절 윈도우 표시/숨김 (body, left_arm, right_arm 중 하나라도 ON이면 표시)."""
        if self._overlay_ui is None:
            return
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
        try:
            indices = resolve_dof_indices(self._articulation, ROS2Config.ALLEX_ACTION_JOINT_FULL_NAMES)
            if len(indices) != ScenarioDisplayConfig.POLICY_ACTION_JOINT_COUNT:
                print(f"❌ Policy action joint indices 불일치: 기대 {ScenarioDisplayConfig.POLICY_ACTION_JOINT_COUNT}, 실제 {len(indices)}")
                return False
            self._policy_action_dof_indices = indices
            return True
        except Exception as e:
            print(f"❌ Policy action DOF indices 캐시 실패: {e}")
            traceback.print_exc()
            return False

    def apply_policy_action(self, joint_values: list) -> bool:
        """Policy action (18개 조인트 위치)을 articulation에 적용. 메인 루프에서만 적용되도록 버퍼에 저장."""
        if self._articulation is None:
            return False
        if len(joint_values) != ScenarioDisplayConfig.POLICY_ACTION_JOINT_COUNT:
            return False
        try:
            if self._policy_action_dof_indices is None and not self._cache_policy_action_dof_indices():
                return False
            self._pending_policy_action = np.array(joint_values, dtype=np.float32)
            return True
        except Exception as e:
            print(f"❌ Policy action 적용 실패: {e}")
            traceback.print_exc()
            return False