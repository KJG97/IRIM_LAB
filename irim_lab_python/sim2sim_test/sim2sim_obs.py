"""
Sim2Sim 모드용 관측: Isaac Sim API로 joint_pos, right_hand_joint_torque, right_hand_base_pos 수집.
"""

from typing import Callable, Dict, Optional

import numpy as np

from .config import ROS2Config
from .utils.articulation import resolve_dof_indices

# ObsDict = Dict[str, np.ndarray]
R_HAND_POSE_PRIM_PATH = "/ALLEX/R_Hand_Pose"


def get_sim2sim_observation(articulation) -> Dict[str, np.ndarray]:
    """
    Isaac Sim articulation에서 OBS_SPECS에 맞는 관측만 추출.
    - joint_pos: 18차원 (ALLEX_ACTION_JOINT_FULL_NAMES)
    - right_hand_joint_torque: 19차원 (어깨3 + elbow1 + 손가락15)
    - right_hand_base_pos: 7차원 (R_Hand_Pose 위치 3 + 쿼터니언 4)
    """
    out = {}
    if articulation is None:
        return out
    try:
        # joint_pos (18)
        joint_indices_18 = resolve_dof_indices(articulation, ROS2Config.ALLEX_ACTION_JOINT_FULL_NAMES)
        if joint_indices_18:
            pos = articulation.get_joint_positions(joint_indices=np.array(joint_indices_18, dtype=np.int32))
            if pos is not None:
                out["joint_pos"] = np.asarray(pos, dtype=np.float32).reshape(-1)

        # right_hand_joint_torque (19)
        torque_indices = resolve_dof_indices(articulation, ROS2Config.RIGHT_HAND_TORQUE_JOINT_NAMES)
        if torque_indices:
            efforts = articulation.get_measured_joint_efforts(joint_indices=np.array(torque_indices, dtype=np.int32))
            if efforts is not None:
                out["right_hand_joint_torque"] = np.asarray(efforts, dtype=np.float32).reshape(-1)

        # right_hand_base_pos (7): R_Hand_Pose world position + quaternion
        try:
            from isaacsim.core.utils.xforms import get_world_pose
            pos_w, quat_w = get_world_pose(R_HAND_POSE_PRIM_PATH)
            if pos_w is not None and quat_w is not None:
                out["right_hand_base_pos"] = np.concatenate([
                    np.asarray(pos_w, dtype=np.float32).reshape(3),
                    np.asarray(quat_w, dtype=np.float32).reshape(4),
                ], axis=0)
        except Exception:
            pass
    except Exception:
        pass
    return out


def make_sim2sim_obs_provider(scenario_ref) -> Callable[[], Dict[str, np.ndarray]]:
    """
    시나리오 참조를 받아, 호출 시점에 scenario._sim2sim_obs_cache 사본을 반환하는 provider 생성.
    관측은 시나리오의 physics 콜백에서만 채워지므로, 디버거는 articulation을 직접 읽지 않아
    'copyInternalStateToCache during simulation' 오류를 피함.
    """
    def provider():
        scenario = scenario_ref() if callable(scenario_ref) else scenario_ref
        if scenario is None:
            return {}
        cache = getattr(scenario, "_sim2sim_obs_cache", None) or {}
        return dict(cache)
    return provider
