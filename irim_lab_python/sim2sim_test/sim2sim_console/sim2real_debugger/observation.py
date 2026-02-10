"""Observation layer: thread-safe store and canonical-key normalizer."""

import threading
from typing import Dict, List

import numpy as np

from .config import ObsDict


class ObservationStore:
    """ROS thread / UI thread 간 데이터 경합 방지용 store."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data: Dict[str, np.ndarray] = {}

    def set(self, key: str, value: np.ndarray) -> None:
        with self._lock:
            self._data[key] = value

    def clear(self) -> None:
        with self._lock:
            self._data.clear()

    def snapshot(self) -> ObsDict:
        with self._lock:
            return dict(self._data)


class ObsNormalizer:
    """raw dict에서 alias를 허용하면서 canonical key로 정규화."""

    def __init__(self):
        self._aliases: Dict[str, List[str]] = {
            "last_actions": ["last_actions", "actions"],
            "hammer_pos": ["hammer_pos"],
            "joint_pos": ["joint_pos"],
            "reference_joint_pos": ["reference_joint_pos", "ref_pos"],
            "right_hand_joint_torque": ["right_hand_joint_torque", "torque"],
            "right_hand_base_pos": ["right_hand_base_pos", "hand_base_pos"],
            "target_right_hand_pose": ["target_right_hand_pose", "target_hand_pose"],
        }

    def normalize(self, raw: ObsDict) -> ObsDict:
        out: ObsDict = {}
        for canonical, keys in self._aliases.items():
            for k in keys:
                if k in raw:
                    out[canonical] = raw[k]
                    break
        return out


__all__ = ["ObservationStore", "ObsNormalizer"]
