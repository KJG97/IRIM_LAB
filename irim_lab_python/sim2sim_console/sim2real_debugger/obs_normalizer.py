from typing import Dict, List

from .types import ObsDict


class ObsNormalizer:
    """raw dict에서 alias를 허용하면서 canonical key로 정규화."""

    def __init__(self):
        # canonical -> aliases (우선순위 순)
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


__all__ = ["ObsNormalizer"]
