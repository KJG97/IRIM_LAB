from typing import List, Optional

import numpy as np

from .config import ALLEX_ACTION_JOINT_NAMES


class TrajectoryLoader:
    @staticmethod
    def load_npz(file_path: str, num_joints: int) -> dict:
        data = np.load(file_path, allow_pickle=True)
        if "positions" not in data:
            raise ValueError("NPZ file does not contain 'positions' array.")

        positions = np.asarray(data["positions"])
        if positions.ndim != 2:
            raise ValueError(f"'positions' must be 2D (T, D), got shape {positions.shape}.")

        num_frames, num_cols = positions.shape

        joint_names: Optional[List[str]] = None
        if "joint_names" in data:
            jn_raw = data["joint_names"]
            joint_names = jn_raw.tolist() if isinstance(jn_raw, np.ndarray) else list(jn_raw)

        action_indices = None
        action_ok = False
        order_ok = False
        missing: List[str] = []

        if joint_names is not None:
            name_to_idx = {n: i for i, n in enumerate(joint_names)}
            idxs = []
            for name in ALLEX_ACTION_JOINT_NAMES:
                if name not in name_to_idx:
                    missing.append(name)
                else:
                    idxs.append(name_to_idx[name])
            if not missing:
                action_ok = True
                action_indices = idxs
                subset_names = [joint_names[i] for i in action_indices]
                order_ok = subset_names == ALLEX_ACTION_JOINT_NAMES

        if action_indices is not None:
            actions_trajectory = positions[:, action_indices].astype(np.float32, copy=False)
        else:
            actions_trajectory = positions[:, :num_joints].astype(np.float32, copy=False)

        return {
            "file_path": file_path,
            "num_frames": int(num_frames),
            "num_cols": int(num_cols),
            "joint_names": joint_names,
            "action_indices": action_indices,
            "positions": positions,
            "actions_trajectory": actions_trajectory,  # (T, 18)
            "order_ok": bool(order_ok),
            "action_ok": bool(action_ok),
        }


__all__ = ["TrajectoryLoader"]
