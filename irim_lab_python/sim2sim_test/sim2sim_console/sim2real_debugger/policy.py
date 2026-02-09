import logging
from typing import Any, Dict, Optional, Tuple

import numpy as np
import torch

LOG = logging.getLogger(__name__)


class PolicyWrapper:
    def __init__(self):
        self.model: Optional[Any] = None
        self.expected_obs_dim: Optional[int] = None
        self.expected_action_dim: Optional[int] = None

    def unload(self) -> None:
        self.model = None
        self.expected_obs_dim = None
        self.expected_action_dim = None

    def load(self, file_path: str) -> Tuple[str, Optional[int], Optional[int]]:
        """return (model_type, obs_dim, action_dim)"""
        try:
            self.model = torch.jit.load(file_path, map_location="cpu")
            model_type = "JIT Script"
        except Exception:
            loaded_obj = torch.load(file_path, map_location="cpu")
            model_type = "PyTorch Model"

            if isinstance(loaded_obj, dict):
                if "model" in loaded_obj:
                    self.model = loaded_obj["model"]
                    model_type = "PyTorch Model (dict['model'])"
                elif "policy" in loaded_obj:
                    self.model = loaded_obj["policy"]
                    model_type = "PyTorch Model (dict['policy'])"
                elif "actor" in loaded_obj:
                    self.model = loaded_obj["actor"]
                    model_type = "PyTorch Model (dict['actor'])"
                elif "model_state_dict" in loaded_obj:
                    raise ValueError(
                        "체크포인트에 'model_state_dict'만 있습니다. 모델 객체가 필요합니다.\n"
                        "1) torch.jit.save()로 저장된 .pt 사용 또는\n"
                        "2) 모델 아키텍처 로드 후 load_state_dict() 사용"
                    )
                else:
                    raise ValueError(f"체크포인트 dict에서 모델 객체 키를 찾지 못했습니다. keys={list(loaded_obj.keys())}")
            else:
                self.model = loaded_obj

        if self.model is None or not callable(self.model):
            raise ValueError(f"Loaded policy is not callable. type={type(self.model)}")

        try:
            if hasattr(self.model, "eval"):
                self.model.eval()
        except Exception:
            pass

        obs_dim, action_dim = self._infer_io_dims(self.model)
        self.expected_obs_dim = obs_dim
        self.expected_action_dim = action_dim
        return model_type, obs_dim, action_dim

    @staticmethod
    def _state_dict_from_any(obj) -> Optional[Dict[str, torch.Tensor]]:
        if isinstance(obj, dict):
            if "model_state_dict" in obj and isinstance(obj["model_state_dict"], dict):
                sd = obj["model_state_dict"]
                if sd and isinstance(next(iter(sd.values())), torch.Tensor):
                    return sd
            if obj and isinstance(next(iter(obj.values())), torch.Tensor):
                return obj

        if hasattr(obj, "state_dict"):
            try:
                sd = obj.state_dict()
                if sd and isinstance(next(iter(sd.values())), torch.Tensor):
                    return sd
            except Exception:
                return None
        return None

    @staticmethod
    def _find_actor_linear_weight(sd: Dict[str, torch.Tensor], first: bool) -> Optional[torch.Tensor]:
        items = []
        for k, v in sd.items():
            if not isinstance(v, torch.Tensor) or v.ndim != 2:
                continue
            if not (k.startswith("actor.") and k.endswith(".weight")):
                continue
            mid = k[len("actor.") : -len(".weight")]
            if mid.isdigit():
                items.append((int(mid), v))
        if not items:
            return None
        items.sort(key=lambda t: t[0])
        return items[0][1] if first else items[-1][1]

    def _infer_io_dims(self, policy_obj) -> Tuple[Optional[int], Optional[int]]:
        sd = self._state_dict_from_any(policy_obj)
        if sd is None:
            return None, None

        obs_dim: Optional[int] = None
        for k in (
            "actor_obs_normalizer._mean",
            "obs_normalizer._mean",
            "actor_obs_normalizer.mean",
            "obs_normalizer.mean",
        ):
            if k in sd and isinstance(sd[k], torch.Tensor):
                obs_dim = int(sd[k].numel())
                break
        if obs_dim is None:
            w0 = self._find_actor_linear_weight(sd, first=True)
            if w0 is not None:
                obs_dim = int(w0.shape[1])

        action_dim: Optional[int] = None
        if "std" in sd and isinstance(sd["std"], torch.Tensor):
            action_dim = int(sd["std"].numel())
        else:
            w_last = self._find_actor_linear_weight(sd, first=False)
            if w_last is not None:
                action_dim = int(w_last.shape[0])

        return obs_dim, action_dim

    def forward(self, obs_vec: np.ndarray, expected_action_dim: int) -> np.ndarray:
        if self.model is None:
            return np.zeros(expected_action_dim, dtype=np.float32)

        try:
            obs_tensor = torch.from_numpy(np.asarray(obs_vec, dtype=np.float32)).unsqueeze(0)
            with torch.inference_mode():
                out = self.model(obs_tensor)

            if isinstance(out, torch.Tensor):
                residual = out.squeeze(0).detach().cpu().numpy().astype(np.float32, copy=False)
            else:
                LOG.warning("Policy output is not torch.Tensor: %s", type(out))
                residual = np.zeros(expected_action_dim, dtype=np.float32)

            if residual.size != expected_action_dim:
                if residual.size < expected_action_dim and residual.size >= 1:
                    # 18차원 출력(관절만) 등: 부족분만 0으로 패딩 (speed 등은 대시보드/기본값 사용)
                    LOG.warning("Policy output dim mismatch: expected=%d got=%d (padding with zeros)", expected_action_dim, residual.size)
                    out = np.zeros(expected_action_dim, dtype=np.float32)
                    out[: residual.size] = residual.reshape(-1)
                    return out
                LOG.warning("Policy output dim mismatch: expected=%d got=%d", expected_action_dim, residual.size)
                return np.zeros(expected_action_dim, dtype=np.float32)

            return residual
        except Exception:
            LOG.exception("Policy forward failed")
            return np.zeros(expected_action_dim, dtype=np.float32)


__all__ = ["PolicyWrapper"]
