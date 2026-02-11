"""
env.yaml / agent.yaml 파싱 및 Isaac Sim(디버거) 환경과 비교.
학습 시 사용한 env·agent 설정과 현재 디버거/정책 구성을 대조해 불일치 시 UI에서 경고.
"""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

LOG = logging.getLogger(__name__)

try:
    import yaml
    _YAML_AVAILABLE = True
except ImportError:
    _YAML_AVAILABLE = False


# env.yaml에서 추출하는 항목 (학습 환경 스펙)
@dataclass
class EnvSummary:
    """env.yaml에서 파싱한 학습 환경 요약."""
    num_joints: Optional[int] = None
    joint_names: List[str] = field(default_factory=list)
    physics_dt: Optional[float] = None
    decimation: Optional[int] = None
    policy_history_length: Optional[int] = None
    residual_scale: Optional[float] = None
    # 관측은 env 구조와 동일하다고 가정 시 총 차원 (last_actions 18*history + proprio)
    expected_obs_dim: Optional[int] = None
    raw: Dict[str, Any] = field(default_factory=dict)

    @property
    def action_dim_from_joints(self) -> Optional[int]:
        """관절 수 기준 액션 차원 (residual 18 + playback_speed 1 = 19)."""
        if self.num_joints is None:
            return None
        return self.num_joints + 1


@dataclass
class AgentSummary:
    """agent.yaml에서 파싱한 요약 (표시용)."""
    experiment_name: Optional[str] = None
    run_name: Optional[str] = None
    class_name: Optional[str] = None
    raw: Dict[str, Any] = field(default_factory=dict)


@dataclass
class EnvAgentCheckResult:
    """비교 결과: 일치 여부와 불일치 메시지 목록."""
    env_summary: Optional[EnvSummary] = None
    agent_summary: Optional[AgentSummary] = None
    mismatches: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    yaml_loaded: bool = False

    @property
    def ok(self) -> bool:
        return self.yaml_loaded and len(self.mismatches) == 0


def _safe_yaml_load(path: str, use_unsafe: bool = False) -> Optional[Dict[str, Any]]:
    """YAML 파일 로드. !!python/ 태그가 있으면 UnsafeLoader 시도."""
    if not _YAML_AVAILABLE:
        LOG.warning("PyYAML not available; cannot parse env/agent YAML")
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = f.read()
    except OSError as e:
        LOG.warning("Failed to read YAML %s: %s", path, e)
        return None

    if use_unsafe:
        try:
            return yaml.load(raw, Loader=getattr(yaml, "UnsafeLoader", yaml.FullLoader))
        except Exception as e:
            LOG.warning("YAML unsafe load failed for %s: %s", path, e)
            return None
    try:
        return yaml.safe_load(raw)
    except yaml.YAMLError as e:
        LOG.debug("YAML safe_load failed (e.g. !!python/ tags), trying full load: %s", e)
        try:
            return yaml.load(raw, Loader=getattr(yaml, "UnsafeLoader", yaml.FullLoader))
        except Exception as e2:
            LOG.warning("YAML load failed for %s: %s", path, e2)
            return None


def load_env_yaml(file_path: str) -> Optional[EnvSummary]:
    """env.yaml 경로에서 EnvSummary 추출."""
    data = _safe_yaml_load(file_path, use_unsafe=True)
    if not data or not isinstance(data, dict):
        return None

    out = EnvSummary(raw=data)

    # sim.dt
    try:
        sim = data.get("sim") or {}
        if isinstance(sim, dict) and "dt" in sim:
            out.physics_dt = float(sim["dt"])
    except (TypeError, ValueError):
        pass

    # decimation
    try:
        if "decimation" in data:
            out.decimation = int(data["decimation"])
    except (TypeError, ValueError):
        pass

    # actions.action: joint_names, residual_scale
    try:
        actions = data.get("actions") or {}
        action_cfg = actions.get("action") if isinstance(actions, dict) else None
        if isinstance(action_cfg, dict):
            jn = action_cfg.get("joint_names")
            if isinstance(jn, list) and jn:
                out.joint_names = [str(x) for x in jn]
                out.num_joints = len(out.joint_names)
            rs = action_cfg.get("residual_scale")
            if rs is not None:
                out.residual_scale = float(rs)
    except (TypeError, ValueError):
        pass

    # observations.policy.history_length
    try:
        obs = data.get("observations") or {}
        policy_obs = obs.get("policy") if isinstance(obs, dict) else None
        if isinstance(policy_obs, dict) and "history_length" in policy_obs:
            out.policy_history_length = int(policy_obs["history_length"])
    except (TypeError, ValueError):
        pass

    # expected_obs_dim: 학습과 동일한 구조 가정 (last_actions 18*history + joint_pos 18 + ref 18 + torque 19 + base_pos 7)
    if out.policy_history_length is not None and out.num_joints is not None:
        last_actions_dim = out.num_joints * out.policy_history_length
        # proprio: joint_pos(18) + reference_joint_pos(18) + right_hand_joint_torque(19) + right_hand_base_pos(7)
        proprio_dim = out.num_joints + out.num_joints + 19 + 7  # 18+18+19+7 when num_joints=18
        out.expected_obs_dim = last_actions_dim + proprio_dim

    return out


def load_agent_yaml(file_path: str) -> Optional[AgentSummary]:
    """agent.yaml 경로에서 AgentSummary 추출 (표시용)."""
    data = _safe_yaml_load(file_path, use_unsafe=False)
    if not data or not isinstance(data, dict):
        return None

    out = AgentSummary(raw=data)
    out.experiment_name = data.get("experiment_name") if isinstance(data.get("experiment_name"), str) else None
    out.run_name = data.get("run_name") if isinstance(data.get("run_name"), str) else None
    out.class_name = data.get("class_name") if isinstance(data.get("class_name"), str) else None
    return out


def find_yaml_dir_for_policy(policy_path: str) -> Optional[str]:
    """
    로드된 .pt/.pth 파일에 대응하는 env/agent YAML이 있을 수 있는 디렉터리 반환.
    - policy/best_260211_1.pt → policy/best_260211_1_yaml/
    - policy/best_260211_1.pt → policy/ (같은 디렉터리의 env.yaml)
    """
    base = os.path.splitext(os.path.basename(policy_path))[0]
    parent = os.path.dirname(os.path.abspath(policy_path))

    # 1) <name>_yaml 폴더
    candidate = os.path.join(parent, base + "_yaml")
    if os.path.isdir(candidate):
        return candidate
    # 2) <name> 폴더
    candidate = os.path.join(parent, base)
    if os.path.isdir(candidate):
        return candidate
    # 3) policy와 같은 디렉터리
    if os.path.isfile(os.path.join(parent, "env.yaml")):
        return parent
    return None


def load_env_agent_from_policy_path(policy_path: str) -> Tuple[Optional[EnvSummary], Optional[AgentSummary], bool]:
    """
    policy_path 기준으로 env.yaml, agent.yaml을 찾아 로드.
    Returns: (EnvSummary or None, AgentSummary or None, yaml_loaded_any).
    """
    yaml_dir = find_yaml_dir_for_policy(policy_path)
    if not yaml_dir:
        return None, None, False

    env_path = os.path.join(yaml_dir, "env.yaml")
    agent_path = os.path.join(yaml_dir, "agent.yaml")

    env_summary = load_env_yaml(env_path) if os.path.isfile(env_path) else None
    agent_summary = load_agent_yaml(agent_path) if os.path.isfile(agent_path) else None
    return env_summary, agent_summary, (env_summary is not None or agent_summary is not None)


def compare_with_debugger(
    env_summary: Optional[EnvSummary],
    agent_summary: Optional[AgentSummary],
    num_joints: int,
    expected_total_obs_dim: int,
    policy_obs_dim: Optional[int],
    policy_action_dim: Optional[int],
    residual_scale_ui: float = 0.1,
) -> EnvAgentCheckResult:
    """
    env/agent 요약과 현재 디버거·정책 구성을 비교해 불일치 목록을 반환.
    """
    result = EnvAgentCheckResult(
        env_summary=env_summary,
        agent_summary=agent_summary,
        yaml_loaded=env_summary is not None or agent_summary is not None,
    )

    if env_summary is None:
        if result.yaml_loaded and agent_summary is not None:
            result.warnings.append("env.yaml를 찾지 못했거나 파싱 실패. 관절/obs 차원 비교 생략.")
        return result

    # 관절 수
    if env_summary.num_joints is not None and env_summary.num_joints != num_joints:
        result.mismatches.append(
            f"관절 수 불일치: env.yaml={env_summary.num_joints}, 디버거 config={num_joints}"
        )

    # 관측 차원 (env 기대 vs 디버거 OBS_SPECS 총합 vs 로드된 정책)
    if env_summary.expected_obs_dim is not None:
        if env_summary.expected_obs_dim != expected_total_obs_dim:
            result.mismatches.append(
                f"관측 총 차원 불일치: env(학습)={env_summary.expected_obs_dim}, 디버거 OBS_SPECS={expected_total_obs_dim}"
            )
        if policy_obs_dim is not None and env_summary.expected_obs_dim != policy_obs_dim:
            result.mismatches.append(
                f"관측 차원 vs 정책: env(학습)={env_summary.expected_obs_dim}, 로드된 정책 obs_dim={policy_obs_dim}"
            )

    if policy_obs_dim is not None and policy_obs_dim != expected_total_obs_dim:
        result.mismatches.append(
            f"정책 obs_dim({policy_obs_dim}) ≠ 디버거 기대 obs_dim({expected_total_obs_dim})"
        )

    # 액션 차원: 정책은 보통 18(관절) 또는 19(관절+playback_speed)
    if env_summary.num_joints is not None and policy_action_dim is not None:
        expected_action = env_summary.action_dim_from_joints  # 19
        if expected_action is not None and policy_action_dim != expected_action:
            # 18만 출력하는 모델도 있음 (speed 별도 처리)
            if policy_action_dim != env_summary.num_joints and policy_action_dim != expected_action:
                result.warnings.append(
                    f"액션 차원: env 기준 {expected_action}(관절+speed), 정책 출력={policy_action_dim}"
                )

    # residual_scale (참고용)
    if env_summary.residual_scale is not None and abs(env_summary.residual_scale - residual_scale_ui) > 1e-6:
        result.warnings.append(
            f"residual_scale: env(학습)={env_summary.residual_scale}, UI={residual_scale_ui}"
        )

    return result


__all__ = [
    "EnvSummary",
    "AgentSummary",
    "EnvAgentCheckResult",
    "load_env_yaml",
    "load_agent_yaml",
    "find_yaml_dir_for_policy",
    "load_env_agent_from_policy_path",
    "compare_with_debugger",
]
