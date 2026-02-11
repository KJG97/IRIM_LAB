import logging
from typing import Callable, List, Optional

import numpy as np

from .config import (
    ALLEX_ACTION_JOINT_NAMES,
    ALLEX_INIT_POSE,
    DebuggerConfig,
    EXPECTED_TOTAL_OBS_DIM,
    OBS_SPECS,
    ObsDict,
)
from .policy import PolicyWrapper

LOG = logging.getLogger(__name__)

# 점진적 리셋 완료 판정: 이 각도(rad) 이하면 완료
RESET_DONE_THRESHOLD_RAD = 0.01


class InferenceEngine:
    """trajectory playback + obs packing + policy forward + action/ref_error injection"""

    def __init__(self, cfg: DebuggerConfig, policy: PolicyWrapper):
        self.cfg = cfg
        self.policy = policy

        self.running = False
        self.base_traj_hz = 50.0
        self.playback_time = 0.0
        self.speed = 1.0  # 학습 시 cfg.playback_speed=1.0과 일치
        self.residual_scale = 0.1
        self.loop = False  # 학습과 동일하게 기본값 False (trajectory 끝나면 마지막 프레임 유지)
        self._trajectory_finished = False
        self._elapsed_time: float = 0.0
        self._max_duration_s: float = float(cfg.infer_duration_s)
        self._last_ref_frame: Optional[np.ndarray] = None
        # streaming obs가 잠시 stale될 때 마지막 유효 값을 사용하는 캐시
        self._streaming_cache: dict[str, np.ndarray] = {}
        # obs_vec이 일시적으로 생성 실패하면 마지막 성공 값을 재사용
        self._last_obs_vec: Optional[np.ndarray] = None

        self.trajectory: Optional[dict] = None

        # publish callback (ROS2 publish_policy_action)
        self.publish_action_cb: Optional[Callable[[np.ndarray], None]] = None
        # debug/telemetry callback (직전 적용 action 송신용)
        self.publish_last_action_cb: Optional[Callable[[np.ndarray], None]] = None

        # streaming freshness checker (canonical name 기준)
        self.is_fresh_cb: Optional[Callable[[str], bool]] = None

        # obs "last_actions" 용: Isaac Lab과 동일하게 action_manager.action = policy 출력(raw residual) 18차원
        self._last_actions: Optional[np.ndarray] = None
        # last_actions history: 최근 3스텝 raw residual 18차원 → concat 54차원
        self._action_history: list[np.ndarray] = []
        self._history_length: int = 3  # 학습 시 PolicyCfg.history_length와 동일
        self._last_actions_obs_dim: int = 18  # num_joints, 학습 last_action과 동일
        # playback_speed 제어 범위 (PPO의 19번째 출력으로 제어)
        # 학습 시 ResidualJointPositionActionCfg의 playback_speed_min/max와 동일
        self._speed_min: float = 0.8
        self._speed_max: float = 1.2
        # inference 중 stop할 때 유지할 마지막 publish된 action (ref + residual*scale)
        self._last_published_action: Optional[np.ndarray] = None
        # 터미널 로그용: 마지막 policy 출력(raw_actions, scale 적용 전)
        self._last_raw_actions_for_log: Optional[np.ndarray] = None
        # 초기 자세로 점진적 리셋 관련 상태
        self._is_resetting: bool = False
        self._reset_start_pose: Optional[np.ndarray] = None  # 리셋 시작 시점의 관절 위치
        self._reset_speed: float = 0.1  # 매 스텝마다 이동할 비율 (0.05 = 5%씩, 약 20 스텝에 완료)

    def set_trajectory(self, traj: Optional[dict]) -> None:
        self.trajectory = traj
        self._trajectory_finished = False
        self._last_ref_frame = None
        self._streaming_cache.clear()
        self._last_obs_vec = None

    def set_publish_callback(self, cb: Optional[Callable[[np.ndarray], None]]) -> None:
        self.publish_action_cb = cb

    def set_last_action_publish_callback(self, cb: Optional[Callable[[np.ndarray], None]]) -> None:
        self.publish_last_action_cb = cb

    def set_freshness_callback(self, cb: Optional[Callable[[str], bool]]) -> None:
        self.is_fresh_cb = cb

    def start(
        self,
        speed: float,
        residual_scale: float,
        max_duration_s: Optional[float] = None,
    ) -> None:
        # 학습 시와 동일하게 playback_speed는 항상 1.0으로 시작
        # PPO가 출력하는 playback_speed로 즉시 업데이트됨
        # (speed 파라미터는 무시됨 - PPO가 제어하므로)
        self.speed = 1.0
        self.residual_scale = float(np.clip(residual_scale, 0.0, 1.0))
        if max_duration_s is not None:
            self._max_duration_s = float(max_duration_s)
        self.playback_time = 0.0
        self._elapsed_time = 0.0
        self.running = True
        self._trajectory_finished = False
        self._last_ref_frame = None
        self._streaming_cache.clear()
        self._last_obs_vec = None
        # last_actions obs = raw residual history (학습과 동일: action_manager.action)
        self._last_actions = None
        self._last_published_action = None
        self._last_raw_actions_for_log = None
        self._is_resetting = False
        self._reset_start_pose = None
        zero_18 = np.zeros(self._last_actions_obs_dim, dtype=np.float32)
        self._action_history = [zero_18.copy() for _ in range(self._history_length)]

    def stop(self) -> None:
        self.running = False
        # stop 시 마지막 publish된 action은 유지 (inference 중 stop한 경우 마지막 액션 유지용)
        # self._last_published_action은 유지됨
        self._last_actions = None  # obs용 last_actions는 초기화
        self._trajectory_finished = True
        # stop 후에도 마지막 ref 프레임을 캐시로 유지
        # streaming 캐시는 유지(스테일 보간용)
        # obs_vec 캐시는 유지 (종료 후 관측 실패 시 재사용)

    def reset_to_init_pose(self) -> None:
        """초기 자세로 점진적 리셋 시작: 현재 위치에서 초기 자세로 천천히 이동."""
        if self._last_published_action is not None and self._last_published_action.size == self.cfg.num_joints:
            self._reset_start_pose = self._last_published_action.copy()
        else:
            self._reset_start_pose = self._get_init_pose().copy()
        self._is_resetting = True
        LOG.info("Starting gradual reset to initial pose")

    def _get_init_pose(self) -> np.ndarray:
        """초기 자세 18차원 (num_joints에 맞춤)."""
        out = np.array(ALLEX_INIT_POSE, dtype=np.float32)
        if out.size != self.cfg.num_joints:
            out = np.zeros(self.cfg.num_joints, dtype=np.float32)
        return out

    def _last_actions_vec(self, default_18: Optional[np.ndarray] = None) -> np.ndarray:
        """last_actions obs 54차원: _action_history가 가득 차 있으면 concat, 아니면 default_18 또는 zero로 패딩."""
        if len(self._action_history) == self._history_length:
            return np.concatenate(self._action_history, axis=0).astype(np.float32)
        zero = np.zeros(self._last_actions_obs_dim, dtype=np.float32)
        filler = default_18 if (default_18 is not None and default_18.size == self._last_actions_obs_dim) else zero
        need = self._history_length - len(self._action_history)
        parts = list(self._action_history) + [filler] * need
        return np.concatenate(parts, axis=0).astype(np.float32)

    def _publish_actions(self, actions_18: np.ndarray, last_actions_54: np.ndarray) -> None:
        if self.publish_action_cb is not None:
            self.publish_action_cb(actions_18)
        if self.publish_last_action_cb is not None:
            self.publish_last_action_cb(last_actions_54)

    def _get_joint_pos(self, data: ObsDict) -> Optional[np.ndarray]:
        """data에서 joint_pos 추출 (freshness·크기 검사)."""
        if "joint_pos" not in data:
            return None
        if self.is_fresh_cb is not None and not self.is_fresh_cb("joint_pos"):
            return None
        jp = np.asarray(data["joint_pos"], dtype=np.float32).reshape(-1)
        return jp if jp.size == self.cfg.num_joints else None

    def _push_action_history(self, action_18: Optional[np.ndarray]) -> None:
        """한 프레임(18차원)을 action history에 넣고 길이 유지."""
        if action_18 is not None and action_18.size == self._last_actions_obs_dim:
            self._action_history.append(action_18.copy())
        else:
            self._action_history.append(np.zeros(self._last_actions_obs_dim, dtype=np.float32))
        if len(self._action_history) > self._history_length:
            self._action_history.pop(0)

    def _pack_obs(self, data: ObsDict) -> Optional[np.ndarray]:
        obs = np.empty((EXPECTED_TOTAL_OBS_DIM,), dtype=np.float32)

        for spec in OBS_SPECS:
            if spec.name not in data:
                LOG.warning("obs pack 실패: key missing: %s", spec.name)
                return None
            arr = np.asarray(data[spec.name], dtype=np.float32).reshape(-1)
            if arr.size != spec.dim:
                LOG.warning("obs pack 실패: dim mismatch: %s expected=%d got=%d", spec.name, spec.dim, arr.size)
                return None
            if spec.streaming and self.is_fresh_cb is not None:
                if not self.is_fresh_cb(spec.name):
                    # freshness 실패 시 마지막 유효 캐시 사용
                    cached = self._streaming_cache.get(spec.name)
                    if cached is None or cached.size != spec.dim:
                        LOG.warning("obs pack 실패: stale streaming without cache: %s", spec.name)
                        return None
                    obs[spec.start : spec.start + spec.dim] = cached
                    continue
            # 신선하거나 비스트리밍이면 캐시 업데이트
            if spec.streaming:
                self._streaming_cache[spec.name] = arr
            obs[spec.start : spec.start + spec.dim] = arr

        return obs

    def _ref_action(self, dt: float) -> Optional[np.ndarray]:
        if self.trajectory is None:
            return None
        traj = self.trajectory.get("actions_trajectory")
        if traj is None:
            return None

        traj = np.asarray(traj, dtype=np.float32)
        if traj.ndim != 2 or traj.shape[1] != self.cfg.num_joints:
            return None
        if traj.shape[0] <= 0 or self.base_traj_hz <= 0:
            return None

        # loop=False로 종료된 후에는 마지막 프레임을 그대로 유지
        if self._trajectory_finished and self._last_ref_frame is not None:
            return self._last_ref_frame

        # 학습과 동일: 첫 호출(playback_time=0)에서는 Frame 0을 반환하고,
        # 이후 호출부터 dt만큼 전진 (IsaacLab: reset→sample(t=0), 이후 _update_command에서 t+=dt)
        if self._last_ref_frame is not None:
            self.playback_time += dt * self.speed

        total_frames = float(traj.shape[0])
        phase = self.playback_time * self.base_traj_hz

        if self.loop:
            idx = int(phase % total_frames)
        else:
            # 한 번만 재생: 끝에 도달하면 마지막 프레임을 유지하고 finished 플래그 설정
            if phase >= total_frames - 1:
                self._trajectory_finished = True
                idx = int(total_frames - 1)
                # 더 이상 시간 누적하지 않고 마지막 프레임 캐시
                self.playback_time = (total_frames - 1) / self.base_traj_hz
            else:
                idx = int(phase)

        ref_frame = traj[idx].astype(np.float32, copy=False)
        self._last_ref_frame = ref_frame
        return ref_frame

    def step_and_inject(self, data: ObsDict, dt: float) -> None:
        """trajectory 있으면 actions 주입; running=False면 last_actions=0 또는 유지; 항상 init pose publish로 토픽 유지."""
        init_pose = self._get_init_pose()

        if self.trajectory is None:
            last_54 = np.zeros(self._last_actions_obs_dim * self._history_length, dtype=np.float32)
            data["last_actions"] = last_54
            self._publish_actions(init_pose, last_54)
            return

        # 최대 duration 초과 시 인퍼런스 자동 종료
        if self.running:
            self._elapsed_time += float(dt)
            if self._max_duration_s > 0.0 and self._elapsed_time >= self._max_duration_s:
                LOG.info("Inference auto-stopped: elapsed_time=%.3f s (max_duration_s=%.3f)", self._elapsed_time, self._max_duration_s)
                # stop() 호출 전에 마지막 publish된 action이 이미 저장되어 있음
                self.stop()
                # stop 후에는 다음 루프에서 _last_published_action을 사용하도록 return
                # 여기서는 바로 publish하지 않고, 다음 step_and_inject 호출 시 처리됨
                return

        if not self.running:
            if self._is_resetting:
                current_pose = self._get_joint_pos(data)
                if current_pose is None and self._last_published_action is not None and self._last_published_action.size == self.cfg.num_joints:
                    current_pose = self._last_published_action.copy()
                if current_pose is None and self._reset_start_pose is not None:
                    current_pose = self._reset_start_pose.copy()
                if current_pose is None:
                    current_pose = init_pose.copy()
                if self._reset_start_pose is None:
                    self._reset_start_pose = current_pose.copy()

                diff = init_pose - current_pose
                if np.max(np.abs(diff)) < RESET_DONE_THRESHOLD_RAD:
                    actions_to_publish = init_pose.copy()
                    self._is_resetting = False
                    self._reset_start_pose = None
                    LOG.info("Reset to initial pose completed")
                else:
                    actions_to_publish = (current_pose + diff * self._reset_speed).astype(np.float32, copy=False)
                self._last_published_action = actions_to_publish.copy()
                last_54 = np.zeros(self._last_actions_obs_dim * self._history_length, dtype=np.float32)
                data["last_actions"] = last_54
                self._publish_actions(actions_to_publish, last_54)
                return

            actions_to_publish = (
                self._last_published_action.astype(np.float32, copy=False)
                if self._last_published_action is not None and self._last_published_action.size == self.cfg.num_joints
                else init_pose
            )
            last_54 = np.zeros(self._last_actions_obs_dim * self._history_length, dtype=np.float32)
            data["last_actions"] = last_54
            self._last_actions = None
            self._publish_actions(actions_to_publish, last_54)
            return

        ref = self._ref_action(dt)
        if ref is None:
            last_54 = np.zeros(self._last_actions_obs_dim * self._history_length, dtype=np.float32)
            data["last_actions"] = last_54
            self._publish_actions(init_pose, last_54)
            return

        data["reference_joint_pos"] = ref.astype(np.float32, copy=False)
        self._push_action_history(self._last_actions)
        data["last_actions"] = self._last_actions_vec(None)

        obs_vec = self._pack_obs(data)
        if obs_vec is not None:
            self._last_obs_vec = obs_vec
        else:
            obs_vec = self._last_obs_vec if self._last_obs_vec is not None else np.zeros(EXPECTED_TOTAL_OBS_DIM, dtype=np.float32)
            if self._last_obs_vec is None:
                LOG.warning("obs_vec 없음: 캐시도 없어 policy 입력 0으로 대체")

        expected_dim = self.policy.expected_action_dim if self.policy.expected_action_dim is not None else self.cfg.action_dim
        policy_output = self.policy.forward(obs_vec, expected_dim)
        raw_actions = policy_output[: self.cfg.num_joints].astype(np.float32, copy=False)
        if policy_output.size > self.cfg.num_joints:
            raw_speed = float(policy_output[self.cfg.num_joints])
            self.speed = self._speed_min + float(np.clip((raw_speed + 1.0) / 2.0, 0.0, 1.0)) * (self._speed_max - self._speed_min)
        data["playback_speed"] = np.array([self.speed], dtype=np.float32)

        actions = (ref + raw_actions * self.residual_scale).astype(np.float32, copy=False)
        self._last_published_action = actions.copy()
        self._last_raw_actions_for_log = raw_actions.copy()
        self._last_actions = raw_actions.copy()
        self._push_action_history(raw_actions)
        data["last_actions"] = self._last_actions_vec(None)
        self._publish_actions(actions, data["last_actions"])


class TrajectoryLoader:
    """NPZ 궤적 파일 로드 (actions_trajectory 등 인퍼런스 입력 형식으로 반환)."""

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

        if joint_names is not None:
            name_to_idx = {n: i for i, n in enumerate(joint_names)}
            idxs = [name_to_idx[name] for name in ALLEX_ACTION_JOINT_NAMES if name in name_to_idx]
            if len(idxs) == len(ALLEX_ACTION_JOINT_NAMES):
                action_indices = idxs
                action_ok = True
                order_ok = all(joint_names[i] == n for i, n in zip(action_indices, ALLEX_ACTION_JOINT_NAMES))

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
            "actions_trajectory": actions_trajectory,
            "order_ok": bool(order_ok),
            "action_ok": bool(action_ok),
        }


__all__ = ["InferenceEngine", "TrajectoryLoader"]
