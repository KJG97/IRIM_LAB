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

        # 직전 스텝에 적용된 관절 위치 18차원 (obs "last_actions" 용, 학습과 동일하게 action_manager.action만)
        self._last_actions: Optional[np.ndarray] = None
        # last_actions history: 최근 3개 스텝의 18차원 관절 위치 → concat 시 54차원
        self._action_history: list[np.ndarray] = []
        self._history_length: int = 3  # 학습 시 PolicyCfg.history_length와 동일
        self._last_actions_obs_dim: int = 18  # num_joints, 학습 last_action과 동일
        # playback_speed 제어 범위 (PPO의 19번째 출력으로 제어)
        # 학습 시 ResidualJointPositionActionCfg의 playback_speed_min/max와 동일
        self._speed_min: float = 0.8
        self._speed_max: float = 1.2
        # inference 중 stop할 때 유지할 마지막 publish된 action (ref + residual*scale)
        self._last_published_action: Optional[np.ndarray] = None
        # 초기 자세로 점진적 리셋 관련 상태
        self._is_resetting: bool = False
        self._reset_start_pose: Optional[np.ndarray] = None  # 리셋 시작 시점의 관절 위치
        self._reset_speed: float = 0.1  # 매 스텝마다 이동할 비율 (0.05 = 5%씩, 약 20 스텝에 완료)
        self._debug_step_cnt: int = 0

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
        # 학습과 동일: last_actions obs는 18차원 관절 위치만 (54차원 = 18*3)
        self._last_actions = None
        self._last_published_action = None  # inference 시작 시 초기화
        self._is_resetting = False  # inference 시작 시 리셋 상태 초기화
        self._reset_start_pose = None
        self._debug_step_cnt = 0  # 디버그 출력 카운터 리셋
        # last_actions obs: 학습과 동일하게 18차원 관절 위치만 history (54차원)
        init_action_18 = np.zeros(self._last_actions_obs_dim, dtype=np.float32)
        self._action_history = [init_action_18.copy() for _ in range(self._history_length)]

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
        # 현재 관절 위치를 시작점으로 설정 (다음 step_and_inject에서 joint_pos를 받아서 설정)
        # 일단 _last_published_action을 시작점으로 사용 (없으면 초기 자세)
        if self._last_published_action is not None and self._last_published_action.size == self.cfg.num_joints:
            self._reset_start_pose = self._last_published_action.copy()
        else:
            # 현재 위치를 모를 경우 초기 자세로 바로 설정
            init_pose = np.array(ALLEX_INIT_POSE, dtype=np.float32)
            if init_pose.size != self.cfg.num_joints:
                init_pose = np.zeros(self.cfg.num_joints, dtype=np.float32)
            self._reset_start_pose = init_pose.copy()
        
        self._is_resetting = True
        # action history 버퍼는 리셋하지 않음 (현재 상태 유지)
        LOG.info("Starting gradual reset to initial pose")

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
        """원본 동작 유지:
        - trajectory가 있으면 actions 주입
        - running=False: last_actions=0
        - running=True: actions=ref + residual*scale, ref_error=joint_pos-actions (가능할 때)
        추가: obs["last_actions"]에는 직전 스텝에 적용된 action을 넣어 학습 시 last_action과 일치시킴.
        추가: inference가 안 돌아가도 대시보드가 켜져있으면 초기 자세(init pose) action을 계속 publish (ROS 토픽 유지).
        """
        # inference가 안 돌아가는 경우에도 초기 자세를 publish하여 ROS 토픽이 계속 유지되도록 함
        init_pose_actions = np.array(ALLEX_INIT_POSE, dtype=np.float32)
        if init_pose_actions.size != self.cfg.num_joints:
            # 안전장치: 크기가 맞지 않으면 0 벡터로 대체
            init_pose_actions = np.zeros(self.cfg.num_joints, dtype=np.float32)
        
        if self.trajectory is None:
            # trajectory가 없어도 초기 자세를 publish (대시보드가 켜져있으면 토픽 유지)
            # last_actions: 학습과 동일 18*3=54차원
            zero_18 = np.zeros(self._last_actions_obs_dim, dtype=np.float32)
            last_actions_history = np.concatenate([zero_18] * self._history_length, axis=0).astype(np.float32)
            if self.publish_action_cb is not None:
                self.publish_action_cb(init_pose_actions)
            if self.publish_last_action_cb is not None:
                self.publish_last_action_cb(last_actions_history)
            data["last_actions"] = last_actions_history
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
            # 초기 자세로 점진적 리셋 중인 경우
            if self._is_resetting:
                # 현재 관절 위치를 확인 (가능하면)
                current_pose = None
                if "joint_pos" in data:
                    ok = True
                    if self.is_fresh_cb is not None:
                        ok = self.is_fresh_cb("joint_pos")
                    if ok:
                        jp = np.asarray(data["joint_pos"], dtype=np.float32).reshape(-1)
                        if jp.size == self.cfg.num_joints:
                            current_pose = jp
                
                # 현재 위치를 모르면 마지막 publish된 action 사용
                if current_pose is None:
                    if self._last_published_action is not None and self._last_published_action.size == self.cfg.num_joints:
                        current_pose = self._last_published_action.copy()
                    elif self._reset_start_pose is not None:
                        current_pose = self._reset_start_pose.copy()
                    else:
                        current_pose = init_pose_actions.copy()
                
                # 시작 위치가 아직 설정되지 않았으면 현재 위치를 시작점으로 설정
                if self._reset_start_pose is None:
                    self._reset_start_pose = current_pose.copy()
                
                # 현재 위치에서 초기 자세로 점진적으로 보간
                target_pose = init_pose_actions
                diff = target_pose - current_pose
                max_diff = np.max(np.abs(diff))
                
                # 거리가 충분히 가까우면 완료
                if max_diff < 0.01:  # 0.01 rad (약 0.57도) 이하면 완료
                    actions_to_publish = target_pose.copy()
                    self._is_resetting = False
                    self._reset_start_pose = None
                    self._last_published_action = actions_to_publish.copy()
                    LOG.info("Reset to initial pose completed")
                else:
                    # 일정 비율씩 이동
                    actions_to_publish = current_pose + diff * self._reset_speed
                    self._last_published_action = actions_to_publish.copy()
                
                # last_actions: 학습과 동일 18*3=54차원
                if len(self._action_history) == self._history_length:
                    last_actions_history = np.concatenate(self._action_history, axis=0).astype(np.float32)
                else:
                    last_actions_history = np.concatenate([actions_to_publish] * self._history_length, axis=0).astype(np.float32)
                data["last_actions"] = last_actions_history
                if self.publish_action_cb is not None:
                    self.publish_action_cb(actions_to_publish)
                if self.publish_last_action_cb is not None:
                    self.publish_last_action_cb(last_actions_history)
                return
            
            # 리셋 중이 아닌 경우: 기존 로직
            # inference가 실행 중이었다가 stop한 경우: 마지막 publish된 action 유지
            # 그 외의 경우 (처음부터 inference가 안 돌아간 경우): 초기 자세 사용
            if self._last_published_action is not None and self._last_published_action.size == self.cfg.num_joints:
                # inference 중 stop한 경우: 마지막 액션 유지
                actions_to_publish = self._last_published_action.astype(np.float32, copy=False)
            else:
                # 처음부터 inference가 안 돌아간 경우: 초기 자세 사용
                actions_to_publish = init_pose_actions
            
            # last_actions: 학습과 동일 18*3=54차원
            if len(self._action_history) == self._history_length:
                last_actions_history = np.concatenate(self._action_history, axis=0).astype(np.float32)
            else:
                if actions_to_publish.size == self.cfg.num_joints:
                    last_actions_history = np.concatenate([actions_to_publish] * self._history_length, axis=0).astype(np.float32)
                else:
                    zero_18 = np.zeros(self._last_actions_obs_dim, dtype=np.float32)
                    last_actions_history = np.concatenate([zero_18] * self._history_length, axis=0).astype(np.float32)
            data["last_actions"] = last_actions_history
            
            self._last_actions = None
            # inference가 안 돌아가도 마지막 액션 또는 초기 자세를 publish하여 토픽 유지
            if self.publish_action_cb is not None:
                self.publish_action_cb(actions_to_publish)
            if self.publish_last_action_cb is not None:
                self.publish_last_action_cb(last_actions_history)
            return

        ref = self._ref_action(dt)
        if ref is None:
            # ref가 없어도 초기 자세를 publish하여 토픽 유지
            zero_18 = np.zeros(self._last_actions_obs_dim, dtype=np.float32)
            last_actions_history = np.concatenate([zero_18] * self._history_length, axis=0).astype(np.float32)
            if self.publish_action_cb is not None:
                self.publish_action_cb(init_pose_actions)
            if self.publish_last_action_cb is not None:
                self.publish_last_action_cb(last_actions_history)
            data["last_actions"] = last_actions_history
            return
        # loop=False일 때 trajectory가 끝나도 즉시 stop하지 않음 (학습과 동일하게 마지막 프레임 유지)
        # Duration에 도달하거나 사용자가 수동으로 Stop할 때까지 inference 계속
        # 참고: _ref_action()에서 이미 _trajectory_finished=True가 설정되고 마지막 프레임을 반환함

        # joint_pos 확보 (freshness 고려)
        joint_pos = None
        if "joint_pos" in data:
            ok = True
            if self.is_fresh_cb is not None:
                ok = self.is_fresh_cb("joint_pos")
            if ok:
                jp = np.asarray(data["joint_pos"], dtype=np.float32).reshape(-1)
                if jp.size == self.cfg.num_joints:
                    joint_pos = jp

        # obs packing용 last_actions: 학습과 동일 18*3=54차원 (직전 3스텝의 적용된 관절 위치)
        if self._last_actions is not None and self._last_actions.size == self._last_actions_obs_dim:
            self._action_history.append(self._last_actions.copy())
            if len(self._action_history) > self._history_length:
                self._action_history.pop(0)
        else:
            zero_18 = np.zeros(self._last_actions_obs_dim, dtype=np.float32)
            self._action_history.append(zero_18)
            if len(self._action_history) > self._history_length:
                self._action_history.pop(0)
        
        if len(self._action_history) == self._history_length:
            data["last_actions"] = np.concatenate(self._action_history, axis=0).astype(np.float32)
        else:
            padded_history = self._action_history + [
                np.zeros(self._last_actions_obs_dim, dtype=np.float32)
                for _ in range(self._history_length - len(self._action_history))
            ]
            data["last_actions"] = np.concatenate(padded_history, axis=0).astype(np.float32)

        # 학습과 동일: reference_joint_pos (궤적 목표 위치)
        data["reference_joint_pos"] = ref.astype(np.float32, copy=False)

        obs_vec = self._pack_obs(data)
        if obs_vec is not None:
            self._last_obs_vec = obs_vec
        else:
            if self._last_obs_vec is not None:
                LOG.debug("obs_vec 없음: 마지막 성공 obs_vec 재사용")
                obs_vec = self._last_obs_vec
            else:
                LOG.warning("obs_vec 없음: 캐시도 없어 policy 입력 0으로 대체")
                obs_vec = np.zeros(EXPECTED_TOTAL_OBS_DIM, dtype=np.float32)

        # PPO 출력: 18차원(관절만) 또는 19차원(18 joint + 1 speed). 로드된 모델 차원 사용 시 경고 없음
        expected_dim = (
            self.policy.expected_action_dim
            if (self.policy.expected_action_dim is not None)
            else self.cfg.action_dim
        )
        policy_output = self.policy.forward(obs_vec, expected_dim)
        raw_actions = policy_output[:self.cfg.num_joints].astype(np.float32, copy=False)
        if policy_output.size > self.cfg.num_joints:
            raw_speed = float(policy_output[self.cfg.num_joints])
            normalized_speed = (raw_speed + 1.0) / 2.0
            normalized_speed = float(np.clip(normalized_speed, 0.0, 1.0))
            self.speed = self._speed_min + normalized_speed * (self._speed_max - self._speed_min)

        data["playback_speed"] = np.array([self.speed], dtype=np.float32)

        # 최종 명령 = ref + raw*residual_scale (먼저 계산 후 재사용)
        actions = (ref + raw_actions * self.residual_scale).astype(np.float32, copy=False)
        self._last_published_action = actions.copy()

        # inference 실행 중일 때만 10스텝마다 출력
        self._debug_step_cnt += 1
        if self._debug_step_cnt % 10 == 1:
            def _fmt(a: np.ndarray) -> str:
                return " ".join(f"{x:+.4f}" for x in a)
            lines = [
                f"[infer] step={self._debug_step_cnt} speed={self.speed:.3f}",
                f"  raw_policy(18): {_fmt(raw_actions)}",
                f"  ref_traj  (18): {_fmt(ref)}",
                f"  final_act (18): {_fmt(actions)}",
            ]
            if joint_pos is not None:
                pos_err = actions - joint_pos
                lines.append(f"  joint_pos (18): {_fmt(joint_pos)}")
                lines.append(f"  pos_error (18): {_fmt(pos_err)}  (target-current, max={np.max(np.abs(pos_err)):.4f})")
            else:
                lines.append("  joint_pos: N/A")
            torque = data.get("right_hand_joint_torque")
            if torque is not None:
                tarr = np.asarray(torque, dtype=np.float32).reshape(-1)
                lines.append(f"  torque   ({tarr.size:2d}): {_fmt(tarr)}")
            print("\n".join(lines))

        # 다음 스텝 obs용: 적용된 관절 위치 18차원 저장
        self._last_actions = actions.copy()
        self._action_history.append(self._last_actions.copy())
        if len(self._action_history) > self._history_length:
            self._action_history.pop(0)
        if len(self._action_history) == self._history_length:
            data["last_actions"] = np.concatenate(self._action_history, axis=0).astype(np.float32)
        else:
            padded_history = self._action_history + [
                np.zeros(self._last_actions_obs_dim, dtype=np.float32)
                for _ in range(self._history_length - len(self._action_history))
            ]
            data["last_actions"] = np.concatenate(padded_history, axis=0).astype(np.float32)

        if self.publish_last_action_cb is not None:
            self.publish_last_action_cb(data["last_actions"])
        if self.publish_action_cb is not None:
            self.publish_action_cb(actions)


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
                action_ok = True
                action_indices = idxs
                order_ok = [joint_names[i] for i in action_indices] == list(ALLEX_ACTION_JOINT_NAMES)

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
