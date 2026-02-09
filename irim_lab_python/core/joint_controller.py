"""
ALLEX Digital Twin 관절 제어 (경량 스텁)

IRIM_LAB 확장에서는 현재 ALLEXJointController의 내부 로직을 사용하지 않고,
외부 코드가 기대하는 인터페이스만 유지하기 위해 최소한의 껍데기만 제공합니다.

- 시뮬레이션에 관절 명령을 실제로 적용하지 않습니다.
- ROS2 콜백/시나리오/텍스트 오버레이 등에서 AttributeError가 나지 않도록
  필요한 속성과 메서드만 빈 구현으로 유지합니다.
"""

from typing import Any, Dict, List, Iterable


class ALLEXJointController:
    """ALLEX 디지털 트윈 관절 제어 스텁 클래스."""

    def __init__(self) -> None:
        # ROS2/시나리오 쪽에서 directly 접근하는 최소 필드들
        self._coupled_joints: Dict[int, Dict[str, Any]] = {}
        self._ros2_subscriber_active: bool = False
        self._topic_mode: str | None = None

        # 텍스트 오버레이/ROS2 상태 표시에서 사용하는 핸드/팔/허리/목 버퍼들
        # 길이는 의미 없고, 존재만 하면 되므로 빈 리스트로 초기화
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

        # 시나리오 참조 (옵션)
        self._scenario_ref: Any | None = None

    # ------------------------------------------------------------------
    # 설정/상태 관련 메서드 (외부에서 호출됨)
    # ------------------------------------------------------------------
    def load_coupled_joint_config(self, config_path: str | None = None) -> None:
        """Coupled Joint 설정 로드 (현재는 아무 것도 하지 않음)."""
        # 기존 코드와의 호환성을 위해 존재만 유지
        if config_path:
            print(f"ℹ️ ALLEXJointController.load_coupled_joint_config() stub – ignore: {config_path}")

    def set_scenario_reference(self, scenario_ref: Any) -> None:
        """Scenario 참조 설정 (시나리오에서 호출)."""
        self._scenario_ref = scenario_ref

    def set_ros2_subscriber_status(self, is_active: bool) -> None:
        """ROS2 Subscriber 활성/비활성 상태 플래그만 유지."""
        self._ros2_subscriber_active = bool(is_active)

    def set_topic_mode(self, topic_mode: str) -> None:
        """토픽 모드 문자열만 캐시."""
        self._topic_mode = topic_mode

    # ------------------------------------------------------------------
    # 관절 명령/목표 관련 메서드 (현재는 no-op)
    # ------------------------------------------------------------------
    def apply_coupled_joints(
        self,
        joint_positions: Iterable[float],
        total_joints: int = 59,
    ) -> List[float]:
        """입력 그대로 또는 길이를 맞춘 리스트를 반환 (실제 Coupled 계산은 수행하지 않음)."""
        positions = list(joint_positions)
        # total_joints 길이에 맞춰 0.0 padding
        if len(positions) < total_joints:
            positions.extend([0.0] * (total_joints - len(positions)))
        return positions

    def update_joint_targets(
        self,
        hand_positions: Iterable[float],
        target_joint_positions: List[float],
        hand_joint_indices: Iterable[int],
    ) -> List[float]:
        """기존 시그니처 유지용 – 간단히 target_joint_positions를 그대로 반환."""
        # 기존 코드와 최대한 비슷한 형태를 유지하되, 실제 로직은 수행하지 않음.
        return list(target_joint_positions)

    def apply_joint_action(self, articulation: Any, action: Any) -> None:
        """관절 액션 적용 – 현재는 실제로 아무 것도 하지 않음."""
        # articulation.apply_action(action) 을 호출하지 않음 (stub)
        return

    def create_joint_control_generator(self, articulation: Any, get_target_positions_func: Any):
        """관절 제어 제너레이터 – 외부 코드와의 호환성을 위한 빈 제너레이터."""
        while True:
            # 원래는 여기서 articulation.apply_action(...) 을 호출했지만
            # 스텁에서는 아무 것도 하지 않고 한 스텝을 넘깁니다.
            yield

    def get_coupled_joints_info(self) -> Dict[int, Dict[str, Any]]:
        """빈 Coupled Joint 정보 반환."""
        return dict(self._coupled_joints)

    def get_unified_target_positions(self) -> List[float]:
        """통합 목표 관절 위치 – 현재는 빈 리스트 반환."""
        # 시나리오/ROS2 측에서는 None/빈 값에 대해 기본 타겟을 사용하도록 이미 방어 코드가 있음.
        return []

    # ------------------------------------------------------------------
    # ROS2CallbackHandler에서 호출하는 그룹 업데이트 메서드 (no-op)
    # ------------------------------------------------------------------
    def update_hand_joint_group(
        self,
        hand_side: str,
        finger_name: str,
        joint_values: Iterable[float],
        mode: str = "current",
    ) -> None:
        """손가락별 관절 업데이트 – 현재는 내부 버퍼에만 대략 반영하거나 무시."""
        # 아주 단순하게, 대응되는 리스트가 있으면 값 몇 개만 복사해 두는 정도로 제한
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
        """팔/허리/목 관절 그룹 업데이트 – 내부 버퍼에 값만 간단히 복사."""
        values = [float(v) for v in joint_values]

        if group_name == "right_arm":
            buf = self.Arm_R_current if mode == "current" else self.Arm_R_desired
        elif group_name == "left_arm":
            buf = self.Arm_L_current if mode == "current" else self.Arm_L_desired
        elif group_name == "waist":
            buf = self.Waist_current if mode == "current" else self.Waist_desired
        elif group_name == "neck":
            buf = self.Neck_current if mode == "current" else self.Neck_desired
        elif group_name.startswith("hand_"):
            # 손가락 그룹은 update_hand_joint_group 에서 처리
            parts = group_name.split("_")
            if len(parts) >= 3:
                hand_side = parts[1]
                finger_name = parts[2]
                self.update_hand_joint_group(hand_side, finger_name, values, mode)
            return
        else:
            # 알려지지 않은 그룹 이름은 무시
            return

        # 버퍼 크기를 맞추고 값 복사
        if buf is not None:
            if len(buf) < len(values):
                buf.extend([0.0] * (len(values) - len(buf)))
            for i in range(len(values)):
                buf[i] = values[i]

