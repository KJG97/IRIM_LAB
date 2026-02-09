import time
from typing import Dict, Optional, TYPE_CHECKING

import numpy as np

from .obs_store import ObservationStore
from .types import ObsDict

ROS2_AVAILABLE = False

if TYPE_CHECKING:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Float32MultiArray, Float64MultiArray
else:  # pragma: no cover - runtime import
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Float32MultiArray
        from std_msgs.msg import Float64MultiArray
        ROS2_AVAILABLE = True
    except ImportError:
        Node = object  # type: ignore[assignment]
        SingleThreadedExecutor = object  # type: ignore[assignment]
        Float32MultiArray = object  # type: ignore[assignment]
        rclpy = None  # type: ignore


class ROS2ObservationSubscriber(Node):  # type: ignore[misc]
    """ROS2 subscriber/publisher. key는 canonical name으로 통일."""

    def __init__(self, store: ObservationStore, domain_id: int = 0):
        super().__init__("sim2real_debugger_obs_subscriber")
        self._store = store
        self._last_update: Dict[str, float] = {}
        self._domain_id = domain_id


        # real robot용 내부 버퍼 (Domain ID == 77에서만 사용)
        self._joint_pos_deg: Optional[np.ndarray] = None
        self._joint_torque: Optional[np.ndarray] = None
        self._base_pos: Optional[np.ndarray] = None
        # real robot용 command publisher 핸들 (Domain ID == 77에서만 초기화)
        self._pub_cmd_arm: Optional["Float64MultiArray"] = None  # type: ignore[assignment]
        self._pub_cmd_thumb: Optional["Float64MultiArray"] = None  # type: ignore[assignment]
        self._pub_cmd_index: Optional["Float64MultiArray"] = None  # type: ignore[assignment]
        self._pub_cmd_middle: Optional["Float64MultiArray"] = None  # type: ignore[assignment]
        self._pub_cmd_ring: Optional["Float64MultiArray"] = None  # type: ignore[assignment]
        self._pub_cmd_little: Optional["Float64MultiArray"] = None  # type: ignore[assignment]

        if self._domain_id == 77:
            self._init_real_robot_subscribers()
        else:
            self._init_default_subscribers()

        # Publishers
        self.pub_policy_action = self.create_publisher(Float32MultiArray, "/policy/action", 10)
        self.pub_last_actions = self.create_publisher(Float32MultiArray, "/observation/last_actions", 10)

        # Domain ID 77 (real robot) 인 경우, /robot_inbound/* joint_command 퍼블리셔 추가
        if self._domain_id == 77:
            self._init_real_robot_command_publishers()

    def _init_default_subscribers(self) -> None:
        # 학습과 동일: reference_joint_pos는 추론 시 궤적 ref로 채움 (ROS 구독 없음)
        self.create_subscription(Float32MultiArray, "/observation/last_actions", self._cb("last_actions"), 10)
        self.create_subscription(Float32MultiArray, "/observation/joint_pos", self._cb("joint_pos"), 10)
        self.create_subscription(Float32MultiArray, "/observation/right_hand_joint_torque", self._cb_torque_default, 10)
        self.create_subscription(Float32MultiArray, "/observation/right_hand_base_pos", self._cb("right_hand_base_pos"), 10)
        self._torque_recv_count: int = 0

    def _init_real_robot_subscribers(self) -> None:
        # 학습과 동일: reference_joint_pos는 추론 시 궤적 ref로 채움 (ROS 구독 없음)
        self.create_subscription(Float32MultiArray, "/observation/last_actions", self._cb("last_actions"), 10)

        # 내부 버퍼 생성
        self._joint_pos_deg = np.zeros(18, dtype=np.float32)
        self._joint_torque = np.zeros(19, dtype=np.float32)  # 어깨3 + 팔꿈치1 + 손가락15 = 19
        self._base_pos = np.zeros(7, dtype=np.float32)

        # 1) joint_positions_deg 구독 (deg -> rad 후 joint_pos Obs로)
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Arm_R_theOne/joint_positions_deg",
            self._cb_arm_joint_deg,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Hand_R_thumb_wir/joint_positions_deg",
            self._cb_thumb_joint_deg,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Hand_R_index_wir/joint_positions_deg",
            self._cb_index_joint_deg,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Hand_R_middle_wir/joint_positions_deg",
            self._cb_middle_joint_deg,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Hand_R_ring_wir/joint_positions_deg",
            self._cb_ring_joint_deg,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Hand_R_little_wir/joint_positions_deg",
            self._cb_little_joint_deg,
            10,
        )

        # 2) joint_torque 구독 (바로 right_hand_joint_torque Obs에 Nm 단위로)
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Hand_R_thumb_wir/joint_torque",
            self._cb_thumb_torque,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Hand_R_index_wir/joint_torque",
            self._cb_index_torque,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Hand_R_middle_wir/joint_torque",
            self._cb_middle_torque,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Hand_R_ring_wir/joint_torque",
            self._cb_ring_torque,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Hand_R_little_wir/joint_torque",
            self._cb_little_torque,
            10,
        )

        # 3) poses에서 right_hand_base_pos (x, y, z, qw, qx, qy, qz)
        self.create_subscription(
            Float64MultiArray,
            "/robot_outbound_data/Arm_R_theOne/poses",
            self._cb_arm_pose,
            10,
        )

    def _init_real_robot_command_publishers(self) -> None:
        """실제 로봇 joint_command 토픽용 퍼블리셔를 초기화."""
        # Arm 7 DoF
        self._pub_cmd_arm = self.create_publisher(
            Float64MultiArray,
            "/robot_inbound/Arm_R_theOne/joint_command",
            10,
        )
        # Thumb 3 DoF (data[0..2])
        self._pub_cmd_thumb = self.create_publisher(
            Float64MultiArray,
            "/robot_inbound/Hand_R_thumb_wir/joint_command",
            10,
        )
        # Index / Middle / Ring / Little: 각 3 DoF 배열을 보내되,
        # data[1], data[2]에만 제어 joint를 채움 (data[0]은 0 유지)
        self._pub_cmd_index = self.create_publisher(
            Float64MultiArray,
            "/robot_inbound/Hand_R_index_wir/joint_command",
            10,
        )
        self._pub_cmd_middle = self.create_publisher(
            Float64MultiArray,
            "/robot_inbound/Hand_R_middle_wir/joint_command",
            10,
        )
        self._pub_cmd_ring = self.create_publisher(
            Float64MultiArray,
            "/robot_inbound/Hand_R_ring_wir/joint_command",
            10,
        )
        self._pub_cmd_little = self.create_publisher(
            Float64MultiArray,
            "/robot_inbound/Hand_R_little_wir/joint_command",
            10,
        )

    def _update_joint_pos_obs(self) -> None:
        """내부 joint_pos_deg 버퍼를 rad로 변환해 joint_pos Obs에 기록."""
        if self._joint_pos_deg is None:
            return
        rad = np.deg2rad(self._joint_pos_deg.astype(np.float32))
        self._store.set("joint_pos", rad)
        self._last_update["joint_pos"] = time.time()

    def _update_torque_obs(self) -> None:
        """내부 torque 버퍼를 right_hand_joint_torque Obs에 기록."""
        if self._joint_torque is None:
            return
        self._store.set("right_hand_joint_torque", self._joint_torque.astype(np.float32))
        self._last_update["right_hand_joint_torque"] = time.time()

    def _update_base_pos_obs(self) -> None:
        """내부 base_pos 버퍼를 right_hand_base_pos Obs에 기록."""
        if self._base_pos is None:
            return
        self._store.set("right_hand_base_pos", self._base_pos.astype(np.float32))
        self._last_update["right_hand_base_pos"] = time.time()

    # -------------------- joint_positions_deg 콜백 --------------------
    def _cb_arm_joint_deg(self, msg: "Float32MultiArray") -> None:
        if self._joint_pos_deg is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size >= 7:
            # 0~6: Arm_R_theOne/joint_positions_deg/data[0..6]
            self._joint_pos_deg[0:7] = arr[0:7]
            self._update_joint_pos_obs()

    def _cb_thumb_joint_deg(self, msg: "Float32MultiArray") -> None:
        if self._joint_pos_deg is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size >= 3:
            # 7~9: Hand_R_thumb_wir/joint_positions_deg/data[0..2]
            self._joint_pos_deg[7:10] = arr[0:3]
            self._update_joint_pos_obs()

    def _cb_index_joint_deg(self, msg: "Float32MultiArray") -> None:
        if self._joint_pos_deg is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size >= 3:
            # 10~11: Hand_R_index_wir/joint_positions_deg/data[1], data[2]
            self._joint_pos_deg[10:12] = arr[1:3]
            self._update_joint_pos_obs()

    def _cb_middle_joint_deg(self, msg: "Float32MultiArray") -> None:
        if self._joint_pos_deg is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size >= 3:
            # 12~13: Hand_R_middle_wir/joint_positions_deg/data[1], data[2]
            self._joint_pos_deg[12:14] = arr[1:3]
            self._update_joint_pos_obs()

    def _cb_ring_joint_deg(self, msg: "Float32MultiArray") -> None:
        if self._joint_pos_deg is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size >= 3:
            # 14~15: Hand_R_ring_wir/joint_positions_deg/data[1], data[2]
            self._joint_pos_deg[14:16] = arr[1:3]
            self._update_joint_pos_obs()

    def _cb_little_joint_deg(self, msg: "Float32MultiArray") -> None:
        if self._joint_pos_deg is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size >= 3:
            # 16~17: Hand_R_little_wir/joint_positions_deg/data[1], data[2]
            self._joint_pos_deg[16:18] = arr[1:3]
            self._update_joint_pos_obs()

    # -------------------- joint_torque 콜백 --------------------
    # 토크 관측 순서 (19차원):
    # [0~2]: 어깨 (R_Shoulder_Pitch, Roll, Yaw) - 실제 로봇에서는 0으로 유지
    # [3]: 팔꿈치 (R_Elbow) - 실제 로봇에서는 0으로 유지
    # [4~6]: 엄지 (R_Thumb_Yaw, CMC, MCP)
    # [7~9]: 검지 (R_Index_Roll, MCP, PIP)
    # [10~12]: 중지 (R_Middle_Roll, MCP, PIP)
    # [13~15]: 약지 (R_Ring_Roll, MCP, PIP)
    # [16~18]: 소지 (R_Little_Roll, MCP, PIP)
    
    def _cb_thumb_torque(self, msg: "Float32MultiArray") -> None:
        if self._joint_torque is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size >= 3:
            # 4~6: Hand_R_thumb_wir/joint_torque/data[0..2]
            self._joint_torque[4:7] = arr[0:3]
            self._update_torque_obs()

    def _cb_index_torque(self, msg: "Float32MultiArray") -> None:
        if self._joint_torque is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size >= 3:
            # 7~9: Hand_R_index_wir/joint_torque/data[0..2]
            self._joint_torque[7:10] = arr[0:3]
            self._update_torque_obs()

    def _cb_middle_torque(self, msg: "Float32MultiArray") -> None:
        if self._joint_torque is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size >= 3:
            # 10~12: Hand_R_middle_wir/joint_torque/data[0..2]
            self._joint_torque[10:13] = arr[0:3]
            self._update_torque_obs()

    def _cb_ring_torque(self, msg: "Float32MultiArray") -> None:
        if self._joint_torque is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size >= 3:
            # 13~15: Hand_R_ring_wir/joint_torque/data[0..2]
            self._joint_torque[13:16] = arr[0:3]
            self._update_torque_obs()

    def _cb_little_torque(self, msg: "Float32MultiArray") -> None:
        if self._joint_torque is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size >= 3:
            # 16~18: Hand_R_little_wir/joint_torque/data[0..2]
            self._joint_torque[16:19] = arr[0:3]
            self._update_torque_obs()

    # -------------------- poses 콜백 (right_hand_base_pos) --------------------
    def _cb_arm_pose(self, msg: "Float32MultiArray") -> None:
        if self._base_pos is None:
            return
        arr = np.asarray(msg.data, dtype=np.float32)
        # x: data[56], y: 57, z: 58, qw: 59, qx: 60, qy: 61, qz: 62
        if arr.size >= 63:
            self._base_pos[:] = arr[56:63]
            self._update_base_pos_obs()


    def _cb_torque_default(self, msg: "Float32MultiArray") -> None:
        """기본 경로(Domain ID ≠ 77) 토크 콜백: store 저장."""
        arr = np.asarray(msg.data, dtype=np.float32)
        self._store.set("right_hand_joint_torque", arr)
        self._last_update["right_hand_joint_torque"] = time.time()

    def _cb(self, key: str):
        def _handler(msg: Float32MultiArray):
            arr = np.asarray(msg.data, dtype=np.float32)
            self._store.set(key, arr)
            self._last_update[key] = time.time()

        return _handler

    def get_last_update_age(self, key: str) -> Optional[float]:
        ts = self._last_update.get(key)
        return None if ts is None else (time.time() - ts)

    def get_last_update_time(self, key: str) -> Optional[float]:
        """마지막 ROS 수신 시각(Unix time, seconds)을 반환."""
        return self._last_update.get(key)

    def publish_policy_action(self, actions: np.ndarray) -> None:
        a = np.asarray(actions).reshape(-1)
        if a.size != 18:
            self.get_logger().warning(f"Invalid actions shape for publish: {a.shape}")
            return
        # Domain ID 77 (real robot) 인 경우에는 /policy/action 대신
        # /robot_inbound/*/joint_command 로 분할 퍼블리시
        if self._domain_id == 77:
            self._publish_real_robot_commands(a)
            return

        # 기본(sim 등) 경로: /policy/action 하나로 퍼블리시
        msg = Float32MultiArray()
        msg.data = [float(x) for x in a]
        self.pub_policy_action.publish(msg)

    def _publish_real_robot_commands(self, a: np.ndarray) -> None:
        """18차원 action을 실제 로봇 joint_command 토픽들로 분해해 퍼블리시."""
        # 퍼블리셔가 초기화되지 않았으면 아무 것도 하지 않음
        if not all(
            [
                self._pub_cmd_arm,
                self._pub_cmd_thumb,
                self._pub_cmd_index,
                self._pub_cmd_middle,
                self._pub_cmd_ring,
                self._pub_cmd_little,
            ]
        ):
            self.get_logger().warning("Real robot command publishers not initialized.")
            return

        # Arm_R_theOne: data[0..6] <= a[0..6]
        arm_msg = Float64MultiArray()
        arm_msg.data = [float(x) for x in a[0:7]]
        self._pub_cmd_arm.publish(arm_msg)  # type: ignore[union-attr]

        # Hand_R_thumb_wir: data[0..2] <= a[7..9]
        thumb_arr = np.zeros(3, dtype=np.float64)
        thumb_arr[0:3] = a[7:10]
        thumb_msg = Float64MultiArray()
        thumb_msg.data = [float(x) for x in thumb_arr]
        self._pub_cmd_thumb.publish(thumb_msg)  # type: ignore[union-attr]

        # Hand_R_index_wir: data[1], data[2] <= a[10], a[11]
        index_arr = np.zeros(3, dtype=np.float64)
        index_arr[1:3] = a[10:12]
        index_msg = Float64MultiArray()
        index_msg.data = [float(x) for x in index_arr]
        self._pub_cmd_index.publish(index_msg)  # type: ignore[union-attr]

        # Hand_R_middle_wir: data[1], data[2] <= a[12], a[13]
        middle_arr = np.zeros(3, dtype=np.float64)
        middle_arr[1:3] = a[12:14]
        middle_msg = Float64MultiArray()
        middle_msg.data = [float(x) for x in middle_arr]
        self._pub_cmd_middle.publish(middle_msg)  # type: ignore[union-attr]

        # Hand_R_ring_wir: data[1], data[2] <= a[14], a[15]
        ring_arr = np.zeros(3, dtype=np.float64)
        ring_arr[1:3] = a[14:16]
        ring_msg = Float64MultiArray()
        ring_msg.data = [float(x) for x in ring_arr]
        self._pub_cmd_ring.publish(ring_msg)  # type: ignore[union-attr]

        # Hand_R_little_wir: data[1], data[2] <= a[16], a[17]
        little_arr = np.zeros(3, dtype=np.float64)
        little_arr[1:3] = a[16:18]
        little_msg = Float64MultiArray()
        little_msg.data = [float(x) for x in little_arr]
        self._pub_cmd_little.publish(little_msg)  # type: ignore[union-attr]

    def publish_last_actions(self, actions: np.ndarray) -> None:
        a = np.asarray(actions).reshape(-1)
        # 학습과 동일: last_action = 처리된 관절 18차원, history_length=3 → 18*3=54차원
        if a.size != 54:
            self.get_logger().warning(f"Invalid last_actions shape for publish: expected 54 (18*3), got {a.shape} (size={a.size})")
            return
        msg = Float32MultiArray()
        msg.data = [float(x) for x in a]
        self.pub_last_actions.publish(msg)


def spin_while(ros_node: ROS2ObservationSubscriber, running_cb) -> None:
    """SingleThreadedExecutor loop."""
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    while running_cb() and rclpy.ok():  # type: ignore[attr-defined]
        executor.spin_once(timeout_sec=0.1)


__all__ = ["ROS2ObservationSubscriber", "ROS2_AVAILABLE", "spin_while"]
