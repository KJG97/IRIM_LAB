"""
ALLEX Digital Twin ROS2 노드 클래스
"""

from ..config import ROS2Config, ROS2Topics, ROS2QoS

def create_allex_ros2_node(joint_callback=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, Float32MultiArray

    class ALLEXRos2Node(Node):
        """ALLEX 디지털 트윈 전용 ROS2 노드"""

        def __init__(self, joint_callback=None):
            super().__init__(ROS2Config.NODE_NAME)
            self._joint_callback = joint_callback
            self._current_topic_mode = ROS2Config.DEFAULT_TOPIC_MODE
            self._joint_publisher = None
            self._joint_obs_publisher = None
            self._right_hand_torque_publisher = None
            self._right_hand_base_pos_publisher = None
            self._current_subscribers = []
            self._desired_subscribers = []
            self._policy_action_subscriber = None
            self._publisher_enabled = False
            self._outbound_subscriber_enabled = False
            
            self.get_logger().info(f'🎯 ALLEX ROS2 Node Ready - Topic Mode: {self._current_topic_mode}')
        
        # ========================================
        # 🔄 토픽 모드 관리
        # ========================================
        def set_topic_mode(self, topic_mode):
            """토픽 모드 설정 및 전환"""
            try:
                if not ROS2Config.is_valid_topic_mode(topic_mode):
                    self.get_logger().error(f"❌ Invalid topic mode: {topic_mode}")
                    return False
                if self._current_topic_mode == topic_mode:
                    self.get_logger().info(f"ℹ️ Already in {topic_mode} mode")
                    return True
                was_enabled = self._outbound_subscriber_enabled
                if was_enabled:
                    self.disable_subscriber()
                old_mode = self._current_topic_mode
                self._current_topic_mode = topic_mode
                if was_enabled:
                    success = self.enable_subscriber()
                    if not success:
                        self._current_topic_mode = old_mode
                        self.enable_subscriber()
                        return False
                mode_display = ROS2Config.get_topic_mode_display_name(topic_mode)
                self.get_logger().info(f"🔄 Topic mode changed to: {mode_display} ({topic_mode})")
                return True
            except Exception as e:
                self.get_logger().error(f"❌ Failed to set topic mode: {e}")
                return False
        
        def get_current_topic_mode(self):
            return self._current_topic_mode

        # ========================================
        # 📤 Publisher 관리
        # ========================================
        def enable_publisher(self):
            try:
                if self._joint_publisher is None:
                    self._joint_publisher = self.create_publisher(
                        JointState, ROS2Topics.JOINT_COMMAND, ROS2QoS.HISTORY_DEPTH
                    )
                    self._joint_obs_publisher = self.create_publisher(
                        Float32MultiArray, ROS2Topics.JOINT_OBS, ROS2QoS.HISTORY_DEPTH,
                    )
                    self._right_hand_torque_publisher = self.create_publisher(
                        Float32MultiArray, ROS2Topics.RIGHT_HAND_JOINT_TORQUE, ROS2QoS.HISTORY_DEPTH,
                    )
                    self._right_hand_base_pos_publisher = self.create_publisher(
                        Float32MultiArray, ROS2Topics.RIGHT_HAND_BASE_POS, ROS2QoS.HISTORY_DEPTH,
                    )
                    self._publisher_enabled = True
                    self.get_logger().info("📡 Joint Position Publisher ENABLED")
                    return True
                return True
            except Exception as e:
                self.get_logger().error(f"❌ Failed to enable publisher: {e}")
                return False
        
        def disable_publisher(self):
            try:
                if self._joint_publisher is not None:
                    self.destroy_publisher(self._joint_publisher)
                    self._joint_publisher = None
                if self._joint_obs_publisher is not None:
                    self.destroy_publisher(self._joint_obs_publisher)
                    self._joint_obs_publisher = None
                if self._right_hand_torque_publisher is not None:
                    self.destroy_publisher(self._right_hand_torque_publisher)
                    self._right_hand_torque_publisher = None
                if self._right_hand_base_pos_publisher is not None:
                    self.destroy_publisher(self._right_hand_base_pos_publisher)
                    self._right_hand_base_pos_publisher = None
                self._publisher_enabled = False
                self.get_logger().info("📡 Joint Position Publisher DISABLED")
                return True
            except Exception as e:
                self.get_logger().error(f"❌ Failed to disable publisher: {e}")
                return False

        def publish_joint_observation(self, joint_values):
            if self._joint_obs_publisher is None:
                return False
            try:
                msg = Float32MultiArray()
                msg.data = [float(v) for v in list(joint_values)]
                self._joint_obs_publisher.publish(msg)
                return True
            except Exception as e:
                self.get_logger().error(f"❌ Failed to publish joint observation: {e}")
                return False

        def publish_right_hand_joint_torque(self, joint_torques):
            if self._right_hand_torque_publisher is None:
                return False
            try:
                msg = Float32MultiArray()
                msg.data = [float(v) for v in list(joint_torques)]
                self._right_hand_torque_publisher.publish(msg)
                return True
            except Exception as e:
                self.get_logger().error(f"❌ Failed to publish right hand joint torque: {e}")
                return False

        def publish_right_hand_base_pos(self, pose_values):
            if self._right_hand_base_pos_publisher is None:
                return False
            try:
                msg = Float32MultiArray()
                msg.data = [float(v) for v in list(pose_values)]
                self._right_hand_base_pos_publisher.publish(msg)
                return True
            except Exception as e:
                self.get_logger().error(f"❌ Failed to publish right hand base pos: {e}")
                return False
        
        # ========================================
        # 📥 Subscriber 관리
        # ========================================
        def enable_subscriber(self):
            if self._outbound_subscriber_enabled:
                return True
            try:
                current_topics = ROS2Config.get_outbound_topics_by_mode(ROS2Config.TOPIC_MODE_CURRENT)
                desired_topics = ROS2Config.get_outbound_topics_by_mode(ROS2Config.TOPIC_MODE_DESIRED)
                self._current_subscribers = []
                for topic, info in current_topics.items():
                    sub = self.create_subscription(
                        Float64MultiArray, topic,
                        self._make_outbound_callback(info['joint_names'], info['group_name'], mode='current'),
                        self._create_qos_profile()
                    )
                    self._current_subscribers.append(sub)
                self._desired_subscribers = []
                for topic, info in desired_topics.items():
                    sub = self.create_subscription(
                        Float64MultiArray, topic,
                        self._make_outbound_callback(info['joint_names'], info['group_name'], mode='desired'),
                        self._create_qos_profile()
                    )
                    self._desired_subscribers.append(sub)
                self._outbound_subscriber_enabled = True
                try:
                    self._policy_action_subscriber = self.create_subscription(
                        Float32MultiArray, "/policy/action",
                        self._make_policy_action_callback(),
                        self._create_qos_profile()
                    )
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to subscribe to /policy/action: {e}")
                self.get_logger().info(f"📡 {len(self._current_subscribers) + len(self._desired_subscribers)} Outbound Subscribers ENABLED (Current + Desired)")
                return True
            except Exception as e:
                self.get_logger().error(f"❌ Failed to enable outbound subscribers: {e}")
                return False

        def _make_outbound_callback(self, joint_names, group_name, mode):
            def callback(msg):
                if self._joint_callback:
                    self._joint_callback(msg.data, joint_names, group_name, mode)
            return callback
        
        def _make_policy_action_callback(self):
            def callback(msg):
                if self._joint_callback:
                    joint_names = ROS2Config.ALLEX_ACTION_JOINT_FULL_NAMES
                    self._joint_callback(msg.data, joint_names, "policy_action", "desired")
            return callback

        def disable_subscriber(self):
            try:
                for sub in self._current_subscribers:
                    self.destroy_subscription(sub)
                self._current_subscribers.clear()
                for sub in self._desired_subscribers:
                    self.destroy_subscription(sub)
                self._desired_subscribers.clear()
                if self._policy_action_subscriber is not None:
                    self.destroy_subscription(self._policy_action_subscriber)
                    self._policy_action_subscriber = None
                self._outbound_subscriber_enabled = False
                self.get_logger().info("📡 Outbound Subscribers DISABLED")
                return True
            except Exception as e:
                self.get_logger().error(f"❌ Failed to disable outbound subscribers: {e}")
                return False

        def toggle_subscriber(self):
            if self._outbound_subscriber_enabled:
                return self.disable_subscriber()
            return self.enable_subscriber()

        def _create_qos_profile(self):
            return QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=ROS2QoS.HISTORY_DEPTH,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        
        def is_publisher_enabled(self):
            return self._publisher_enabled
        
        def is_subscriber_enabled(self):
            return self._outbound_subscriber_enabled

    return ALLEXRos2Node(joint_callback=joint_callback)
