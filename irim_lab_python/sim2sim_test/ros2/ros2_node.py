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
            self._joint_subscriber = None
            self._outbound_subscribers = []
            self._current_subscribers = []
            self._desired_subscribers = []
            self._policy_action_subscriber = None
            self._policy_action_subscriber_enabled = False
            self._publisher_enabled = False
            self._subscriber_enabled = False
            self._outbound_subscriber_enabled = False
            
            self.get_logger().info(f'🎯 ALLEX ROS2 Node Ready - Topic Mode: {self._current_topic_mode}')
        
        # ========================================
        # 🔄 토픽 모드 관리
        # ========================================
        def set_topic_mode(self, topic_mode):
            """토픽 모드 설정 및 전환
            
            Args:
                topic_mode (str): 설정할 토픽 모드
                
            Returns:
                bool: 전환 성공 여부
            """
            try:
                # 유효성 검증
                if not ROS2Config.is_valid_topic_mode(topic_mode):
                    self.get_logger().error(f"❌ Invalid topic mode: {topic_mode}")
                    return False
                
                # 현재 모드와 동일하면 스킵
                if self._current_topic_mode == topic_mode:
                    self.get_logger().info(f"ℹ️ Already in {topic_mode} mode")
                    return True
                
                # 기존 subscribers가 활성화되어 있다면 재구독 필요
                was_enabled = self._outbound_subscriber_enabled
                
                if was_enabled:
                    # 기존 subscribers 비활성화
                    self.disable_subscriber()
                
                # 토픽 모드 변경
                old_mode = self._current_topic_mode
                self._current_topic_mode = topic_mode
                
                # 활성화되어 있었다면 새 모드로 재활성화
                if was_enabled:
                    success = self.enable_subscriber()
                    if not success:
                        # 실패 시 이전 모드로 롤백
                        self._current_topic_mode = old_mode
                        self.enable_subscriber()  # 원래 모드로 복구 시도
                        return False
                
                mode_display = ROS2Config.get_topic_mode_display_name(topic_mode)
                self.get_logger().info(f"🔄 Topic mode changed to: {mode_display} ({topic_mode})")
                return True
                
            except Exception as e:
                self.get_logger().error(f"❌ Failed to set topic mode: {e}")
                return False
        
        def get_current_topic_mode(self):
            """현재 토픽 모드 반환"""
            return self._current_topic_mode
        
        def _get_current_outbound_topics(self):
            """현재 토픽 모드에 맞는 14개 outbound 토픽 반환"""
            return ROS2Config.get_outbound_topics_by_mode(self._current_topic_mode)
        
        # ========================================
        # 📤 Publisher 관리
        # ========================================
        def enable_publisher(self):
            """Joint Command Publisher 활성화"""
            try:
                if self._joint_publisher is None:
                    self._joint_publisher = self.create_publisher(
                        JointState, 
                        ROS2Topics.JOINT_COMMAND, 
                        ROS2QoS.HISTORY_DEPTH
                    )
                    # Isaac Sim → Sim2Real Debugger 관측용 Publisher (18개 관절)
                    self._joint_obs_publisher = self.create_publisher(
                        Float32MultiArray,
                        ROS2Topics.JOINT_OBS,
                        ROS2QoS.HISTORY_DEPTH,
                    )
                    # Isaac Sim → Sim2Real Debugger 오른손 손가락 토크 Publisher (11개 관절)
                    self._right_hand_torque_publisher = self.create_publisher(
                        Float32MultiArray,
                        ROS2Topics.RIGHT_HAND_JOINT_TORQUE,
                        ROS2QoS.HISTORY_DEPTH,
                    )
                    # Isaac Sim → Sim2Real Debugger Right_Hand_base pose Publisher (7D: pos+quat)
                    self._right_hand_base_pos_publisher = self.create_publisher(
                        Float32MultiArray,
                        ROS2Topics.RIGHT_HAND_BASE_POS,
                        ROS2QoS.HISTORY_DEPTH,
                    )
                    self._publisher_enabled = True
                    self.get_logger().info("📡 Joint Position Publisher ENABLED")
                    return True
                else:
                    self.get_logger().warn("⚠️ Joint Publisher already exists")
                    return True
                    
            except Exception as e:
                self.get_logger().error(f"❌ Failed to enable publisher: {e}")
                return False
        
        def disable_publisher(self):
            """Joint Command Publisher 비활성화"""
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
                else:
                    self.get_logger().warn("⚠️ Joint Publisher already disabled")
                    return True
                    
            except Exception as e:
                self.get_logger().error(f"❌ Failed to disable publisher: {e}")
                return False
        
        def publish_joint_command(self, joint_values, joint_names):
            """관절 명령 발행
            
            Args:
                joint_values (list): 관절 위치 값들
                joint_names (list): 관절 이름들
                
            Returns:
                bool: 발행 성공 여부
            """
            if not self._publisher_enabled or self._joint_publisher is None:
                self.get_logger().warn("⚠️ Joint Publisher is not enabled")
                return False
            
            try:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = ''
                msg.name = joint_names
                msg.position = joint_values
                msg.velocity = []
                msg.effort = []
                
                self._joint_publisher.publish(msg)
                return True
                
            except Exception as e:
                self.get_logger().error(f"❌ Failed to publish joint command: {e}")
                return False

        def publish_joint_observation(self, joint_values):
            """Sim2Real Debugger용 관절 관측 값(18개)을 Float32MultiArray로 발행"""
            if self._joint_obs_publisher is None:
                self.get_logger().warn("⚠️ Joint Observation Publisher is not enabled")
                return False
            try:
                msg = Float32MultiArray()
                # numpy array 또는 리스트 모두 지원
                msg.data = [float(v) for v in list(joint_values)]
                self._joint_obs_publisher.publish(msg)
                return True
            except Exception as e:
                self.get_logger().error(f"❌ Failed to publish joint observation: {e}")
                return False

        def publish_right_hand_joint_torque(self, joint_torques):
            """오른손 손가락 15개 관절에 대한 토크를 Float32MultiArray로 발행"""
            if self._right_hand_torque_publisher is None:
                self.get_logger().warn("⚠️ Right hand joint torque Publisher is not enabled")
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
            """Right_Hand_base pose (Origin_Body 기준, 7D: xyz + qwqxq yqz) 를 Float32MultiArray로 발행"""
            if self._right_hand_base_pos_publisher is None:
                self.get_logger().warn("⚠️ Right hand base pos Publisher is not enabled")
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
        # 📥 Subscriber 관리 (토픽 모드 지원)
        # ========================================
        def enable_subscriber(self):
            """14개 Outbound Subscribers 활성화 (current와 desired 토픽을 동시에 구독)"""
            if self._outbound_subscriber_enabled:
                self.get_logger().warn("⚠️ Outbound Subscribers already enabled")
                return True
            
            try:
                # 🆕 current 토픽들 가져오기
                current_topics = ROS2Config.get_outbound_topics_by_mode(ROS2Config.TOPIC_MODE_CURRENT)
                
                # 🆕 desired 토픽들 가져오기
                desired_topics = ROS2Config.get_outbound_topics_by_mode(ROS2Config.TOPIC_MODE_DESIRED)
                
                # 🆕 current subscribers 생성
                self._current_subscribers = []  # 새로운 리스트 추가
                for topic, info in current_topics.items():
                    try:
                        sub = self.create_subscription(
                            Float64MultiArray,
                            topic,
                            self._make_outbound_callback(info['joint_names'], info['group_name'], mode='current'),  # 🆕 mode 추가
                            self._create_qos_profile()
                        )
                        self._current_subscribers.append(sub)
                    except Exception as e:
                        self.get_logger().error(f"❌ Failed to subscribe to current {topic}: {e}")
                        self.disable_subscriber()
                        return False
                
                # 🆕 desired subscribers 생성
                self._desired_subscribers = []  # 새로운 리스트 추가
                for topic, info in desired_topics.items():
                    try:
                        sub = self.create_subscription(
                            Float64MultiArray,
                            topic,
                            self._make_outbound_callback(info['joint_names'], info['group_name'], mode='desired'),  # 🆕 mode 추가
                            self._create_qos_profile()
                        )
                        self._desired_subscribers.append(sub)
                    except Exception as e:
                        self.get_logger().error(f"❌ Failed to subscribe to desired {topic}: {e}")
                        self.disable_subscriber()
                        return False
                
                self._outbound_subscriber_enabled = True
                
                # 🆕 Policy action subscriber 활성화
                try:
                    self._policy_action_subscriber = self.create_subscription(
                        Float32MultiArray,
                        "/policy/action",
                        self._make_policy_action_callback(),
                        self._create_qos_profile()
                    )
                    self._policy_action_subscriber_enabled = True
                    self.get_logger().info("📡 Policy Action Subscriber ENABLED (/policy/action)")
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to subscribe to /policy/action: {e}")
                    # Policy action subscriber 실패해도 outbound subscribers는 계속 사용 가능
                
                self.get_logger().info(f"📡 {len(self._current_subscribers) + len(self._desired_subscribers)} Outbound Subscribers ENABLED (Current + Desired)")
                return True
                
            except Exception as e:
                self.get_logger().error(f"❌ Failed to enable outbound subscribers: {e}")
                return False

        def _make_outbound_callback(self, joint_names, group_name, mode):  # 🆕 mode 매개변수 추가
            """14개 토픽용 콜백 생성"""
            def callback(msg):
                if self._joint_callback:
                    self._joint_callback(msg.data, joint_names, group_name, mode)  # 🆕 mode 전달
            return callback
        
        def _make_policy_action_callback(self):
            """Policy action 토픽용 콜백 생성"""
            def callback(msg):
                if self._joint_callback:
                    # 18개 값을 받아서 ALLEX_ACTION_JOINT_FULL_NAMES 순서대로 처리
                    # mode는 'desired'로 설정 (명령값이므로)
                    from ..config.ros2_config import ROS2Config
                    joint_names = ROS2Config.ALLEX_ACTION_JOINT_FULL_NAMES
                    self._joint_callback(msg.data, joint_names, "policy_action", "desired")
            return callback

        def disable_subscriber(self):
            """14개 Outbound Subscribers 비활성화"""
            try:
                # 🆕 current subscribers 정리
                for sub in self._current_subscribers:
                    self.destroy_subscription(sub)
                self._current_subscribers.clear()
                
                # 🆕 desired subscribers 정리
                for sub in self._desired_subscribers:
                    self.destroy_subscription(sub)
                self._desired_subscribers.clear()
                
                # 🆕 Policy action subscriber 정리
                if self._policy_action_subscriber is not None:
                    self.destroy_subscription(self._policy_action_subscriber)
                    self._policy_action_subscriber = None
                    self._policy_action_subscriber_enabled = False
                    self.get_logger().info("📡 Policy Action Subscriber DISABLED")
                
                self._outbound_subscriber_enabled = False
                self.get_logger().info("📡 Outbound Subscribers DISABLED")
                return True
            except Exception as e:
                self.get_logger().error(f"❌ Failed to disable outbound subscribers: {e}")
                return False

        def toggle_subscriber(self):
            """Subscriber 토글 (UI에서 사용)"""
            if self._outbound_subscriber_enabled:
                return self.disable_subscriber()
            else:
                return self.enable_subscriber()

        # ========================================
        # 🔧 헬퍼 메서드들
        # ========================================
        def _create_qos_profile(self):
            """QoS 프로파일 생성"""
            return QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=ROS2QoS.HISTORY_DEPTH,
                reliability=ReliabilityPolicy.RELIABLE, 
                durability=DurabilityPolicy.VOLATILE
            )
        
        # ========================================
        # 📊 상태 확인 메서드들
        # ========================================
        def is_publisher_enabled(self):
            """Publisher 활성화 상태 확인"""
            return self._publisher_enabled
        
        def is_subscriber_enabled(self):
            """Outbound Subscriber 활성화 상태 확인"""
            return self._outbound_subscriber_enabled

        def get_outbound_topic_count(self):
            """구독 중인 Outbound 토픽 수 반환"""
            return len(self._current_subscribers) + len(self._desired_subscribers)

        def get_status_summary(self):
            """상태 요약 반환"""
            mode_display = ROS2Config.get_topic_mode_display_name(self._current_topic_mode)
            total_subscribers = len(self._current_subscribers) + len(self._desired_subscribers)
            return {
                'publisher': 'ON' if self._publisher_enabled else 'OFF',
                'subscriber': 'ON' if self._outbound_subscriber_enabled else 'OFF',
                'outbound_subscribers': f'ON ({total_subscribers})' if self._outbound_subscriber_enabled else 'OFF',
                'topic_mode': self._current_topic_mode,
                'topic_mode_display': mode_display
            }

    return ALLEXRos2Node(joint_callback=joint_callback)