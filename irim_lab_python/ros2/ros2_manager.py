"""
ALLEX Digital Twin ROS2 통합 관리자
"""

import threading

from .ros2_node import create_allex_ros2_node
from .ros2_callbacks import ROS2CallbackHandler
from ..config import ROS2Config
from ..core.joint_controller import ALLEXJointController



class ROS2IntegratedManager:
    """ROS2 통신을 통합 관리하는 클래스"""
    
    def __init__(self, scenario_ref=None):
        """ROS2 관리자 초기화
        
        Args:
            scenario_ref: ALLEXDigitalTwin 시나리오 인스턴스 참조
        """
        # 핵심 구성요소들
        self._ros2_node = None
        self._callback_handler = None
        self._executor = None
        self._ros_thread = None
        self._joint_controller = None
        
        # 상태 관리
        self._initialized = False
        self._scenario_ref = scenario_ref
        
        # 🆕 토픽 모드 상태 관리
        self._topic_mode = ROS2Config.DEFAULT_TOPIC_MODE
        
        # 관절 제어 참조 (UI Builder에서 설정)
        self._effective_joints = None
        self._joint_values_ref = None
    
    # ========================================
    # 🏗️ 초기화 및 정리
    # ========================================
    def initialize(self):
        """ROS2 관리자 초기화"""
        try:
            if self._initialized:
                print("⚠️ ROS2 Manager already initialized!")
                return True
            
            # 완전 정리 후 시작
            self.cleanup()
            
            # ROS2 Bridge Extension 활성화
            from isaacsim.core.utils.extensions import enable_extension
            enable_extension(ROS2Config.BRIDGE_EXTENSION)

            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            
            # ROS2 초기화
            if not rclpy.ok():
                rclpy.init()
            
            # 🔧 Scenario의 Joint Controller 사용 (새로 생성하지 않음)
            if self._scenario_ref and hasattr(self._scenario_ref, '_joint_controller'):
                self._joint_controller = self._scenario_ref._joint_controller
                print("✅ Using existing Joint Controller from Scenario")
            else:
                # 백업용: 새로 생성 (이전 코드)
                self._joint_controller = ALLEXJointController()
                print("⚠️ Created new Joint Controller (no scenario reference)")


            self._callback_handler = ROS2CallbackHandler(self._joint_controller )

            # 🆕 시나리오 참조 설정 (unified callback용)
            if self._scenario_ref:
                self._callback_handler.set_scenario_reference(self._scenario_ref)

            if self._effective_joints and self._joint_values_ref:
                self._callback_handler.set_joint_control_reference(
                    self._effective_joints, 
                    self._joint_values_ref
                )
            
            # ROS2 노드 생성
            self._ros2_node = create_allex_ros2_node(
                joint_callback=self._callback_handler.on_external_joint_command,
                joint_torque_callback=self._callback_handler.on_joint_torque_command
            )
            
            # 🆕 Manager의 토픽 모드를 Node에 동기화
            if self._ros2_node:
                self._ros2_node.set_topic_mode(self._topic_mode)
            
            # Executor 설정
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._ros2_node)
            
            # 안전한 스레드 시작
            self._start_executor_thread()
            
            self._initialized = True
            return True
            
        except Exception as e:
            print(f"❌ ROS2 Manager initialization failed: {e}")
            self.cleanup()
            return False
    
    def cleanup(self):
        """ROS2 리소스 완전 정리"""
        try:
            # Executor 정리
            if self._executor:
                try:
                    self._executor.shutdown()
                except Exception:
                    pass
                self._executor = None

            # Node 정리
            if self._ros2_node:
                try:
                    self._ros2_node.destroy_node()
                except Exception:
                    pass
                self._ros2_node = None

            # 콜백 핸들러/컨트롤러 참조 해제
            self._callback_handler = None
            self._joint_controller = None

            # 스레드 정리: join 시도
            if self._ros_thread and self._ros_thread.is_alive():
                try:
                    self._ros_thread.join(timeout=1.0)
                except Exception:
                    pass
            self._ros_thread = None

            # ROS2 컨텍스트 종료
            try:
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

            self._initialized = False

        except Exception as e:
            print(f"⚠️ ROS2 cleanup warning: {e}")
            # 강제 초기화
            self._ros2_node = None
            self._executor = None
            self._ros_thread = None
            self._initialized = False
    
    def _start_executor_thread(self):
        """Executor 스레드 시작"""
        def safe_spin():
            try:
                self._executor.spin()
            except Exception as e:
                print(f"ROS2 Executor error: {e}")
        
        self._ros_thread = threading.Thread(
            target=safe_spin, 
            daemon=ROS2Config.THREAD_DAEMON
        )
        self._ros_thread.start()
    
    # ========================================
    # 🔄 토픽 모드 관리
    # ========================================
    def toggle_topic_mode(self):
        """토픽 모드 토글 (positions ↔ targets)
        
        Returns:
            tuple: (success: bool, status_message: str)
        """
        if not self._initialized or not self._ros2_node:
            print("❌ ROS2 Manager not initialized!")
            return False, "Topic Mode: ERROR (No Manager)"
        
        try:
            # 현재 모드 확인
            current_mode = self._topic_mode
            
            # 다음 모드 결정
            if current_mode == ROS2Config.TOPIC_MODE_CURRENT:
                new_mode = ROS2Config.TOPIC_MODE_DESIRED
            else:
                new_mode = ROS2Config.TOPIC_MODE_CURRENT
            
            # 모드 전환 시도
            success = self.set_topic_mode(new_mode)
            
            if success:
                mode_display = ROS2Config.get_topic_mode_display_name(new_mode)
                status = f"Topic Mode: {mode_display}"
                return True, status
            else:
                return False, "Topic Mode: FAILED"
                
        except Exception as e:
            error_msg = f"Topic Mode: ERROR ({str(e)[:15]})"
            print(f"❌ Topic mode toggle error: {e}")
            return False, error_msg
    
    def set_topic_mode(self, topic_mode):
        """토픽 모드 설정
        
        Args:
            topic_mode (str): 설정할 토픽 모드
            
        Returns:
            bool: 설정 성공 여부
        """
        if not self._initialized or not self._ros2_node:
            print("❌ ROS2 Manager not initialized!")
            return False
        
        try:
            # 유효성 검증
            if not ROS2Config.is_valid_topic_mode(topic_mode):
                print(f"❌ Invalid topic mode: {topic_mode}")
                return False
            
            # Node에 모드 전환 요청
            success = self._ros2_node.set_topic_mode(topic_mode)
            
            if success:
                # Manager 상태 동기화
                self._topic_mode = topic_mode
                if self._joint_controller:
                    self._joint_controller.set_topic_mode(topic_mode)
                mode_display = ROS2Config.get_topic_mode_display_name(topic_mode)
                print(f"✅ Topic mode changed to: {mode_display}")
                return True
            else:
                print(f"❌ Failed to set topic mode to: {topic_mode}")
                return False
                
        except Exception as e:
            print(f"❌ Topic mode setting error: {e}")
            return False
    
    def get_current_topic_mode(self):
        """현재 토픽 모드 반환
        
        Returns:
            str: 현재 토픽 모드
        """
        if self._initialized and self._ros2_node:
            # Node에서 실제 모드 가져와서 동기화
            node_mode = self._ros2_node.get_current_topic_mode()
            self._topic_mode = node_mode
            return node_mode
        else:
            # 초기화되지 않은 경우 Manager의 상태 반환
            return self._topic_mode
    
    def get_available_topic_modes(self):
        """사용 가능한 토픽 모드 목록 반환
        
        Returns:
            list: 사용 가능한 토픽 모드 리스트
        """
        return ROS2Config.get_available_topic_modes()
    
    # ========================================
    # 🔧 설정 메서드들
    # ========================================
    def set_joint_control_reference(self, effective_joints, joint_values_ref):
        """관절 제어 참조 설정
        
        Args:
            effective_joints (int): 유효 관절 수
            joint_values_ref (list): 관절 값 리스트 참조
        """
        self._effective_joints = effective_joints
        self._joint_values_ref = joint_values_ref
        
        # 콜백 핸들러에도 전달
        if self._callback_handler:
            self._callback_handler.set_joint_control_reference(
                effective_joints, 
                joint_values_ref
            )
    
    # ========================================
    # 📤📥 Publisher/Subscriber 제어
    # ========================================
    def toggle_publisher(self):
        """Publisher 토글"""
        if not self._initialized or not self._ros2_node:
            print("❌ ROS2 Manager not initialized!")
            return False, "Publisher : ERROR (No Manager)"
        
        try:
            if self._ros2_node.is_publisher_enabled():
                success = self._ros2_node.disable_publisher()
                status = "Publisher: OFF" if success else "Publisher: ERROR"
            else:
                success = self._ros2_node.enable_publisher()
                status = "Publisher: ON" if success else "Publisher: FAILED"
            
            return success, status

        except Exception as e:
            error_msg = f"Publisher: ERROR ({str(e)[:20]})"
            print(f"❌ Publisher toggle error: {e}")
            return False, error_msg

    # ========================================
    # 📤 관절 관측(18 DOF) 퍼블리셔 헬퍼
    #   - 물리 시뮬레이션 콜백(시나리오 update)에서 호출
    # ========================================
    def is_publisher_enabled(self):
        """현재 Joint Publisher 활성화 여부 반환"""
        return bool(self._ros2_node and self._ros2_node.is_publisher_enabled())

    def publish_joint_observation(self, joint_values):
        """/observation/joint_pos 발행 (시나리오에서 관절 읽은 뒤 호출)"""
        if not self._initialized or not self._ros2_node:
            return False
        return self._ros2_node.publish_joint_observation(joint_values)
    
    def publish_right_hand_joint_torque(self, joint_torques):
        """/observation/right_hand_joint_torque 발행 (시나리오에서 토크 읽은 뒤 호출)"""
        if not self._initialized or not self._ros2_node:
            return False
        return self._ros2_node.publish_right_hand_joint_torque(joint_torques)
    
    def publish_right_hand_base_pos(self, pose_values):
        """/observation/right_hand_base_pos 발행 (Right_Hand_base pose, Origin_Body 기준 7D)"""
        if not self._initialized or not self._ros2_node:
            return False
        return self._ros2_node.publish_right_hand_base_pos(pose_values)
    
    def toggle_subscriber(self):
        """🆕 14개 Outbound Subscribers 토글 (기존 메서드명 유지)"""
        if not self._initialized or not self._ros2_node:
            print("❌ ROS2 Manager not initialized!")
            return False, "Subscriber: ERROR (No Manager)"
        
        try:
            # 14개 토픽 토글
            success = self._ros2_node.toggle_subscriber()
            
            if self._ros2_node.is_subscriber_enabled():
                # 🆕 현재 토픽 모드 표시 추가
                current_mode = self.get_current_topic_mode()  # 🆕 실시간 모드 확인
                mode_display = ROS2Config.get_topic_mode_display_name(current_mode)
                status = f"Subscriber: ON ({mode_display})"
                # Joint Controller에 구독 상태 전달
                if self._joint_controller:
                    self._joint_controller.set_ros2_subscriber_status(True)
            else:
                status = "Subscriber: OFF"
                if self._joint_controller:
                    self._joint_controller.set_ros2_subscriber_status(False)
                    
            return success, status
            
        except Exception as e:
            error_msg = f"Subscriber: ERROR ({str(e)[:20]})"
            print(f"❌ Subscriber toggle error: {e}")
            return False, error_msg
            
    def toggle_joint_torque_subscriber(self):
        """14개 토크 토픽 Subscriber 토글"""
        if not self._initialized or not self._ros2_node:
            print("❌ ROS2 Manager not initialized!")
            return False, "Torque Topics: ERROR (No Manager)"
        
        try:
            if self._ros2_node.is_joint_torque_subscriber_enabled():  # 메서드명 변경
                success = self._ros2_node.disable_joint_torque_subscriber()  # 메서드명 변경
                status = "Torque Topics: OFF" if success else "Torque Topics: ERROR"
                
                # Scenario를 simulation mode로 전환
                if success and self._scenario_ref:
                    self._scenario_ref.set_torque_source_mode('disabled')
            else:
                success = self._ros2_node.enable_joint_torque_subscriber()  # 메서드명 변경
                # 14개 토크 토픽 구독 성공 시 개수 표시
                if success:
                    topic_count = len(self._ros2_node._joint_torque_subscriber)
                    status = f"Torque Topics: ON ({topic_count})"
                else:
                    status = "Torque Topics: FAILED"
                
                # Scenario를 external mode로 전환
                if success and self._scenario_ref:
                    self._scenario_ref.set_torque_source_mode('external')
            
            return success, status
            
        except Exception as e:
            error_msg = f"Torque Topics: ERROR ({str(e)[:15]})"
            print(f"❌ Torque Topics toggle error: {e}")
            return False, error_msg

    def get_joint_controller(self):
        """Joint Controller 인스턴스 반환"""
        return self._joint_controller

    def get_unified_target_positions(self):
        """통합된 목표 관절 위치 반환 (모든 관절 그룹 포함)"""
        if self._joint_controller:
            return self._joint_controller.get_unified_target_positions()
        return None

    def get_active_joint_groups(self):
        """활성화된 관절 그룹 목록 반환"""
        if self._callback_handler:
            return self._callback_handler.get_active_joint_groups()
        return []

    # ========================================
    # 📤 Publishing 메서드들
    # ========================================
    def publish_joint_command(self, joint_values, joint_names):
        """관절 명령 발행
        
        Args:
            joint_values (list): 관절 위치 값들
            joint_names (list): 관절 이름들
            
        Returns:
            bool: 발행 성공 여부
        """
        if not self._initialized or not self._ros2_node:
            print("⚠️ ROS2 Manager not initialized")
            return False
        
        return self._ros2_node.publish_joint_command(joint_values, joint_names)
    
    # ========================================
    # 📊 상태 확인 메서드들
    # ========================================
    def is_initialized(self):
        """초기화 상태 확인"""
        return self._initialized
    
    def get_status_summary(self):
        """전체 상태 요약"""
        if not self._initialized or not self._ros2_node:
            return {
                'initialized': False,
                'publisher': 'OFF',
                'outbound_subscribers': 'OFF', 
                'unified_subscriber': 'OFF',
                'active_groups': [],
                'topic_count': 0,
                'topic_mode': self._topic_mode,
                'topic_mode_display': ROS2Config.get_topic_mode_display_name(self._topic_mode)
            }
        
        status = self._ros2_node.get_status_summary()
        status['initialized'] = True
        status['active_groups'] = self.get_active_joint_groups()
        status['topic_count'] = self._ros2_node.get_outbound_topic_count()
        
        # 🆕 Manager와 Node 간 토픽 모드 동기화
        self._topic_mode = status.get('topic_mode', self._topic_mode)
        
        return status
    
    def get_message_statistics(self):
        """메시지 통계"""
        if self._callback_handler:
            return self._callback_handler.get_message_statistics()
        return {'legacy_joint_messages': 0, 'outbound_messages': 0, 'unified_messages': 0}
    
    def get_detailed_status(self):
        """🆕 상세 상태 정보 반환"""
        basic_status = self.get_status_summary()
        statistics = self.get_message_statistics()
        
        return {
            **basic_status,
            'statistics': statistics,
            'joint_controller_active': self._joint_controller is not None,
            'callback_handler_active': self._callback_handler is not None,
            'available_topic_modes': self.get_available_topic_modes()
        }
    
    def print_status_report(self):
        """🆕 상태 리포트 출력"""
        print("📊 ROS2 Manager Status Report:")
        status = self.get_detailed_status()
        
        print(f"   Initialized: {status['initialized']}")
        print(f"   Publisher: {status.get('publisher', 'N/A')}")
        print(f"   Outbound Subscribers: {status.get('outbound_subscribers', 'N/A')}")
        print(f"   Unified Subscriber: {status.get('unified_subscriber', 'N/A')}")
        print(f"   Topic Mode: {status.get('topic_mode_display', 'N/A')} ({status.get('topic_mode', 'N/A')})")
        print(f"   Active Groups: {', '.join(status['active_groups']) if status['active_groups'] else 'None'}")
        print(f"   Topic Count: {status.get('topic_count', 0)}")
        
        if 'statistics' in status:
            stats = status['statistics']
            print(f"   Message Stats:")
            print(f"     - Legacy: {stats.get('legacy_joint_messages', 0)}")
            print(f"     - Outbound: {stats.get('outbound_messages', 0)}")
            print(f"     - Unified: {stats.get('unified_messages', 0)}")
        
        # 콜백 핸들러 상태 출력
        if self._callback_handler:
            self._callback_handler.print_status_summary()