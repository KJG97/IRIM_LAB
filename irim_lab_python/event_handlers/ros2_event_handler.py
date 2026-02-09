"""
ROS2 통신 관련 이벤트 핸들러
"""

from ..config import ROS2Config 

class ROS2EventHandler:
    """ROS2 통신 이벤트를 처리하는 클래스"""
    
    def __init__(self, ui_builder_ref):
        """이벤트 핸들러 초기화
        
        Args:
            ui_builder_ref: UIBuilder 인스턴스 참조
        """
        self._ui_builder = ui_builder_ref
    
    def setup_ros2_manager(self):
        """🆕 ROS2 관리자 초기화 (간소화됨)"""
        success = self._ui_builder._ros2_manager.initialize()
        if not success:
            print("❌ ROS2 Manager initialization failed!")
        else:
            # 🆕 초기 토픽 모드 상태 설정
            self._update_topic_mode_status()
        return success
    
    def toggle_publisher(self):
        """🎛️ Publisher 토글 (간소화됨)"""
        success, status = self._ui_builder._ros2_manager.toggle_publisher()
        if hasattr(self._ui_builder, '_pub_status_label'):
            self._ui_builder._pub_status_label.text = status
        
        # UI 상태 업데이트
        self._ui_builder._publisher_enabled = 'ON' in status
        return success
    
    def toggle_subscriber(self):
        """🎛️ Subscriber 토글 (간소화됨)"""
        success, status = self._ui_builder._ros2_manager.toggle_subscriber()
        if hasattr(self._ui_builder, '_sub_status_label'):
            self._ui_builder._sub_status_label.text = status
        
        # UI 상태 업데이트
        self._ui_builder._subscriber_enabled = 'ON' in status
        
        # 🆕 토픽 모드 상태도 함께 업데이트
        self._update_topic_mode_status()
        
        return success
    
    def toggle_joint_torque_subscriber(self):
        """🚀 Torque Subscriber 토글 (간소화됨)"""
        success, status = self._ui_builder._ros2_manager.toggle_joint_torque_subscriber()
        
        # 🔧 올바른 라벨 참조로 수정
        if hasattr(self._ui_builder, '_joint_torque_status_label'):
            # "Torque Topics: ON/OFF" → "Joint Torque: ON/OFF" 형태로 변환
            if 'ON' in status:
                self._ui_builder._joint_torque_status_label.text = "Joint Torque: ON"
            else:
                self._ui_builder._joint_torque_status_label.text = "Joint Torque: OFF"
        
        # UI 상태 업데이트
        self._ui_builder._joint_torque_subscriber_enabled = 'ON' in status
        return success
    
    def cleanup_ros2(self):
        """ROS2 정리 및 UI 상태 업데이트"""
        print("🔄 ROS2 Handler 정리 중...")
        
        # 🎯 올바른 경로로 수정: self._ui_builder._ros2_manager
        if hasattr(self._ui_builder, '_ros2_manager') and self._ui_builder._ros2_manager:
            self._ui_builder._ros2_manager.cleanup()
            self._ui_builder._ros2_manager = None
        
        # 🛡️ 안전한 UI 라벨 업데이트 (try-catch 추가)
        try:
            if (hasattr(self._ui_builder, '_sub_status_label') and 
                self._ui_builder._sub_status_label is not None):
                self._ui_builder._sub_status_label.text = "Subscriber: OFF"
                
            if (hasattr(self._ui_builder, '_joint_torque_status_label') and 
                self._ui_builder._joint_torque_status_label is not None):
                self._ui_builder._joint_torque_status_label.text = "Joint Torque: OFF"

            if (hasattr(self._ui_builder, '_topic_mode_status_label') and 
                self._ui_builder._topic_mode_status_label is not None):
                self._ui_builder._topic_mode_status_label.text = "Control Mode: Current"
                
        except Exception as e:
            print(f"⚠️ UI 정리 중 오류 (무시됨): {e}")


    # 🆕 새로 추가할 메서드
    def _update_topic_mode_status(self):
        """토픽 모드 상태 라벨 업데이트"""
        if (hasattr(self._ui_builder, '_topic_mode_status_label') and 
            self._ui_builder._topic_mode_status_label and
            self._ui_builder._ros2_manager and 
            self._ui_builder._ros2_manager.is_initialized()):
            try:
                current_mode = self._ui_builder._ros2_manager.get_current_topic_mode()
                mode_display = ROS2Config.get_topic_mode_display_name(current_mode)
                self._ui_builder._topic_mode_status_label.text = f"Control Mode: {mode_display}"
            except Exception as e:
                print(f"⚠️ Topic mode status update error: {e}")

    def _sync_all_ros2_status(self):
        """🆕 모든 ROS2 상태를 UI에 동기화"""
        if not self._ui_builder._ros2_manager or not self._ui_builder._ros2_manager.is_initialized():
            return
        
        try:
            # Publisher 상태 동기화
            if hasattr(self._ui_builder, '_pub_status_label'):
                pub_enabled = self._ui_builder._ros2_manager._ros2_node.is_publisher_enabled()
                self._ui_builder._pub_status_label.text = "Publisher: ON" if pub_enabled else "Publisher: OFF"
            
            # Subscriber 상태 동기화 (토픽 모드 포함)
            if hasattr(self._ui_builder, '_sub_status_label'):
                sub_enabled = self._ui_builder._ros2_manager._ros2_node.is_subscriber_enabled()
                if sub_enabled:
                    current_mode = self._ui_builder._ros2_manager.get_current_topic_mode()
                    mode_display = ROS2Config.get_topic_mode_display_name(current_mode)
                    self._ui_builder._sub_status_label.text = f"Subscriber: ON ({mode_display})"
                else:
                    self._ui_builder._sub_status_label.text = "Subscriber: OFF"
            
            # 토픽 모드 상태 동기화
            self._update_topic_mode_status()
            
        except Exception as e:
            print(f"⚠️ ROS2 status sync error: {e}")