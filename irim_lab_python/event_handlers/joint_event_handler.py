"""
관절 제어 관련 이벤트 핸들러
"""

from ..config import JointConfig


class JointEventHandler:
    """관절 제어 이벤트를 처리하는 클래스"""
    
    def __init__(self, ui_builder_ref):
        """이벤트 핸들러 초기화
        
        Args:
            ui_builder_ref: UIBuilder 인스턴스 참조
        """
        self._ui_builder = ui_builder_ref
    
    def on_joint_slider_changed(self, joint_index, model, label):
        """🔄 슬라이더 값 변경 시 호출 (간소화됨)"""
        value = model.get_value_as_float()
        self._ui_builder._joint_values[joint_index] = value
        
        # UI 레이블 업데이트
        label.text = f"{value:.2f}"
        
        # 🎯 Publisher가 ON일 때만 ROS2 토픽 발행
        if self._ui_builder._publisher_enabled:
            success = self._ui_builder._ros2_manager.publish_joint_command(
                self._ui_builder._joint_values, 
                self._ui_builder._joint_names
            )
            if not success:
                print(f"⚠️ Failed to publish joint command")
        else:
            print(f"📝 Slider moved (Publisher OFF): Joint {joint_index+1} = {value:.3f}")
    
        