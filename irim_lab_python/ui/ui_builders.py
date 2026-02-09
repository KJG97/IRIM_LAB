"""
각 섹션별 UI 빌더들
"""

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.gui.components.element_wrappers import StateButton

from .ui_components import UIComponentFactory
from .ui_styles import ButtonStyleManager
from ..config import UIConfig, JointConfig, ROS2Config
from ..config.ui_config import UIColors, UILayout


class WorldControlsBuilder:
    """World Controls 섹션 UI 빌더"""
    
    def __init__(self, ui_builder_ref):
        """
        Args:
            ui_builder_ref: UIBuilder 인스턴스 참조
        """
        self.ui_builder = ui_builder_ref
    
    def build(self):
        """World Controls UI 생성"""
        world_controls_frame = CollapsableFrame(
            "World Controls", 
            collapsed=UIConfig.WORLD_CONTROLS_COLLAPSED
        )
        
        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=UILayout.SPACING_SMALL):
                
                # 🟢 LOAD 버튼
                self._build_load_button()
                
                # 🔴 RESET 버튼
                self._build_reset_button()
                
                # 🟢 RUN/STOP 버튼
                self._build_scenario_button()
                
                # ✨ 버튼 스타일 적용
                self._apply_button_styles()
    
    def _build_load_button(self):
        """LOAD 버튼 생성"""
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_SUCCESS)
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)
            
            # LoadButton 생성
            self.ui_builder._load_btn = LoadButton(
                "Scene Loader", "LOAD", 
                setup_scene_fn=self.ui_builder._setup_scene, 
                setup_post_load_fn=self.ui_builder._setup_scenario
            )
            self.ui_builder._load_btn.set_world_settings(
                physics_dt=UIConfig.PHYSICS_DT, 
                rendering_dt=UIConfig.RENDERING_DT, 
                sim_params={"gravity": UIConfig.GRAVITY}
            )
            self.ui_builder.wrapped_ui_elements.append(self.ui_builder._load_btn)
            
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
    
    def _build_reset_button(self):
        """RESET 버튼 생성"""
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_RESET)
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)
            
            # ResetButton 생성
            self.ui_builder._reset_btn = ResetButton(
                "System Reset", "RESET", 
                pre_reset_fn=None, 
                post_reset_fn=self.ui_builder._on_post_reset_btn
            )
            self.ui_builder._reset_btn.enabled = False
            self.ui_builder.wrapped_ui_elements.append(self.ui_builder._reset_btn)
            
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
    
    def _build_scenario_button(self):
        """RUN/STOP 시나리오 버튼 생성"""
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_STATE)
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)

            self.ui_builder._scenario_state_btn = StateButton(
                "Run Scenario",
                "RUN",
                "STOP",
                on_a_click_fn=self.ui_builder._on_run_scenario_a_text,
                on_b_click_fn=self.ui_builder._on_run_scenario_b_text,
                physics_callback_fn=self.ui_builder._update_scenario,
            )
            self.ui_builder._scenario_state_btn.enabled = False
            self.ui_builder.wrapped_ui_elements.append(self.ui_builder._scenario_state_btn)
    
    def _apply_button_styles(self):
        """버튼 스타일 적용"""
        ButtonStyleManager.apply_button_styles(
            self.ui_builder._load_btn,
            self.ui_builder._reset_btn,
            self.ui_builder._scenario_state_btn
        )

class JointControlsBuilder:
    """Joint Controls 섹션 UI 빌더"""
    
    def __init__(self, ui_builder_ref):
        """
        Args:
            ui_builder_ref: UIBuilder 인스턴스 참조
        """
        self.ui_builder = ui_builder_ref
    
    def build(self):
        """Joint Controls UI 생성"""
        joint_control_frame = CollapsableFrame(
            "ROS2 Studio", 
            collapsed=UIConfig.JOINT_CONTROL_COLLAPSED
        )
        
        with joint_control_frame:
            with ui.VStack(style=get_style(), spacing=UILayout.SPACING_SMALL, height=0):
                
                # ROS2 초기화 버튼
                self._build_ros2_init_button()
                
                # Publisher/Subscriber 제어
                self._build_ros2_controls()
                                                    
    def _build_ros2_init_button(self):
        """ROS2 초기화 및 토픽 모드 제어 버튼들 생성"""
        with ui.VStack(spacing=UILayout.SPACING_SMALL):
            # Initialize ROS2 버튼
            UIComponentFactory.create_styled_button(
                "Initialize ROS2", 
                callback=self.ui_builder._setup_ros2_manager, 
                color_scheme = 'green',
                height=UILayout.BUTTON_HEIGHT
            )

            UIComponentFactory.create_styled_button(
                "Torque Sub", 
                callback=self.ui_builder._toggle_joint_torque_subscriber,
                color_scheme='blue',
                height=UILayout.BUTTON_HEIGHT
            )

            # 🆕 Joint Sub Mode 토글 버튼과 상태 표시
            UIComponentFactory.create_styled_button(
                "Current | Desired", 
                callback=self.ui_builder._toggle_topic_mode,
                color_scheme='blue',
                height=UILayout.BUTTON_HEIGHT
            )

            # Unified 상태 표시
            with ui.HStack(height=UILayout.LABEL_HEIGHT):
                self.ui_builder._joint_torque_status_label = UIComponentFactory.create_status_label(
                    "Joint Torque: OFF", UILayout.LABEL_WIDTH_XLARGE
                )

                self.ui_builder._topic_mode_status_label = UIComponentFactory.create_status_label(
                    "Control Mode: Current", UILayout.LABEL_WIDTH_LARGE
                )
            
            UIComponentFactory.create_separator(UILayout.SEPARATOR_HEIGHT)
    
    def _build_ros2_controls(self):
        """Publisher/Subscriber 제어 버튼들 생성"""
        # 버튼들
        with ui.HStack(height=UILayout.BUTTON_HEIGHT_LARGE):
            UIComponentFactory.create_styled_button(
                "Publisher", 
                callback=self.ui_builder._toggle_publisher, 
                color_scheme = 'yellow',
                height=UILayout.BUTTON_HEIGHT
            )
            UIComponentFactory.create_styled_button(
                "Subscriber", 
                callback=self.ui_builder._toggle_subscriber, 
                color_scheme = 'yellow',
                height=UILayout.BUTTON_HEIGHT
            )
        
        # 상태 표시
        with ui.HStack(height=UILayout.LABEL_HEIGHT):
            self.ui_builder._pub_status_label = UIComponentFactory.create_status_label(
                "Publisher: OFF", UILayout.LABEL_WIDTH_LARGE
            )
            self.ui_builder._sub_status_label = UIComponentFactory.create_status_label(
                "Subscriber: OFF", UILayout.LABEL_WIDTH_LARGE
            )
        
        self._build_joint_group_checkboxes()

    def _create_group_checkbox_callback(self, group_name: str):
        """그룹별 체크박스 콜백 생성 팩토리 메서드"""
        def callback(checked: bool):
            # 🔧 scenario → _scenario로 변경
            if hasattr(self.ui_builder, '_scenario') and self.ui_builder._scenario:
                self.ui_builder._scenario.set_joint_group_enabled(group_name, checked)
                print(f"✅ {group_name} 그룹: {'활성화' if checked else '비활성화'}")
            else:
                print(f"⚠️ Scenario 미로드 - {group_name} 상태 변경 무시됨")
        return callback

    def _build_joint_group_checkboxes(self):
        """관절 그룹별 표시 제어 체크박스들 생성"""

        # 체크박스 설정
        checkbox_configs = [
            ("Left Arm", False, self._create_group_checkbox_callback('left_arm')),
            ("Right Arm", False, self._create_group_checkbox_callback('right_arm')),
            ("Body", False, self._create_group_checkbox_callback('body')),
            ("Hand", False, self._create_group_checkbox_callback('hand'))
        ]
        
        # 체크박스 그룹 생성 (2열로 배치)
        self.ui_builder._joint_group_checkboxes = UIComponentFactory.create_checkbox_group_with_labels(
            checkbox_configs, columns= 2  # 3열로 배치
        )
        
        # 전체 선택/해제 버튼
        with ui.HStack(height=UILayout.BUTTON_HEIGHT):
            UIComponentFactory.create_styled_button(
                "All ON", 
                callback=self._on_all_groups_enable,
                color_scheme='green',
                height=UILayout.BUTTON_HEIGHT_SMALL
            )
            UIComponentFactory.create_styled_button(
                "All OFF", 
                callback=self._on_all_groups_disable,
                color_scheme='red', 
                height=UILayout.BUTTON_HEIGHT_SMALL
            )
            ui.Spacer()  # 오른쪽 여백
        
        UIComponentFactory.create_separator(UILayout.SEPARATOR_HEIGHT)



    def _on_all_groups_enable(self):
        """모든 그룹 활성화"""
        # 🔧 scenario → _scenario로 변경
        if hasattr(self.ui_builder, '_scenario') and self.ui_builder._scenario:
            self.ui_builder._scenario.set_all_groups_enabled(True)
            # 🔄 UI 체크박스들도 동기화
            self._sync_checkboxes_to_scenario()
        else:
            print("⚠️ Scenario 미로드 - All ON 무시됨")

    def _on_all_groups_disable(self):
        """모든 그룹 비활성화"""
        # 🔧 scenario → _scenario로 변경
        if hasattr(self.ui_builder, '_scenario') and self.ui_builder._scenario:
            self.ui_builder._scenario.set_all_groups_enabled(False)
            # 🔄 UI 체크박스들도 동기화
            self._sync_checkboxes_to_scenario()
        else:
            print("⚠️ Scenario 미로드 - All OFF 무시됨")

    def _sync_checkboxes_to_scenario(self):
        """시나리오 상태를 UI 체크박스에 동기화"""
        # 🔧 scenario → _scenario로 변경
        if not (hasattr(self.ui_builder, '_scenario') and self.ui_builder._scenario) or not hasattr(self.ui_builder, '_joint_group_checkboxes'):
            return
        
        # 시나리오에서 현재 상태 가져오기
        group_states = self.ui_builder._scenario.get_all_group_states()
        
        # 체크박스 UI 매핑
        checkbox_mapping = {
            'body': 'Body',
            'left_arm': 'Left Arm', 
            'right_arm': 'Right Arm',
            'hand': 'Hand'
        }
        
        # 각 체크박스 상태 업데이트
        for group_key, checkbox_text in checkbox_mapping.items():
            if checkbox_text in self.ui_builder._joint_group_checkboxes:
                current_state = group_states.get(group_key, False)  # 🔧 True → False로 변경
                checkbox = self.ui_builder._joint_group_checkboxes[checkbox_text]
                checkbox.model.set_value(current_state)


class SensorLabWorldControlsBuilder:
    """Sensor Lab용 World Controls 섹션 UI 빌더"""
    
    def __init__(self, sensor_lab_ui_builder_ref):
        """
        Args:
            sensor_lab_ui_builder_ref: SensorLabUIBuilder 인스턴스 참조
        """
        self.sensor_lab_ui_builder = sensor_lab_ui_builder_ref
    
    def build(self):
        """Sensor Lab용 World Controls UI 생성"""
        world_controls_frame = CollapsableFrame(
            "World Controls", 
            collapsed=UIConfig.WORLD_CONTROLS_COLLAPSED
        )
        
        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=UILayout.SPACING_SMALL):
                
                # 🟢 LOAD 버튼
                self._build_load_button()
                
                # 🔴 Joint Torque Test 버튼
                self._build_torque_test_button()
                
                # 🟢 구 생성 버튼들 (3개)
                self._build_sphere_buttons()
                
                # ✨ 버튼 스타일 적용
                self._apply_button_styles()
    
    def _build_load_button(self):
        """LOAD 버튼 생성 (센서 테스트 에셋 로드용)"""
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_SUCCESS)
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)
            
            # LoadButton 생성 - 센서 테스트 에셋 로드용
            self.sensor_lab_ui_builder._load_btn = LoadButton(
                "Scene Loader", "LOAD", 
                setup_scene_fn=self.sensor_lab_ui_builder._setup_sensor_scene, 
                setup_post_load_fn=None
            )
            self.sensor_lab_ui_builder._load_btn.set_world_settings(
                physics_dt=UIConfig.PHYSICS_DT, 
                rendering_dt=UIConfig.RENDERING_DT, 
                sim_params={"gravity": UIConfig.GRAVITY}
            )
            self.sensor_lab_ui_builder.wrapped_ui_elements.append(self.sensor_lab_ui_builder._load_btn)
            
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
    
    def _build_torque_test_button(self):
        """Joint Torque Test 버튼 생성"""
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_RESET)  # 빨간색 사이드바
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)
            
            # 빨간색 스타일 버튼 생성
            self.sensor_lab_ui_builder._torque_test_btn = UIComponentFactory.create_styled_button(
                "Joint Torque Test",
                callback=self.sensor_lab_ui_builder._toggle_joint_torque_test,
                color_scheme='red',
                height=UILayout.BUTTON_HEIGHT
            )
            
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
    
    def _build_sphere_buttons(self):
        """구 생성 버튼들 (3개) - 크기/무게 다름"""
        # 라벨
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            ui.Label("Test Sphere:", style={"color": 0xFFAAAAAA, "font_size": 12})
        
        # 버튼 3개를 한 줄에 배치
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_SUCCESS)  # 초록색 사이드바
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)
            
            # 10g 구 버튼
            self.sensor_lab_ui_builder._sphere_large_btn = UIComponentFactory.create_styled_button(
                "10g",
                callback=self.sensor_lab_ui_builder._create_sphere_large,
                color_scheme='green',
                height=UILayout.BUTTON_HEIGHT,
                width=60
            )
            
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            
            # 5g 구 버튼
            self.sensor_lab_ui_builder._sphere_medium_btn = UIComponentFactory.create_styled_button(
                "5g",
                callback=self.sensor_lab_ui_builder._create_sphere_medium,
                color_scheme='green',
                height=UILayout.BUTTON_HEIGHT,
                width=60
            )
            
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            
            # 0.3g 구 버튼
            self.sensor_lab_ui_builder._sphere_small_btn = UIComponentFactory.create_styled_button(
                "0.3g",
                callback=self.sensor_lab_ui_builder._create_sphere_small,
                color_scheme='green',
                height=UILayout.BUTTON_HEIGHT,
                width=60
            )
            
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
    
    def _apply_button_styles(self):
        """버튼 스타일 적용 (센서 랩용 - LOAD 버튼만)"""
        # 센서 랩에는 LOAD 버튼만 있으므로 직접 스타일 적용
        if hasattr(self.sensor_lab_ui_builder, '_load_btn') and self.sensor_lab_ui_builder._load_btn:
            try:
                if hasattr(self.sensor_lab_ui_builder._load_btn, '_button') and self.sensor_lab_ui_builder._load_btn._button:
                    self.sensor_lab_ui_builder._load_btn._button.style = ButtonStyleManager.get_load_button_style()
            except Exception as e:
                print(f"⚠️ LOAD 버튼 스타일 적용 실패: {e}")
