# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.timeline
import omni.ui as ui
from omni.usd import StageEventType
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.gui.components.element_wrappers import StateButton

from .config import ROS2Config, JointConfig, UIConfig, VisibilityConfig
from .config.ui_config import UIColors, UILayout
from .ros2 import ROS2IntegratedManager
from .scenario import ALLEXDigitalTwin
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import create_new_stage


# -----------------------------------------------------------------------------
# UI 컴포넌트·스타일·섹션 빌더 (기존 ui_components, ui_styles, ui_builders 통합)
# -----------------------------------------------------------------------------

class UIComponentFactory:
    """UI 컴포넌트 생성 팩토리"""

    @staticmethod
    def _create_ui_button(text, callback=None, height=UILayout.BUTTON_HEIGHT, style=None, width=None):
        if style is not None:
            return ui.Button(text, clicked_fn=callback, height=height, width=width, style=style) if width else ui.Button(text, clicked_fn=callback, height=height, style=style)
        return ui.Button(text, clicked_fn=callback, height=height, width=width) if width else ui.Button(text, clicked_fn=callback, height=height)

    @staticmethod
    def create_section_header(text, height=UILayout.LABEL_HEIGHT):
        return ui.Label(text, height=height)

    @staticmethod
    def create_separator(height=UILayout.SEPARATOR_HEIGHT):
        return ui.Separator(height=height)

    @staticmethod
    def create_spacer(width=UILayout.SPACING_SMALL):
        return ui.Spacer(width=width)

    @staticmethod
    def create_status_label(text, width=UILayout.LABEL_WIDTH_LARGE):
        return ui.Label(text, width=width)

    @staticmethod
    def create_colored_sidebar(color, width=UILayout.SIDEBAR_WIDTH, height=UILayout.BUTTON_HEIGHT):
        return ui.Rectangle(width=width, height=height, style={"background_color": color, "border_radius": UILayout.BUTTON_BORDER_RADIUS})

    @staticmethod
    def create_styled_button(text, callback=None, color_scheme="default", height=UILayout.BUTTON_HEIGHT, width=None):
        style_map = {
            "red": {"Button": {"background_color": UIColors.RED_BUTTON_BG, "border_width": UILayout.BUTTON_BORDER_WIDTH, "border_color": UIColors.RED_BUTTON_BORDER, "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE}, "Button:hovered": {"background_color": UIColors.RED_BUTTON_HOVER}},
            "yellow": {"Button": {"background_color": UIColors.YELLOW_BUTTON_BG, "border_width": UILayout.BUTTON_BORDER_WIDTH, "border_color": UIColors.YELLOW_BUTTON_BORDER, "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE}, "Button:hovered": {"background_color": UIColors.YELLOW_BUTTON_HOVER}},
            "green": {"Button": {"background_color": UIColors.STATE_BUTTON_BG, "border_width": UILayout.BUTTON_BORDER_WIDTH, "border_color": UIColors.STATE_BUTTON_BORDER, "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE}, "Button:hovered": {"background_color": UIColors.STATE_BUTTON_HOVER}},
            "blue": {"Button": {"background_color": UIColors.BLUE_BUTTON_BG, "border_width": UILayout.BUTTON_BORDER_WIDTH, "border_color": UIColors.BLUE_BUTTON_BORDER, "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE}, "Button:hovered": {"background_color": UIColors.BLUE_BUTTON_HOVER}},
            "object_viz": {"Button": {"background_color": UIColors.OBJECT_VIZ_BUTTON_BG, "border_width": UILayout.BUTTON_BORDER_WIDTH, "border_color": UIColors.OBJECT_VIZ_BUTTON_BORDER, "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE}, "Button:hovered": {"background_color": UIColors.OBJECT_VIZ_BUTTON_HOVER}},
            "transparency": {"Button": {"background_color": UIColors.TRANSPARENCY_BUTTON_BG, "border_width": UILayout.BUTTON_BORDER_WIDTH, "border_color": UIColors.TRANSPARENCY_BUTTON_BORDER, "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE}, "Button:hovered": {"background_color": UIColors.TRANSPARENCY_BUTTON_HOVER}},
            "default": None,
        }
        style = style_map.get(color_scheme)
        return UIComponentFactory._create_ui_button(text, callback, height, style, width)

    @staticmethod
    def create_joint_slider(joint_index, callback, joint_name="Joint"):
        with ui.HStack(height=UILayout.BUTTON_HEIGHT):
            ui.Label(f"{joint_name}:", width=UILayout.LABEL_WIDTH_MEDIUM)
            slider = ui.FloatSlider(min=JointConfig.JOINT_MIN_ANGLE, max=JointConfig.JOINT_MAX_ANGLE, step=JointConfig.JOINT_STEP)
            value_label = ui.Label("0.00", width=UILayout.LABEL_WIDTH_SMALL)
            slider.model.add_value_changed_fn(lambda model, i=joint_index, label=value_label: callback(i, model, label))
            return slider, value_label

    @staticmethod
    def create_checkbox(text, initial_value=True, callback=None, width=None):
        style = {"CheckBox": {"background_color": UIColors.BACKGROUND, "border_radius": 3, "font_size": 14}, "CheckBox:checked": {"background_color": UIColors.STATE_BUTTON_BG, "border_color": UIColors.STATE_BUTTON_BORDER}}
        checkbox = ui.CheckBox(text=text, width=width or UILayout.LABEL_WIDTH_MEDIUM, style=style)
        checkbox.model.set_value(initial_value)
        if callback:
            checkbox.model.add_value_changed_fn(lambda model: callback(model.get_value_as_bool()))
        return checkbox

    @staticmethod
    def create_checkbox_with_label(text, initial_value=True, callback=None, width=None):
        checkbox_style = {"CheckBox": {"background_color": UIColors.BACKGROUND, "border_radius": 3}, "CheckBox:checked": {"background_color": UIColors.STATE_BUTTON_BG, "border_color": UIColors.STATE_BUTTON_BORDER}}
        with ui.HStack(spacing=5, height=UILayout.BUTTON_HEIGHT):
            checkbox = ui.CheckBox(width=20, style=checkbox_style)
            label = ui.Label(text, width=width or UILayout.LABEL_WIDTH_MEDIUM - 25, style={"color": 0xFFFFFFFF, "font_size": 14})
        checkbox.model.set_value(initial_value)
        if callback:
            checkbox.model.add_value_changed_fn(lambda model: callback(model.get_value_as_bool()))
        return checkbox

    @staticmethod
    def create_checkbox_group_with_labels(checkboxes_config, columns=2):
        checkboxes = {}
        rows = (len(checkboxes_config) + columns - 1) // columns
        for row in range(rows):
            with ui.HStack(height=UILayout.BUTTON_HEIGHT, spacing=10):
                for col in range(columns):
                    index = row * columns + col
                    if index < len(checkboxes_config):
                        text, initial_value, callback = checkboxes_config[index]
                        checkbox = UIComponentFactory.create_checkbox_with_label(text, initial_value, callback, width=100)
                        checkboxes[text] = checkbox
                    else:
                        ui.Spacer()
        return checkboxes


class ButtonStyleManager:
    """버튼 스타일 관리"""

    @staticmethod
    def get_load_button_style():
        return {
            **get_style(),
            "Button": {"background_color": UIColors.LOAD_BUTTON_BG, "border_width": UILayout.BUTTON_BORDER_WIDTH_THICK, "border_color": UIColors.LOAD_BUTTON_BORDER, "border_radius": UILayout.BUTTON_BORDER_RADIUS_XLARGE, "margin": UILayout.BUTTON_MARGIN, "padding": UILayout.BUTTON_PADDING},
            "Button:hovered": {"background_color": UIColors.LOAD_BUTTON_HOVER},
            "Button.Label": {"color": UIColors.TEXT_PRIMARY},
        }

    @staticmethod
    def get_reset_button_style():
        return {
            **get_style(),
            "Button": {"background_color": UIColors.RED_BUTTON_BG, "border_width": UILayout.BUTTON_BORDER_WIDTH_THICK, "border_color": UIColors.RED_BUTTON_BORDER, "border_radius": UILayout.BUTTON_BORDER_RADIUS_XLARGE, "margin": UILayout.BUTTON_MARGIN, "padding": UILayout.BUTTON_PADDING},
            "Button:hovered": {"background_color": UIColors.RED_BUTTON_HOVER},
            "Button.Label": {"color": UIColors.TEXT_PRIMARY},
        }

    @staticmethod
    def get_state_button_style():
        return {
            **get_style(),
            "Button": {"background_color": UIColors.STATE_BUTTON_BG, "border_width": UILayout.BUTTON_BORDER_WIDTH_THICK, "border_color": UIColors.STATE_BUTTON_BORDER, "border_radius": UILayout.BUTTON_BORDER_RADIUS_XLARGE, "margin": UILayout.BUTTON_MARGIN, "padding": UILayout.BUTTON_PADDING},
            "Button:hovered": {"background_color": UIColors.STATE_BUTTON_HOVER},
            "Button.Label": {"color": UIColors.TEXT_PRIMARY},
        }

    @staticmethod
    def apply_button_styles(load_btn, reset_btn, state_btn):
        try:
            if hasattr(load_btn, "_button") and load_btn._button:
                load_btn._button.style = ButtonStyleManager.get_load_button_style()
            if hasattr(reset_btn, "_button") and reset_btn._button:
                reset_btn._button.style = ButtonStyleManager.get_reset_button_style()
            if hasattr(state_btn, "_state_button") and state_btn._state_button:
                state_btn._state_button.style = ButtonStyleManager.get_state_button_style()
        except Exception as e:
            print(f"⚠️ Could not apply button styles: {e}")


class WorldControlsBuilder:
    """World Controls 섹션 UI 빌더"""

    def __init__(self, ui_builder_ref):
        self.ui_builder = ui_builder_ref

    def build(self):
        world_controls_frame = CollapsableFrame("World Controls", collapsed=UIConfig.WORLD_CONTROLS_COLLAPSED)
        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=UILayout.SPACING_SMALL):
                self._build_load_button()
                self._build_reset_button()
                self._build_scenario_button()
                self._apply_button_styles()

    def _build_load_button(self):
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_SUCCESS)
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)
            self.ui_builder._load_btn = LoadButton("Scene Loader", "LOAD", setup_scene_fn=self.ui_builder._setup_scene, setup_post_load_fn=self.ui_builder._setup_scenario)
            self.ui_builder._load_btn.set_world_settings(physics_dt=UIConfig.PHYSICS_DT, rendering_dt=UIConfig.RENDERING_DT, sim_params={"gravity": UIConfig.GRAVITY})
            self.ui_builder.wrapped_ui_elements.append(self.ui_builder._load_btn)
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)

    def _build_reset_button(self):
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_RESET)
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)
            self.ui_builder._reset_btn = ResetButton("System Reset", "RESET", pre_reset_fn=None, post_reset_fn=self.ui_builder._on_post_reset_btn)
            self.ui_builder._reset_btn.enabled = False
            self.ui_builder.wrapped_ui_elements.append(self.ui_builder._reset_btn)
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)

    def _build_scenario_button(self):
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_STATE)
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)
            self.ui_builder._scenario_state_btn = StateButton("Run Scenario", "RUN", "STOP", on_a_click_fn=self.ui_builder._on_run_scenario_a_text, on_b_click_fn=self.ui_builder._on_run_scenario_b_text, physics_callback_fn=self.ui_builder._update_scenario)
            self.ui_builder._scenario_state_btn.enabled = False
            self.ui_builder.wrapped_ui_elements.append(self.ui_builder._scenario_state_btn)

    def _apply_button_styles(self):
        ButtonStyleManager.apply_button_styles(self.ui_builder._load_btn, self.ui_builder._reset_btn, self.ui_builder._scenario_state_btn)


class JointControlsBuilder:
    """Joint Controls 섹션 UI 빌더"""

    def __init__(self, ui_builder_ref):
        self.ui_builder = ui_builder_ref

    def build(self):
        joint_control_frame = CollapsableFrame("ROS2 Studio", collapsed=UIConfig.JOINT_CONTROL_COLLAPSED)
        with joint_control_frame:
            with ui.VStack(style=get_style(), spacing=UILayout.SPACING_SMALL, height=0):
                self._build_ros2_init_button()
                self._build_ros2_controls()

    def _build_ros2_init_button(self):
        with ui.VStack(spacing=UILayout.SPACING_SMALL):
            UIComponentFactory.create_styled_button("Initialize ROS2", callback=self.ui_builder._setup_ros2_manager, color_scheme="green", height=UILayout.BUTTON_HEIGHT)
            UIComponentFactory.create_styled_button("Current | Desired", callback=self.ui_builder._toggle_topic_mode, color_scheme="blue", height=UILayout.BUTTON_HEIGHT)
            with ui.HStack(height=UILayout.LABEL_HEIGHT):
                self.ui_builder._topic_mode_status_label = UIComponentFactory.create_status_label("Control Mode: Current", UILayout.LABEL_WIDTH_LARGE)
            UIComponentFactory.create_separator(UILayout.SEPARATOR_HEIGHT)

    def _build_ros2_controls(self):
        with ui.HStack(height=UILayout.BUTTON_HEIGHT_LARGE):
            UIComponentFactory.create_styled_button("Publisher", callback=self.ui_builder._toggle_publisher, color_scheme="yellow", height=UILayout.BUTTON_HEIGHT)
            UIComponentFactory.create_styled_button("Subscriber", callback=self.ui_builder._toggle_subscriber, color_scheme="yellow", height=UILayout.BUTTON_HEIGHT)
        with ui.HStack(height=UILayout.LABEL_HEIGHT):
            self.ui_builder._pub_status_label = UIComponentFactory.create_status_label("Publisher: OFF", UILayout.LABEL_WIDTH_LARGE)
            self.ui_builder._sub_status_label = UIComponentFactory.create_status_label("Subscriber: OFF", UILayout.LABEL_WIDTH_LARGE)
        self._build_joint_group_checkboxes()

    def _create_group_checkbox_callback(self, group_name: str):
        def callback(checked: bool):
            if hasattr(self.ui_builder, "_scenario") and self.ui_builder._scenario:
                self.ui_builder._scenario.set_joint_group_enabled(group_name, checked)
        return callback

    def _build_joint_group_checkboxes(self):
        checkbox_configs = [
            ("Left Arm", False, self._create_group_checkbox_callback("left_arm")),
            ("Right Arm", False, self._create_group_checkbox_callback("right_arm")),
            ("Body", False, self._create_group_checkbox_callback("body")),
            ("Hand", False, self._create_group_checkbox_callback("hand")),
        ]
        self.ui_builder._joint_group_checkboxes = UIComponentFactory.create_checkbox_group_with_labels(checkbox_configs, columns=2)
        with ui.HStack(height=UILayout.BUTTON_HEIGHT):
            UIComponentFactory.create_styled_button("All ON", callback=self._on_all_groups_enable, color_scheme="green", height=UILayout.BUTTON_HEIGHT_SMALL)
            UIComponentFactory.create_styled_button("All OFF", callback=self._on_all_groups_disable, color_scheme="red", height=UILayout.BUTTON_HEIGHT_SMALL)
            ui.Spacer()
        UIComponentFactory.create_separator(UILayout.SEPARATOR_HEIGHT)

    def _on_all_groups_enable(self):
        if hasattr(self.ui_builder, "_scenario") and self.ui_builder._scenario:
            self.ui_builder._scenario.set_all_groups_enabled(True)
            self._sync_checkboxes_to_scenario()

    def _on_all_groups_disable(self):
        if hasattr(self.ui_builder, "_scenario") and self.ui_builder._scenario:
            self.ui_builder._scenario.set_all_groups_enabled(False)
            self._sync_checkboxes_to_scenario()

    def _sync_checkboxes_to_scenario(self):
        if not (hasattr(self.ui_builder, "_scenario") and self.ui_builder._scenario) or not hasattr(self.ui_builder, "_joint_group_checkboxes"):
            return
        group_states = self.ui_builder._scenario.get_all_group_states()
        checkbox_mapping = {"body": "Body", "left_arm": "Left Arm", "right_arm": "Right Arm", "hand": "Hand"}
        for group_key, checkbox_text in checkbox_mapping.items():
            if checkbox_text in self.ui_builder._joint_group_checkboxes:
                current_state = group_states.get(group_key, False)
                checkbox = self.ui_builder._joint_group_checkboxes[checkbox_text]
                checkbox.model.set_value(current_state)


class SensorLabWorldControlsBuilder:
    """Sensor Lab용 World Controls 섹션 UI 빌더 (sensor_test에서 import용)."""

    def __init__(self, sensor_lab_ui_builder_ref):
        self.sensor_lab_ui_builder = sensor_lab_ui_builder_ref

    def build(self):
        world_controls_frame = CollapsableFrame("World Controls", collapsed=UIConfig.WORLD_CONTROLS_COLLAPSED)
        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=UILayout.SPACING_SMALL):
                self._build_load_button()
                self._build_torque_test_button()
                self._build_sphere_buttons()
                self._apply_button_styles()

    def _build_load_button(self):
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_SUCCESS)
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)
            self.sensor_lab_ui_builder._load_btn = LoadButton("Scene Loader", "LOAD", setup_scene_fn=self.sensor_lab_ui_builder._setup_sensor_scene, setup_post_load_fn=None)
            self.sensor_lab_ui_builder._load_btn.set_world_settings(physics_dt=UIConfig.PHYSICS_DT, rendering_dt=UIConfig.RENDERING_DT, sim_params={"gravity": UIConfig.GRAVITY})
            self.sensor_lab_ui_builder.wrapped_ui_elements.append(self.sensor_lab_ui_builder._load_btn)
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)

    def _build_torque_test_button(self):
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_RESET)
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)
            self.sensor_lab_ui_builder._torque_test_btn = UIComponentFactory.create_styled_button("Joint Torque Test", callback=self.sensor_lab_ui_builder._toggle_joint_torque_test, color_scheme="red", height=UILayout.BUTTON_HEIGHT)
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)

    def _build_sphere_buttons(self):
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            ui.Label("Test Sphere:", style={"color": 0xFFAAAAAA, "font_size": 12})
        with ui.HStack():
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            UIComponentFactory.create_colored_sidebar(UIColors.SIDEBAR_SUCCESS)
            UIComponentFactory.create_spacer(UILayout.SPACING_MEDIUM)
            self.sensor_lab_ui_builder._sphere_large_btn = UIComponentFactory.create_styled_button("10g", callback=self.sensor_lab_ui_builder._create_sphere_large, color_scheme="green", height=UILayout.BUTTON_HEIGHT, width=60)
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            self.sensor_lab_ui_builder._sphere_medium_btn = UIComponentFactory.create_styled_button("5g", callback=self.sensor_lab_ui_builder._create_sphere_medium, color_scheme="green", height=UILayout.BUTTON_HEIGHT, width=60)
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)
            self.sensor_lab_ui_builder._sphere_small_btn = UIComponentFactory.create_styled_button("0.3g", callback=self.sensor_lab_ui_builder._create_sphere_small, color_scheme="green", height=UILayout.BUTTON_HEIGHT, width=60)
            UIComponentFactory.create_spacer(UILayout.SPACING_SMALL)

    def _apply_button_styles(self):
        if hasattr(self.sensor_lab_ui_builder, "_load_btn") and self.sensor_lab_ui_builder._load_btn and hasattr(self.sensor_lab_ui_builder._load_btn, "_button") and self.sensor_lab_ui_builder._load_btn._button:
            self.sensor_lab_ui_builder._load_btn._button.style = ButtonStyleManager.get_load_button_style()


# -----------------------------------------------------------------------------
# UIBuilder (Sim2Sim 메인 UI)
# -----------------------------------------------------------------------------

class UIBuilder:

    # ========================================
    # 🏗️ 초기화 및 설정
    # ========================================
    def __init__(self):
        # UI 요소들 - UIElementWrapper 인스턴스로 생성된 UI 구성요소들
        self.wrapped_ui_elements = []

        # 타임라인 접근 - 정지/일시정지/재생을 프로그램으로 제어하기 위함
        self._timeline = omni.timeline.get_timeline_interface()


        self._hand_joint_indices = JointConfig.HAND_JOINT_INDICES
        self._effective_joints = len(self._hand_joint_indices)
        self._joint_values = [0.0] * self._effective_joints
        self._joint_names = [JointConfig.JOINT_NAME_TEMPLATE.format(i + 1) for i in self._hand_joint_indices]

        # 🎯 ROS2 관리 변수
        self._publisher_enabled = False
        self._subscriber_enabled = False
        self._unified_subscriber_enabled = False
        self._latest_unified_data = {}
        self._topic_mode_status_label = None

        # 🆕 Articulation 초기화 추적 변수
        self._articulation_initialized = False
        self._physics_step_count = 0
        self._initialization_attempts = 0
        self._max_initialization_attempts = JointConfig.MAX_INITIALIZATION_ATTEMPTS

        # 예제 초기화 실행 - 제공된 예제의 초기 설정 수행
        self._on_init()

    def _on_init(self):
        self._articulation = None
        self._cuboid = None
        self._scenario = ALLEXDigitalTwin()

        # �� 시나리오에 UIBuilder 참조 설정
        self._scenario.set_ui_builder_ref(self)

        # 🆕 ROS2 관리자에 scenario 참조 설정
        self._ros2_manager = ROS2IntegratedManager(scenario_ref=self._scenario)

        self._scenario.set_ros2_manager(self._ros2_manager)

        self._ros2_manager.set_joint_control_reference(
            self._effective_joints, 
            self._joint_values
        )

    # ========================================
    # ROS2 통신 관리
    # ========================================
    def _setup_ros2_manager(self):
        """ROS2 관리자 초기화"""
        if self._ros2_manager is None:
            self._ros2_manager = ROS2IntegratedManager(scenario_ref=self._scenario)
            self._scenario.set_ros2_manager(self._ros2_manager)
            self._ros2_manager.set_joint_control_reference(
                self._effective_joints, self._joint_values
            )

        success = self._ros2_manager.initialize()
        if not success:
            print("❌ ROS2 Manager initialization failed!")
        else:
            self._update_topic_mode_status()

        if success and self._ros2_manager:
            self._ros2_manager.set_joint_control_reference(
                self._effective_joints, self._joint_values
            )
            print("🔧 Joint Position Initialized")
            if self._scenario and self._scenario._joint_controller:
                self._scenario._joint_controller.set_ros2_subscriber_status(False)
            self._sync_all_ros2_status()
        return success

    def _toggle_topic_mode(self):
        """토픽 모드 토글"""
        if not self._ros2_manager or not self._ros2_manager.is_initialized():
            print("❌ ROS2 Manager not initialized!")
            return
        
        try:
            success, status_message = self._ros2_manager.toggle_topic_mode()
            
            if success:
                # 기존 업데이트
                current_mode = self._ros2_manager.get_current_topic_mode()
                mode_display = ROS2Config.get_topic_mode_display_name(current_mode)
                self._topic_mode_status_label.text = f"Control Mode: {mode_display}"
                
                # Scenario 캐시 및 오버레이 텍스트 갱신
                if hasattr(self, "_scenario") and self._scenario:
                    self._scenario._update_cached_topic_mode()
                    self._scenario.update_joint_display_text()
                    self._scenario.update_hand_display_text()
                
                print(f"✅ {status_message}")
            else:
                print(f"❌ {status_message}")
                
        except Exception as e:
            print(f"❌ Topic mode toggle error: {e}")

    def _toggle_publisher(self):
        """Publisher 토글"""
        success, status = self._ros2_manager.toggle_publisher()
        if hasattr(self, "_pub_status_label"):
            self._pub_status_label.text = status
        self._publisher_enabled = "ON" in status
        return success

    def _toggle_subscriber(self):
        """Subscriber 토글"""
        success, status = self._ros2_manager.toggle_subscriber()
        if hasattr(self, "_sub_status_label"):
            self._sub_status_label.text = status
        self._subscriber_enabled = "ON" in status
        self._update_topic_mode_status()
        return success

    def _cleanup_ros2(self):
        """ROS2 정리 및 UI 상태 업데이트"""
        print("🔄 ROS2 정리 중...")
        if hasattr(self, "_ros2_manager") and self._ros2_manager:
            self._ros2_manager.cleanup()
            self._ros2_manager = None
        try:
            if hasattr(self, "_sub_status_label") and self._sub_status_label is not None:
                self._sub_status_label.text = "Subscriber: OFF"
            if hasattr(self, "_topic_mode_status_label") and self._topic_mode_status_label is not None:
                self._topic_mode_status_label.text = "Control Mode: Current"
        except Exception as e:
            print(f"⚠️ UI 정리 중 오류 (무시됨): {e}")

    def _update_topic_mode_status(self):
        """토픽 모드 상태 라벨 업데이트"""
        if (
            hasattr(self, "_topic_mode_status_label")
            and self._topic_mode_status_label
            and self._ros2_manager
            and self._ros2_manager.is_initialized()
        ):
            try:
                current_mode = self._ros2_manager.get_current_topic_mode()
                mode_display = ROS2Config.get_topic_mode_display_name(current_mode)
                self._topic_mode_status_label.text = f"Control Mode: {mode_display}"
            except Exception as e:
                print(f"⚠️ Topic mode status update error: {e}")

    def _sync_all_ros2_status(self):
        """모든 ROS2 상태를 UI에 동기화"""
        if not self._ros2_manager or not self._ros2_manager.is_initialized():
            return
        try:
            if hasattr(self, "_pub_status_label"):
                pub_enabled = self._ros2_manager._ros2_node.is_publisher_enabled()
                self._pub_status_label.text = "Publisher: ON" if pub_enabled else "Publisher: OFF"
            if hasattr(self, "_sub_status_label"):
                sub_enabled = self._ros2_manager._ros2_node.is_subscriber_enabled()
                if sub_enabled:
                    current_mode = self._ros2_manager.get_current_topic_mode()
                    mode_display = ROS2Config.get_topic_mode_display_name(current_mode)
                    self._sub_status_label.text = f"Subscriber: ON ({mode_display})"
                else:
                    self._sub_status_label.text = "Subscriber: OFF"
            self._update_topic_mode_status()
        except Exception as e:
            print(f"⚠️ ROS2 status sync error: {e}")

    #  ========================================
    # 🎮 Isaac Sim 이벤트 콜백 (자동 호출)
    # ========================================
    def on_menu_callback(self):
        """
        툴바에서 UI가 열릴 때 호출되는 콜백 함수.
        build_ui() 함수 실행 후 직접 호출됨.
        """
        pass

    def on_timeline_event(self, event):
        """
        타임라인 이벤트 (재생, 일시정지, 정지)에 대한 콜백 함수.
        """
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            if hasattr(self, "_scenario_state_btn") and self._scenario_state_btn:
                self._scenario_state_btn.reset()
                self._scenario_state_btn.enabled = False

    def on_physics_step(self, step: float):
        """
        물리 시뮬레이션 단계에 대한 콜백 함수.
        Physics Step은 타임라인이 재생 중일 때만 발생함.

        Args:
            step (float): Size of physics step
        """
        pass

    def on_stage_event(self, event):
        """
        스테이지 이벤트에 대한 콜백 함수.

        Args:
            event (omni.usd.StageEventType): 이벤트 타입
        """
        if event.type == int(StageEventType.OPENED):
            # If the user opens a new stage, the extension should completely reset
            self._reset_extension()

    def cleanup(self):
        # UI 요소 정리
        for ui_elem in self.wrapped_ui_elements:
            try:
                ui_elem.cleanup()
            except Exception:
                pass

        # 리스트 비우기 및 강한 참조 제거 (누수 차단)
        self.wrapped_ui_elements.clear()
        if hasattr(self, "_load_btn"):
            self._load_btn = None
        if hasattr(self, "_reset_btn"):
            self._reset_btn = None
        if hasattr(self, "_scenario_state_btn"):
            self._scenario_state_btn = None
        if hasattr(self, "_pub_status_label"):
            self._pub_status_label = None
        if hasattr(self, "_sub_status_label"):
            self._sub_status_label = None
        if hasattr(self, "_topic_mode_status_label"):
            self._topic_mode_status_label = None
        if hasattr(self, "_joint_group_checkboxes"):
            self._joint_group_checkboxes = None

        # ROS2 정리
        self._cleanup_ros2()

    def build_ui(self):
        """
        확장 기능 실행을 위한 맞춤형 UI 도구 구성.
        UI 창이 닫혔다가 다시 열릴 때마다 이 함수가 호출됨.
        """
        
        # 🟢 World Controls 섹션
        world_controls_builder = WorldControlsBuilder(self)
        world_controls_builder.build()
        
        # 🦾 Joint Controls 섹션
        joint_controls_builder = JointControlsBuilder(self)
        joint_controls_builder.build()
            
    # ========================================
    # 🦾 관절 제어
    # ======================================== 
    def _on_joint_slider_changed(self, joint_index, model, label):
        """슬라이더 값 변경 시: UI·joint 값 갱신, Publisher ON이면 ROS2 발행"""
        value = model.get_value_as_float()
        self._joint_values[joint_index] = value
        label.text = f"{value:.2f}"
        if self._publisher_enabled:
            success = self._ros2_manager.publish_joint_command(
                self._joint_values, self._joint_names
            )
            if not success:
                print("⚠️ Failed to publish joint command")
        else:
            print(f"📝 Slider moved (Publisher OFF): Joint {joint_index+1} = {value:.3f}")

    # ========================================
    # 시나리오 관리
    # ========================================
    def _setup_scene(self):
        """Scene 설정 - LOAD 버튼 콜백"""
        create_new_stage()
        try:
            from omni.kit.viewport.menubar.lighting.actions import _set_lighting_mode
            _set_lighting_mode(lighting_mode="Stage Lights")
        except Exception:
            pass
        loaded_objects = self._scenario.load_example_assets()
        if loaded_objects is None:
            return
        world = World.instance()
        if isinstance(loaded_objects, (list, tuple)):
            for loaded_object in loaded_objects:
                world.scene.add(loaded_object)
        else:
            world.scene.add(loaded_objects)

    def _setup_scenario(self):
        """Scenario 설정 후 RUN/Reset 버튼 활성화"""
        self._scenario.setup()
        if hasattr(self, "_scenario_state_btn") and self._scenario_state_btn:
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = True
        if hasattr(self, "_reset_btn") and self._reset_btn:
            self._reset_btn.enabled = True

    def _on_post_reset_btn(self):
        """Reset 버튼 콜백: ROS2 정리 → 시나리오 리셋 → ROS2 재설정"""
        self._cleanup_ros2()
        self._scenario.reset()
        self._setup_ros2_manager()
        if hasattr(self, "_scenario_state_btn") and self._scenario_state_btn:
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = True

    def _update_scenario(self, step: float):
        """물리 스텝마다: 지연 초기화 시도, 시나리오 업데이트"""
        self._physics_step_count += 1
        if (
            not self._articulation_initialized
            and self._physics_step_count >= 3
            and self._initialization_attempts < self._max_initialization_attempts
        ):
            self._try_delayed_initialization()
        done = self._scenario.update(step)
        if done and hasattr(self, "_scenario_state_btn") and self._scenario_state_btn:
            self._scenario_state_btn.enabled = False

    def _try_delayed_initialization(self):
        """지연된 Articulation 초기화 시도"""
        self._initialization_attempts += 1
        success = self._scenario.delayed_initialization()
        if success:
            self._articulation_initialized = True
        else:
            print(f"⚠️ Articulation 초기화 실패 (시도 {self._initialization_attempts})")
            if self._initialization_attempts >= self._max_initialization_attempts:
                print("❌ Articulation 초기화 최대 시도 횟수 초과")
                print("🔧 수동 초기화 버튼을 사용해보세요")

    def _on_run_scenario_a_text(self):
        """RUN 클릭: 타임라인 재생, 초기화 상태 리셋"""
        self._timeline.play()
        self._articulation_initialized = False
        self._physics_step_count = 0
        self._initialization_attempts = 0

    def _on_run_scenario_b_text(self):
        """STOP 클릭: 시뮬레이션 일시정지"""
        print("⏸️ 시뮬레이션 일시정지...")
        self._timeline.pause()

    # ----------------------------------------
    # Table/Can 시각화 (필요 시 UI에서 버튼 연결)
    # ----------------------------------------
    def toggle_table_can_visibility(self):
        """Table/Can prim visibility·collision 토글"""
        try:
            import isaacsim.core.utils.prims as prims_utils
            from isaacsim.core.prims import XFormPrim
            import omni.usd
            from pxr import UsdPhysics

            table_can_prims = VisibilityConfig.TABLE_CAN_PRIMS
            collision_prims = VisibilityConfig.TABLE_CAN_COLLISION_PRIMS
            stage = omni.usd.get_context().get_stage()
            if not stage:
                print("❌ USD Stage를 찾을 수 없습니다")
                return False

            valid_prims = [p for p in table_can_prims if prims_utils.is_prim_path_valid(p)]
            if not valid_prims:
                print("❌ Table/Can prim들을 찾을 수 없습니다")
                if hasattr(self, "_table_can_status_label"):
                    self._table_can_status_label.text = "Status: NO PRIMS FOUND"
                return False

            first_prim = XFormPrim(prim_paths_expr=valid_prims[0])
            new_visibility = not first_prim.get_visibilities()[0]

            for prim_path in valid_prims:
                try:
                    prim = XFormPrim(prim_paths_expr=prim_path)
                    prim.set_visibilities(visibilities=[new_visibility])
                except Exception as e:
                    print(f"⚠️ {prim_path} visibility 변경 실패: {e}")

            from pxr import Usd
            for collision_path in collision_prims:
                try:
                    collision_prim = stage.GetPrimAtPath(collision_path)
                    if collision_prim:
                        UsdPhysics.CollisionAPI.Apply(collision_prim)
                        attr = collision_prim.GetAttribute("physics:collisionEnabled")
                        if attr:
                            attr.Set(not attr.Get())
                        else:
                            attr = collision_prim.CreateAttribute(
                                "physics:collisionEnabled",
                                Usd.GetDefaultTypeForType(Usd.GetTypeForTypeName("bool")),
                            )
                            attr.Set(not new_visibility)
                except Exception as e:
                    print(f"⚠️ {collision_path} collision 변경 실패: {e}")

            self._table_can_visible = new_visibility
            if hasattr(self, "_table_can_status_label"):
                status_text = "VISIBLE" if new_visibility else "HIDDEN"
                self._table_can_status_label.text = f"Table/Can: {status_text}"
            return True
        except Exception as e:
            print(f"❌ Table/Can visibility/collision 변경 실패: {e}")
            if hasattr(self, "_table_can_status_label"):
                self._table_can_status_label.text = f"Status: ERROR ({str(e)[:15]})"
            return False

    def reset_table_can_visibility(self):
        """Table/Can visibility·collision 보이도록 리셋"""
        try:
            import isaacsim.core.utils.prims as prims_utils
            from isaacsim.core.prims import XFormPrim
            from pxr import UsdPhysics

            table_can_prims = VisibilityConfig.TABLE_CAN_PRIMS
            collision_prims = VisibilityConfig.TABLE_CAN_COLLISION_PRIMS
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return False

            for prim_path in table_can_prims:
                if prims_utils.is_prim_path_valid(prim_path):
                    try:
                        prim = XFormPrim(prim_paths_expr=prim_path)
                        prim.set_visibilities(visibilities=[True])
                    except Exception as e:
                        print(f"⚠️ {prim_path} visibility reset 실패: {e}")

            from pxr import Usd
            for collision_path in collision_prims:
                try:
                    collision_prim = stage.GetPrimAtPath(collision_path)
                    if collision_prim:
                        UsdPhysics.CollisionAPI.Apply(collision_prim)
                        attr = collision_prim.GetAttribute("physics:collisionEnabled")
                        if attr:
                            attr.Set(True)
                        else:
                            attr = collision_prim.CreateAttribute(
                                "physics:collisionEnabled",
                                Usd.GetDefaultTypeForType(Usd.GetTypeForTypeName("bool")),
                            )
                            attr.Set(True)
                except Exception as e:
                    print(f"⚠️ {collision_path} collision reset 실패: {e}")

            self._table_can_visible = True
            if hasattr(self, "_table_can_status_label"):
                self._table_can_status_label.text = "Table/Can: VISIBLE"
            return True
        except Exception as e:
            print(f"⚠️ Table/Can reset 실패: {e}")
            return False

    def _reset_extension(self):
        """
        사용자가 self.on_stage_event()에서 새 스테이지를 열 때 호출됨.
        모든 상태가 리셋되어야 함.  
        """
        self._on_init()
        self._reset_ui()

    def _reset_ui(self):
        if hasattr(self, "_scenario_state_btn") and self._scenario_state_btn:
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False
        if hasattr(self, "_reset_btn") and self._reset_btn:
            self._reset_btn.enabled = False

