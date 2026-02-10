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

from .config import (
    ROS2Config,
    JointConfig,
    SimulationConfig,
    UIConfig,
    UIColors,
    UILayout,
    OverlayConfig,
)
from .ros2 import ROS2IntegratedManager
from .scenario import ALLEXDigitalTwin
from .ui_helpers import (
    OVERLAY_WINDOW_FLAGS,
    apply_button_styles,
    create_checkbox_group,
    create_colored_sidebar,
    create_separator,
    create_spacer,
    create_status_label,
    create_styled_button,
)
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import create_new_stage


# -----------------------------------------------------------------------------
# UIBuilder — Sim2Sim 메인 UI (World / ROS2 / 오버레이 모두 여기서 처리)
# -----------------------------------------------------------------------------

class UIBuilder:

    def __init__(self):
        self.wrapped_ui_elements = []
        self._timeline = omni.timeline.get_timeline_interface()

        self._hand_joint_indices = JointConfig.HAND_JOINT_INDICES
        self._effective_joints = len(self._hand_joint_indices)
        self._joint_values = [0.0] * self._effective_joints

        self._topic_mode_status_label = None

        self._articulation_initialized = False
        self._physics_step_count = 0
        self._initialization_attempts = 0
        self._max_initialization_attempts = JointConfig.MAX_INITIALIZATION_ATTEMPTS

        # 오버레이 윈도우/라벨 (기존 JointOverlayUI 역할)
        self._text_overlay_window = None
        self._joint_data_label = None
        self._hand_overlay_window = None
        self._hand_data_label = None

        self._on_init()

    def _on_init(self):
        self._scenario = ALLEXDigitalTwin()
        self._ros2_manager = ROS2IntegratedManager(scenario_ref=self._scenario)
        self._scenario.set_ros2_manager(self._ros2_manager)
        self._ros2_manager.set_joint_control_reference(self._effective_joints, self._joint_values)
        self._scenario.set_overlay_ui(self)

    # ----- World Controls (LOAD / RESET / RUN) -----
    def _build_world_controls(self):
        world_controls_frame = CollapsableFrame("World Controls", collapsed=UIConfig.WORLD_CONTROLS_COLLAPSED)
        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=UILayout.SPACING_SMALL):
                self._build_load_button()
                self._build_reset_button()
                self._build_scenario_button()
                apply_button_styles(self._load_btn, self._reset_btn, self._scenario_state_btn)

    def _build_action_row(self, sidebar_color, build_button_fn):
        """사이드바 + 버튼 한 행 공통 빌드."""
        with ui.HStack():
            create_spacer(UILayout.SPACING_SMALL)
            create_colored_sidebar(sidebar_color)
            create_spacer(UILayout.SPACING_MEDIUM)
            build_button_fn()
            create_spacer(UILayout.SPACING_SMALL)

    def _build_load_button(self):
        def add():
            self._load_btn = LoadButton("Scene Loader", "LOAD", setup_scene_fn=self._setup_scene, setup_post_load_fn=self._setup_scenario)
            self._load_btn.set_world_settings(physics_dt=SimulationConfig.PHYSICS_DT, rendering_dt=SimulationConfig.RENDERING_DT, sim_params={"gravity": SimulationConfig.GRAVITY})
            self.wrapped_ui_elements.append(self._load_btn)
        self._build_action_row(UIColors.SIDEBAR_GREEN, add)

    def _build_reset_button(self):
        def add():
            self._reset_btn = ResetButton("System Reset", "RESET", pre_reset_fn=None, post_reset_fn=self._on_post_reset_btn)
            self._reset_btn.enabled = False
            self.wrapped_ui_elements.append(self._reset_btn)
        self._build_action_row(UIColors.SIDEBAR_BLUE, add)

    def _build_scenario_button(self):
        def add():
            self._scenario_state_btn = StateButton("Run Scenario", "RUN", "STOP", on_a_click_fn=self._on_run_scenario_a_text, on_b_click_fn=self._on_run_scenario_b_text, physics_callback_fn=self._update_scenario)
            self._scenario_state_btn.enabled = False
            self.wrapped_ui_elements.append(self._scenario_state_btn)
        self._build_action_row(UIColors.SIDEBAR_GREEN, add)

    # ----- ROS2 Studio (Joint Controls) -----
    def _build_joint_controls(self):
        joint_control_frame = CollapsableFrame("ROS2 Studio", collapsed=UIConfig.JOINT_CONTROL_COLLAPSED)
        with joint_control_frame:
            with ui.VStack(style=get_style(), spacing=UILayout.SPACING_SMALL, height=0):
                self._build_ros2_init_button()
                self._build_ros2_controls()

    def _build_ros2_init_button(self):
        with ui.VStack(spacing=UILayout.SPACING_SMALL):
            create_styled_button("Initialize ROS2", callback=self._setup_ros2_manager, color_scheme="green", height=UILayout.BUTTON_HEIGHT)
            create_styled_button("Current | Desired", callback=self._toggle_topic_mode, color_scheme="blue", height=UILayout.BUTTON_HEIGHT)
            with ui.HStack(height=UILayout.LABEL_HEIGHT):
                self._topic_mode_status_label = create_status_label("Control Mode: Current", UILayout.LABEL_WIDTH_LARGE)
            create_separator(UILayout.SEPARATOR_HEIGHT)

    def _build_ros2_controls(self):
        with ui.HStack(height=UILayout.BUTTON_HEIGHT_LARGE):
            create_styled_button("Publisher", callback=self._toggle_publisher, color_scheme="yellow", height=UILayout.BUTTON_HEIGHT)
            create_styled_button("Subscriber", callback=self._toggle_subscriber, color_scheme="yellow", height=UILayout.BUTTON_HEIGHT)
        with ui.HStack(height=UILayout.LABEL_HEIGHT):
            self._pub_status_label = create_status_label("Publisher: OFF", UILayout.LABEL_WIDTH_LARGE)
            self._sub_status_label = create_status_label("Subscriber: OFF", UILayout.LABEL_WIDTH_LARGE)
        self._build_joint_group_checkboxes()

    def _create_group_checkbox_callback(self, group_name: str):
        def callback(checked: bool):
            if self._scenario:
                self._scenario.set_joint_group_enabled(group_name, checked)
        return callback

    def _build_joint_group_checkboxes(self):
        checkbox_configs = [
            ("Left Arm", False, self._create_group_checkbox_callback("left_arm")),
            ("Right Arm", False, self._create_group_checkbox_callback("right_arm")),
            ("Body", False, self._create_group_checkbox_callback("body")),
            ("Hand", False, self._create_group_checkbox_callback("hand")),
        ]
        self._joint_group_checkboxes = create_checkbox_group(checkbox_configs, columns=2)
        with ui.HStack(height=UILayout.BUTTON_HEIGHT):
            create_styled_button("All ON", callback=self._on_all_groups_enable, color_scheme="green", height=UILayout.BUTTON_HEIGHT_SMALL)
            create_styled_button("All OFF", callback=self._on_all_groups_disable, color_scheme="red", height=UILayout.BUTTON_HEIGHT_SMALL)
            ui.Spacer()
        create_separator(UILayout.SEPARATOR_HEIGHT)

    def _on_all_groups_enable(self):
        if self._scenario:
            self._scenario.set_all_groups_enabled(True)
            self._sync_checkboxes_to_scenario()

    def _on_all_groups_disable(self):
        if self._scenario:
            self._scenario.set_all_groups_enabled(False)
            self._sync_checkboxes_to_scenario()


    _GROUP_KEY_TO_CHECKBOX_LABEL = {"body": "Body", "left_arm": "Left Arm", "right_arm": "Right Arm", "hand": "Hand"}

    def _sync_checkboxes_to_scenario(self):
        if not self._scenario or not hasattr(self, "_joint_group_checkboxes"):
            return
        group_states = self._scenario.get_all_group_states()
        for group_key, checkbox_text in self._GROUP_KEY_TO_CHECKBOX_LABEL.items():
            if checkbox_text in self._joint_group_checkboxes:
                current_state = group_states.get(group_key, False)
                self._joint_group_checkboxes[checkbox_text].model.set_value(current_state)

    # ----- Sim2sim Deploy -----
    def _build_sim2sim_deploy(self):
        sim2sim_frame = CollapsableFrame("Sim2sim Deploy", collapsed=UIConfig.JOINT_CONTROL_COLLAPSED)
        with sim2sim_frame:
            with ui.VStack(style=get_style(), spacing=UILayout.SPACING_SMALL, height=0):
                create_styled_button(
                    "Open Sim2Real Debugger",
                    callback=self._on_open_sim2real_debugger,
                    color_scheme="green",
                    height=UILayout.BUTTON_HEIGHT,
                )

    def _on_open_sim2real_debugger(self):
        try:
            from .sim2sim_console.sim2real_debugger_gui import show_debugger
            show_debugger()
        except Exception as e:
            import traceback
            print(f"Sim2Real Debugger GUI 실행 실패: {e}")
            traceback.print_exc()

    # ----- 오버레이 윈도우 (관절/손 텍스트) — scenario가 self를 overlay_ui로 사용 -----
    _OVERLAY_LABEL_STYLE = {"color": UIColors.TEXT_PRIMARY, "font_size": OverlayConfig.FONT_SIZE, "margin": OverlayConfig.LABEL_MARGIN, "word_wrap": False}

    def _create_overlay_window(self, title, width, height, pos_x, pos_y, border_color, placeholder_text, window_attr, label_attr, success_msg, error_msg):
        try:
            window = ui.Window(title, width=width, height=height, flags=OVERLAY_WINDOW_FLAGS, position_x=pos_x, position_y=pos_y)
            window.visible = False
            setattr(self, window_attr, window)
            with window.frame:
                with ui.ZStack():
                    ui.Rectangle(style={"border_radius": OverlayConfig.BORDER_RADIUS, "border_width": OverlayConfig.BORDER_WIDTH, "border_color": border_color})
                    with ui.VStack(spacing=OverlayConfig.VSTACK_SPACING):
                        ui.Spacer(height=OverlayConfig.SPACER_HEIGHT)
                        label = ui.Label(placeholder_text, alignment=ui.Alignment.LEFT_TOP, style=self._OVERLAY_LABEL_STYLE)
                        ui.Spacer(height=OverlayConfig.SPACER_HEIGHT)
            setattr(self, label_attr, label)
        except Exception as e:
            import traceback
            traceback.print_exc()

    def create_joint_overlay_window(self):
        self._create_overlay_window(
            "Text Overlay",
            OverlayConfig.JOINT_OVERLAY_WIDTH, OverlayConfig.JOINT_OVERLAY_HEIGHT,
            OverlayConfig.JOINT_OVERLAY_POSITION_X, OverlayConfig.JOINT_OVERLAY_POSITION_Y,
            OverlayConfig.JOINT_OVERLAY_BORDER_COLOR, "Loading Joint Data...",
            "_text_overlay_window", "_joint_data_label",
            "✅ Joint Data 텍스트 오버레이 생성 완료", "❌ 텍스트 오버레이 생성 실패",
        )

    def create_hand_overlay_window(self):
        self._create_overlay_window(
            "Hand Joint Overlay",
            OverlayConfig.HAND_OVERLAY_WIDTH, OverlayConfig.HAND_OVERLAY_HEIGHT,
            OverlayConfig.HAND_OVERLAY_POSITION_X, OverlayConfig.HAND_OVERLAY_POSITION_Y,
            OverlayConfig.HAND_OVERLAY_BORDER_COLOR, "Loading Hand Joint Data...",
            "_hand_overlay_window", "_hand_data_label",
            "✅ 손 관절 오버레이 생성 완료", "❌ 손 관절 오버레이 생성 실패",
        )

    def _update_overlay_label(self, label, format_fn, error_prefix):
        if label is None:
            return
        try:
            label.text = format_fn()
        except Exception as e:
            label.text = f"❌ {error_prefix}:\n{str(e)}"

    def update_joint_display_text(self, scenario):
        self._update_overlay_label(self._joint_data_label, scenario.format_all_joint_text, "Display Error")

    def update_hand_display_text(self, scenario):
        self._update_overlay_label(self._hand_data_label, scenario.format_hand_joint_text, "Hand Data Error")

    def _set_overlay_visibility(self, window, show: bool, window_name: str):
        try:
            if window is not None:
                window.visible = show
        except Exception:
            pass

    def set_hand_window_visibility(self, show: bool):
        self._set_overlay_visibility(self._hand_overlay_window, show, "손 관절 윈도우")

    def set_joint_window_visibility(self, should_show: bool):
        self._set_overlay_visibility(self._text_overlay_window, should_show, "관절 윈도우")

    # ----- ROS2 -----
    def _setup_ros2_manager(self):
        if self._ros2_manager is None:
            self._ros2_manager = ROS2IntegratedManager(scenario_ref=self._scenario)
            self._scenario.set_ros2_manager(self._ros2_manager)

        success = self._ros2_manager.initialize()
        if not success:
            print("❌ ROS2 Manager initialization failed!")
            return False
        self._ros2_manager.set_joint_control_reference(self._effective_joints, self._joint_values)
        self._update_topic_mode_status()
        if self._scenario:
            self._scenario.set_ros2_subscriber_status(False)
        self._sync_all_ros2_status()
        return True

    def _toggle_topic_mode(self):
        if not self._ros2_manager or not self._ros2_manager.is_initialized():
            return
        try:
            success, status_message = self._ros2_manager.toggle_topic_mode()
            if success:
                current_mode = self._ros2_manager.get_current_topic_mode()
                mode_display = ROS2Config.get_topic_mode_display_name(current_mode)
                self._topic_mode_status_label.text = f"Control Mode: {mode_display}"
                if self._scenario:
                    self._scenario._update_cached_topic_mode()
                    self._scenario.update_joint_display_text()
                    self._scenario.update_hand_display_text()
        except Exception:
            pass

    def _toggle_publisher(self):
        if not self._ros2_manager or not self._ros2_manager.is_initialized():
            return False
        success, status = self._ros2_manager.toggle_publisher()
        if hasattr(self, "_pub_status_label"):
            self._pub_status_label.text = status
        return success

    def _toggle_subscriber(self):
        if not self._ros2_manager or not self._ros2_manager.is_initialized():
            return False
        success, status = self._ros2_manager.toggle_subscriber()
        if hasattr(self, "_sub_status_label"):
            self._sub_status_label.text = status
        self._update_topic_mode_status()
        return success

    def _cleanup_ros2(self):
        if hasattr(self, "_ros2_manager") and self._ros2_manager:
            self._ros2_manager.cleanup()
            self._ros2_manager = None
        try:
            if hasattr(self, "_sub_status_label") and self._sub_status_label is not None:
                self._sub_status_label.text = "Subscriber: OFF"
            if hasattr(self, "_topic_mode_status_label") and self._topic_mode_status_label is not None:
                self._topic_mode_status_label.text = "Control Mode: Current"
        except Exception:
            pass

    def _update_topic_mode_status(self):
        if self._topic_mode_status_label and self._ros2_manager and self._ros2_manager.is_initialized():
            try:
                current_mode = self._ros2_manager.get_current_topic_mode()
                mode_display = ROS2Config.get_topic_mode_display_name(current_mode)
                self._topic_mode_status_label.text = f"Control Mode: {mode_display}"
            except Exception:
                pass

    def _sync_all_ros2_status(self):
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
        except Exception:
            pass

    # ----- Isaac Sim 이벤트 콜백 -----
    def on_menu_callback(self):
        pass

    def on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            if hasattr(self, "_scenario_state_btn") and self._scenario_state_btn:
                self._scenario_state_btn.reset()
                self._scenario_state_btn.enabled = False

    def on_physics_step(self, step: float):
        pass

    def on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED):
            self._reset_extension()

    _CLEANUP_UI_ATTRS = (
        "_load_btn", "_reset_btn", "_scenario_state_btn",
        "_pub_status_label", "_sub_status_label", "_topic_mode_status_label",
        "_joint_group_checkboxes",
    )

    def cleanup(self):
        for ui_elem in self.wrapped_ui_elements:
            try:
                ui_elem.cleanup()
            except Exception:
                pass
        self.wrapped_ui_elements.clear()
        for attr in self._CLEANUP_UI_ATTRS:
            if hasattr(self, attr):
                setattr(self, attr, None)
        self._cleanup_ros2()

    def build_ui(self):
        self._build_world_controls()
        self._build_joint_controls()
        self._build_sim2sim_deploy()

    # ----- 시나리오 (LOAD / Reset / RUN) -----
    def _setup_scene(self):
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
        self._scenario.setup()
        if hasattr(self, "_scenario_state_btn") and self._scenario_state_btn:
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = True
        if hasattr(self, "_reset_btn") and self._reset_btn:
            self._reset_btn.enabled = True

    def _on_post_reset_btn(self):
        self._cleanup_ros2()
        self._scenario.reset()
        self._setup_ros2_manager()
        if hasattr(self, "_scenario_state_btn") and self._scenario_state_btn:
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = True

    def _update_scenario(self, step: float):
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
        self._initialization_attempts += 1
        success = self._scenario.delayed_initialization()
        if success:
            self._articulation_initialized = True
        elif self._initialization_attempts >= self._max_initialization_attempts:
            print("❌ Articulation 초기화 최대 시도 횟수 초과 (수동 초기화 버튼 사용 권장)")

    def _on_run_scenario_a_text(self):
        self._timeline.play()
        self._articulation_initialized = False
        self._physics_step_count = 0
        self._initialization_attempts = 0

    def _on_run_scenario_b_text(self):
        self._timeline.pause()

    def _reset_extension(self):
        self._on_init()
        self._reset_ui()

    def _reset_ui(self):
        if hasattr(self, "_scenario_state_btn") and self._scenario_state_btn:
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False
        if hasattr(self, "_reset_btn") and self._reset_btn:
            self._reset_btn.enabled = False
