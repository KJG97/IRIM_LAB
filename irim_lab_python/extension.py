# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import gc

import omni
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType

from .global_variables import EXTENSION_TITLE, SENSOR_LAB_TITLE, SIM2SIM_TITLE
from .ui_builder import UIBuilder
from .sensor_lab_ui_builder import SensorLabUIBuilder
from .core.asset_manager import ALLEXAssetManager

"""
This file serves as a basic template for the standard boilerplate operations
that make a UI-based extension appear on the toolbar.

This implementation is meant to cover most use-cases without modification.
Various callbacks are hooked up to a seperate class UIBuilder in .ui_builder.py
Most users will be able to make their desired UI extension by interacting solely with
UIBuilder.

This class sets up standard useful callback functions in UIBuilder:
    on_menu_callback: Called when extension is opened
    on_timeline_event: Called when timeline is stopped, paused, or played
    on_physics_step: Called on every physics step
    on_stage_event: Called when stage is opened or closed
    cleanup: Called when resources such as physics subscriptions should be cleaned up
    build_ui: User function that creates the UI they want.
"""


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""

        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ui.Window(
            title=SIM2SIM_TITLE, 
            width=400, 
            height=500, 
            visible=False, 
            flags=ui.WINDOW_FLAGS_NO_COLLAPSE | ui.WINDOW_FLAGS_NO_RESIZE,
            padding_x=10,
            padding_y=10,
            #dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{SIM2SIM_TITLE}",
            self._menu_callback,
            description=f"Add {SIM2SIM_TITLE} Extension to UI toolbar",
        )
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{SENSOR_LAB_TITLE}",
            self._sensor_lab_menu_callback,
            description="Load ALLEX Sensor Test Asset",
        )
        self._menu_items = [
            MenuItemDescription(name=SIM2SIM_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{SIM2SIM_TITLE}")),
            MenuItemDescription(name=SENSOR_LAB_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{SENSOR_LAB_TITLE}"))
        ]

        add_menu_items(self._menu_items, EXTENSION_TITLE)

        # Filled in with User Functions
        self.ui_builder = UIBuilder()
        
        # Sensor Lab Window
        self._sensor_lab_window = ui.Window(
            title="ALLEX Sensor Lab", 
            width=400, 
            height=300, 
            visible=False, 
            flags=ui.WINDOW_FLAGS_NO_COLLAPSE | ui.WINDOW_FLAGS_NO_RESIZE,
            padding_x=10,
            padding_y=10,
        )
        self._sensor_lab_window.set_visibility_changed_fn(self._on_sensor_lab_window)
        self.sensor_lab_ui_builder = SensorLabUIBuilder()

        # Events
        self._usd_context = omni.usd.get_context()
        self._physxIFace = _physx.acquire_physx_interface()
        self._physx_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self):
        self._models = {}
        remove_menu_items(self._menu_items, EXTENSION_TITLE)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{SIM2SIM_TITLE}")
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{SENSOR_LAB_TITLE}")

        if self._window:
            self._window = None
        if self._sensor_lab_window:
            self._sensor_lab_window = None
        self.ui_builder.cleanup()
        if hasattr(self, 'sensor_lab_ui_builder'):
            self.sensor_lab_ui_builder.cleanup()
        gc.collect()

    def _on_window(self, visible):
        if not getattr(self, "_window", None):
            return
        if self._window.visible:
            # Subscribe to Stage and Timeline Events (중복 구독 방지)
            if not hasattr(self, '_timeline_event_sub') or self._timeline_event_sub is None:
                stream = self._timeline.get_timeline_event_stream()
                self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)
            
            if not hasattr(self, '_stage_event_sub') or self._stage_event_sub is None:
                self._usd_context = omni.usd.get_context()
                events = self._usd_context.get_stage_event_stream()
                self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)

            self._build_ui()
        else:
            # 메인 윈도우가 닫혀도 센서 랩이 열려있으면 구독 유지
            if not (hasattr(self, '_sensor_lab_window') and self._sensor_lab_window and self._sensor_lab_window.visible):
                self._usd_context = None
                self._stage_event_sub = None
                self._timeline_event_sub = None
            self.ui_builder.cleanup()

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        # async def dock_window():
        #     await omni.kit.app.get_app().next_update_async()

        #     def dock(space, name, location, pos=0.5):
        #         window = omni.ui.Workspace.get_window(name)
        #         if window and space:
        #             window.dock_in(space, location, pos)
        #         return window

        #     tgt = ui.Workspace.get_window("Viewport")
        #     dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)
        #     await omni.kit.app.get_app().next_update_async()

        # self._task = asyncio.ensure_future(dock_window())

    #################################################################
    # Functions below this point call user functions
    #################################################################

    def _menu_callback(self):
        if not getattr(self, "_window", None):
            return
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _sensor_lab_menu_callback(self):
        """{SENSOR_LAB_TITLE} 메뉴 클릭 시 센서 랩 창 열기"""
        if not getattr(self, "_sensor_lab_window", None):
            return
        self._sensor_lab_window.visible = not self._sensor_lab_window.visible
        self.sensor_lab_ui_builder.on_menu_callback()
    
    def _on_sensor_lab_window(self, visible):
        """센서 랩 창 가시성 변경 시 호출"""
        if not getattr(self, "_sensor_lab_window", None):
            return
        if self._sensor_lab_window.visible:
            # 타임라인 이벤트 구독 (메인 윈도우와 독립적으로)
            if not hasattr(self, '_timeline_event_sub') or self._timeline_event_sub is None:
                stream = self._timeline.get_timeline_event_stream()
                self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)
            
            # 스테이지 이벤트 구독
            if not hasattr(self, '_stage_event_sub') or self._stage_event_sub is None:
                self._usd_context = omni.usd.get_context()
                events = self._usd_context.get_stage_event_stream()
                self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            
            self._build_sensor_lab_ui()
        else:
            if hasattr(self, 'sensor_lab_ui_builder'):
                self.sensor_lab_ui_builder.cleanup()
    
    def _build_sensor_lab_ui(self):
        """센서 랩 UI 빌드"""
        with self._sensor_lab_window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_sensor_lab_extension_ui()
    
    def _build_sensor_lab_extension_ui(self):
        """센서 랩 확장 UI 빌드"""
        self.sensor_lab_ui_builder.build_ui()

    def _on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if not self._physx_subscription:
                try:
                    self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)
                except Exception:
                    pass
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physx_subscription = None

        self.ui_builder.on_timeline_event(event)

    def _on_physics_step(self, step):
        self.ui_builder.on_physics_step(step)
        if hasattr(self, 'sensor_lab_ui_builder'):
            self.sensor_lab_ui_builder.on_physics_step(step)

    def _on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            # stage was opened or closed, cleanup
            self._physx_subscription = None
            self.ui_builder.cleanup()

            # 창이 열려있다면 UI 즉시 재구축
            if getattr(self, "_window", None) and self._window.visible:
                self._build_ui()

        self.ui_builder.on_stage_event(event)

    def _build_extension_ui(self):
        # Call user function for building UI
        self.ui_builder.build_ui()
