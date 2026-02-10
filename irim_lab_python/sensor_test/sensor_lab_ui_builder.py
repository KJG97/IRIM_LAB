"""
Sensor Lab용 간단한 UI 빌더 (sim2sim_test 무의존)
"""

import omni.ui as ui
import omni.usd
from pxr import UsdGeom, Gf, Vt, UsdPhysics, UsdShade

from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.api.world import World
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.examples.extension.core_connectors import LoadButton

from .sensor_lab_config import (
    SENSOR_LAB_CAMERA_EYE,
    SENSOR_LAB_CAMERA_TARGET,
    SENSOR_LAB_CAMERA_PRIM_PATH,
    PHYSICS_DT,
    RENDERING_DT,
    GRAVITY,
    WORLD_CONTROLS_COLLAPSED,
    SPACING_SMALL,
    SPACING_MEDIUM,
    SIDEBAR_WIDTH,
    BUTTON_HEIGHT,
    BUTTON_BORDER_RADIUS,
    SIDEBAR_SUCCESS,
    SIDEBAR_RESET,
    LOAD_BUTTON_BG,
    LOAD_BUTTON_BORDER,
    LOAD_BUTTON_HOVER,
    RED_BUTTON_BG,
    RED_BUTTON_BORDER,
    RED_BUTTON_HOVER,
    STATE_BUTTON_BG,
    STATE_BUTTON_BORDER,
    STATE_BUTTON_HOVER,
    TEXT_PRIMARY,
    BUTTON_BORDER_WIDTH_THICK,
    BUTTON_BORDER_RADIUS_XLARGE,
    BUTTON_MARGIN,
    BUTTON_PADDING,
)
from .sensor_lab_asset import load_sensor_test_asset
from .sensor_lab_joint_torque_test import JointTorqueTest


def _load_button_style():
    return {
        **get_style(),
        "Button": {
            "background_color": LOAD_BUTTON_BG,
            "border_width": BUTTON_BORDER_WIDTH_THICK,
            "border_color": LOAD_BUTTON_BORDER,
            "border_radius": BUTTON_BORDER_RADIUS_XLARGE,
            "margin": BUTTON_MARGIN,
            "padding": BUTTON_PADDING,
        },
        "Button:hovered": {"background_color": LOAD_BUTTON_HOVER},
        "Button.Label": {"color": TEXT_PRIMARY},
    }


def _red_button_style():
    return {
        "Button": {
            "background_color": RED_BUTTON_BG,
            "border_width": 1,
            "border_color": RED_BUTTON_BORDER,
            "border_radius": 3,
        },
        "Button:hovered": {"background_color": RED_BUTTON_HOVER},
    }


def _green_button_style():
    return {
        "Button": {
            "background_color": STATE_BUTTON_BG,
            "border_width": 1,
            "border_color": STATE_BUTTON_BORDER,
            "border_radius": 3,
        },
        "Button:hovered": {"background_color": STATE_BUTTON_HOVER},
    }


class SensorLabWorldControlsBuilder:
    """Sensor Lab용 World Controls 섹션 UI 빌더."""

    def __init__(self, sensor_lab_ui_builder_ref):
        self.sensor_lab_ui_builder = sensor_lab_ui_builder_ref

    def build(self):
        world_controls_frame = CollapsableFrame("World Controls", collapsed=WORLD_CONTROLS_COLLAPSED)
        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=SPACING_SMALL):
                self._build_load_button()
                self._build_torque_test_button()
                self._build_sphere_buttons()
                self._apply_button_styles()

    def _build_load_button(self):
        with ui.HStack():
            ui.Spacer(width=SPACING_SMALL)
            ui.Rectangle(width=SIDEBAR_WIDTH, height=BUTTON_HEIGHT, style={"background_color": SIDEBAR_SUCCESS, "border_radius": BUTTON_BORDER_RADIUS})
            ui.Spacer(width=SPACING_MEDIUM)
            self.sensor_lab_ui_builder._load_btn = LoadButton(
                "Scene Loader",
                "LOAD",
                setup_scene_fn=self.sensor_lab_ui_builder._setup_sensor_scene,
                setup_post_load_fn=None,
            )
            self.sensor_lab_ui_builder._load_btn.set_world_settings(
                physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT, sim_params={"gravity": GRAVITY}
            )
            self.sensor_lab_ui_builder.wrapped_ui_elements.append(self.sensor_lab_ui_builder._load_btn)
            ui.Spacer(width=SPACING_SMALL)

    def _build_torque_test_button(self):
        with ui.HStack():
            ui.Spacer(width=SPACING_SMALL)
            ui.Rectangle(width=SIDEBAR_WIDTH, height=BUTTON_HEIGHT, style={"background_color": SIDEBAR_RESET, "border_radius": BUTTON_BORDER_RADIUS})
            ui.Spacer(width=SPACING_MEDIUM)
            self.sensor_lab_ui_builder._torque_test_btn = ui.Button(
                "Joint Torque Test",
                clicked_fn=self.sensor_lab_ui_builder._toggle_joint_torque_test,
                height=BUTTON_HEIGHT,
                style=_red_button_style(),
            )
            ui.Spacer(width=SPACING_SMALL)

    def _build_sphere_buttons(self):
        with ui.HStack():
            ui.Spacer(width=SPACING_SMALL)
            ui.Label("Test Sphere:", style={"color": 0xFFAAAAAA, "font_size": 12})
        with ui.HStack():
            ui.Spacer(width=SPACING_SMALL)
            ui.Rectangle(width=SIDEBAR_WIDTH, height=BUTTON_HEIGHT, style={"background_color": SIDEBAR_SUCCESS, "border_radius": BUTTON_BORDER_RADIUS})
            ui.Spacer(width=SPACING_MEDIUM)
            self.sensor_lab_ui_builder._sphere_large_btn = ui.Button("10g", clicked_fn=self.sensor_lab_ui_builder._create_sphere_large, height=BUTTON_HEIGHT, width=60, style=_green_button_style())
            ui.Spacer(width=SPACING_SMALL)
            self.sensor_lab_ui_builder._sphere_medium_btn = ui.Button("5g", clicked_fn=self.sensor_lab_ui_builder._create_sphere_medium, height=BUTTON_HEIGHT, width=60, style=_green_button_style())
            ui.Spacer(width=SPACING_SMALL)
            self.sensor_lab_ui_builder._sphere_small_btn = ui.Button("0.3g", clicked_fn=self.sensor_lab_ui_builder._create_sphere_small, height=BUTTON_HEIGHT, width=60, style=_green_button_style())
            ui.Spacer(width=SPACING_SMALL)

    def _apply_button_styles(self):
        if hasattr(self.sensor_lab_ui_builder, "_load_btn") and self.sensor_lab_ui_builder._load_btn and hasattr(self.sensor_lab_ui_builder._load_btn, "_button") and self.sensor_lab_ui_builder._load_btn._button:
            self.sensor_lab_ui_builder._load_btn._button.style = _load_button_style()


class SensorLabUIBuilder:
    """센서 랩용 간단한 UI 빌더"""

    def __init__(self):
        self.wrapped_ui_elements = []
        self._joint_torque_test = JointTorqueTest()
        self._torque_test_initialized = False

    def build_ui(self):
        world_controls_builder = SensorLabWorldControlsBuilder(self)
        world_controls_builder.build()

    def _setup_sensor_scene(self):
        if self._torque_test_initialized:
            self._joint_torque_test.cleanup()
            self._torque_test_initialized = False

        sensor_articulation = load_sensor_test_asset()
        if sensor_articulation is not None:
            try:
                world = World.instance()
                if world is not None:
                    world.scene.add(sensor_articulation)
            except Exception:
                pass
            self._setup_sensor_camera_view()

    def _setup_sensor_camera_view(self):
        try:
            set_camera_view(
                eye=SENSOR_LAB_CAMERA_EYE,
                target=SENSOR_LAB_CAMERA_TARGET,
                camera_prim_path=SENSOR_LAB_CAMERA_PRIM_PATH,
            )
        except Exception:
            pass

    SPHERE_CONFIGS = {
        "large": {"mass": 0.010, "scale": 0.005, "label": "10g"},
        "medium": {"mass": 0.005, "scale": 0.0035, "label": "5g"},
        "small": {"mass": 0.0003, "scale": 0.002, "label": "0.3g"},
    }

    def _create_rigid_body_sphere(self, config_name: str, static_friction=5.0, dynamic_friction=5.0):
        try:
            config = self.SPHERE_CONFIGS.get(config_name)
            if config is None:
                return

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return

            sphere_path = "/World/TestSphere"
            material_path = "/World/TestSphere/PhysicsMaterial"

            existing_prim = stage.GetPrimAtPath(sphere_path)
            if existing_prim.IsValid():
                stage.RemovePrim(sphere_path)

            sphere = UsdGeom.Sphere.Define(stage, sphere_path)
            sphere_prim = sphere.GetPrim()
            xformable = UsdGeom.Xformable(sphere_prim)
            xformable.AddTranslateOp().Set(Gf.Vec3d(0.66781, -0.23008, 1.1))
            scale = config["scale"]
            xformable.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
            sphere.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.0, 1.0, 0.0)]))

            rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(sphere_prim)
            rigid_body_api.CreateRigidBodyEnabledAttr(True)
            mass_api = UsdPhysics.MassAPI.Apply(sphere_prim)
            mass_api.CreateMassAttr(config["mass"])

            physics_material = UsdShade.Material.Define(stage, material_path)
            physics_material_api = UsdPhysics.MaterialAPI.Apply(physics_material.GetPrim())
            physics_material_api.CreateStaticFrictionAttr(static_friction)
            physics_material_api.CreateDynamicFrictionAttr(dynamic_friction)
            physics_material_api.CreateRestitutionAttr(0.0)

            UsdPhysics.CollisionAPI.Apply(sphere_prim)
            collision_api = UsdPhysics.MeshCollisionAPI.Apply(sphere_prim)
            collision_api.CreateApproximationAttr("convexHull")

            binding_api = UsdShade.MaterialBindingAPI.Apply(sphere_prim)
            binding_api.Bind(physics_material, UsdShade.Tokens.weakerThanDescendants, "physics")

            mass_grams = config["mass"] * 1000
            self._joint_torque_test.set_sphere_mass_display(mass_grams)
        except Exception as e:
            print(f"❌ Rigid Body 구 생성 실패: {e}")

    def _create_sphere_large(self):
        self._create_rigid_body_sphere("large")

    def _create_sphere_medium(self):
        self._create_rigid_body_sphere("medium")

    def _create_sphere_small(self):
        self._create_rigid_body_sphere("small")

    def on_menu_callback(self):
        pass

    def on_timeline_event(self, event):
        pass

    def on_physics_step(self, step):
        if self._joint_torque_test.is_active:
            self._joint_torque_test.update()

    def on_stage_event(self, event):
        pass

    def _toggle_joint_torque_test(self):
        if not self._torque_test_initialized:
            success = self._joint_torque_test.initialize()
            if success:
                self._torque_test_initialized = True
            else:
                return
        if self._joint_torque_test.is_active:
            self._joint_torque_test.stop()
        else:
            self._joint_torque_test.start()

    def cleanup(self):
        if hasattr(self, "_joint_torque_test"):
            self._joint_torque_test.cleanup()
        self.wrapped_ui_elements = []
