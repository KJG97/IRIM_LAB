"""
Sensor Lab용 간단한 UI 빌더
"""

import omni.usd
from pxr import UsdGeom, Gf, Vt, UsdPhysics, UsdShade

from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation

from ..sim2sim_test.core.asset_manager import ALLEXAssetManager
from ..sim2sim_test.ui_builder import SensorLabWorldControlsBuilder
from ..sim2sim_test.utils.constants import SENSOR_LAB_CAMERA_EYE, SENSOR_LAB_CAMERA_TARGET, SENSOR_LAB_CAMERA_PRIM_PATH
from .sensor_lab_joint_torque_test import JointTorqueTest


class SensorLabUIBuilder:
    """센서 랩용 간단한 UI 빌더"""
    
    def __init__(self):
        """초기화"""
        self.wrapped_ui_elements = []
        self._asset_manager = ALLEXAssetManager()
        self._joint_torque_test = JointTorqueTest()
        self._torque_test_initialized = False
    
    def build_ui(self):
        """센서 랩용 UI 구성"""
        # World Controls 섹션
        world_controls_builder = SensorLabWorldControlsBuilder(self)
        world_controls_builder.build()
    
    def _setup_sensor_scene(self):
        """센서 테스트 에셋 로드 및 카메라 뷰 설정"""
        # 기존 토크 테스트 리셋 (두 번째 LOAD 시 무효화된 Articulation 참조 방지)
        if self._torque_test_initialized:
            self._joint_torque_test.cleanup()
            self._torque_test_initialized = False
        
        sensor_articulation = self._asset_manager.load_sensor_test_asset()
        if sensor_articulation is not None:
            try:
                world = World.instance()
                if world is not None:
                    world.scene.add(sensor_articulation)
            except Exception:
                pass
            
            self._setup_sensor_camera_view()
    
    def _setup_sensor_camera_view(self):
        """센서 랩용 카메라 뷰 설정"""
        try:
            set_camera_view(
                eye=SENSOR_LAB_CAMERA_EYE,
                target=SENSOR_LAB_CAMERA_TARGET,
                camera_prim_path=SENSOR_LAB_CAMERA_PRIM_PATH
            )
        except Exception:
            pass
    
    # 구 생성 설정 (Mass: kg, Scale: m)
    SPHERE_CONFIGS = {
        "large":  {"mass": 0.010,   "scale": 0.005,   "label": "10g"},   # 10g
        "medium": {"mass": 0.005,   "scale": 0.0035,  "label": "5g"},    # 5g
        "small":  {"mass": 0.0003,  "scale": 0.002,   "label": "0.3g"},  # 0.3g
    }
    
    def _create_rigid_body_sphere(self, config_name: str, static_friction=5.0, dynamic_friction=5.0):
        """Rigid Body 구 생성
        
        Args:
            config_name: 구 설정 이름 ("large", "medium", "small")
            static_friction: 정지 마찰 계수
            dynamic_friction: 동적 마찰 계수
        """
        try:
            config = self.SPHERE_CONFIGS.get(config_name)
            if config is None:
                print(f"⚠️ 알 수 없는 구 설정: {config_name}")
                return
            
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return
            
            sphere_path = "/World/TestSphere"
            material_path = "/World/TestSphere/PhysicsMaterial"
            
            # 기존 프림이 있으면 삭제
            existing_prim = stage.GetPrimAtPath(sphere_path)
            if existing_prim.IsValid():
                stage.RemovePrim(sphere_path)
            
            # Sphere 생성
            sphere = UsdGeom.Sphere.Define(stage, sphere_path)
            sphere_prim = sphere.GetPrim()
            
            # 위치 설정
            xformable = UsdGeom.Xformable(sphere_prim)
            xformable.AddTranslateOp().Set(Gf.Vec3d(0.66781, -0.23008, 1.1))
            
            # 스케일 설정
            scale = config["scale"]
            xformable.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
            
            # 초록색 설정 (displayColor)
            sphere.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.0, 1.0, 0.0)]))
            
            # Rigid Body API 적용
            rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(sphere_prim)
            rigid_body_api.CreateRigidBodyEnabledAttr(True)
            
            # Mass API 적용
            mass_api = UsdPhysics.MassAPI.Apply(sphere_prim)
            mass_api.CreateMassAttr(config["mass"])
            
            # Physics Material 생성 및 마찰 계수 설정
            physics_material = UsdShade.Material.Define(stage, material_path)
            physics_material_api = UsdPhysics.MaterialAPI.Apply(physics_material.GetPrim())
            physics_material_api.CreateStaticFrictionAttr(static_friction)
            physics_material_api.CreateDynamicFrictionAttr(dynamic_friction)
            physics_material_api.CreateRestitutionAttr(0.0)  # 반발 계수 (탄성)
            
            # Collision API 적용
            UsdPhysics.CollisionAPI.Apply(sphere_prim)
            collision_api = UsdPhysics.MeshCollisionAPI.Apply(sphere_prim)
            collision_api.CreateApproximationAttr("convexHull")
            
            # Material 바인딩
            binding_api = UsdShade.MaterialBindingAPI.Apply(sphere_prim)
            binding_api.Bind(physics_material, UsdShade.Tokens.weakerThanDescendants, "physics")
            
            print(f"✅ Rigid Body 구 생성 완료: {sphere_path}")
            print(f"   크기: {config['label']}, Mass: {config['mass']*1000:.1f}g, Scale: {scale}")
            
            # Viewport에 구 무게 표시 업데이트
            mass_grams = config['mass'] * 1000  # kg -> g 변환
            self._joint_torque_test.set_sphere_mass_display(mass_grams)
            
        except Exception as e:
            print(f"⚠️ Rigid Body 구 생성 실패: {e}")
    
    def _create_sphere_large(self):
        """큰 구 생성 (10g, scale 0.005)"""
        self._create_rigid_body_sphere("large")
    
    def _create_sphere_medium(self):
        """중간 구 생성 (5g, scale 0.0035)"""
        self._create_rigid_body_sphere("medium")
    
    def _create_sphere_small(self):
        """작은 구 생성 (0.3g, scale 0.002)"""
        self._create_rigid_body_sphere("small")
    
    def on_menu_callback(self):
        """메뉴 콜백 (필요시 구현)"""
        pass
    
    def on_timeline_event(self, event):
        """타임라인 이벤트 (필요시 구현)"""
        pass
    
    def on_physics_step(self, step):
        """물리 스텝 이벤트 - 토크 테스트 업데이트"""
        if self._joint_torque_test.is_active:
            self._joint_torque_test.update()
    
    def on_stage_event(self, event):
        """스테이지 이벤트 (필요시 구현)"""
        pass
    
    def _toggle_joint_torque_test(self):
        """관절 토크 테스트 토글"""
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
        """리소스 정리"""
        if hasattr(self, '_joint_torque_test'):
            self._joint_torque_test.cleanup()
        self.wrapped_ui_elements = []
