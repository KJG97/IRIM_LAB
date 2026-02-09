"""
시나리오 관련 이벤트 핸들러
"""

from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import create_new_stage


class ScenarioEventHandler:
    """시나리오 관련 이벤트를 처리하는 클래스"""
    
    def __init__(self, ui_builder_ref):
        """이벤트 핸들러 초기화
        
        Args:
            ui_builder_ref: UIBuilder 인스턴스 참조
        """
        self._ui_builder = ui_builder_ref
    
    def setup_scene(self):
        """🔄 Scene 설정 - LOAD 버튼 콜백"""
        create_new_stage()

        # 🆕 조명 모드 설정 추가
        from omni.kit.viewport.menubar.lighting.actions import _set_lighting_mode
        
        # 원하는 조명 모드 선택 (예: Default 조명 리그)
        _set_lighting_mode(lighting_mode="Stage Lights")

        # RTX 설정 제거 - 기본값 사용 (DLSS 에러 방지)

        loaded_objects = self._ui_builder._scenario.load_example_assets()

        # Add user-loaded objects to the World
        world = World.instance()
        
        if isinstance(loaded_objects, (list, tuple)):
            for loaded_object in loaded_objects:
                world.scene.add(loaded_object)
        else:
            world.scene.add(loaded_objects)
        
    
    def setup_scenario(self):
        """🔄 Scenario 설정 - ROS2는 UI Builder가 관리"""
        self._ui_builder._scenario.setup()

        # UI management (널 가드)
        if hasattr(self._ui_builder, "_scenario_state_btn") and self._ui_builder._scenario_state_btn:
            self._ui_builder._scenario_state_btn.reset()
            self._ui_builder._scenario_state_btn.enabled = True   # ← RUN 활성화
        if hasattr(self._ui_builder, "_reset_btn") and self._ui_builder._reset_btn:
            self._ui_builder._reset_btn.enabled = True
        
    
    def on_post_reset_btn(self):
        """🔄 Reset 버튼 콜백"""
        # ROS2 정리
        from .ros2_event_handler import ROS2EventHandler
        ros2_handler = ROS2EventHandler(self._ui_builder)
        ros2_handler.cleanup_ros2()
        
        # scenario 리셋
        self._ui_builder._scenario.reset()

        # 🆕 ROS2 Manager 재설정
        self._ui_builder._setup_ros2_manager()
        
        # Visibility 리셋
        from .visibility_event_handler import VisibilityEventHandler
        visibility_handler = VisibilityEventHandler(self._ui_builder)
        visibility_handler.reset_hand_force_visibility()
        
        # UI management
        self._ui_builder._scenario_state_btn.reset()
        self._ui_builder._scenario_state_btn.enabled = True
            
    def update_scenario(self, step: float):
        """시나리오 업데이트 - physics step마다 호출"""
        # Physics step 카운트 증가
        self._ui_builder._physics_step_count += 1
        
        # 첫 몇 step 후에 Articulation 초기화 시도
        if (not self._ui_builder._articulation_initialized and 
            self._ui_builder._physics_step_count >= 3 and 
            self._ui_builder._initialization_attempts < self._ui_builder._max_initialization_attempts):
            
            self._try_delayed_initialization()
        
        # 기존 시나리오 업데이트
        done = self._ui_builder._scenario.update(step)
        if done:
            self._ui_builder._scenario_state_btn.enabled = False
    
    def _try_delayed_initialization(self):
        """🆕 지연된 Articulation 초기화 시도"""
        self._ui_builder._initialization_attempts += 1        
        success = self._ui_builder._scenario.delayed_initialization()
        
        if success:
            self._ui_builder._articulation_initialized = True
        else:
            print(f"⚠️ Articulation 초기화 실패 (시도 {self._ui_builder._initialization_attempts})")
            
            if self._ui_builder._initialization_attempts >= self._ui_builder._max_initialization_attempts:
                print("❌ Articulation 초기화 최대 시도 횟수 초과")
                print("🔧 수동 초기화 버튼을 사용해보세요")
    
    def on_run_scenario_start(self):
        """🎬 시뮬레이션 시작 - RUN 버튼 클릭 시"""
        self._ui_builder._timeline.play()
        
        # 초기화 상태 리셋
        self._ui_builder._articulation_initialized = False
        self._ui_builder._physics_step_count = 0
        self._ui_builder._initialization_attempts = 0
    
    def on_run_scenario_stop(self):
        """⏸️ 시뮬레이션 일시정지 - STOP 버튼 클릭 시"""
        print("⏸️ 시뮬레이션 일시정지...")
        self._ui_builder._timeline.pause()