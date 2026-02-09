"""
UI 관련 설정 상수들
"""

# ========================================
# 🎨 UI 색상 설정
# ========================================
class UIColors:
    """UI 색상 상수들"""



    BACKGROUND = 0xFF23211F 

    # 버튼 색상
    LOAD_BUTTON_BG = 0xFF0D4F1C        # 진한 초록
    LOAD_BUTTON_BORDER = 0xFF2ECC40    # 밝은 초록 테두리
    LOAD_BUTTON_HOVER = 0xFF1A6B2A     # 호버 초록
    
    RED_BUTTON_BG = 0xFF0D0D4F       # 진한 빨강  
    RED_BUTTON_BORDER = 0xFF2E2ECC   # 밝은 빨강 테두리
    RED_BUTTON_HOVER = 0xFF1A1A6B    # 호버 빨강
    
    STATE_BUTTON_BG = 0xFF0D4F1C       # 진한 초록
    STATE_BUTTON_BORDER = 0xFF2ECC40   # 밝은 초록 테두리
    STATE_BUTTON_HOVER = 0xFF1A6B2A    # 호버 초록
    
    YELLOW_BUTTON_BG = 0xFF007ACC     # 진한 노란색 (BGR: 0, 122, 204)
    YELLOW_BUTTON_BORDER = 0xFF00CCFF # 밝은 노란색 테두리 (BGR: 0, 204, 255)
    YELLOW_BUTTON_HOVER = 0xFF0099E6  # 호버 노란색 (BGR: 0, 153, 230)

    # Object Viz 버튼 색상 (보라색 계열)
    OBJECT_VIZ_BUTTON_BG = 0xFFCC007A      # 진한 보라색 (BGR: 204, 0, 122)
    OBJECT_VIZ_BUTTON_BORDER = 0xFFE60099  # 밝은 보라색 테두리 (BGR: 230, 0, 153)
    OBJECT_VIZ_BUTTON_HOVER = 0xFFB30066   # 호버 보라색 (BGR: 179, 0, 102)

    # Material Opacity/Transparency 버튼 색상 (연한 회색 비슷한 하얀색)
    TRANSPARENCY_BUTTON_BG = 0xFF808080      # 연한 회색 배경 (BGR: 240, 240, 240)
    TRANSPARENCY_BUTTON_BORDER = 0xFF606060  # 회색 테두리 (BGR: 221, 221, 221)
    TRANSPARENCY_BUTTON_HOVER = 0xFF909090   # 호버 시 약간 더 진한 회색 (BGR: 229, 229, 229)
    
    # 상태 표시줄 색상
    SIDEBAR_SUCCESS = 0xFF2ECC40       # 초록 사이드바
    SIDEBAR_RESET = 0xFF2E2ECC         # 빨강 사이드바
    SIDEBAR_STATE = 0xFF2ECC40         # 초록 사이드바
    
    TEXT_PRIMARY = 0xFFFFFFFF          # 흰색 텍스트
    TEXT_BLACK = 0xFF000000            # 검은색 텍스트 (BGR: 0, 0, 0)
    TEXT_DARK_GRAY = 0xFF333333        # 진한 회색 텍스트 (읽기 쉬운 대안)

    TEXT_MODE_ACTIVE = 0xFF00AA00      # 진한 초록색 (활성 모드)
    TEXT_MODE_INACTIVE = 0xFFFFFFFF    # 흰색 (비활성 모드)

    BLUE_BUTTON_BG = 0xFF7E231E        
    BLUE_BUTTON_BORDER = 0xFFD27619   
    BLUE_BUTTON_HOVER = 0xFFC06515  

# ========================================  
# 📐 UI 레이아웃 설정
# ========================================
class UILayout:
    """UI 레이아웃 상수들"""
    # 기본 크기
    BUTTON_HEIGHT = 30
    BUTTON_HEIGHT_SMALL = 25 
    BUTTON_HEIGHT_LARGE = 35
    LABEL_HEIGHT = 25
    SEPARATOR_HEIGHT = 5
    SEPARATOR_HEIGHT_LARGE = 10
    
    # 간격
    SPACING_SMALL = 5
    SPACING_MEDIUM = 8
    
    # 너비
    SIDEBAR_WIDTH = 4
    LABEL_WIDTH_SMALL = 60
    LABEL_WIDTH_MEDIUM = 80
    LABEL_WIDTH_LARGE = 150
    LABEL_WIDTH_XLARGE = 200
    
    # 버튼 스타일
    BUTTON_BORDER_WIDTH = 1
    BUTTON_BORDER_WIDTH_THICK = 2
    BUTTON_BORDER_RADIUS = 2
    BUTTON_BORDER_RADIUS_LARGE = 3
    BUTTON_BORDER_RADIUS_XLARGE = 4
    BUTTON_MARGIN = 0
    BUTTON_PADDING = 5


# ========================================
# 🎛️ UI 기본 설정
# ========================================  
class UIConfig:
    """UI 전반적인 설정"""
    # 프레임 설정
    WORLD_CONTROLS_COLLAPSED = True
    JOINT_CONTROL_COLLAPSED = True  
    VISIBILITY_CONTROL_COLLAPSED = True
    TRAJECTORY_STUDIO_COLLAPSED = True
    
    # 물리 설정
    # Note: 물리 시뮬레이션은 100Hz로 돌아가고, 관측은 시간 누적 방식으로 50Hz로 샘플링
    # (scenario.py의 _publish_joint_observation_if_needed에서 target_period = 0.02s로 구현)
    PHYSICS_DT = 1 / 50.0  # 100Hz 물리 시뮬레이션 (sim.dt = 0.01s)
    RENDERING_DT = 1 / 50.0  # 50Hz 렌더링 (물리보다 낮은 주기로 충분)
    GRAVITY = [0, 0, -9.81]
    
    # RTX 설정
    RTX_FRACTIONAL_CUTOUT_OPACITY_KEY = "/rtx/raytracing/fractionalCutoutOpacity"
    RTX_FRACTIONAL_CUTOUT_OPACITY_VALUE = True