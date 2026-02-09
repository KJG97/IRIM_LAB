"""
Sensor Lab 관절 토크 테스트 모듈
토크값에 따라 시각화 프림의 scale과 orientation 조절
+ Viewport에 토크값 텍스트 표시 (omni.ui.scene)
+ 자코비안 기반 Fingertip Force 계산
"""

import numpy as np
from typing import Dict, Optional, Tuple
from pxr import Gf, Usd, UsdGeom
import omni.usd
import omni.ui as ui
from isaacsim.core.prims import SingleArticulation
from isaacsim.sensors.physics import _sensor


def rotation_matrix_x(angle: float) -> np.ndarray:
    """X축 회전 행렬"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])


def rotation_matrix_y(angle: float) -> np.ndarray:
    """Y축 회전 행렬"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])


def rotation_matrix_z(angle: float) -> np.ndarray:
    """Z축 회전 행렬"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """RPY (Roll-Pitch-Yaw) to Rotation Matrix (ZYX convention)"""
    return rotation_matrix_z(yaw) @ rotation_matrix_y(pitch) @ rotation_matrix_x(roll)


def vector_to_quaternion(direction: np.ndarray) -> np.ndarray:
    """방향 벡터를 quaternion으로 변환 (z축이 해당 방향을 향하도록)
    
    Args:
        direction: 목표 방향 벡터 (3,)
        
    Returns:
        quaternion: [w, x, y, z] 형태의 quaternion
    """
    norm = np.linalg.norm(direction)
    if norm < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0])
    
    d = direction / norm
    z_axis = np.array([1.0, 0.0, 0.0])
    
    dot = np.dot(z_axis, d)
    
    if dot > 0.9999:
        return np.array([1.0, 0.0, 0.0, 0.0])
    elif dot < -0.9999:
        return np.array([0.0, 1.0, 0.0, 0.0])
    
    axis = np.cross(z_axis, d)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(np.clip(dot, -1.0, 1.0))
    
    half_angle = angle / 2.0
    w = np.cos(half_angle)
    xyz = axis * np.sin(half_angle)
    
    return np.array([w, xyz[0], xyz[1], xyz[2]])


class JointTorqueTest:
    """관절 토크 테스트 클래스"""
    
    # Articulation 경로
    ARTICULATION_PATH = "/ALLEX_Sensor_Test/ALLEX"
    
    # 관절 이름 리스트
    JOINT_NAMES = [
        # "R_Little_PIP_Joint",
        # "R_Little_MCP_Joint",
        # "R_Ring_PIP_Joint",
        # "R_Ring_MCP_Joint",
        # "R_Middle_PIP_Joint",
        # "R_Middle_MCP_Joint",
        "R_Index_Roll_Joint",
        "R_Index_PIP_Joint",
        "R_Index_MCP_Joint",
    ]
    
    # 관절 이름과 시각화 프림 경로 매핑
    JOINT_TO_VIZ_MAPPING = {
        # "R_Little_PIP_Joint": "/ALLEX_Sensor_Test/ALLEX/R_Hand_Little_Middle/torque_viz",
        # "R_Little_MCP_Joint": "/ALLEX_Sensor_Test/ALLEX/R_Hand_Little_Proximal/torque_viz",
        # "R_Ring_PIP_Joint": "/ALLEX_Sensor_Test/ALLEX/R_Hand_Ring_Middle/torque_viz",
        # "R_Ring_MCP_Joint": "/ALLEX_Sensor_Test/ALLEX/R_Hand_Ring_Proximal/torque_viz",
        # "R_Middle_PIP_Joint": "/ALLEX_Sensor_Test/ALLEX/R_Hand_Middle_Middle/torque_viz",
        # "R_Middle_MCP_Joint": "/ALLEX_Sensor_Test/ALLEX/R_Hand_Middle_Proximal/torque_viz",
        "R_Index_Roll_Joint": "/ALLEX_Sensor_Test/ALLEX/R_Hand_Index_Roll/torque_viz",
        "R_Index_PIP_Joint": "/ALLEX_Sensor_Test/ALLEX/R_Hand_Index_Middle/torque_viz",
        "R_Index_MCP_Joint": "/ALLEX_Sensor_Test/ALLEX/R_Hand_Index_Proximal/torque_viz",
    }
    
    # Fingertip Force 시각화 프림 경로
    FORCE_VIZ_PATH = "/ALLEX_Sensor_Test/ALLEX/R_Hand_Index_Distal/force_viz"
    
    # Contact Sensor 경로
    CONTACT_SENSOR_PATH = "/ALLEX_Sensor_Test/ALLEX/R_Hand_Index_Distal/collisions/ALLEX_Hand_Distal_Pad/ALLEX_Hand_Distal_Pad/Contact_Sensor"
    
    # Force 시각화 설정
    # Baseline은 start() 호출 시 동적으로 설정됨
    FORCE_VIZ_BASE_SCALE_XY = 0.3  # x, y축 고정 스케일
    FORCE_VIZ_SCALE_Z_PER_NEWTON = 5.0  # 1N당 z축 스케일
    FORCE_VIZ_MIN_SCALE_Z = 0.1  # 최소 z축 스케일
    FORCE_VIZ_MAX_SCALE_Z = 10.0  # 최대 z축 스케일
    FORCE_VIZ_SCALE_FACTOR = 0.7  # Force 시각화 전체 스케일 보정 계수
    
    # 토크 임계점
    # PIP/MCP: 양수 토크만 임계값 적용, 음수는 바로 표시
    # Roll: 음수 토크만 임계값 적용 (절대값 0.230), 양수는 바로 표시
    TORQUE_THRESHOLD_ROLL = 0.0023  # 음수 방향 임계값 (절대값)
    TORQUE_THRESHOLD_PIP = 0.003
    TORQUE_THRESHOLD_MCP = 0.015
    
    # 최대 토크값 - 통일값
    MAX_TORQUE = 0.03
    
    # 시각화 스케일 보정 계수 (관절 타입별)
    # MCP는 초기값이 커서 스케일을 줄여줌
    SCALE_FACTOR_ROLL = 0.7
    SCALE_FACTOR_PIP = 0.7
    SCALE_FACTOR_MCP = 0.7
    
    # 양수 토크용 quaternion [w, x, y, z] - PIP/MCP용
    QUAT_POSITIVE = np.array([0.5, 0.5, 0.5, -0.5])
    
    # 음수 토크용 quaternion [w, x, y, z] - PIP/MCP용
    QUAT_NEGATIVE = np.array([0.66389, -0.66388, -0.24342, -0.24342])
    
    # Roll 관절용 quaternion [w, x, y, z]
    QUAT_POSITIVE_ROLL = np.array([0.5, -0.5, 0.5, -0.5])
    QUAT_NEGATIVE_ROLL = np.array([0.5, -0.5, -0.5, 0.5])
    
    # 화면에 표시할 관절
    DISPLAY_JOINTS = ["R_Index_Roll_Joint", "R_Index_PIP_Joint", "R_Index_MCP_Joint"]
    
    # ============================================================
    # URDF 기하학 정보 (Index Finger Kinematic Chain)
    # Chain: Hand_base -> Roll -> MCP -> PIP -> Fingertip
    # ============================================================
    
    # 각 관절의 원점 위치 (부모 링크 기준, meters)
    JOINT_ORIGINS = {
        "R_Index_Roll_Joint": np.array([-0.0140329, -0.033055, -0.0433875]),
        "R_Index_MCP_Joint": np.array([-0.003817, 0.0, -0.0061245]),
        "R_Index_PIP_Joint": np.array([0.0, 0.0, -0.052]),
    }
    
    # 각 관절의 RPY (부모 링크 기준, radians)
    JOINT_RPY = {
        "R_Index_Roll_Joint": np.array([-0.1666661, -0.3339816, 0.1298149]),
        "R_Index_MCP_Joint": np.array([-0.3490659, -0.0000001, -1.5707961]),
        "R_Index_PIP_Joint": np.array([-0.0000001, 0.0, 0.0000001]),
    }
    
    # 각 관절의 회전 축 (로컬 좌표계 기준)
    JOINT_AXES = {
        "R_Index_Roll_Joint": np.array([1.0, 0.0, 0.0]),
        "R_Index_MCP_Joint": np.array([1.0, 0.0, 0.0]),
        "R_Index_PIP_Joint": np.array([1.0, 0.0, 0.0]),
    }
    
    # Fingertip 오프셋 (PIP 관절에서 fingertip까지, R_Hand_Index_Middle 링크 끝)
    FINGERTIP_OFFSET = np.array([0.0, 0.0, -0.035])  # 약 35mm (추정값, 필요시 조정)
    
    # Jacobian 계산용 관절 순서 (Roll -> MCP -> PIP)
    JACOBIAN_JOINT_ORDER = ["R_Index_Roll_Joint", "R_Index_MCP_Joint", "R_Index_PIP_Joint"]
    
    # PIP 토크 계수 - 이 값을 조절하면 Force 방향이 변함
    # 교수님 피드백: PIP에 계수를 곱해서 당기는 힘과 수직 방향으로 만들 수 있음
    # 시도할 값: 0.5, 1.0, 2.0, 2.5 (~L_total/L_distal), 5.0 등
    PIP_TORQUE_COEFFICIENT = 0.85  # 기본값 1.0, 조절 필요
    
    def __init__(self):
        """초기화"""
        self._articulation: Optional[SingleArticulation] = None
        self._joint_indices: Dict[str, int] = {}  # 관절 이름 -> DOF 인덱스
        self._viz_prims: Dict[str, "UsdGeom.Xformable"] = {}  # 관절 이름 -> UsdGeom.Xformable
        self._is_active = False
        self._initialized = False
        
        # 센서 데이터 표시 윈도우 관련
        self._sensor_window: Optional[ui.Window] = None
        self._torque_labels: Dict[str, ui.Label] = {}  # 관절 이름 -> ui.Label
        self._sphere_mass_label: Optional[ui.Label] = None  # 구 무게 표시 라벨
        self._force_labels: Dict[str, ui.Label] = {}          # Fingertip force 라벨 (Fx, Fy, Fz)
        self._contact_force_label: Optional[ui.Label] = None  # Contact Force 스칼라 라벨
        self._contact_force_vector_labels: Dict[str, ui.Label] = {}  # Contact Force 벡터 라벨 (Fx, Fy, Fz)
        self._sensor_window_initialized = False
        
        # Fingertip force 계산 결과 저장
        self._fingertip_force: np.ndarray = np.zeros(3)
        
        # Force Baseline (동적으로 설정됨 - start() 호출 시)
        self._force_baseline: Optional[np.ndarray] = None  # [Fx, Fy, Fz] baseline
        self._force_baseline_magnitude: float = 0.0  # |F| baseline
        
        # Force 시각화 프림
        self._force_viz_prim: Optional["UsdGeom.Xformable"] = None
        
        # Torque Baseline (동적으로 설정됨 - start() 호출 시)
        self._torque_baseline: Dict[str, float] = {}  # 관절 이름 -> baseline 토크값
        
        # Contact Sensor 인터페이스
        self._contact_sensor_interface = None
        
        # Contact Sensor Force 벡터 (Fx, Fy, Fz)
        self._contact_force_vector: np.ndarray = np.zeros(3)
        self._contact_force_baseline: Optional[np.ndarray] = None  # Contact Force baseline
        
        # Contact Sensor 현재 프레임 데이터
        self._contact_current_frame: Dict = {
            "force": 0.0,           # 스칼라 힘 크기 (N)
            "in_contact": False,    # 접촉 여부
            "is_valid": False,      # 데이터 유효성
        }
        
    def initialize(self) -> bool:
        """Articulation 초기화 및 관절 DOF 인덱스 찾기"""
        try:
            # Articulation 생성
            try:
                self._articulation = SingleArticulation(prim_path=self.ARTICULATION_PATH)
                print(f"✅ Articulation 생성 완료: {self.ARTICULATION_PATH}")
            except Exception as e:
                print(f"⚠️ Articulation 생성 실패: {e}")
                return False
            
            # Articulation 초기화 (물리 시뮬레이션 시작 후 호출되어야 함)
            # 여기서는 초기화만 시도하고, 실제 초기화는 물리 스텝에서 수행
            try:
                self._articulation.initialize()
                self._initialized = True
                print("✅ Articulation 초기화 완료")
            except Exception as e:
                print(f"⚠️ Articulation 초기화 실패 (나중에 재시도): {e}")
                self._initialized = False
            
            # 각 관절의 DOF 인덱스 찾기 및 시각화 프림 초기화
            if self._initialized:
                for joint_name in self.JOINT_NAMES:
                    try:
                        dof_index = self._articulation.get_dof_index(joint_name)
                        self._joint_indices[joint_name] = dof_index
                        
                        # 시각화 프림 초기화 시도
                        self._try_initialize_viz_prim(joint_name)
                        
                        print(f"✅ 관절 DOF 인덱스 찾기 완료: {joint_name} -> {dof_index}")
                    except Exception as e:
                        print(f"⚠️ 관절 DOF 인덱스 찾기 실패 ({joint_name}): {e}")
                        return False
            
            print(f"✅ 관절 토크 테스트 초기화 완료 ({len(self._joint_indices)}개 관절)")
            
            # 센서 데이터 윈도우 초기화
            self._initialize_sensor_window()
            
            return True
            
        except Exception as e:
            print(f"❌ 관절 토크 테스트 초기화 실패: {e}")
            return False
    
    def _initialize_sensor_window(self):
        """센서 데이터 표시용 독립 윈도우 초기화"""
        if self._sensor_window_initialized:
            return
        
        try:
            # 스타일 정의
            title_style = {"color": 0xFFFFFFFF, "font_size": 24}
            section_title_style = {"color": 0xFF88CCFF, "font_size": 18}
            label_style = {"font_size": 18}
            value_style = {"font_size": 18}
            
            # 독립 윈도우 생성
            self._sensor_window = ui.Window(
                "Index Finger Sensor Data",
                width=320,
                height=520,
                flags=ui.WINDOW_FLAGS_NO_SCROLLBAR
            )
            
            with self._sensor_window.frame:
                with ui.VStack(spacing=4):
                    ui.Spacer(height=8)
                    
                    # ═══════════════════════════════════════
                    # 메인 타이틀
                    # ═══════════════════════════════════════
                    ui.Label(
                        "Index Finger Sensor",
                        style=title_style,
                        alignment=ui.Alignment.CENTER,
                        height=28
                    )
                    ui.Spacer(height=4)
                    
                    # 구분선
                    with ui.HStack(height=2):
                        ui.Spacer(width=10)
                        ui.Rectangle(style={"background_color": 0xFF3D5AFE}, height=2)
                        ui.Spacer(width=10)
                    
                    ui.Spacer(height=6)
                    
                    # ═══════════════════════════════════════
                    # Joint Torque 섹션
                    # ═══════════════════════════════════════
                    ui.Label("Joint Torque", style=section_title_style, 
                            alignment=ui.Alignment.CENTER, height=24)
                    ui.Spacer(height=2)
                    
                    # Roll
                    with ui.HStack(height=24):
                        ui.Spacer(width=12)
                        ui.Label("Roll:", style={**label_style, "color": 0xFFFF9800}, width=50)
                        self._torque_labels["R_Index_Roll_Joint"] = ui.Label(
                            "+0.00000 Nm", style={**value_style, "color": 0xFFFF9800})
                    # MCP
                    with ui.HStack(height=24):
                        ui.Spacer(width=12)
                        ui.Label("MCP:", style={**label_style, "color": 0xFFFFEB3B}, width=50)
                        self._torque_labels["R_Index_MCP_Joint"] = ui.Label(
                            "+0.00000 Nm", style={**value_style, "color": 0xFFFFEB3B})
                    # PIP
                    with ui.HStack(height=24):
                        ui.Spacer(width=12)
                        ui.Label("PIP:", style={**label_style, "color": 0xFF8BC34A}, width=50)
                        self._torque_labels["R_Index_PIP_Joint"] = ui.Label(
                            "+0.00000 Nm", style={**value_style, "color": 0xFF8BC34A})
                    
                    ui.Spacer(height=6)
                    
                    # ═══════════════════════════════════════
                    # Fingertip Force 섹션 (Jacobian)
                    # ═══════════════════════════════════════
                    ui.Label("Fingertip Force (Jacobian)", style=section_title_style, 
                            alignment=ui.Alignment.CENTER, height=24)
                    ui.Spacer(height=2)
                    
                    # Fx, Fy
                    with ui.HStack(height=24):
                        ui.Spacer(width=12)
                        ui.Label("Fx:", style={**label_style, "color": 0xFFEF5350}, width=30)
                        self._force_labels["Fx"] = ui.Label(
                            "+0.000", style={**value_style, "color": 0xFFEF5350}, width=70)
                        ui.Label("Fy:", style={**label_style, "color": 0xFF66BB6A}, width=30)
                        self._force_labels["Fy"] = ui.Label(
                            "+0.000", style={**value_style, "color": 0xFF66BB6A}, width=70)
                    # Fz, |F|
                    with ui.HStack(height=24):
                        ui.Spacer(width=12)
                        ui.Label("Fz:", style={**label_style, "color": 0xFF42A5F5}, width=30)
                        self._force_labels["Fz"] = ui.Label(
                            "+0.000", style={**value_style, "color": 0xFF42A5F5}, width=70)
                        ui.Label("|F|:", style={**label_style, "color": 0xFFFFFFFF}, width=30)
                        self._force_labels["Mag"] = ui.Label(
                            "0.000 N", style={**value_style, "color": 0xFFFFFFFF}, width=70)
                    
                    ui.Spacer(height=6)
                    
                    # ═══════════════════════════════════════
                    # Contact Sensor 섹션
                    # ═══════════════════════════════════════
                    ui.Label("Contact Sensor", style=section_title_style, 
                            alignment=ui.Alignment.CENTER, height=24)
                    ui.Spacer(height=2)
                    
                    # 스칼라 Force
                    with ui.HStack(height=24):
                        ui.Spacer(width=12)
                        ui.Label("|F|:", style={**label_style, "color": 0xFFE040FB}, width=50)
                        self._contact_force_label = ui.Label(
                            "-- N", style={**value_style, "color": 0xFFE040FB})
                    
                    # 3축 Force 벡터 (Fx, Fy)
                    with ui.HStack(height=24):
                        ui.Spacer(width=12)
                        ui.Label("Fx:", style={**label_style, "color": 0xFFFF5722}, width=30)
                        self._contact_force_vector_labels["Fx"] = ui.Label(
                            "+0.000", style={**value_style, "color": 0xFFFF5722}, width=70)
                        ui.Label("Fy:", style={**label_style, "color": 0xFF4CAF50}, width=30)
                        self._contact_force_vector_labels["Fy"] = ui.Label(
                            "+0.000", style={**value_style, "color": 0xFF4CAF50}, width=70)
                    # Fz
                    with ui.HStack(height=24):
                        ui.Spacer(width=12)
                        ui.Label("Fz:", style={**label_style, "color": 0xFF2196F3}, width=30)
                        self._contact_force_vector_labels["Fz"] = ui.Label(
                            "+0.000", style={**value_style, "color": 0xFF2196F3}, width=70)
                    
                    ui.Spacer(height=6)
                    
                    # ═══════════════════════════════════════
                    # Sphere Info
                    # ═══════════════════════════════════════
                    with ui.HStack(height=24):
                        ui.Spacer(width=12)
                        ui.Label("Sphere:", style={**label_style, "color": 0xFF00E5FF}, width=80)
                        self._sphere_mass_label = ui.Label(
                            "-- g", style={**value_style, "color": 0xFF00E5FF})
                    
                    ui.Spacer(height=8)
            
            self._sensor_window_initialized = True
            print("✅ 센서 데이터 윈도우 초기화 완료")
            
        except Exception as e:
            print(f"⚠️ 센서 데이터 윈도우 초기화 실패: {e}")
    
    def _destroy_sensor_window(self):
        """센서 데이터 윈도우 제거"""
        if self._sensor_window is not None:
            try:
                self._sensor_window.destroy()
            except Exception:
                pass
            self._sensor_window = None
        
        self._torque_labels.clear()
        self._force_labels.clear()
        self._contact_force_label = None
        self._contact_force_vector_labels.clear()
        self._sensor_window_initialized = False
    
    def _update_torque_display(self, joint_name: str, torque_value: float):
        """특정 관절의 토크값 표시 업데이트 (Baseline 대비 변화량)"""
        if joint_name not in self._torque_labels:
            return
        
        label = self._torque_labels[joint_name]
        if label is not None:
            # Baseline이 설정되어 있으면 변화량 계산, 아니면 절대값 표시
            if joint_name in self._torque_baseline:
                delta_torque = torque_value - self._torque_baseline[joint_name]
            else:
                delta_torque = torque_value
            
            # 토크값 포맷팅 (부호 포함, 소수점 5자리)
            label.text = f"{delta_torque:+.5f} Nm"
    
    def set_sphere_mass_display(self, mass_grams: float):
        """구 무게 표시 업데이트
        
        Args:
            mass_grams: 구 무게 (그램 단위)
        """
        if self._sphere_mass_label is not None:
            if mass_grams >= 1.0:
                self._sphere_mass_label.text = f"{mass_grams:.1f} g"
            else:
                self._sphere_mass_label.text = f"{mass_grams:.2f} g"
    
    def start(self):
        """토크 테스트 시작"""
        if not self._articulation:
            print("⚠️ Articulation이 초기화되지 않았습니다. initialize()를 먼저 호출하세요.")
            return
        
        # Articulation이 아직 초기화되지 않았다면 시도
        if not self._initialized:
            try:
                self._articulation.initialize()
                self._initialized = True
                print("✅ Articulation 초기화 완료 (지연 초기화)")
                
                # DOF 인덱스 찾기 및 시각화 프림 초기화
                if not self._joint_indices:
                    for joint_name in self.JOINT_NAMES:
                        try:
                            dof_index = self._articulation.get_dof_index(joint_name)
                            self._joint_indices[joint_name] = dof_index
                            
                            # 시각화 프림 초기화 시도
                            self._try_initialize_viz_prim(joint_name)
                            
                            print(f"✅ 관절 DOF 인덱스 찾기 완료: {joint_name} -> {dof_index}")
                        except Exception as e:
                            print(f"⚠️ 관절 DOF 인덱스 찾기 실패 ({joint_name}): {e}")
            except Exception as e:
                print(f"⚠️ Articulation 초기화 실패: {e}")
                return
        
        self._is_active = True
        
        # 현재 Force와 Torque를 Baseline으로 설정
        self._capture_force_baseline()
        self._capture_torque_baseline()
        
        # Contact Sensor 인터페이스 초기화
        try:
            self._contact_sensor_interface = _sensor.acquire_contact_sensor_interface()
            print(f"✅ Contact Sensor 인터페이스 초기화 완료")
            
            # Contact Force baseline 캡처
            self._capture_contact_force_baseline()
        except Exception as e:
            print(f"⚠️ Contact Sensor 인터페이스 초기화 실패: {e}")
            self._contact_sensor_interface = None
        
        print("🔄 관절 토크 테스트 시작")
    
    def _capture_force_baseline(self):
        """현재 Fingertip Force를 Baseline으로 캡처"""
        try:
            if not self._joint_indices or self._articulation is None:
                self._force_baseline = np.zeros(3)
                self._force_baseline_magnitude = 0.0
                return
            
            # 현재 토크와 각도 읽기
            dof_indices = np.array(list(self._joint_indices.values()))
            efforts = self._articulation.get_measured_joint_efforts(joint_indices=dof_indices)
            positions = self._articulation.get_joint_positions(joint_indices=dof_indices)
            
            if efforts is None or len(efforts) < len(self.JOINT_NAMES):
                self._force_baseline = np.zeros(3)
                self._force_baseline_magnitude = 0.0
                return
            
            # 관절 토크/각도 딕셔너리 구성
            joint_torques = {}
            joint_angles = {}
            for i, joint_name in enumerate(self.JOINT_NAMES):
                if i < len(efforts):
                    joint_torques[joint_name] = float(efforts[i])
                if positions is not None and i < len(positions):
                    joint_angles[joint_name] = float(positions[i])
                else:
                    joint_angles[joint_name] = 0.0
            
            # Fingertip Force 계산
            if all(name in joint_torques for name in self.JACOBIAN_JOINT_ORDER):
                raw_force = self.compute_fingertip_force(joint_torques, joint_angles)
                self._force_baseline = self._swap_fx_fz(raw_force)
                self._force_baseline_magnitude = np.linalg.norm(self._force_baseline)
                print(f"✅ Force Baseline 설정: Fx={self._force_baseline[0]:.4f}, "
                      f"Fy={self._force_baseline[1]:.4f}, Fz={self._force_baseline[2]:.4f}, "
                      f"|F|={self._force_baseline_magnitude:.4f} N")
            else:
                self._force_baseline = np.zeros(3)
                self._force_baseline_magnitude = 0.0
                
        except Exception as e:
            print(f"⚠️ Force Baseline 캡처 실패: {e}")
            self._force_baseline = np.zeros(3)
            self._force_baseline_magnitude = 0.0
    
    def _capture_torque_baseline(self):
        """현재 Joint Torque를 Baseline으로 캡처"""
        try:
            if not self._joint_indices or self._articulation is None:
                self._torque_baseline = {}
                return
            
            # 현재 토크 읽기
            dof_indices = np.array(list(self._joint_indices.values()))
            efforts = self._articulation.get_measured_joint_efforts(joint_indices=dof_indices)
            
            if efforts is None or len(efforts) < len(self.JOINT_NAMES):
                self._torque_baseline = {}
                return
            
            # 각 관절의 baseline 토크값 저장
            self._torque_baseline = {}
            baseline_str = []
            for i, joint_name in enumerate(self.JOINT_NAMES):
                if i < len(efforts):
                    self._torque_baseline[joint_name] = float(efforts[i])
                    # 짧은 이름으로 로그 출력
                    short_name = joint_name.replace("R_Index_", "").replace("_Joint", "")
                    baseline_str.append(f"{short_name}={efforts[i]:.5f}")
            
            print(f"✅ Torque Baseline 설정: {', '.join(baseline_str)} Nm")
                
        except Exception as e:
            print(f"⚠️ Torque Baseline 캡처 실패: {e}")
            self._torque_baseline = {}
    
    def _capture_contact_force_baseline(self):
        """현재 Contact Sensor Force 벡터를 Baseline으로 캡처"""
        try:
            if self._contact_sensor_interface is None:
                self._contact_force_baseline = np.zeros(3)
                return
            
            # Raw contact data에서 impulse 읽기
            raw_data = self._contact_sensor_interface.get_contact_sensor_raw_data(self.CONTACT_SENSOR_PATH)
            
            if raw_data is not None and len(raw_data) > 0:
                total_impulse = np.zeros(3)
                for i in range(len(raw_data)):
                    impulse = raw_data["impulse"][i]
                    total_impulse[0] += impulse[0]
                    total_impulse[1] += impulse[1]
                    total_impulse[2] += impulse[2]
                
                # dt 가져오기
                dt = 1.0 / 60.0
                try:
                    from isaacsim.core.utils.stage import traverse_stage
                    from pxr import PhysxSchema
                    stage = omni.usd.get_context().get_stage()
                    if stage:
                        for prim in traverse_stage():
                            if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                                physx_api = PhysxSchema.PhysxSceneAPI(prim)
                                freq = physx_api.GetTimeStepsPerSecondAttr().Get()
                                if freq and freq > 0:
                                    dt = 1.0 / freq
                                break
                except Exception:
                    pass
                
                self._contact_force_baseline = total_impulse / dt
                print(f"✅ Contact Force Baseline 설정: Fx={self._contact_force_baseline[0]:.4f}, "
                      f"Fy={self._contact_force_baseline[1]:.4f}, Fz={self._contact_force_baseline[2]:.4f} N")
            else:
                self._contact_force_baseline = np.zeros(3)
                print("✅ Contact Force Baseline 설정: 0.0, 0.0, 0.0 N (접촉 없음)")
                
        except Exception as e:
            print(f"⚠️ Contact Force Baseline 캡처 실패: {e}")
            self._contact_force_baseline = np.zeros(3)
    
    def stop(self):
        """토크 테스트 중지"""
        self._is_active = False
        print("⏸️ 관절 토크 테스트 중지")
    
    # ============================================================
    # Forward Kinematics & Jacobian 계산
    # ============================================================
    def _get_world_rotation_matrix(self, prim_path: str) -> np.ndarray:
        """prim_path의 Local->World 회전(3x3) 행렬"""
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return np.eye(3)

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return np.eye(3)

        xf = UsdGeom.Xformable(prim)
        m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())  # Gf.Matrix4d
        r = m.ExtractRotationMatrix()  # Gf.Matrix3d

        return np.array([
            [r[0][0], r[0][1], r[0][2]],
            [r[1][0], r[1][1], r[1][2]],
            [r[2][0], r[2][1], r[2][2]],
        ], dtype=float)

    def _ensure_orient_then_scale_order(self, xformable: "UsdGeom.Xformable"):
        orient_op = None
        scale_op = None

        ops = list(xformable.GetOrderedXformOps())
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                orient_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                scale_op = op

        # 중요: orient 먼저 만들어야 함
        if orient_op is None:
            orient_op = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        if scale_op is None:
            scale_op = xformable.AddScaleOp()

        ops = list(xformable.GetOrderedXformOps())
        others = [op for op in ops if op != orient_op and op != scale_op]
        xformable.SetXformOpOrder(others + [orient_op, scale_op])

    def _compute_forward_kinematics(self, joint_angles: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray, Dict[str, Tuple[np.ndarray, np.ndarray]]]:
        """Forward Kinematics 계산
        
        Args:
            joint_angles: 관절 이름 -> 각도 (radians) 딕셔너리
            
        Returns:
            fingertip_pos: Fingertip 위치 (3,) - Hand base 기준
            fingertip_rot: Fingertip 회전 행렬 (3, 3)
            joint_frames: 각 관절의 (위치, 축 방향) 딕셔너리
        """
        # 현재 프레임 (Hand base 기준)
        pos = np.zeros(3)
        rot = np.eye(3)
        
        joint_frames = {}
        
        for joint_name in self.JACOBIAN_JOINT_ORDER:
            # 관절 원점으로 이동
            origin = self.JOINT_ORIGINS[joint_name]
            rpy = self.JOINT_RPY[joint_name]
            local_axis = self.JOINT_AXES[joint_name]
            
            # 부모 프레임에서 관절 프레임으로 변환
            joint_rot_offset = rpy_to_rotation_matrix(rpy[0], rpy[1], rpy[2])
            pos = pos + rot @ origin
            rot = rot @ joint_rot_offset
            
            # 관절 회전 적용
            angle = joint_angles.get(joint_name, 0.0)
            joint_rotation = rotation_matrix_x(angle)  # 모든 관절이 x축 회전
            rot = rot @ joint_rotation
            
            # 월드 프레임에서의 관절 축 방향
            world_axis = rot @ local_axis
            
            joint_frames[joint_name] = (pos.copy(), world_axis)
        
        # Fingertip 위치 계산 (PIP 관절 이후 오프셋)
        fingertip_pos = pos + rot @ self.FINGERTIP_OFFSET
        
        return fingertip_pos, rot, joint_frames
    
    def _compute_jacobian(self, joint_angles: Dict[str, float]) -> np.ndarray:
        """기하학적 자코비안 계산 (선속도 부분만, 3x3)
        
        Args:
            joint_angles: 관절 이름 -> 각도 (radians) 딕셔너리
            
        Returns:
            J: 자코비안 행렬 (3, 3) - 각 열은 관절, 각 행은 xyz 속도
        """
        fingertip_pos, _, joint_frames = self._compute_forward_kinematics(joint_angles)
        
        J = np.zeros((3, 3))
        
        for i, joint_name in enumerate(self.JACOBIAN_JOINT_ORDER):
            joint_pos, joint_axis = joint_frames[joint_name]
            
            # 회전 관절의 선속도 자코비안: z_i x (p_e - p_i)
            r = fingertip_pos - joint_pos
            J[:, i] = np.cross(joint_axis, r)
        
        return J
    
    def compute_fingertip_force(self, joint_torques: Dict[str, float], joint_angles: Dict[str, float]) -> np.ndarray:
        """관절 토크로부터 Fingertip Force 계산
        
        τ = J^T * F  =>  F = (J^T)^(-1) * τ = J^(-T) * τ
        
        Args:
            joint_torques: 관절 이름 -> 토크 (Nm) 딕셔너리
            joint_angles: 관절 이름 -> 각도 (radians) 딕셔너리
            
        Returns:
            force: Fingertip force (3,) - [Fx, Fy, Fz] in Newtons
        """
        try:
            # 자코비안 계산
            J = self._compute_jacobian(joint_angles)
            
            # 토크 벡터 구성 (Roll, MCP, PIP 순서)
            # PIP에만 계수를 곱해서 Force 방향 조절
            tau = np.array([
                joint_torques.get("R_Index_Roll_Joint", 0.0),
                joint_torques.get("R_Index_MCP_Joint", 0.0),
                joint_torques.get("R_Index_PIP_Joint", 0.0) * self.PIP_TORQUE_COEFFICIENT
            ])
            
            # J^T의 역행렬 계산
            J_T = J.T
            
            # 특이값 분해를 이용한 안정적인 역행렬 계산
            try:
                # 조건수 확인
                cond = np.linalg.cond(J_T)
                if cond > 1e6:
                    # 특이점 근처 - pseudo-inverse 사용
                    J_T_inv = np.linalg.pinv(J_T)
                else:
                    J_T_inv = np.linalg.inv(J_T)
            except np.linalg.LinAlgError:
                # 역행렬 계산 실패 시 pseudo-inverse 사용
                J_T_inv = np.linalg.pinv(J_T)
            
            # F = J^(-T) * τ
            force = J_T_inv @ tau
            
            return force
            
        except Exception as e:
            print(f"⚠️ Fingertip force 계산 실패: {e}")
            return np.zeros(3)

    def _swap_fx_fz(self, force: np.ndarray) -> np.ndarray:
        """Force 벡터에서 Fx와 Fz를 교환 (Fx<->Fz)"""
        f = np.asarray(force, dtype=float).reshape(3,)
        return np.array([f[2], f[1], f[0]], dtype=float)

    def _update_force_display(self, force: np.ndarray):
        """Fingertip Force 표시 업데이트 (Baseline 대비 변화량)"""
        if not self._force_labels:
            return
        
        try:
            # Baseline이 설정되어 있으면 변화량 계산, 아니면 절대값 표시
            if self._force_baseline is not None:
                delta_force = force - self._force_baseline
                magnitude = np.linalg.norm(force)
                delta_magnitude = magnitude - self._force_baseline_magnitude
            else:
                delta_force = force
                delta_magnitude = np.linalg.norm(force)
            
            if "Fx" in self._force_labels:
                self._force_labels["Fx"].text = f"{delta_force[0]:+.3f}"
            if "Fy" in self._force_labels:
                self._force_labels["Fy"].text = f"{delta_force[1]:+.3f}"
            if "Fz" in self._force_labels:
                self._force_labels["Fz"].text = f"{delta_force[2]:+.3f}"
            if "Mag" in self._force_labels:
                self._force_labels["Mag"].text = f"{abs(delta_magnitude):.3f} N"
        except Exception:
            pass
    
    def _try_initialize_force_viz_prim(self) -> bool:
        """Force 시각화 프림 초기화 시도"""
        if self._force_viz_prim is not None:
            return True
        
        try:
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return False
            
            prim = stage.GetPrimAtPath(self.FORCE_VIZ_PATH)
            if not prim.IsValid():
                return False
            
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                return False
            
            self._force_viz_prim = xformable
            
            self._ensure_orient_then_scale_order(self._force_viz_prim)

            # 초기 scale을 0으로 설정
            self._set_scale_xyz(xformable, 0.0, 0.0, 0.0)
            
            print(f"✅ Force 시각화 프림 초기화 완료: {self.FORCE_VIZ_PATH}")
            return True
        except Exception as e:
            print(f"⚠️ Force 시각화 프림 초기화 실패: {e}")
            return False
    
    def _set_scale_xyz(self, xformable: "UsdGeom.Xformable", sx: float, sy: float, sz: float):
        """USD prim의 xyz scale 개별 설정"""
        try:
            scale_op = None
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    scale_op = op
                    break
            
            if scale_op is None:
                scale_op = xformable.AddScaleOp()
            
            # precision에 따라 적절한 타입 사용
            try:
                scale_op.Set(Gf.Vec3d(sx, sy, sz))
            except Exception:
                scale_op.Set(Gf.Vec3f(sx, sy, sz))
        except Exception as e:
            print(f"⚠️ scale xyz 설정 실패: {e}")
    
    def _update_force_visualization(self, force: np.ndarray):
        """Force 벡터 시각화 업데이트 (월드 좌표계 기준, prim orientation은 Isaac Sim에서 설정)
        
        Args:
            force: Fingertip force 벡터 [Fx, Fy, Fz] (N)
        
        Note:
            현재 테스트: Fz만 z축으로 시각화
            다음 단계: Fx를 x축으로 시각화 추가
        """
        # Force viz 프림 초기화 시도
        if self._force_viz_prim is None:
            if not self._try_initialize_force_viz_prim():
                return
        
        try:
            # 1) delta (Hand_base 기준)
            if self._force_baseline is not None:
                delta_base = force - self._force_baseline
            else:
                delta_base = force
            
            # 2) Hand_base -> World 회전 적용
            # ⚠️ FK 기준 프레임 prim_path가 ARTICULATION_PATH와 다르면 그 prim path로 바꿔야 정확합니다.
            R = self._get_world_rotation_matrix(self.ARTICULATION_PATH)
            delta_world = R @ delta_base

            # 3) 길이(scale)는 |delta_world| × 보정계수
            mag = float(np.linalg.norm(delta_world))
            scale_z = self.FORCE_VIZ_MIN_SCALE_Z + mag * self.FORCE_VIZ_SCALE_Z_PER_NEWTON
            scale_z = min(scale_z, self.FORCE_VIZ_MAX_SCALE_Z)
            scale_z *= self.FORCE_VIZ_SCALE_FACTOR  # 보정 계수 적용

            scale_x = self.FORCE_VIZ_BASE_SCALE_XY * self.FORCE_VIZ_SCALE_FACTOR
            scale_y = self.FORCE_VIZ_BASE_SCALE_XY * self.FORCE_VIZ_SCALE_FACTOR
            self._set_scale_xyz(self._force_viz_prim, scale_x, scale_y, scale_z)

            # 4) 방향은 delta_world 방향 (Fx 반전하여 올바른 방향 표시)
            direction = np.array([delta_world[0], delta_world[1],- delta_world[2]])
            quat = vector_to_quaternion(direction)
            self._set_orientation(self._force_viz_prim, quat)

        except Exception as e:
            print(f"⚠️ Force 시각화 업데이트 실패: {e}")
    
    def _update_contact_force_display(self):
        """Contact Sensor Force를 UI에 업데이트 (스칼라 + 3축 벡터)"""
        if self._contact_sensor_interface is None:
            return
        
        try:
            # get_sensor_reading()으로 스칼라 Force 값 읽기
            # 참고: ContactSensor 원본 코드처럼 파라미터 없이 호출
            reading = self._contact_sensor_interface.get_sensor_reading(self.CONTACT_SENSOR_PATH)
            
            # 현재 프레임 데이터 업데이트 (ContactSensor API 스타일)
            # reading.value: 접촉력의 크기 (스칼라, N)
            # reading.in_contact: 접촉 여부 (bool)
            # reading.is_valid: 센서 데이터 유효성 (bool)
            self._contact_current_frame["force"] = float(reading.value) if reading.is_valid else 0.0
            self._contact_current_frame["in_contact"] = bool(reading.in_contact) if reading.is_valid else False
            self._contact_current_frame["is_valid"] = reading.is_valid
            
            # 스칼라 Force 업데이트 (UI)
            # 접촉 중일 때만 값 표시, 아니면 0 표시
            if self._contact_force_label is not None:
                if reading.is_valid and reading.in_contact:
                    self._contact_force_label.text = f"{reading.value:.6f} N"
                elif reading.is_valid:
                    # 유효하지만 접촉 없음
                    self._contact_force_label.text = f"{reading.value:.6f} N"
                else:
                    self._contact_force_label.text = "-- N"
            
            # Raw contact data에서 3축 impulse 벡터 읽기
            raw_data = self._contact_sensor_interface.get_contact_sensor_raw_data(self.CONTACT_SENSOR_PATH)
            
            if raw_data is not None and len(raw_data) > 0:
                # 모든 접촉점의 impulse 합산
                total_impulse = np.zeros(3)
                for i in range(len(raw_data)):
                    impulse = raw_data["impulse"][i]
                    total_impulse[0] += impulse[0]
                    total_impulse[1] += impulse[1]
                    total_impulse[2] += impulse[2]
                
                # Impulse -> Force 변환 (dt로 나눔)
                # Contact sensor의 dt 가져오기 (기본값 사용)
                dt = 1.0 / 60.0  # 기본 60Hz 물리 시뮬레이션
                try:
                    # PhysX scene에서 실제 dt 가져오기 시도
                    from isaacsim.core.utils.stage import traverse_stage
                    from pxr import PhysxSchema
                    stage = omni.usd.get_context().get_stage()
                    if stage:
                        for prim in traverse_stage():
                            if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                                physx_api = PhysxSchema.PhysxSceneAPI(prim)
                                freq = physx_api.GetTimeStepsPerSecondAttr().Get()
                                if freq and freq > 0:
                                    dt = 1.0 / freq
                                break
                except Exception:
                    pass
                
                # Force 벡터 계산
                force_vector = total_impulse / dt
                
                # Baseline 적용 (설정되어 있으면)
                if self._contact_force_baseline is not None:
                    delta_force = force_vector - self._contact_force_baseline
                else:
                    delta_force = force_vector
                
                self._contact_force_vector = delta_force
                
                # UI 업데이트
                if "Fx" in self._contact_force_vector_labels:
                    self._contact_force_vector_labels["Fx"].text = f"{delta_force[0]:+.3f}"
                if "Fy" in self._contact_force_vector_labels:
                    self._contact_force_vector_labels["Fy"].text = f"{delta_force[1]:+.3f}"
                if "Fz" in self._contact_force_vector_labels:
                    self._contact_force_vector_labels["Fz"].text = f"{delta_force[2]:+.3f}"
            else:
                # 접촉 없음
                self._contact_force_vector = np.zeros(3)
                if "Fx" in self._contact_force_vector_labels:
                    self._contact_force_vector_labels["Fx"].text = "+0.000"
                if "Fy" in self._contact_force_vector_labels:
                    self._contact_force_vector_labels["Fy"].text = "+0.000"
                if "Fz" in self._contact_force_vector_labels:
                    self._contact_force_vector_labels["Fz"].text = "+0.000"
                    
        except Exception as e:
            if self._contact_force_label is not None:
                self._contact_force_label.text = "-- N"
    
    def _try_initialize_viz_prim(self, joint_name: str) -> bool:
        """시각화 프림 초기화 시도 (USD prim 직접 사용)"""
        if joint_name in self._viz_prims:
            return True
        
        viz_path = self.JOINT_TO_VIZ_MAPPING.get(joint_name)
        if not viz_path:
            return False
        
        try:
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return False
            
            prim = stage.GetPrimAtPath(viz_path)
            if not prim.IsValid():
                return False
            
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                return False
            
            self._viz_prims[joint_name] = xformable
            
            # 초기 scale을 0으로 설정
            self._set_scale(xformable, 0.0)
            return True
        except Exception as e:
            print(f"⚠️ viz_prim 초기화 예외 ({joint_name}): {e}")
            return False
    
    def _set_scale(self, xformable: "UsdGeom.Xformable", scale: float):
        """USD prim의 scale 설정"""
        try:
            # 기존 scale op 찾기 또는 생성
            scale_op = None
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    scale_op = op
                    break
            
            if scale_op is None:
                scale_op = xformable.AddScaleOp()
            
            scale_op.Set(Gf.Vec3f(scale, scale, scale))
        except Exception as e:
            print(f"⚠️ scale 설정 실패: {e}")
    
    def _set_orientation(self, xformable: "UsdGeom.Xformable", quat: np.ndarray):
        """USD prim의 orientation 설정 (quaternion: [w, x, y, z])"""
        try:
            # 기존 orient op 찾기 또는 생성
            orient_op = None
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    orient_op = op
                    break
            
            if orient_op is None:
                orient_op = xformable.AddOrientOp()
            
            # Gf.Quatd(w, x, y, z) - double precision 사용
            orient_op.Set(Gf.Quatd(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))
        except Exception as e:
            print(f"⚠️ orientation 설정 실패: {e}")
    
    def update(self):
        """토크값을 읽고 시각화 프림의 scale과 orientation 업데이트"""
        if not self._is_active or not self._initialized or self._articulation is None:
            return
        
        try:
            if not self._joint_indices:
                return
            
            # Physics Simulation View가 준비되지 않았을 수 있으므로 예외 처리
            try:
                dof_indices = np.array(list(self._joint_indices.values()))
                efforts = self._articulation.get_measured_joint_efforts(joint_indices=dof_indices)
                positions = self._articulation.get_joint_positions(joint_indices=dof_indices)
            except Exception:
                # Physics Simulation View가 아직 준비되지 않음 (시뮬레이션 리셋 중 등)
                return
            
            if efforts is None or len(efforts) < len(self.JOINT_NAMES):
                return
            
            # 관절 토크/각도 딕셔너리 구성 (Fingertip Force 계산용)
            joint_torques = {}
            joint_angles = {}
            for i, joint_name in enumerate(self.JOINT_NAMES):
                if i < len(efforts):
                    joint_torques[joint_name] = float(efforts[i])
                if positions is not None and i < len(positions):
                    joint_angles[joint_name] = float(positions[i])
                else:
                    joint_angles[joint_name] = 0.0
            
            # Fingertip Force 계산 및 표시
            if all(name in joint_torques for name in self.JACOBIAN_JOINT_ORDER):
                raw_force = self.compute_fingertip_force(joint_torques, joint_angles)
                self._fingertip_force = self._swap_fx_fz(raw_force)

                self._update_force_display(self._fingertip_force)         # UI도 스왑 반영
                self._update_force_visualization(self._fingertip_force)   # prim 방향도 스왑 반영

            
            # 각 관절의 토크값에 따라 시각화 업데이트 (Baseline 대비 변화량 기준)
            for i, joint_name in enumerate(self.JOINT_NAMES):
                if i >= len(efforts):
                    continue
                
                torque_value = float(efforts[i])
                
                # Baseline 대비 변화량 계산
                if joint_name in self._torque_baseline:
                    delta_torque = torque_value - self._torque_baseline[joint_name]
                else:
                    delta_torque = torque_value
                delta_abs = abs(delta_torque)
                
                # 화면에 토크값 표시 (변화량)
                if joint_name in self.DISPLAY_JOINTS:
                    self._update_torque_display(joint_name, torque_value)
                
                # 시각화 프림 가져오기 (없으면 초기화 시도)
                viz_prim = self._viz_prims.get(joint_name)
                if viz_prim is None:
                    # 지연 초기화 시도
                    if not self._try_initialize_viz_prim(joint_name):
                        continue
                    viz_prim = self._viz_prims.get(joint_name)
                    if viz_prim is None:
                        continue
                
                # 관절 타입별 스케일 보정 계수 결정
                is_roll = "Roll" in joint_name
                is_mcp = "MCP" in joint_name
                
                # 스케일 팩터 결정
                if is_roll:
                    scale_factor = self.SCALE_FACTOR_ROLL
                elif is_mcp:
                    scale_factor = self.SCALE_FACTOR_MCP
                else:
                    scale_factor = self.SCALE_FACTOR_PIP
                
                max_torque = self.MAX_TORQUE
                
                # Roll 관절용 quaternion 선택
                if is_roll:
                    quat_positive = self.QUAT_POSITIVE_ROLL
                    quat_negative = self.QUAT_NEGATIVE_ROLL
                else:
                    quat_positive = self.QUAT_POSITIVE
                    quat_negative = self.QUAT_NEGATIVE
                
                # 변화량 기반 시각화: delta > 0이면 양수 방향, delta < 0이면 음수 방향
                # 변화량의 절대값이 임계값 이상일 때만 시각화
                MIN_DELTA_THRESHOLD = 0.001  # 최소 변화량 임계값 (노이즈 필터링)
                
                if delta_abs < MIN_DELTA_THRESHOLD:
                    # 변화량이 너무 작으면 숨김
                    self._set_scale(viz_prim, 0.0)
                else:
                    # 변화량에 비례하여 스케일 계산
                    normalized_torque = min((delta_abs / max_torque) * 0.7 + 0.2, 0.7)
                    scale = normalized_torque * scale_factor
                    
                    self._set_scale(viz_prim, scale)
                    
                    # 변화량 부호에 따라 방향 설정
                    if delta_torque > 0:
                        self._set_orientation(viz_prim, quat_positive)
                    else:
                        self._set_orientation(viz_prim, quat_negative)
            
            # Contact Sensor Force UI 업데이트
            self._update_contact_force_display()
                    
        except Exception as e:
            print(f"토크 테스트 업데이트 실패: {e}")
    
    def cleanup(self):
        """리소스 정리"""
        self.stop()
        
        # 센서 데이터 윈도우 정리
        self._destroy_sensor_window()
        
        # 모든 시각화 프림의 scale을 0으로 리셋
        for joint_name, viz_prim in self._viz_prims.items():
            try:
                self._set_scale(viz_prim, 0.0)
            except Exception as e:
                print(f"⚠️ Cleanup scale 리셋 실패 ({joint_name}): {e}")
        
        # Force 시각화 프림 정리
        if self._force_viz_prim is not None:
            try:
                self._set_scale_xyz(self._force_viz_prim, 0.0, 0.0, 0.0)
            except Exception:
                pass
            self._force_viz_prim = None
        
        # Baseline 초기화
        self._force_baseline = None
        self._force_baseline_magnitude = 0.0
        self._torque_baseline = {}
        self._contact_force_baseline = None
        
        self._articulation = None
        self._joint_indices.clear()
        self._viz_prims.clear()
        self._initialized = False
        print("🧹 관절 토크 테스트 리소스 정리 완료")
    
    @property
    def is_active(self) -> bool:
        """테스트 활성화 상태 반환"""
        return self._is_active
    
    def get_contact_current_frame(self) -> Dict:
        """Contact Sensor 현재 프레임 데이터 반환
        
        Returns:
            dict: {
                "force": float,      # 스칼라 힘 크기 (N)
                "in_contact": bool,  # 접촉 여부
                "is_valid": bool,    # 데이터 유효성
            }
        """
        return self._contact_current_frame.copy()
    
    @property
    def contact_force(self) -> float:
        """Contact Sensor 스칼라 힘 크기 반환 (N)"""
        return self._contact_current_frame["force"]
    
    @property
    def contact_force_vector(self) -> np.ndarray:
        """Contact Sensor 3축 힘 벡터 반환 [Fx, Fy, Fz] (N)"""
        return self._contact_force_vector.copy()
