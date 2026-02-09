"""
ALLEX Digital Twin 에셋 관리
"""

from pathlib import Path
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage


class ALLEXAssetManager:
    """ALLEX 디지털 트윈 에셋 로딩 및 관리를 담당하는 클래스"""
    
    def __init__(self):
        """에셋 관리자 초기화"""
        self._articulation = None
        self._robot_prim_path = "/ALLEX"

    def load_robot_asset(self):
        """ALLEX 로봇 에셋 로딩
        
        Returns:
            SingleArticulation: 로딩된 로봇 Articulation 객체
        """
        try:
            current_file = Path(__file__)
            # sim2sim_test/core → 확장 루트(IRIM_LAB)로 이동하여 asset 폴더 찾기
            extension_root = current_file.parent.parent.parent  # irim_lab_python
            asset_path = extension_root.parent / "asset" / "ALLEX" / "ALLEX.usd"
            path_to_robot_usd = str(asset_path.resolve())

            # 에셋이 존재하는지 확인
            if not asset_path.exists():
                raise FileNotFoundError(f"로봇 에셋 파일을 찾을 수 없습니다: {asset_path}")

            # 스테이지에 로봇 에셋 추가
            add_reference_to_stage(path_to_robot_usd, self._robot_prim_path)
            
            # Articulation 객체 생성 (초기화는 나중에)
            self._articulation = SingleArticulation(self._robot_prim_path)
            
            return self._articulation
            
        except Exception as e:
            print(f"❌ 로봇 에셋 로딩 실패: {e}")
            return None

    def initialize_articulation(self):
        """Articulation 초기화 (시뮬레이션 시작 후 호출)
        
        Returns:
            bool: 초기화 성공 여부
        """
        if self._articulation is None:
            print("⚠️ Articulation 객체가 없습니다")
            return False
            
        try:
            self._articulation.initialize()
            return True
        except Exception:
            return False

    def get_articulation(self):
        """현재 로딩된 Articulation 객체 반환
        
        Returns:
            SingleArticulation: 로봇 Articulation 객체
        """
        return self._articulation

    def get_robot_prim_path(self):
        """로봇 프림 경로 반환
        
        Returns:
            str: 로봇 프림 경로
        """
        return self._robot_prim_path

    def is_robot_loaded(self):
        """로봇이 로딩되었는지 확인
        
        Returns:
            bool: 로봇 로딩 상태
        """
        return self._articulation is not None

    def get_joint_info(self):
        """관절 정보 반환
        
        Returns:
            dict: 관절 관련 정보 (이름, 개수 등)
        """
        if self._articulation is None:
            return None
            
        try:
            joint_names = self._articulation.dof_names
            joint_count = len(joint_names) if joint_names else 0
            
            return {
                'joint_names': joint_names,
                'joint_count': joint_count,
                'articulation_path': self._robot_prim_path
            }
        except Exception as e:
            print(f"⚠️ 관절 정보 조회 실패: {e}")
            return None

    def load_sensor_test_asset(self, prim_path: str = "/ALLEX_Sensor_Test"):
        """ALLEX 센서 테스트 에셋 로딩
        
        Args:
            prim_path: 스테이지에 추가할 프림 경로 (기본값: "/ALLEX_Sensor_Test")
        
        Returns:
            SingleArticulation: 로딩된 Articulation 객체 (실패 시 None)
        """
        try:
            current_file = Path(__file__)
            # sim2sim_test/core → 확장 루트(IRIM_LAB)로 이동하여 asset 폴더 찾기
            extension_root = current_file.parent.parent.parent  # irim_lab_python
            asset_path = extension_root.parent / "asset" / "ALLEX" / "ALLEX_sensor_test.usd"
            path_to_sensor_usd = str(asset_path.resolve())

            # 에셋이 존재하는지 확인
            if not asset_path.exists():
                raise FileNotFoundError(f"센서 테스트 에셋 파일을 찾을 수 없습니다: {asset_path}")

            # 스테이지에 센서 테스트 에셋 추가
            add_reference_to_stage(path_to_sensor_usd, prim_path)
            
            # Articulation 객체 생성 (초기화는 나중에)
            articulation_path = f"{prim_path}/ALLEX"
            sensor_articulation = SingleArticulation(prim_path=articulation_path)
            
            print(f"✅ 센서 테스트 에셋 로딩 성공: {prim_path}")
            return sensor_articulation
            
        except Exception as e:
            print(f"❌ 센서 테스트 에셋 로딩 실패: {e}")
            return None