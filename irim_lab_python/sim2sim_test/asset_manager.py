"""
ALLEX Digital Twin 에셋 관리
"""

from pathlib import Path
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage


class ALLEXAssetManager:
    """ALLEX 디지털 트윈 에셋 로딩 및 관리를 담당하는 클래스"""

    def __init__(self):
        self._articulation = None
        self._robot_prim_path = "/ALLEX"

    def load_robot_asset(self):
        try:
            current_file = Path(__file__)
            # sim2sim_test/ → irim_lab_python
            extension_root = current_file.parent.parent
            asset_path = extension_root.parent / "asset" / "ALLEX" / "ALLEX.usd"
            path_to_robot_usd = str(asset_path.resolve())
            if not asset_path.exists():
                raise FileNotFoundError(f"로봇 에셋 파일을 찾을 수 없습니다: {asset_path}")
            add_reference_to_stage(path_to_robot_usd, self._robot_prim_path)
            self._articulation = SingleArticulation(self._robot_prim_path)
            return self._articulation
        except Exception as e:
            print(f"❌ 로봇 에셋 로딩 실패: {e}")
            return None

    def initialize_articulation(self):
        if self._articulation is None:
            return False
        try:
            self._articulation.initialize()
            return True
        except Exception:
            return False
