"""
Sensor Lab 전용 에셋 로딩 (sim2sim_test 무의존)
"""

from pathlib import Path
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage


def load_sensor_test_asset(prim_path: str = "/ALLEX_Sensor_Test"):
    """센서 테스트 USD 에셋을 스테이지에 로드하고 Articulation 반환."""
    try:
        current_file = Path(__file__)
        extension_root = current_file.parent.parent
        asset_path = extension_root.parent / "asset" / "ALLEX" / "ALLEX_sensor_test.usd"
        path_to_sensor_usd = str(asset_path.resolve())
        if not asset_path.exists():
            raise FileNotFoundError(f"센서 테스트 에셋 파일을 찾을 수 없습니다: {asset_path}")
        add_reference_to_stage(path_to_sensor_usd, prim_path)
        articulation_path = f"{prim_path}/ALLEX"
        sensor_articulation = SingleArticulation(prim_path=articulation_path)
        print(f"✅ 센서 테스트 에셋 로딩 성공: {prim_path}")
        return sensor_articulation
    except Exception as e:
        print(f"❌ 센서 테스트 에셋 로딩 실패: {e}")
        return None
