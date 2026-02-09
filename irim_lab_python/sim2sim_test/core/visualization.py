"""
ALLEX Digital Twin 시각화
"""

import numpy as np
import math

class ALLEXVisualization:
    """ALLEX 디지털 트윈 시각화를 담당하는 클래스"""
    
    def __init__(self):
        """시각화 관리자 초기화"""
        # 🆕 Force 벡터 시각화 관련 설정
        self._force_base_scale = 0.1
        self._force_scale_factor = 0.1

    def get_visualization_info(self):
        """시각화 설정 정보 반환
        
        Returns:
            dict: 현재 시각화 설정 정보
        """
        return {}

    def calculate_vector_quaternion(self, final_x, final_y, final_z):
        """3차원 벡터를 쿼터니언으로 변환
        
        Args:
            vector_x, vector_y, vector_z (float): 3차원 벡터 성분
            
        Returns:
            tuple: quaternion (w, x, y, z)
        """
        # 🎯 벡터 정규화
        vector = np.array([final_x, final_y, final_z])
        vector_length = np.linalg.norm(vector)
        
        if vector_length < 1e-10:  # 벡터가 너무 작으면 기본 방향 반환
            return (1.0, 0.0, 0.0, 0.0)
        
        normalized_vector = vector / vector_length
        
        # 🎯 기준 벡터 (예: Z축 방향)와의 회전 계산
        reference_vector = np.array([0.0, 0.0, 1.0])  # Z축을 기준으로 사용
        
        # 내적으로 cos(각도) 계산
        dot_product = np.dot(normalized_vector, reference_vector)
        
        # 거의 같은 방향이면 회전 없음
        if dot_product > 0.9999999:
            return (1.0, 0.0, 0.0, 0.0)
        
        # 반대 방향이면 180도 회전
        if dot_product < -0.9999999:
            # X축 중심으로 180도 회전
            return (0.0, 1.0, 0.0, 0.0)
        
        # 외적으로 회전축 계산
        cross_product = np.cross(reference_vector, normalized_vector)
        cross_product = cross_product / np.linalg.norm(cross_product)
        
        # 각도 계산
        angle = math.acos(np.clip(dot_product, -1.0, 1.0))
        
        # 쿼터니언 계산 (axis-angle에서 quaternion으로)
        half_angle = angle * 0.5
        sin_half_angle = math.sin(half_angle)
        cos_half_angle = math.cos(half_angle)
        
        # Quaternion (w, x, y, z)
        quaternion = (
            cos_half_angle,                    # w
            cross_product[0] * sin_half_angle, # x
            cross_product[1] * sin_half_angle, # y
            cross_product[2] * sin_half_angle  # z
        )
        
        return quaternion

    def calculate_vector_magnitude_scale(self, prim_path,final_x, final_y, final_z):
        """벡터 크기에 따른 Z축 스케일 계산 (기존 X,Y 스케일 유지)
        
        Args:
            prim_path (str): Prim 경로  
            vector_x, vector_y, vector_z (float): 3차원 벡터 성분
            
        Returns:
            list: [x_scale, y_scale, z_scale] (x,y는 기존 값 유지)
        """
        try:
            import isaacsim.core.utils.prims as prims_utils
            from isaacsim.core.prims import XFormPrim
            
            # XFormPrim 객체 생성하여 현재 스케일 얻기
            force_prim = XFormPrim(prim_paths_expr=prim_path)
            current_scales = force_prim.get_local_scales()[0]  # (1, 3) 배열에서 첫 번째 요소
            
            # 기존 x, y 스케일 유지
            current_x_scale = current_scales[0]
            current_y_scale = current_scales[1]
            
            # 벡터 크기 계산
            magnitude = math.sqrt(final_x**2 + final_y**2 + final_z**2)
            
            # Z축 스케일만 벡터 크기에 비례하여 조정
            z_scale = self._force_base_scale + magnitude * self._force_scale_factor
            
            return [current_x_scale, current_y_scale, z_scale]
            
        except Exception as e:
            print(f"⚠️ Error getting current scales: {e}")
 
    def update_force_vector_visualization(self, prim_path, vector_x, vector_y, vector_z):
        """Force 벡터 시각화 업데이트 (최종 리맵핑 포함)"""
        try:
            import isaacsim.core.utils.prims as prims_utils
            from isaacsim.core.prims import XFormPrim
            
            # Prim 존재 확인
            if not prims_utils.is_prim_path_valid(prim_path):
                print(f"❌ Prim not found: {prim_path}")
                return False
            
            # 🔥 최종 축 리맵핑 (prim_path 기반)
            if "TCP_force" in prim_path:
                # TCP: (x,y,z) → (z,-x,-y)
                final_x = vector_z
                final_y = -vector_x
                final_z = -vector_y
            else:
                # Fingers: (x,y,z) → (x,-z,y)
                final_x = vector_x
                final_y = -vector_z
                final_z = vector_y

             
            # XFormPrim 객체 생성
            force_prim = XFormPrim(prim_paths_expr=prim_path)
            
            # 🔧 1. 안전한 방향 계산
            try:
                quaternion = self.calculate_vector_quaternion(final_x, final_y, final_z)
                orientations_array = np.array([quaternion], dtype=np.float32)
                
                # 방향 업데이트
                force_prim.set_local_poses(orientations=orientations_array)
                
            except Exception as e:
                print(f"⚠️ Orientation update failed: {e}")
                # 방향 업데이트 실패해도 계속 진행
            
            # 🔧 2. 안전한 스케일 계산
            try:
                scale = self.calculate_vector_magnitude_scale(prim_path,final_x, final_y, final_z)
                scales_array = np.array([scale], dtype=np.float32)
                
                # 스케일 업데이트
                force_prim.set_local_scales(scales=scales_array)
                
            except Exception as e:
                print(f"⚠️ Scale update failed: {e}")
                # 스케일 업데이트 실패해도 계속 진행
            
            return True
            
        except Exception as e:
            print(f"❌ Force vector visualization update failed: {e}")
            import traceback
            traceback.print_exc()
            return False










