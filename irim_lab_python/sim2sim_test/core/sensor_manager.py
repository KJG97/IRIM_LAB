"""
ALLEX Digital Twin 센서 관리
"""


class ALLEXSensorManager:
    """ALLEX 디지털 트윈 센서 데이터를 관리하는 클래스"""
    
    def __init__(self):
        """센서 관리자 초기화"""
        self._current_torque = 0.0
        self._force_data = [0.0, 0.0, 0.0]
        self._torque_data = [0.0, 0.0, 0.0]

    def get_joint_torque(self, articulation, joint_name="LSP"):
        """특정 관절의 토크 값 읽기
        
        Args:
            articulation: 로봇 Articulation 객체
            joint_name (str): 관절 이름
            
        Returns:
            float: 관절 토크 값
        """
        if articulation is None:
            return 0.0
            
        try:
            # 모든 관절 이름과 인덱스 확인
            joint_names = articulation.dof_names
            joint_indices = articulation._articulation_view._metadata.joint_indices
            
            # 지정된 관절 인덱스 찾기
            if joint_name in joint_indices:
                joint_index = joint_indices[joint_name]
                
                # 특정 관절의 토크 값 읽기
                joint_torque = articulation.get_measured_joint_efforts(
                    joint_indices=[joint_index]
                )[0]

                # 토크 값 저장 및 반환
                self._current_torque = joint_torque
                
                return self._current_torque
                
            else:
                # 관절 이름을 찾을 수 없을 때 모든 관절 이름 출력
                print(f"❌ {joint_name} 관절을 찾을 수 없습니다.")
                print(f"📋 사용 가능한 관절들: {list(joint_indices.keys())}")
                
        except Exception as e:
            print(f"⚠️ {joint_name} 토크 읽기 중 오류: {e}")
            
        return 0.0

    def get_all_joint_torques(self, articulation):
        """모든 관절의 토크 값 읽기
        
        Args:
            articulation: 로봇 Articulation 객체
            
        Returns:
            list: 모든 관절의 토크 값
        """
        if articulation is None:
            return []
            
        try:
            torques = articulation.get_measured_joint_efforts()
            return torques.tolist() if torques is not None else []
        except Exception as e:
            print(f"⚠️ 전체 관절 토크 읽기 중 오류: {e}")
            return []

    def get_joint_positions(self, articulation):
        """현재 관절 위치 반환
        
        Args:
            articulation: 로봇 Articulation 객체
            
        Returns:
            list: 현재 관절 위치
        """
        if articulation is None:
            return []
            
        try:
            positions = articulation.get_joint_positions()
            return positions.tolist() if positions is not None else []
        except Exception as e:
            print(f"⚠️ 관절 위치 읽기 중 오류: {e}")
            return []

    def get_joint_velocities(self, articulation):
        """현재 관절 속도 반환
        
        Args:
            articulation: 로봇 Articulation 객체
            
        Returns:
            list: 현재 관절 속도
        """
        if articulation is None:
            return []
            
        try:
            velocities = articulation.get_joint_velocities()
            return velocities.tolist() if velocities is not None else []
        except Exception as e:
            print(f"⚠️ 관절 속도 읽기 중 오류: {e}")
            return []

    def set_force_data(self, force):
        """Force 데이터 설정
        
        Args:
            force (list): [Fx, Fy, Fz] Force 데이터
        """
        if len(force) == 3:
            self._force_data = force

    def set_torque_data(self, torque):
        """Torque 데이터 설정
        
        Args:
            torque (list): [Tx, Ty, Tz] Torque 데이터
        """
        if len(torque) == 3:
            self._torque_data = torque

    def get_force_data(self):
        """Force 데이터 반환
        
        Returns:
            list: [Fx, Fy, Fz] Force 데이터
        """
        return self._force_data.copy()

    def get_torque_data(self):
        """Torque 데이터 반환
        
        Returns:
            list: [Tx, Ty, Tz] Torque 데이터
        """
        return self._torque_data.copy()

    def get_multiple_joint_torques(self, articulation, joint_names_list):
        """🆕 여러 관절의 토크 값을 한 번에 읽기
        
        Args:
            articulation: 로봇 Articulation 객체
            joint_names_list (list): 읽을 관절 이름 리스트
            
        Returns:
            dict: {joint_name: torque_value} 형태의 딕셔너리
        """
        torque_dict = {}
        
        if articulation is None:
            return torque_dict
            
        try:
            # 모든 관절 인덱스 정보 확인
            joint_indices = articulation._articulation_view._metadata.joint_indices
            
            for joint_name in joint_names_list:
                if joint_name in joint_indices:
                    joint_index = joint_indices[joint_name]
                    
                    # 특정 관절의 토크 값 읽기
                    joint_torque = articulation.get_measured_joint_efforts(
                        joint_indices=[joint_index]
                    )[0]
                    
                    torque_dict[joint_name] = float(joint_torque)
                    
                else:
                    print(f"⚠️ {joint_name} 관절을 찾을 수 없습니다.")
                    torque_dict[joint_name] = 0.0
                    
        except Exception as e:
            print(f"❌ 다중 관절 토크 읽기 중 오류: {e}")
            
        return torque_dict

    def get_torque_prims_data(self, articulation):
        """🆕 토크 프림에 대응하는 모든 관절의 토크 값 읽기
        
        Args:
            articulation: 로봇 Articulation 객체
            
        Returns:
            dict: {joint_index: torque_value} 형태의 딕셔너리
        """
        from ..utils.constants import JOINT_TORQUE_PRIMS
        
        torque_data = {}
        
        if articulation is None:
            return torque_data
            
        try:
            # JOINT_TORQUE_PRIMS에서 관절 이름 리스트 추출
            joint_names = [joint_name for _, joint_name in JOINT_TORQUE_PRIMS.values()]
            
            # 다중 토크 읽기
            torque_dict = self.get_multiple_joint_torques(articulation, joint_names)
            
            # joint_index를 키로 하는 딕셔너리로 변환
            for joint_index, (_, joint_name) in JOINT_TORQUE_PRIMS.items():
                torque_data[joint_index] = torque_dict.get(joint_name, 0.0)
            
            # 내부 저장소 업데이트
            self._current_torques = torque_data.copy()
            
        except Exception as e:
            print(f"❌ 토크 프림 데이터 읽기 중 오류: {e}")
            
        return torque_data

    def get_current_torques(self):
        """🆕 현재 모든 토크 값 딕셔너리 반환"""
        return getattr(self, '_current_torques', {}).copy()


    @property
    def current_torque(self):
        """현재 토크 값 반환"""
        return self._current_torque