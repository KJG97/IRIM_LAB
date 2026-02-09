"""
ALLEX Digital Twin 시뮬레이션 루프
"""


class ALLEXSimulationLoop:
    """ALLEX 디지털 트윈 시뮬레이션 루프를 관리하는 클래스"""
    
    def __init__(self):
        """시뮬레이션 루프 관리자 초기화"""
        self._script_generator = None
        self._is_running = False
        self._callbacks = []  # 콜백 리스트 추가


    def set_script_generator(self, generator):
        """스크립트 제너레이터 설정
        
        Args:
            generator: 시뮬레이션 스크립트 제너레이터
        """
        self._script_generator = generator
        self._is_running = True

    def add_callback(self, callback_func):
        """🆕 콜백 함수를 추가합니다.
        
        Args:
            callback_func: 호출할 콜백 함수 (step_size: float 인자 받음)
        """
        if callback_func not in self._callbacks:
            self._callbacks.append(callback_func)

    def remove_callback(self, callback_func):
        """콜백 함수를 제거합니다."""
        if callback_func in self._callbacks:
            self._callbacks.remove(callback_func)

    def update(self, step: float):
        """시뮬레이션 스텝 업데이트
        
        Args:
            step (float): 시뮬레이션 스텝
            
        Returns:
            bool: 시뮬레이션 계속 실행 여부 (True: 종료, False: 계속)
        """
        # �� 등록된 콜백들 실행
        for callback in self._callbacks:
            try:
                callback(step)
            except Exception as e:
                print(f"❌ Callback error in {callback.__name__}: {e}")
        
        if self._script_generator is None:
            return True
            
        try:
            result = next(self._script_generator)
            return False  # 계속 실행
        except StopIteration:
            self._is_running = False
            return True  # 시뮬레이션 종료

    def stop(self):
        """시뮬레이션 중지"""
        self._is_running = False
        self._script_generator = None

    def is_running(self):
        """시뮬레이션 실행 상태 확인
        
        Returns:
            bool: 실행 중 여부
        """
        return self._is_running

    def reset(self):
        """시뮬레이션 루프 리셋"""
        self._script_generator = None
        self._is_running = False
        self._callbacks.clear()  # 🆕 콜백 리스트도 초기화