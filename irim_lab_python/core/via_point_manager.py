import os
import csv
import numpy as np



class IRIMCubicHermiteSpline:
    """Cubic Hermite Spline - tension 파라미터로 부드러움 조절 가능"""
    
    def __init__(self, dim: int, tension: float = 0.5):
        """
        Args:
            dim: 조인트 개수 (차원)
            tension: 부드러움 조절 파라미터
                     - 0.0: 직선 보간 (가장 딱딱함)
                     - 0.5: 기본 Catmull-Rom (기본값)
                     - 1.0: 더 곡선적
                     - 1.5~2.0: 매우 부드러움 (오버슈팅 주의)
        """
        self.BUFFER_SIZE = 4
        self.EPS = 1e-6  # small epsilon for denom checks
        self.tension = max(0.0, min(2.0, tension))  # 0~2 범위로 클램프
        
        # Ring buffer for via points
        self.buffer = [None] * self.BUFFER_SIZE  # SViaPoint objects
        self.valid = [False] * self.BUFFER_SIZE  # flags for filled slots
        self.head = 0      # index of current segment start (p0)
        self.filled = 0    # number of valid points in buffer
        self.dim = dim     # dimension of positions
        
        # Initialize buffer
        for i in range(self.BUFFER_SIZE):
            self.buffer[i] = {
                'duration_s': 0.0,
                'positions': [0.0] * dim  # numpy 대신 리스트 사용
            }
    
    def add_back_via_point(self, duration_s: float, positions: np.ndarray):
        """새로운 via point를 ring buffer 뒤쪽에 추가"""
        # tail index 계산: head_에서 filled_만큼 떨어진 위치
        tail_idx = (self.head + self.filled) % self.BUFFER_SIZE
        
        if self.filled < self.BUFFER_SIZE:
            # 아직 공간이 있을 때
            self.buffer[tail_idx]['duration_s'] = duration_s
            self.buffer[tail_idx]['positions'] = positions.copy()
            self.valid[tail_idx] = True
            self.filled += 1
        else:
            # 가득 찼을 때: 가장 오래된 것을 버리고 head_ 이동 → 새 tail에 덮어쓰기
            self.head = (self.head + 1) % self.BUFFER_SIZE
            tail_idx = (self.head + self.filled - 1) % self.BUFFER_SIZE
            self.buffer[tail_idx]['duration_s'] = duration_s
            self.buffer[tail_idx]['positions'] = positions.copy()
            self.valid[tail_idx] = True
    
    def add_front_via_point(self, duration_s: float, positions: np.ndarray):
        """새로운 via point를 ring buffer 앞쪽에 추가"""
        if self.filled == 0:
            self.add_back_via_point(max(duration_s, self.EPS), positions)
            return
        
        # 1) head 한 칸 뒤로 이동 → 자연스럽게 꼬리 하나는 drop
        self.head = (self.head - 1 + self.BUFFER_SIZE) % self.BUFFER_SIZE
        self.buffer[self.head]['duration_s'] = max(duration_s, self.EPS)
        self.buffer[self.head]['positions'] = positions.copy()
        self.valid[self.head] = True
        
        # 2) 빈 공간이 있으면 filled_ 증가, 꽉 찼으면 그대로
        if self.filled < self.BUFFER_SIZE:
            self.filled += 1
    
    def override_via_point_idx(self, duration_s: float, positions: np.ndarray, idx: int):
        """특정 인덱스의 via point를 덮어쓰기"""
        # 1) 유효 인덱스 체크
        if idx < 0 or idx >= self.filled:
            print(f"[Spline] ERROR: Invalid idx={idx} (valid range 0..{self.filled - 1})")
            return
        
        # 2) 실제 버퍼 인덱스 계산
        buf_idx = (self.head + idx) % self.BUFFER_SIZE
        
        # 3) duration과 positions 덮어쓰기
        self.buffer[buf_idx]['duration_s'] = max(duration_s, self.EPS)
        self.buffer[buf_idx]['positions'] = positions.copy()
        
        # 4) 해당 슬롯을 valid로 표시
        self.valid[buf_idx] = True
    
    def get_target(self, t_ms: float) -> tuple:
        """
        C++ CCentripetalCatmullRomSpline::getTarget와 동일한 구조.
        Returns:
            (success, pose_out, vel_out)
            - success: 0=정상, -1=뒤로, +1=앞으로, 404=오류
            - pose_out: 위치 배열
            - vel_out: 속도 배열
        """
        # 1) 최소 4개 포인트 필요
        if self.filled < 4:
            return 404, None, None

        # 2) 시간을 초로 변환
        time_s = t_ms * 0.001

        # 3) C++와 동일한 인덱스 계산
        i0 = self.head
        i1 = (i0 + 1) % self.BUFFER_SIZE
        i2 = (i1 + 1) % self.BUFFER_SIZE
        i3 = (i2 + 1) % self.BUFFER_SIZE

        # 4) valid 체크 추가
        if not (self.valid[i0] and self.valid[i1] and self.valid[i2] and self.valid[i3]):
            return 404, None, None

        # 5) duration
        seg_dur = self.buffer[i2]['duration_s']
        if seg_dur < self.EPS:
            print("[Spline] ERROR: segDur<EPS")
            return 1, None, None

        # 6) 구간 경계 체크
        if time_s > seg_dur:
            return 1, None, None
        if time_s < 0:
            return -1, None, None

        # 7) Hermite Spline 보간
        u_norm = time_s / seg_dur  # ✅ time_s 사용
        pose_out = np.zeros(self.dim)
        vel_out = np.zeros(self.dim)


        for d in range(self.dim):
            P0 = self.buffer[i0]['positions'][d]
            P1 = self.buffer[i1]['positions'][d]
            P2 = self.buffer[i2]['positions'][d]
            P3 = self.buffer[i3]['positions'][d]

            # 기울기 계산 (C++와 동일)
            s01 = 0.0
            s12 = (P2 - P1) / self.buffer[i2]['duration_s']
            s23 = 0.0
            if self.buffer[i1]['duration_s'] > self.EPS:
                s01 = (P1 - P0) / self.buffer[i1]['duration_s']
            if self.buffer[i3]['duration_s'] > self.EPS:
                s23 = (P3 - P2) / self.buffer[i3]['duration_s']

            # 탄젠트 계산 (tension으로 부드러움 조절)
            m1 = 0.0
            if s01 * s12 > 0:
                m1 = self.tension * (s01 + s12)
            m2 = 0.0
            if s12 * s23 > 0:
                m2 = self.tension * (s12 + s23)

            T1 = m1 * seg_dur
            T2 = m2 * seg_dur

            pose_out[d] = (
                self._h00(u_norm) * P1 +
                self._h10(u_norm) * T1 +
                self._h01(u_norm) * P2 +
                self._h11(u_norm) * T2
            )
            vel_out[d] = (
                (self._h00p(u_norm) * P1 +
                self._h10p(u_norm) * T1 +
                self._h01p(u_norm) * P2 +
                self._h11p(u_norm) * T2) / seg_dur
            )

        return 0, pose_out, vel_out
    
    def _compute_knots(self, idx0: int) -> np.ndarray:
        """knot parameters 계산"""
        # 1) 네 점 index
        i1 = (idx0 + 1) % self.BUFFER_SIZE
        i2 = (i1 + 1) % self.BUFFER_SIZE
        i3 = (i2 + 1) % self.BUFFER_SIZE
        
        # 2) duration_s 끌어오기
        d01 = self.buffer[i1]['duration_s']
        d12 = self.buffer[i2]['duration_s']
        d23 = self.buffer[i3]['duration_s']
        
        # 3) 누적해서 t 설정
        t = np.zeros(4)
        t[0] = 0.0
        t[1] = t[0] + d01
        t[2] = t[1] + d12
        t[3] = t[2] + d23
        
        return t
    
    def set_second_buffer_duration(self, sec: float):
        """두 번째 버퍼의 duration 설정"""
        # 최소한 3개 이상 포인트가 채워져 있어야 i2가 유효합니다.
        if self.filled < 3:
            print("[Spline] ERROR: Not enough points to set middle duration")
            return
        
        if sec < self.EPS:
            print(f"[Spline] 씁... {sec}로 2번째 duration 설정 잘못한걸껄..? 일단 해줌.")
        
        # head_ 기준으로 세 번째 포인트(index 2) 계산
        idx2 = (self.head + 2) % self.BUFFER_SIZE
        
        # 둘 중 큰걸로 하자~ -> 더 보수적으로.
        if self.buffer[idx2]['duration_s'] < sec:
            self.buffer[idx2]['duration_s'] = sec
    
    def print_buffer(self):
        """버퍼 상태 출력 (디버깅용)"""
        print("=== Catmull-Rom Buffer State ===")
        print(f"head_ = {self.head}, filled_ = {self.filled}")
        for i in range(self.BUFFER_SIZE):
            if not self.valid[i]:
                print(f"[{i}] invalid")
                continue
            
            print(f"[{i}] duration_s: {self.buffer[i]['duration_s']}, positions: {self.buffer[i]['positions']}")
        print("================================")
    
    # ========================================
    # Hermite Basis Functions (C++와 동일)
    # ========================================
    @staticmethod
    def _h00(u: float) -> float:
        """Cubic Hermite basis function h00"""
        return 2*u*u*u - 3*u*u + 1
    
    @staticmethod
    def _h10(u: float) -> float:
        """Cubic Hermite basis function h10"""
        return u*u*u - 2*u*u + u
    
    @staticmethod
    def _h01(u: float) -> float:
        """Cubic Hermite basis function h01"""
        return -2*u*u*u + 3*u*u
    
    @staticmethod
    def _h11(u: float) -> float:
        """Cubic Hermite basis function h11"""
        return u*u*u - u*u
    
    # Derivatives (속도 계산용)
    @staticmethod
    def _h00p(u: float) -> float:
        """Derivative of h00"""
        return 6*u*u - 6*u
    
    @staticmethod
    def _h10p(u: float) -> float:
        """Derivative of h10"""
        return 3*u*u - 4*u + 1
    
    @staticmethod
    def _h01p(u: float) -> float:
        """Derivative of h01"""
        return -6*u*u + 6*u
    
    @staticmethod
    def _h11p(u: float) -> float:
        """Derivative of h11"""
        return 3*u*u - 2*u


class ViaPointManager:
    def __init__(self):
        self.via_points = []

    def add_via_point(self, joint_positions):
        """현재 조인트 포지션을 리스트에 추가"""
        self.via_points.append(np.array(joint_positions))
        print(f"현재 저장된 via point 개수: {len(self.via_points)}")

    def clear(self):
        """Via Point 리스트 초기화"""
        self.via_points = []

    def save_to_csv(self, folder_path, group_mapping, joint_name_map, duration=None):
        """Via Point 리스트를 CSV로 저장
        - 첫 번째 via point의 duration은 3.0
        - 나머지 via point의 duration은 1.0
        """
        os.makedirs(folder_path, exist_ok=True)

        for file_name, joint_names in group_mapping.items():
            indices = [joint_name_map[joint] for joint in joint_names]
            csv_path = os.path.join(folder_path, file_name)
            n_joints = len(joint_names)
            header = ["duration"] + [f"joint_{i+1}" for i in range(n_joints)]
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                for idx, via_point in enumerate(self.via_points):
                    # 첫 번째 via point는 duration=3.0, 나머지는 duration=1.0
                    if idx == 0:
                        row_duration = 3.0
                    else:
                        row_duration = 1.0
                    # 소수점 1자리로 반올림하여 저장
                    row = [row_duration] + [round(np.degrees(via_point[idx]), 1) for idx in indices]
                    writer.writerow(row)

    def create_spline_for_joint_group(self, joint_names: list) -> IRIMCubicHermiteSpline:
        """특정 조인트 그룹에 대한 Spline 객체 생성"""
        return IRIMCubicHermiteSpline(dim=len(joint_names))

    def add_via_points_to_spline(self, spline: IRIMCubicHermiteSpline, positions_list: list, durations_list: list):
        """Via points를 Spline에 추가"""
        for i, (positions, duration) in enumerate(zip(positions_list, durations_list)):
            spline.add_back_via_point(duration, positions)

