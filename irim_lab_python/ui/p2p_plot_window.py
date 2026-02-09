import omni.ui as ui
import numpy as np

class P2PPlotWindow:
    def __init__(self, title="P2P Trajectory Plot"):
        self._window = ui.Window(
            title=title,
            width=800,
            height=600,
            visible=True,
            flags=0,
        )
        self._plot_containers = {}
        self._plots = {}
        self._joint_names = []
        self._data = None
        self._p2p_total_duration = 0.0  
        self._p2p_playback_elapsed_time = 0.0  
        self._indicator_lines = {}
        self._indicator_line_offsets = {} 

        self._build_ui()

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=10):
                self._main_plot_frame = ui.Frame(height=ui.Length(0))
                # 실제 Plot은 plot() 메서드에서 동적으로 생성

    def show(self):
        self._window.visible = True

    def clear(self):
        self._main_plot_frame.clear()
        self._plot_containers.clear()
        self._plots.clear()
        self._indicator_lines.clear() 
        self._joint_names = []
        self._data = None

    def plot(self, joint_names, data, total_duration, via_times=None, via_positions=None):
        print(f"[DEBUG] plot() called with joint_names: {joint_names}")
        self._joint_names = joint_names
        self._data = data
        self._via_times = via_times
        self._via_positions = via_positions
        self._p2p_total_duration = total_duration 
        self._indicator_lines = {} 

        n_frames = data.shape[0]
        x_data = np.linspace(0, total_duration, n_frames)
        
        self.clear()
        with self._main_plot_frame:
            with ui.VStack(spacing=5):
                for idx, joint_name in enumerate(joint_names):
                    y_data = data[:, idx]
                    min_val = float(np.min(y_data))
                    max_val = float(np.max(y_data))
                    mid_val = (min_val + max_val) / 2
                    marker_x = []
                    marker_y = []
                    self._indicator_line_offsets[joint_name] = 0.0 
                   

                    if via_times is not None and via_positions is not None:
                        n_frames = len(y_data)
                        x_data = np.linspace(0, total_duration, n_frames)
                        marker_x = []
                        marker_y = []
                        for t, pos in zip(via_times, via_positions):
                            # via point가 몇 번째 프레임에 해당하는지 index 계산
                            idx_closest = (np.abs(x_data - t)).argmin()
                            x_norm = idx_closest / (n_frames - 1)  # 0~1 사이 비율
                            y_val = pos[idx]  # joint별로
                            # y축 정규화
                            y_norm = (y_val - (min_val)) / ((max_val) - (min_val))
                            marker_x.append(x_norm)
                            marker_y.append(y_norm)

                    with ui.VStack():
                        ui.Label(f"{joint_name}", style={"font_size": 18})
                        # 2. Plot과 y축 눈금 라벨을 HStack으로 묶기
                        with ui.HStack():
                            # 왼쪽: y축 눈금 라벨
                            with ui.VStack(width=ui.Length(40), height=ui.Length(180)):  # ← 여기 width를 명시적으로!
                                ui.Label(f"{max_val:.1f}", alignment=ui.Alignment.RIGHT, width=ui.Length(40))
                                ui.Spacer(height=ui.Length(70)) 
                                ui.Label(f"{mid_val:.1f}", alignment=ui.Alignment.RIGHT, width=ui.Length(40))
                                ui.Spacer(height=ui.Length(60)) 
                                ui.Label(f"{min_val:.1f}", alignment=ui.Alignment.RIGHT, width=ui.Length(40))

                            with ui.ZStack(width=ui.Fraction(1.0), height=ui.Length(180)):
                                # Trajectory Plot
                                ui.Plot(
                                    ui.Type.LINE,
                                    min_val - 5, max_val + 5,
                                    *y_data.tolist(),
                                    width=ui.Fraction(1.0),
                                    height=ui.Length(180),
                                    style={
                                        "color": 0xFF00CCFF,
                                        "background_color": 0xFF222222,
                                    }
                                )
                                
                                if via_times is not None:
                                    n_frames = len(y_data)
                                    x_data = np.linspace(0, total_duration, n_frames)
                                    for t in via_times[1:]:  # 0번은 건너뜀!
                                        idx_closest = (np.abs(x_data - t)).argmin()
                                        x_norm = idx_closest / (n_frames - 1)  # 0~1 사이 비율
                                        with ui.Placer(draggable=False, offset_x=ui.Percent(x_norm * 100)):
                                            ui.Line(
                                                width=ui.Length(2),
                                                height=ui.Length(180),
                                                alignment=ui.Alignment.LEFT,
                                                style={"color": 0xFF0000FF}  # 원하는 색상
                                            )

                            # x축 라벨 (기존 코드 유지)
                        with ui.HStack():
                            ui.Spacer(width=ui.Length(40)) 
                            ui.Label("0s", width=ui.Length(40))
                            ui.Spacer()
                            ui.Label(f"{total_duration/2:.2f}s", width=ui.Length(60))
                            ui.Spacer()
                            ui.Label(f"{total_duration:.2f}s", width=ui.Length(0))
                        ui.Spacer(height=ui.Length(50)) 