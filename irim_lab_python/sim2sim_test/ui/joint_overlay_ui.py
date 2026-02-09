# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.

"""
관절·손 관절 텍스트 오버레이 UI 전용 모듈.
시나리오의 포맷/데이터 로직은 scenario에 두고, 윈도우 생성·라벨 갱신만 담당.
"""

import omni.ui as ui


# 오버레이 윈도우 기본 플래그 (타이틀/리사이즈/스크롤/배경/이동 비활성화)
OVERLAY_WINDOW_FLAGS = (
    ui.WINDOW_FLAGS_NO_TITLE_BAR
    | ui.WINDOW_FLAGS_NO_RESIZE
    | ui.WINDOW_FLAGS_NO_SCROLLBAR
    | ui.WINDOW_FLAGS_NO_BACKGROUND
    | ui.WINDOW_FLAGS_NO_MOVE
)

# 관절 오버레이 위치·크기
JOINT_OVERLAY_WIDTH = 175
JOINT_OVERLAY_HEIGHT = 525
JOINT_OVERLAY_POSITION_X = 15
JOINT_OVERLAY_POSITION_Y = 75
JOINT_OVERLAY_BORDER_COLOR = 0xFF00AAFF

# 손 관절 오버레이 위치·크기
HAND_OVERLAY_WIDTH = 175
HAND_OVERLAY_HEIGHT = 810
HAND_OVERLAY_POSITION_X = 190
HAND_OVERLAY_POSITION_Y = 75
HAND_OVERLAY_BORDER_COLOR = 0xFFFF6600


class JointOverlayUI:
    """관절/손 관절 텍스트 오버레이 윈도우 생성 및 라벨 갱신만 담당. 데이터·포맷은 scenario에 위임."""

    def __init__(self):
        self._text_overlay_window = None
        self._joint_data_label = None
        self._hand_overlay_window = None
        self._hand_data_label = None

    def create_joint_overlay_window(self):
        """관절 데이터용 텍스트 오버레이 윈도우 생성 (초기에는 숨김)."""
        try:
            self._text_overlay_window = ui.Window(
                "Text Overlay",
                width=JOINT_OVERLAY_WIDTH,
                height=JOINT_OVERLAY_HEIGHT,
                flags=OVERLAY_WINDOW_FLAGS,
                position_x=JOINT_OVERLAY_POSITION_X,
                position_y=JOINT_OVERLAY_POSITION_Y,
            )
            self._text_overlay_window.visible = False

            with self._text_overlay_window.frame:
                with ui.ZStack():
                    ui.Rectangle(
                        style={
                            "border_radius": 8,
                            "border_width": 1,
                            "border_color": JOINT_OVERLAY_BORDER_COLOR,
                        }
                    )
                    with ui.VStack(spacing=3):
                        ui.Spacer(height=5)
                        self._joint_data_label = ui.Label(
                            "Loading Joint Data...",
                            alignment=ui.Alignment.LEFT_TOP,
                            style={
                                "color": 0xFFFFFFFF,
                                "font_size": 16,
                                "margin": 8,
                                "word_wrap": False,
                            },
                        )
                        ui.Spacer(height=5)

            print("✅ Joint Data 텍스트 오버레이 생성 완료")
        except Exception as e:
            print(f"❌ 텍스트 오버레이 생성 실패: {e}")
            import traceback
            traceback.print_exc()

    def create_hand_overlay_window(self):
        """손 관절 전용 텍스트 오버레이 윈도우 생성 (초기에는 숨김)."""
        try:
            self._hand_overlay_window = ui.Window(
                "Hand Joint Overlay",
                width=HAND_OVERLAY_WIDTH,
                height=HAND_OVERLAY_HEIGHT,
                flags=OVERLAY_WINDOW_FLAGS,
                position_x=HAND_OVERLAY_POSITION_X,
                position_y=HAND_OVERLAY_POSITION_Y,
            )
            self._hand_overlay_window.visible = False

            with self._hand_overlay_window.frame:
                with ui.ZStack():
                    ui.Rectangle(
                        style={
                            "border_radius": 8,
                            "border_width": 1,
                            "border_color": HAND_OVERLAY_BORDER_COLOR,
                        }
                    )
                    with ui.VStack(spacing=3):
                        ui.Spacer(height=5)
                        self._hand_data_label = ui.Label(
                            "Loading Hand Joint Data...",
                            alignment=ui.Alignment.LEFT_TOP,
                            style={
                                "color": 0xFFFFFFFF,
                                "font_size": 16,
                                "margin": 8,
                                "word_wrap": False,
                            },
                        )
                        ui.Spacer(height=5)

            print("✅ 손 관절 오버레이 생성 완료")
        except Exception as e:
            print(f"❌ 손 관절 오버레이 생성 실패: {e}")
            import traceback
            traceback.print_exc()

    def update_joint_display_text(self, scenario):
        """시나리오의 format_all_joint_text() 결과로 관절 라벨 갱신."""
        if self._joint_data_label is None:
            return
        try:
            self._joint_data_label.text = scenario.format_all_joint_text()
        except Exception as e:
            self._joint_data_label.text = f"❌ Display Error:\n{str(e)}"
            print(f"❌ Joint display update failed: {e}")

    def update_hand_display_text(self, scenario):
        """시나리오의 format_hand_joint_text() 결과로 손 관절 라벨 갱신."""
        if self._hand_data_label is None:
            return
        try:
            self._hand_data_label.text = scenario.format_hand_joint_text()
        except Exception as e:
            self._hand_data_label.text = f"❌ Hand Data Error:\n{str(e)}"
            print(f"❌ Hand display update failed: {e}")

    def set_hand_window_visibility(self, show: bool):
        """손 관절 윈도우 표시/숨김. 표시 시 scenario로 내용 갱신은 호출측에서 수행."""
        try:
            if self._hand_overlay_window is not None:
                self._hand_overlay_window.visible = show
                if show:
                    print("✅ 손 관절 윈도우 표시")
                else:
                    print("✅ 손 관절 윈도우 숨김")
            else:
                print("⚠️ 손 관절 윈도우가 초기화되지 않았습니다")
        except Exception as e:
            print(f"❌ 손 윈도우 가시성 제어 실패: {e}")

    def set_joint_window_visibility(self, should_show: bool):
        """관절(본체) 윈도우 표시/숨김. 표시 시 내용 갱신은 호출측에서 수행."""
        try:
            if self._text_overlay_window is not None:
                self._text_overlay_window.visible = should_show
                if should_show:
                    print("✅ 관절 윈도우 표시")
                else:
                    print("✅ 관절 윈도우 숨김")
            else:
                print("⚠️ 관절 윈도우가 초기화되지 않았습니다")
        except Exception as e:
            print(f"❌ 관절 윈도우 가시성 제어 실패: {e}")
