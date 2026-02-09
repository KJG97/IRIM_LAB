"""
재사용 가능한 UI 컴포넌트들
"""

import omni.ui as ui
from ..config.ui_config import UIColors, UILayout


class UIComponentFactory:
    """UI 컴포넌트 생성 팩토리"""

    @staticmethod
    def _create_ui_button(text, callback=None, height=UILayout.BUTTON_HEIGHT, style=None, width=None):
        """내부 헬퍼: style=None 처리를 안전하게 하는 ui.Button 생성"""
        if style is not None:
            if width is not None:
                return ui.Button(text, clicked_fn=callback, height=height, width=width, style=style)
            return ui.Button(text, clicked_fn=callback, height=height, style=style)
        else:
            if width is not None:
                return ui.Button(text, clicked_fn=callback, height=height, width=width)
            return ui.Button(text, clicked_fn=callback, height=height)
    
    @staticmethod
    def create_section_header(text, height=UILayout.LABEL_HEIGHT):
        """섹션 헤더 생성"""
        return ui.Label(text, height=height)
    
    @staticmethod
    def create_separator(height=UILayout.SEPARATOR_HEIGHT):
        """구분선 생성"""
        return ui.Separator(height=height)
    
    @staticmethod
    def create_spacer(width=UILayout.SPACING_SMALL):
        """여백 생성"""
        return ui.Spacer(width=width)
    
    @staticmethod
    def create_status_label(text, width=UILayout.LABEL_WIDTH_LARGE):
        """상태 표시 레이블 생성"""
        return ui.Label(text, width=width)
    
    @staticmethod
    def create_colored_sidebar(color, width=UILayout.SIDEBAR_WIDTH, height=UILayout.BUTTON_HEIGHT):
        """컬러 사이드바 생성"""
        return ui.Rectangle(
            width=width, 
            height=height, 
            style={
                "background_color": color, 
                "border_radius": UILayout.BUTTON_BORDER_RADIUS
            }
        )
    
    @staticmethod
    def create_styled_button(text, callback=None, color_scheme='default', height=UILayout.BUTTON_HEIGHT, width=None):
        """스타일이 적용된 버튼 생성"""
        style_map = {
            'red': {
                "Button": {
                    "background_color": UIColors.RED_BUTTON_BG,
                    "border_width": UILayout.BUTTON_BORDER_WIDTH,
                    "border_color": UIColors.RED_BUTTON_BORDER,
                    "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE
                },
                "Button:hovered": {"background_color": UIColors.RED_BUTTON_HOVER}
            },
            'yellow': {
                "Button": {
                    "background_color": UIColors.YELLOW_BUTTON_BG,
                    "border_width": UILayout.BUTTON_BORDER_WIDTH,
                    "border_color": UIColors.YELLOW_BUTTON_BORDER,
                    "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE
                },
                "Button:hovered": {
                    "background_color": UIColors.YELLOW_BUTTON_HOVER
                }
            },
            'object_viz': {
                "Button": {
                    "background_color": UIColors.OBJECT_VIZ_BUTTON_BG,
                    "border_width": UILayout.BUTTON_BORDER_WIDTH,
                    "border_color": UIColors.OBJECT_VIZ_BUTTON_BORDER,
                    "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE
                },
                "Button:hovered": {"background_color": UIColors.OBJECT_VIZ_BUTTON_HOVER}
            },
            'transparency': {
                "Button": {
                    "background_color": UIColors.TRANSPARENCY_BUTTON_BG,
                    "border_width": UILayout.BUTTON_BORDER_WIDTH,
                    "border_color": UIColors.TRANSPARENCY_BUTTON_BORDER,
                    "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE,
                },
                "Button:hovered": {
                    "background_color": UIColors.TRANSPARENCY_BUTTON_HOVER,
                }
            },
            'green': {
                "Button": {
                    "background_color": UIColors.STATE_BUTTON_BG,      # 진한 초록
                    "border_width": UILayout.BUTTON_BORDER_WIDTH,
                    "border_color": UIColors.STATE_BUTTON_BORDER,     # 밝은 초록 테두리
                    "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE
                },
                "Button:hovered": {
                    "background_color": UIColors.STATE_BUTTON_HOVER    # 호버 시 초록
                }
            },
            'blue': {
                "Button": {
                    "background_color": UIColors.BLUE_BUTTON_BG,      # 진한 초록
                    "border_width": UILayout.BUTTON_BORDER_WIDTH,
                    "border_color": UIColors.BLUE_BUTTON_BORDER,     # 밝은 초록 테두리
                    "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE
                },
                "Button:hovered": {
                    "background_color": UIColors.BLUE_BUTTON_HOVER    # 호버 시 초록
                }
            },
            'default': None
        }
        
        style = style_map.get(color_scheme, None)
        return UIComponentFactory._create_ui_button(text, callback, height, style, width)
    
    @staticmethod
    def create_joint_slider(joint_index, callback, joint_name="Joint"):
        """관절 슬라이더 생성"""
        from ..config import JointConfig
        
        with ui.HStack(height=UILayout.BUTTON_HEIGHT):
            ui.Label(f"{joint_name}:", width=UILayout.LABEL_WIDTH_MEDIUM)
            
            # 슬라이더
            slider = ui.FloatSlider(
                min=JointConfig.JOINT_MIN_ANGLE, 
                max=JointConfig.JOINT_MAX_ANGLE, 
                step=JointConfig.JOINT_STEP
            )
            
            # 값 표시 레이블
            value_label = ui.Label("0.00", width=UILayout.LABEL_WIDTH_SMALL)
            
            # 콜백 연결
            slider.model.add_value_changed_fn(
                lambda model, i=joint_index, label=value_label: callback(i, model, label)
            )
            
            return slider, value_label
    
    @staticmethod
    def create_status_row(labels_and_widths):
        """상태 표시 행 생성
        
        Args:
            labels_and_widths: [(label_text, width), ...] 형식의 리스트
        """
        with ui.HStack(height=UILayout.LABEL_HEIGHT):
            status_labels = []
            for label_text, width in labels_and_widths:
                label = ui.Label(label_text, width=width)
                status_labels.append(label)
            return status_labels
    
    @staticmethod
    def create_button_row(buttons_config, height=UILayout.BUTTON_HEIGHT_LARGE):
        """버튼 행 생성"""
        with ui.HStack(height=height):
            buttons = []
            for button_text, callback, style in buttons_config:
                button = UIComponentFactory._create_ui_button(
                    button_text, callback, UILayout.BUTTON_HEIGHT, style
                )
                buttons.append(button)
            return buttons

    @staticmethod
    def create_checkbox(text, initial_value=True, callback=None, width=None):
        """체크박스 생성
        
        Args:
            text (str): 체크박스 라벨 텍스트
            initial_value (bool): 초기 체크 상태
            callback (callable): 상태 변경 시 호출될 콜백 함수
            width (int): 체크박스 너비
            
        Returns:
            ui.CheckBox: 생성된 체크박스 위젯
        """
        style = {
            "CheckBox": {
                "background_color": UIColors.BACKGROUND,
                "border_radius": 3,
                "font_size": 14
            },
            "CheckBox:checked": {
                "background_color": UIColors.STATE_BUTTON_BG,
                "border_color": UIColors.STATE_BUTTON_BORDER
            }
        }
        
        checkbox = ui.CheckBox(
            text=text,
            width=width or UILayout.LABEL_WIDTH_MEDIUM,
            style=style
        )
        
        # 초기값 설정
        checkbox.model.set_value(initial_value)
        
        # 콜백 연결
        if callback:
            checkbox.model.add_value_changed_fn(
                lambda model: callback(model.get_value_as_bool())
            )
        
        return checkbox
    
    @staticmethod
    def create_checkbox_group(checkboxes_config, columns=2):
        """체크박스 그룹 생성
        
        Args:
            checkboxes_config: [(text, initial_value, callback), ...] 리스트
            columns (int): 열 개수
            
        Returns:
            dict: {text: checkbox} 형태의 딕셔너리
        """
        checkboxes = {}
        
        # 행 개수 계산
        rows = (len(checkboxes_config) + columns - 1) // columns
        
        for row in range(rows):
            with ui.HStack(height=UILayout.BUTTON_HEIGHT):
                for col in range(columns):
                    index = row * columns + col
                    if index < len(checkboxes_config):
                        text, initial_value, callback = checkboxes_config[index]
                        checkbox = UIComponentFactory.create_checkbox(
                            text, initial_value, callback
                        )
                        checkboxes[text] = checkbox
                    else:
                        # 빈 공간 채우기
                        ui.Spacer()
        
        return checkboxes

    @staticmethod
    def create_checkbox_with_label(text, initial_value=True, callback=None, width=None):
        """체크박스와 라벨을 분리해서 생성 (텍스트 표시 보장)"""
        checkbox_style = {
            "CheckBox": {
                "background_color": UIColors.BACKGROUND,
                "border_radius": 3,
            },
            "CheckBox:checked": {
                "background_color": UIColors.STATE_BUTTON_BG,
                "border_color": UIColors.STATE_BUTTON_BORDER
            }
        }
        
        with ui.HStack(spacing=5, height=UILayout.BUTTON_HEIGHT):
            # 체크박스 (텍스트 없이)
            checkbox = ui.CheckBox(
                width=20,  # 체크박스 고정 크기
                style=checkbox_style
            )
            
            # 라벨 (텍스트 표시)
            label = ui.Label(
                text,
                width=width or UILayout.LABEL_WIDTH_MEDIUM - 25,  # 체크박스 크기만큼 빼기
                style={
                    "color": 0xFFFFFFFF,  # 흰색 텍스트
                    "font_size": 14
                }
            )
        
        # 초기값 설정
        checkbox.model.set_value(initial_value)
        
        # 콜백 연결
        if callback:
            checkbox.model.add_value_changed_fn(
                lambda model: callback(model.get_value_as_bool())
            )
        
        return checkbox

    @staticmethod
    def create_checkbox_group_with_labels(checkboxes_config, columns=2):
        """라벨이 포함된 체크박스 그룹 생성"""
        checkboxes = {}
        
        # 행 개수 계산
        rows = (len(checkboxes_config) + columns - 1) // columns
        
        for row in range(rows):
            with ui.HStack(height=UILayout.BUTTON_HEIGHT, spacing=10):
                for col in range(columns):
                    index = row * columns + col
                    if index < len(checkboxes_config):
                        text, initial_value, callback = checkboxes_config[index]
                        
                        # 🆕 새로운 방식으로 체크박스 생성
                        checkbox = UIComponentFactory.create_checkbox_with_label(
                            text, initial_value, callback, width=100
                        )
                        checkboxes[text] = checkbox
                    else:
                        # 빈 공간 채우기
                        ui.Spacer()
        
        return checkboxes