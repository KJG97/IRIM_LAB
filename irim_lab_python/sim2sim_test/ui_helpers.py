"""
Sim2Sim UI 헬퍼: 버튼/체크박스/스타일 등 재사용 가능한 위젯·스타일 팩토리.
ui_builder에서 사용; omni.ui 및 config(UIColors, UILayout, OverlayConfig) 의존.
"""

import omni.ui as ui
from isaacsim.gui.components.ui_utils import get_style

from .config import UIColors, UILayout, OverlayConfig


# -----------------------------------------------------------------------------
# 오버레이 윈도우 플래그
# -----------------------------------------------------------------------------

OVERLAY_WINDOW_FLAGS = (
    ui.WINDOW_FLAGS_NO_TITLE_BAR
    | ui.WINDOW_FLAGS_NO_RESIZE
    | ui.WINDOW_FLAGS_NO_SCROLLBAR
    | ui.WINDOW_FLAGS_NO_BACKGROUND
    | ui.WINDOW_FLAGS_NO_MOVE
)


# -----------------------------------------------------------------------------
# 버튼·레이아웃 요소
# -----------------------------------------------------------------------------

def _make_button(text, callback=None, height=UILayout.BUTTON_HEIGHT, style=None, width=None):
    if style is not None:
        return (
            ui.Button(text, clicked_fn=callback, height=height, width=width, style=style)
            if width
            else ui.Button(text, clicked_fn=callback, height=height, style=style)
        )
    return (
        ui.Button(text, clicked_fn=callback, height=height, width=width)
        if width
        else ui.Button(text, clicked_fn=callback, height=height)
    )


def create_separator(height=UILayout.SEPARATOR_HEIGHT):
    return ui.Separator(height=height)


def create_spacer(width=UILayout.SPACING_SMALL):
    return ui.Spacer(width=width)


def create_status_label(text, width=UILayout.LABEL_WIDTH_LARGE):
    return ui.Label(text, width=width)


def create_colored_sidebar(color, width=UILayout.SIDEBAR_WIDTH, height=UILayout.BUTTON_HEIGHT):
    return ui.Rectangle(
        width=width,
        height=height,
        style={"background_color": color, "border_radius": UILayout.BUTTON_BORDER_RADIUS},
    )


def create_styled_button(
    text, callback=None, color_scheme="default", height=UILayout.BUTTON_HEIGHT, width=None
):
    style_map = {
        "red": {
            "Button": {
                "background_color": UIColors.BLUE_BG,
                "border_width": UILayout.BUTTON_BORDER_WIDTH,
                "border_color": UIColors.BLUE_BORDER,
                "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE,
            },
            "Button:hovered": {"background_color": UIColors.BLUE_HOVER},
        },
        "yellow": {
            "Button": {
                "background_color": UIColors.ORANGE_BG,
                "border_width": UILayout.BUTTON_BORDER_WIDTH,
                "border_color": UIColors.ORANGE_BORDER,
                "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE,
            },
            "Button:hovered": {"background_color": UIColors.ORANGE_HOVER},
        },
        "green": {
            "Button": {
                "background_color": UIColors.GREEN_BG,
                "border_width": UILayout.BUTTON_BORDER_WIDTH,
                "border_color": UIColors.GREEN_BORDER,
                "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE,
            },
            "Button:hovered": {"background_color": UIColors.GREEN_HOVER},
        },
        "blue": {
            "Button": {
                "background_color": UIColors.AMBER_BG,
                "border_width": UILayout.BUTTON_BORDER_WIDTH,
                "border_color": UIColors.AMBER_BORDER,
                "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE,
            },
            "Button:hovered": {"background_color": UIColors.AMBER_HOVER},
        },
        "object_viz": {
            "Button": {
                "background_color": UIColors.MAGENTA_BG,
                "border_width": UILayout.BUTTON_BORDER_WIDTH,
                "border_color": UIColors.MAGENTA_BORDER,
                "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE,
            },
            "Button:hovered": {"background_color": UIColors.MAGENTA_HOVER},
        },
        "transparency": {
            "Button": {
                "background_color": UIColors.GRAY_BG,
                "border_width": UILayout.BUTTON_BORDER_WIDTH,
                "border_color": UIColors.GRAY_BORDER,
                "border_radius": UILayout.BUTTON_BORDER_RADIUS_LARGE,
            },
            "Button:hovered": {"background_color": UIColors.GRAY_HOVER},
        },
        "default": None,
    }
    style = style_map.get(color_scheme)
    return _make_button(text, callback, height, style, width)


def create_checkbox_with_label(text, initial_value=True, callback=None, width=None):
    checkbox_style = {
        "CheckBox": {
            "background_color": UIColors.BACKGROUND,
            "border_radius": OverlayConfig.CHECKBOX_BORDER_RADIUS,
        },
        "CheckBox:checked": {
            "background_color": UIColors.GREEN_BG,
            "border_color": UIColors.GREEN_BORDER,
        },
    }
    with ui.HStack(spacing=UILayout.SPACING_SMALL, height=UILayout.BUTTON_HEIGHT):
        checkbox = ui.CheckBox(width=OverlayConfig.CHECKBOX_WIDTH, style=checkbox_style)
        label = ui.Label(
            text,
            width=width or UILayout.LABEL_WIDTH_MEDIUM - OverlayConfig.CHECKBOX_LABEL_WIDTH_OFFSET,
            style={
                "color": UIColors.TEXT_PRIMARY,
                "font_size": OverlayConfig.CHECKBOX_LABEL_FONT_SIZE,
            },
        )
    checkbox.model.set_value(initial_value)
    if callback:
        checkbox.model.add_value_changed_fn(lambda model: callback(model.get_value_as_bool()))
    return checkbox


def create_checkbox_group(checkboxes_config, columns=2):
    checkboxes = {}
    rows = (len(checkboxes_config) + columns - 1) // columns
    for row in range(rows):
        with ui.HStack(height=UILayout.BUTTON_HEIGHT, spacing=10):
            for col in range(columns):
                index = row * columns + col
                if index < len(checkboxes_config):
                    text, initial_value, callback = checkboxes_config[index]
                    checkbox = create_checkbox_with_label(
                        text, initial_value, callback, width=OverlayConfig.CHECKBOX_GROUP_ITEM_WIDTH
                    )
                    checkboxes[text] = checkbox
                else:
                    ui.Spacer()
    return checkboxes


# -----------------------------------------------------------------------------
# Load/Reset/State 버튼 스타일 (isaacsim core_connectors 연동)
# -----------------------------------------------------------------------------

def get_load_button_style():
    return {
        **get_style(),
        "Button": {
            "background_color": UIColors.GREEN_BG,
            "border_width": UILayout.BUTTON_BORDER_WIDTH_THICK,
            "border_color": UIColors.GREEN_BORDER,
            "border_radius": UILayout.BUTTON_BORDER_RADIUS_XLARGE,
            "margin": UILayout.BUTTON_MARGIN,
            "padding": UILayout.BUTTON_PADDING,
        },
        "Button:hovered": {"background_color": UIColors.GREEN_HOVER},
        "Button.Label": {"color": UIColors.TEXT_PRIMARY},
    }


def get_reset_button_style():
    return {
        **get_style(),
        "Button": {
            "background_color": UIColors.BLUE_BG,
            "border_width": UILayout.BUTTON_BORDER_WIDTH_THICK,
            "border_color": UIColors.BLUE_BORDER,
            "border_radius": UILayout.BUTTON_BORDER_RADIUS_XLARGE,
            "margin": UILayout.BUTTON_MARGIN,
            "padding": UILayout.BUTTON_PADDING,
        },
        "Button:hovered": {"background_color": UIColors.BLUE_HOVER},
        "Button.Label": {"color": UIColors.TEXT_PRIMARY},
    }


def get_state_button_style():
    return {
        **get_style(),
        "Button": {
            "background_color": UIColors.GREEN_BG,
            "border_width": UILayout.BUTTON_BORDER_WIDTH_THICK,
            "border_color": UIColors.GREEN_BORDER,
            "border_radius": UILayout.BUTTON_BORDER_RADIUS_XLARGE,
            "margin": UILayout.BUTTON_MARGIN,
            "padding": UILayout.BUTTON_PADDING,
        },
        "Button:hovered": {"background_color": UIColors.GREEN_HOVER},
        "Button.Label": {"color": UIColors.TEXT_PRIMARY},
    }


def apply_button_styles(load_btn, reset_btn, state_btn):
    try:
        if hasattr(load_btn, "_button") and load_btn._button:
            load_btn._button.style = get_load_button_style()
        if hasattr(reset_btn, "_button") and reset_btn._button:
            reset_btn._button.style = get_reset_button_style()
        if hasattr(state_btn, "_state_button") and state_btn._state_button:
            state_btn._state_button.style = get_state_button_style()
    except Exception as e:
        print(f"⚠️ Could not apply button styles: {e}")
