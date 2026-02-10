"""
관절 표시용 텍스트 포맷 (current | desired).
"""

from ..config import ROS2Config


def format_joint_line(
    name: str,
    current_val: float,
    desired_val: float,
    topic_mode: str,
    name_width: int = 3,
    prefix: str = "     ",
) -> str:
    """current | desired 한 줄 포맷 (모드에 따라 강조)."""
    current_part = (
        f"[{current_val:6.1f}]" if topic_mode == ROS2Config.TOPIC_MODE_CURRENT else f" {current_val:6.1f}"
    )
    desired_part = (
        f" {desired_val:6.1f}" if topic_mode == ROS2Config.TOPIC_MODE_CURRENT else f"[{desired_val:6.1f}]"
    )
    return f"{prefix}{name:{name_width}s}: {current_part} | {desired_part}"
