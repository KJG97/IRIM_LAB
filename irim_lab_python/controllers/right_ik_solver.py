from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from isaacsim.core.prims import Articulation
from isaacsim.core.api.articulations import ArticulationSubset
from typing import Optional
import numpy as np
import os

# 확장 루트 경로를 동적으로 찾기
def _get_extension_root():
    """확장 루트 경로 반환"""
    current_file = os.path.abspath(__file__)
    extension_root = os.path.dirname(os.path.dirname(current_file))
    return extension_root


class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(self, robot_articulation: Articulation, end_effector_frame_name: Optional[str] = None) -> None:
        # 🎯 오른팔 관절 이름들만 정의 (허리 3개 조인트 제외)
        # cspace에서 WY, WP, CP를 제외한 나머지 조인트들
        self._right_arm_joint_names = [
            "RSP",    # Right Shoulder Pitch  
            "RSR",    # Right Shoulder Roll
            "RSY",    # Right Shoulder Yaw
            "REP",    # Right Elbow Pitch
            "RWY",    # Right Wrist Yaw
            "RWR",    # Right Wrist Roll
            "RWP"     # Right Wrist Pitch
        ]
        
        # 🎯 ArticulationSubset 생성 - 오른팔 조인트만 포함
        self._right_arm_subset = ArticulationSubset(robot_articulation, self._right_arm_joint_names)
        
        extension_root = _get_extension_root()
        robot_desc_path = os.path.join(extension_root, "rmpflow", "allex_right_arm_descriptor.yaml")
        urdf_path = os.path.join(extension_root, "controllers", "ALLEX", "urdf", "ALLEX.urdf")
        self._kinematics = LulaKinematicsSolver(robot_description_path=robot_desc_path,
                                                urdf_path=urdf_path)
        if end_effector_frame_name is None:
            end_effector_frame_name = "right_hand"
        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)
    
    def compute_inverse_kinematics(self, target_position, target_orientation=None):
        """오른팔 조인트만으로 IK 계산"""
        # 기존 IK 계산
        actions, success = super().compute_inverse_kinematics(target_position, target_orientation)
        
        if success and actions is not None:
            # 🎯 ArticulationSubset을 통해 전체 articulation 액션으로 변환
            full_action = self._right_arm_subset.make_articulation_action(
                joint_positions=actions.joint_positions,
                joint_velocities=actions.joint_velocities if actions.joint_velocities is not None else np.zeros_like(actions.joint_positions)
            )
            return full_action, success
        else:
            return None, False

    @property 
    def right_arm_subset(self):
        """오른팔 조인트 서브셋 반환"""
        return self._right_arm_subset