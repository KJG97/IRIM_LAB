"""
Lula 기반 ALLEX Inverse Kinematics Controller
root_link  : "chest"
end_effector: "left_hand"
"""

import os
from typing import Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from isaacsim.core.api.scenes import Scene
from .left_ik_solver import KinematicsSolver
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.xforms import get_world_pose
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.api.controllers.articulation_controller import ArticulationController
from isaacsim.robot.manipulators.manipulators.single_manipulator import SingleManipulator


class ALLEXLeftIKController:
    """ALLEX 오른팔을 제어하는 IK 컨트롤러 (허리 조인트 제외)"""

    def __init__(self):
        self.initialized = None
        self.robot_prim_path = "/ALLEX"
        self.ee_prim_path = "/ALLEX/left_hand"
        self.robot_name = "ALLEX"
        self.ee_name = "left_hand"

        self.scene = Scene()

        # 🎯 SingleManipulator 대신 일반 Articulation 사용
        # (오른팔만 제어하므로 Manipulator 클래스 불필요)
        self.allex_articulation = SingleArticulation(
            prim_path=self.robot_prim_path,
            name=self.robot_name
        )
        self.scene.add(self.allex_articulation)
        self.allex_articulation.initialize()

        # 🔧 수정된 KinematicsSolver 사용 (ArticulationSubset 내장)
        self.my_ik_solver = KinematicsSolver(
            self.allex_articulation, 
            end_effector_frame_name=self.ee_name
        )

        self.allex_articulation_controller = self.allex_articulation.get_articulation_controller()

    def get_current_joint_positions(self):
        """현재 오른팔 조인트 위치 반환"""
        return self.my_ik_solver.right_arm_subset.get_joint_positions()

    def get_end_effector_pose(self):
        """End effector 현재 위치/방향 반환"""
        return get_world_pose(self.ee_prim_path)

    def compute_ik(self, target_position, target_orientation=None):
        """IK 계산 (오른팔 조인트만)"""
        return self.my_ik_solver.compute_inverse_kinematics(target_position, target_orientation)

    def apply_joint_positions(self, joint_positions):
        """오른팔 조인트 위치 적용"""
        self.my_ik_solver.right_arm_subset.set_joint_positions(joint_positions)
