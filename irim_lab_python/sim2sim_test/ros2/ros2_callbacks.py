"""
ALLEX Digital Twin ROS2 콜백 처리 로직
"""

from ..config.ros2_config import ROS2Config


class ROS2CallbackHandler:
    """ROS2 메시지 콜백 처리를 담당하는 클래스"""
    
    def __init__(self, joint_controller):
        """콜백 핸들러 초기화
        
        Args:
            joint_controller: ALLEXJointController 인스턴스
        """
        self._scenario = None
        self._joint_controller = joint_controller
        
        # 관절 제어 참조 (레거시 지원용)
        self._effective_joints = None
        self._joint_values = None
        
        # 디버깅용 통계
        self._joint_message_count = 0
        self._outbound_message_count = 0
        
        # 그룹별 메시지 카운터
        self._group_message_counts = {
            'right_hand': 0,
            'left_hand': 0,
            'right_arm': 0,
            'left_arm': 0,
            'waist': 0,
            'neck': 0
        }

    
    def set_joint_control_reference(self, effective_joints, joint_values):
        """관절 제어 참조 설정
        
        Args:
            effective_joints (int): 유효 관절 수
            joint_values (list): 관절 값 리스트 참조
        """
        self._effective_joints = effective_joints
        self._joint_values = joint_values
    
    def on_external_joint_command(self, joint_values, joint_names, group_name=None, mode='current'):  # 🆕 mode 추가
        """확장된 관절 명령 콜백"""
        try:
            # 🆕 Policy action 처리
            if group_name == "policy_action":
                self._process_policy_action(joint_values, joint_names, mode)
                self._outbound_message_count += 1
            elif group_name is None:
                self._process_outbound_joint_command(joint_values, joint_names, group_name, mode)  # 🆕 mode 전달
                self._joint_message_count += 1
            else:
                self._process_outbound_joint_command(joint_values, joint_names, group_name, mode)  # 🆕 mode 전달
                self._outbound_message_count += 1
                
        except Exception as e:
            print(f"❌ Joint command processing error: {e}")
            import traceback
            traceback.print_exc()

    def _process_policy_action(self, joint_values, joint_names, mode):
        """Policy action 처리: 18개 관절을 articulation에 직접 적용
        
        Args:
            joint_values: 18개 관절 값 (ALLEX_ACTION_JOINT_FULL_NAMES 순서)
            joint_names: ALLEX_ACTION_JOINT_FULL_NAMES 리스트
            mode: 'desired' (명령값이므로, 사용하지 않음)
        """
        try:
            if len(joint_values) != 18:
                print(f"⚠️ Policy action expects 18 joints, got {len(joint_values)}")
                return
            
            # Scenario의 apply_policy_action 메서드 호출
            if self._scenario:
                success = self._scenario.apply_policy_action(joint_values)
                if success:
                    # 통계 업데이트
                    if "right_arm" in self._group_message_counts:
                        self._group_message_counts["right_arm"] += 1
                    if "right_hand" in self._group_message_counts:
                        self._group_message_counts["right_hand"] += 1
                else:
                    print("⚠️ Failed to apply policy action to articulation")
            else:
                print("⚠️ Scenario reference not set, cannot apply policy action")
                
        except Exception as e:
            print(f"❌ Policy action processing error: {e}")
            import traceback
            traceback.print_exc()
    
    def _process_outbound_joint_command(self, joint_values, joint_names, group_name, mode):  # 🆕 mode 추가
        """새로운 14개 토픽 처리"""
        try:
            target_group = self._determine_joint_group(group_name)
            
            if target_group:
                self._joint_controller.update_joint_group(target_group, joint_values, mode)  # 🆕 mode 전달
                
                if target_group in self._group_message_counts:
                    self._group_message_counts[target_group] += 1                
            else:
                print(f"⚠️ Unknown group mapping for: {group_name}")
                
        except Exception as e:
            print(f"❌ Outbound joint processing error for {group_name} ({mode}): {e}")

    def _determine_joint_group(self, group_name):
        """그룹명을 기반으로 관절 그룹 결정
        
        Args:
            group_name (str): 토픽에서 추출한 그룹명 (예: "Hand_R_thumb_wir", "Hand_R_index_wir")
            
        Returns:
            str: 관절 그룹명 ('hand_right_thumb', 'hand_left_index', 등) 또는 None
        """
        # 손가락별 세부 매핑
        if "Hand_R_thumb" in group_name:
            return 'hand_right_thumb'
        elif "Hand_R_index" in group_name:
            return 'hand_right_index'
        elif "Hand_R_middle" in group_name:
            return 'hand_right_middle'
        elif "Hand_R_ring" in group_name:
            return 'hand_right_ring'
        elif "Hand_R_little" in group_name:
            return 'hand_right_little'
        elif "Hand_L_thumb" in group_name:
            return 'hand_left_thumb'
        elif "Hand_L_index" in group_name:
            return 'hand_left_index'
        elif "Hand_L_middle" in group_name:
            return 'hand_left_middle'
        elif "Hand_L_ring" in group_name:
            return 'hand_left_ring'
        elif "Hand_L_little" in group_name:
            return 'hand_left_little'
        # 기존 팔, 허리, 목 매핑은 그대로
        elif "Arm_R" in group_name:
            return 'right_arm'
        elif "Arm_L" in group_name:
            return 'left_arm'
        elif "waist" in group_name.lower():
            return 'waist'
        elif "neck" in group_name.lower():
            return 'neck'
        else:
            return None

    # ========================================
    # 📊 통계 및 상태 메서드들
    # ========================================
    def get_message_statistics(self):
        """메시지 통계 반환"""
        return {
            'legacy_joint_messages': self._joint_message_count,
            'outbound_messages': self._outbound_message_count,
            'group_counts': self._group_message_counts.copy()
        }
    
    def get_group_statistics(self):
        """관절 그룹별 통계 반환"""
        return self._group_message_counts.copy()
    
    def reset_statistics(self):
        """통계 리셋"""
        self._joint_message_count = 0
        self._outbound_message_count = 0
        self._group_message_counts = {
            'right_hand': 0,
            'left_hand': 0,
            'right_arm': 0,
            'left_arm': 0,
            'waist': 0,
            'neck': 0
        }
    
    def set_scenario_reference(self, scenario_ref):
        """시나리오 참조 설정 (unified callback용)"""
        self._scenario = scenario_ref
    
    # ========================================
    # 🔧 헬퍼 메서드들
    # ========================================
    def _find_joint_index_by_name(self, name):
        """관절 이름으로 인덱스 찾기 (레거시 지원용)"""
        joint_name_map = {
            "R11": 24, "R12": 34, "R13": 44,
            "R21": 25, "R22": 35, "R23": 45,
            "R31": 26, "R32": 36, "R33": 46,
            "R41": 27, "R42": 37, "R43": 47,
            "R51": 28, "R52": 38, "R53": 48,
        }
        return joint_name_map.get(name, None)
    
    def get_active_joint_groups(self):
        """활성화된 관절 그룹 목록 반환"""
        active_groups = []
        for group, count in self._group_message_counts.items():
            if count > 0:
                active_groups.append(group)
        return active_groups
    
    def print_status_summary(self):
        """상태 요약 출력"""
        stats = self.get_message_statistics()
        print(f"📊 Callback Statistics:")
        print(f"   Legacy Joint: {stats['legacy_joint_messages']}")
        print(f"   Outbound: {stats['outbound_messages']}")
        print(f"   Unified: {stats['unified_messages']}")
        print(f"   Active Groups: {', '.join(self.get_active_joint_groups())}")