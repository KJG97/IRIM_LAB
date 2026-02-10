"""
ALLEX Digital Twin ROS2 통합 관리자 (콜백 처리 로직 포함)
"""

import threading

from .ros2_node import create_allex_ros2_node
from ..config import ROS2Config


class _JointControllerStub:
    """scenario_ref 없을 때 사용하는 최소 스텁 (update_joint_group 등 no-op)."""
    def set_topic_mode(self, topic_mode): ...
    def set_ros2_subscriber_status(self, is_active): ...
    def get_unified_target_positions(self): return []
    def update_joint_group(self, group_name, joint_values, mode="current"): ...


# -----------------------------------------------------------------------------
# ROS2 메시지 콜백 처리 (기존 ros2_callbacks 통합)
# -----------------------------------------------------------------------------
class ROS2CallbackHandler:
    """ROS2 메시지 콜백 처리를 담당하는 클래스"""

    def __init__(self, joint_controller):
        self._scenario = None
        self._joint_controller = joint_controller

    def on_external_joint_command(self, joint_values, joint_names, group_name=None, mode='current'):
        try:
            if group_name == "policy_action":
                self._process_policy_action(joint_values, joint_names, mode)
            else:
                self._process_outbound_joint_command(joint_values, joint_names, group_name, mode)
        except Exception as e:
            print(f"❌ Joint command processing error: {e}")
            import traceback
            traceback.print_exc()

    def _process_policy_action(self, joint_values, joint_names, mode):
        try:
            if len(joint_values) != 18:
                print(f"⚠️ Policy action expects 18 joints, got {len(joint_values)}")
                return
            if self._scenario:
                success = self._scenario.apply_policy_action(joint_values)
                if not success:
                    print("⚠️ Failed to apply policy action to articulation")
            else:
                print("⚠️ Scenario reference not set, cannot apply policy action")
        except Exception as e:
            print(f"❌ Policy action processing error: {e}")
            import traceback
            traceback.print_exc()

    def _process_outbound_joint_command(self, joint_values, joint_names, group_name, mode):
        try:
            target_group = self._determine_joint_group(group_name)
            if target_group:
                self._joint_controller.update_joint_group(target_group, joint_values, mode)
            else:
                print(f"⚠️ Unknown group mapping for: {group_name}")
        except Exception as e:
            print(f"❌ Outbound joint processing error for {group_name} ({mode}): {e}")

    def _determine_joint_group(self, group_name):
        if group_name is None:
            return None
        if "Hand_R_thumb" in group_name:
            return 'hand_right_thumb'
        if "Hand_R_index" in group_name:
            return 'hand_right_index'
        if "Hand_R_middle" in group_name:
            return 'hand_right_middle'
        if "Hand_R_ring" in group_name:
            return 'hand_right_ring'
        if "Hand_R_little" in group_name:
            return 'hand_right_little'
        if "Hand_L_thumb" in group_name:
            return 'hand_left_thumb'
        if "Hand_L_index" in group_name:
            return 'hand_left_index'
        if "Hand_L_middle" in group_name:
            return 'hand_left_middle'
        if "Hand_L_ring" in group_name:
            return 'hand_left_ring'
        if "Hand_L_little" in group_name:
            return 'hand_left_little'
        if "Arm_R" in group_name:
            return 'right_arm'
        if "Arm_L" in group_name:
            return 'left_arm'
        if "waist" in group_name.lower():
            return 'waist'
        if "neck" in group_name.lower():
            return 'neck'
        return None

    def set_scenario_reference(self, scenario_ref):
        self._scenario = scenario_ref

    def set_joint_control_reference(self, effective_joints, joint_values_ref):
        """매니저에서 호출. 콜백 핸들러는 관절 수/참조를 사용하지 않으며 no-op."""
        pass


# -----------------------------------------------------------------------------
# ROS2 통합 관리자
# -----------------------------------------------------------------------------
class ROS2IntegratedManager:
    """ROS2 통신을 통합 관리하는 클래스"""

    def __init__(self, scenario_ref=None):
        self._ros2_node = None
        self._callback_handler = None
        self._executor = None
        self._ros_thread = None
        self._joint_controller = None
        self._initialized = False
        self._scenario_ref = scenario_ref
        self._topic_mode = ROS2Config.DEFAULT_TOPIC_MODE
        self._effective_joints = None
        self._joint_values_ref = None

    def initialize(self):
        try:
            if self._initialized:
                print("⚠️ ROS2 Manager already initialized!")
                return True
            self.cleanup()
            from isaacsim.core.utils.extensions import enable_extension
            enable_extension(ROS2Config.BRIDGE_EXTENSION)
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            if not rclpy.ok():
                rclpy.init()
            if self._scenario_ref is not None:
                self._joint_controller = self._scenario_ref
                print("✅ Using Scenario as joint controller")
            else:
                self._joint_controller = _JointControllerStub()
                print("⚠️ No scenario reference – using stub joint controller")
            self._callback_handler = ROS2CallbackHandler(self._joint_controller)
            if self._scenario_ref:
                self._callback_handler.set_scenario_reference(self._scenario_ref)
            self._ros2_node = create_allex_ros2_node(
                joint_callback=self._callback_handler.on_external_joint_command
            )
            if self._ros2_node:
                self._ros2_node.set_topic_mode(self._topic_mode)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._ros2_node)
            self._start_executor_thread()
            self._initialized = True
            return True
        except Exception as e:
            print(f"❌ ROS2 Manager initialization failed: {e}")
            self.cleanup()
            return False

    def cleanup(self):
        try:
            if self._executor:
                try:
                    self._executor.shutdown()
                except Exception:
                    pass
                self._executor = None
            if self._ros2_node:
                try:
                    self._ros2_node.destroy_node()
                except Exception:
                    pass
                self._ros2_node = None
            self._callback_handler = None
            self._joint_controller = None
            if self._ros_thread and self._ros_thread.is_alive():
                try:
                    self._ros_thread.join(timeout=1.0)
                except Exception:
                    pass
            self._ros_thread = None
            try:
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
            self._initialized = False
        except Exception as e:
            print(f"⚠️ ROS2 cleanup warning: {e}")
            self._ros2_node = None
            self._executor = None
            self._ros_thread = None
            self._initialized = False

    def _start_executor_thread(self):
        def safe_spin():
            try:
                self._executor.spin()
            except Exception as e:
                print(f"ROS2 Executor error: {e}")
        self._ros_thread = threading.Thread(
            target=safe_spin, daemon=ROS2Config.THREAD_DAEMON
        )
        self._ros_thread.start()

    def toggle_topic_mode(self):
        if not self._initialized or not self._ros2_node:
            return False, "Topic Mode: ERROR (No Manager)"
        try:
            current_mode = self._topic_mode
            new_mode = ROS2Config.TOPIC_MODE_DESIRED if current_mode == ROS2Config.TOPIC_MODE_CURRENT else ROS2Config.TOPIC_MODE_CURRENT
            success = self.set_topic_mode(new_mode)
            if success:
                return True, f"Topic Mode: {ROS2Config.get_topic_mode_display_name(new_mode)}"
            return False, "Topic Mode: FAILED"
        except Exception as e:
            print(f"❌ Topic mode toggle error: {e}")
            return False, f"Topic Mode: ERROR ({str(e)[:15]})"

    def set_topic_mode(self, topic_mode):
        if not self._initialized or not self._ros2_node:
            return False
        try:
            if not ROS2Config.is_valid_topic_mode(topic_mode):
                return False
            success = self._ros2_node.set_topic_mode(topic_mode)
            if success:
                self._topic_mode = topic_mode
                if self._joint_controller:
                    self._joint_controller.set_topic_mode(topic_mode)
                print(f"✅ Topic mode changed to: {ROS2Config.get_topic_mode_display_name(topic_mode)}")
                return True
            return False
        except Exception as e:
            print(f"❌ Topic mode setting error: {e}")
            return False

    def get_current_topic_mode(self):
        if self._initialized and self._ros2_node:
            node_mode = self._ros2_node.get_current_topic_mode()
            self._topic_mode = node_mode
            return node_mode
        return self._topic_mode

    def get_available_topic_modes(self):
        return ROS2Config.get_available_topic_modes()

    def set_joint_control_reference(self, effective_joints, joint_values_ref):
        self._effective_joints = effective_joints
        self._joint_values_ref = joint_values_ref
        if self._callback_handler:
            self._callback_handler.set_joint_control_reference(effective_joints, joint_values_ref)

    def toggle_publisher(self):
        if not self._initialized or not self._ros2_node:
            return False, "Publisher : ERROR (No Manager)"
        try:
            if self._ros2_node.is_publisher_enabled():
                success = self._ros2_node.disable_publisher()
                status = "Publisher: OFF" if success else "Publisher: ERROR"
            else:
                success = self._ros2_node.enable_publisher()
                status = "Publisher: ON" if success else "Publisher: FAILED"
            return success, status
        except Exception as e:
            print(f"❌ Publisher toggle error: {e}")
            return False, f"Publisher: ERROR ({str(e)[:20]})"

    def is_publisher_enabled(self):
        return bool(self._ros2_node and self._ros2_node.is_publisher_enabled())

    def publish_joint_observation(self, joint_values):
        if not self._initialized or not self._ros2_node:
            return False
        return self._ros2_node.publish_joint_observation(joint_values)

    def publish_right_hand_joint_torque(self, joint_torques):
        if not self._initialized or not self._ros2_node:
            return False
        return self._ros2_node.publish_right_hand_joint_torque(joint_torques)

    def publish_right_hand_base_pos(self, pose_values):
        if not self._initialized or not self._ros2_node:
            return False
        return self._ros2_node.publish_right_hand_base_pos(pose_values)

    def toggle_subscriber(self):
        if not self._initialized or not self._ros2_node:
            return False, "Subscriber: ERROR (No Manager)"
        try:
            success = self._ros2_node.toggle_subscriber()
            if self._ros2_node.is_subscriber_enabled():
                current_mode = self.get_current_topic_mode()
                mode_display = ROS2Config.get_topic_mode_display_name(current_mode)
                status = f"Subscriber: ON ({mode_display})"
                if self._joint_controller:
                    self._joint_controller.set_ros2_subscriber_status(True)
            else:
                status = "Subscriber: OFF"
                if self._joint_controller:
                    self._joint_controller.set_ros2_subscriber_status(False)
            return success, status
        except Exception as e:
            print(f"❌ Subscriber toggle error: {e}")
            return False, f"Subscriber: ERROR ({str(e)[:20]})"

    def get_unified_target_positions(self):
        if self._joint_controller:
            return self._joint_controller.get_unified_target_positions()
        return None

    def is_initialized(self):
        return self._initialized
