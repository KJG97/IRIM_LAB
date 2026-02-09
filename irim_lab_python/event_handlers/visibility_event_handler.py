"""
시각화 제어 관련 이벤트 핸들러
"""

from ..config import VisibilityConfig


class VisibilityEventHandler:
    """시각화 제어 이벤트를 처리하는 클래스"""
    
    def __init__(self, ui_builder_ref):
        """이벤트 핸들러 초기화
        
        Args:
            ui_builder_ref: UIBuilder 인스턴스 참조
        """
        self._ui_builder = ui_builder_ref
    
    def toggle_hand_force_visibility(self):
        """Hand Force Visualization prims의 visibility 토글"""
        try:
            hand_force_prims = VisibilityConfig.HAND_FORCE_PRIMS
            
            import isaacsim.core.utils.prims as prims_utils
            from isaacsim.core.prims import XFormPrim
            
            # 유효한 Prim들만 찾기
            valid_prims = []
            invalid_prims = []
            
            for prim_path in hand_force_prims:
                if prims_utils.is_prim_path_valid(prim_path):
                    valid_prims.append(prim_path)
                else:
                    invalid_prims.append(prim_path)
            
            if not valid_prims:
                print("❌ Hand Force Visualization prim들을 찾을 수 없습니다")
                if hasattr(self._ui_builder, '_hand_force_status_label'):
                    self._ui_builder._hand_force_status_label.text = "Status: NO PRIMS FOUND"
                return False
            
            # 첫 번째 유효한 Prim의 현재 상태를 기준으로 토글
            first_prim = XFormPrim(prim_paths_expr=valid_prims[0])
            current_visibility = first_prim.get_visibilities()[0]
            new_visibility = not current_visibility
            
            # 모든 유효한 Prim들에 대해 동일한 visibility 적용
            success_count = 0
            failed_count = 0
            
            for prim_path in valid_prims:
                try:
                    prim = XFormPrim(prim_paths_expr=prim_path)
                    prim.set_visibilities(visibilities=[new_visibility])
                    success_count += 1
                except Exception as e:
                    print(f"⚠️ {prim_path} visibility 변경 실패: {e}")
                    failed_count += 1
            
            # 상태 업데이트
            self._ui_builder._hand_force_visible = new_visibility
            status_text = "VISIBLE" if new_visibility else "HIDDEN"
            
            if hasattr(self._ui_builder, '_hand_force_status_label'):
                self._ui_builder._hand_force_status_label.text = f"Hand Force Viz: {status_text} ({success_count}/{len(valid_prims)})"
            
            if failed_count > 0:
                print(f"   ❌ 실패: {failed_count} prims")
            if invalid_prims:
                print(f"   ⚠️ 찾을 수 없는 prims: {len(invalid_prims)}")
            
            return success_count > 0
            
        except Exception as e:
            print(f"❌ Hand Force Visualization 변경 실패: {e}")
            if hasattr(self._ui_builder, '_hand_force_status_label'):
                self._ui_builder._hand_force_status_label.text = f"Status: ERROR ({str(e)[:15]})"
            return False
    
    def toggle_joint_torque_visibility(self):
        """Joint Torque Visualization prims의 visibility 토글"""
        try:
            joint_torque_prims = VisibilityConfig.JOINT_TORQUE_PRIMS
            
            import isaacsim.core.utils.prims as prims_utils
            from isaacsim.core.prims import XFormPrim
            
            # 유효한 Prim들만 찾기
            valid_prims = []
            invalid_prims = []
            
            for prim_path in joint_torque_prims:
                if prims_utils.is_prim_path_valid(prim_path):
                    valid_prims.append(prim_path)
                else:
                    invalid_prims.append(prim_path)
            
            if not valid_prims:
                print("❌ Joint Torque Visualization prim들을 찾을 수 없습니다")
                if hasattr(self._ui_builder, '_joint_torque_status_label'):
                    self._ui_builder._joint_torque_status_label.text = "Joint Torque: OFF"
                return False
            
            # 첫 번째 유효한 Prim의 현재 상태를 기준으로 토글
            first_prim = XFormPrim(prim_paths_expr=valid_prims[0])
            current_visibility = first_prim.get_visibilities()[0]
            new_visibility = not current_visibility
            
            # 모든 유효한 Prim들에 대해 동일한 visibility 적용
            success_count = 0
            failed_count = 0
            
            for prim_path in valid_prims:
                try:
                    prim = XFormPrim(prim_paths_expr=prim_path)
                    prim.set_visibilities(visibilities=[new_visibility])
                    success_count += 1
                except Exception as e:
                    print(f"⚠️ {prim_path} visibility 변경 실패: {e}")
                    failed_count += 1
            
            # 상태 업데이트
            self._ui_builder._joint_torque_visible = new_visibility
            status_text = "VISIBLE" if new_visibility else "HIDDEN"
            
            if hasattr(self._ui_builder, '_joint_torque_status_label'):
                self._ui_builder._joint_torque_status_label.text = f"Joint Torque: OFF"
                        
            if failed_count > 0:
                print(f"   ❌ 실패: {failed_count} prims")
            if invalid_prims:
                print(f"   ⚠️ 찾을 수 없는 prims: {len(invalid_prims)}")
            
            return success_count > 0
            
        except Exception as e:
            print(f"❌ Joint Torque Visualization 변경 실패: {e}")

            return False

    def toggle_table_can_visibility(self):
        """Table과 Can prims의 visibility와 collision 토글"""
        try:
            table_can_prims = VisibilityConfig.TABLE_CAN_PRIMS
            collision_prims = VisibilityConfig.TABLE_CAN_COLLISION_PRIMS
            
            import isaacsim.core.utils.prims as prims_utils
            from isaacsim.core.prims import XFormPrim
            import omni.usd
            from pxr import Usd, UsdGeom, UsdPhysics
            
            # USD Stage 가져오기
            stage = omni.usd.get_context().get_stage()
            if not stage:
                print("❌ USD Stage를 찾을 수 없습니다")
                return False
            
            # 유효한 Prim들만 찾기
            valid_prims = []
            invalid_prims = []
            
            for prim_path in table_can_prims:
                if prims_utils.is_prim_path_valid(prim_path):
                    valid_prims.append(prim_path)
                else:
                    invalid_prims.append(prim_path)
            
            if not valid_prims:
                print("❌ Table과 Can prim들을 찾을 수 없습니다")
                if hasattr(self._ui_builder, '_table_can_status_label'):
                    self._ui_builder._table_can_status_label.text = "Status: NO PRIMS FOUND"
                return False
            
            # 첫 번째 유효한 Prim의 현재 상태를 기준으로 토글
            first_prim = XFormPrim(prim_paths_expr=valid_prims[0])
            current_visibility = first_prim.get_visibilities()[0]
            new_visibility = not current_visibility
            
            # 1. Visibility 변경
            visibility_success_count = 0
            visibility_failed_count = 0
            
            for prim_path in valid_prims:
                try:
                    prim = XFormPrim(prim_paths_expr=prim_path)
                    prim.set_visibilities(visibilities=[new_visibility])
                    visibility_success_count += 1
                except Exception as e:
                    print(f"⚠️ {prim_path} visibility 변경 실패: {e}")
                    visibility_failed_count += 1
            
            # 2. Collision 변경
            collision_success_count = 0
            collision_failed_count = 0
            
            for collision_path in collision_prims:
                try:
                    # Collision Prim 가져오기
                    collision_prim = stage.GetPrimAtPath(collision_path)
                    if collision_prim:
                        # PhysicsCollisionAPI 적용
                        collision_api = UsdPhysics.CollisionAPI.Apply(collision_prim)
                        
                        # collisionEnabled 속성 토글
                        collision_enabled_attr = collision_prim.GetAttribute("physics:collisionEnabled")
                        if collision_enabled_attr:
                            current_collision = collision_enabled_attr.Get()
                            collision_enabled_attr.Set(not current_collision)
                            collision_success_count += 1
                            print(f"✅ {collision_path} collision 토글: {current_collision} → {not current_collision}")
                        else:
                            # 속성이 없으면 새로 생성
                            collision_enabled_attr = collision_prim.CreateAttribute(
                                "physics:collisionEnabled", 
                                Usd.GetDefaultTypeForType(Usd.GetTypeForTypeName("bool"))
                            )
                            collision_enabled_attr.Set(not new_visibility)  # visibility와 반대로 설정
                            collision_success_count += 1
                            print(f"✅ {collision_path} collision 속성 생성 및 설정")
                    else:
                        print(f"⚠️ {collision_path} collision prim을 찾을 수 없습니다")
                        collision_failed_count += 1
                        
                except Exception as e:
                    print(f"⚠️ {collision_path} collision 변경 실패: {e}")
                    collision_failed_count += 1
            
            # 상태 업데이트
            self._ui_builder._table_can_visible = new_visibility
            status_text = "VISIBLE" if new_visibility else "HIDDEN"
            
            if hasattr(self._ui_builder, '_table_can_status_label'):
                self._ui_builder._table_can_status_label.text = f"Table/Can: {status_text} (V:{visibility_success_count}, C:{collision_success_count})"
            
            # 결과 출력
            print(f"🎯 Table/Can 토글 결과:")
            print(f"   👁️ Visibility: {visibility_success_count}/{len(valid_prims)} 성공")
            print(f"   💥 Collision: {collision_success_count}/{len(collision_prims)} 성공")
            
            if visibility_failed_count > 0:
                print(f"   ❌ Visibility 실패: {visibility_failed_count} prims")
            if collision_failed_count > 0:
                print(f"   ❌ Collision 실패: {collision_failed_count} prims")
            if invalid_prims:
                print(f"   ⚠️ 찾을 수 없는 prims: {len(invalid_prims)}")
            
            return visibility_success_count > 0 or collision_success_count > 0
            
        except Exception as e:
            print(f"❌ Table/Can visibility/collision 변경 실패: {e}")
            import traceback
            traceback.print_exc()
            if hasattr(self._ui_builder, '_table_can_status_label'):
                self._ui_builder._table_can_status_label.text = f"Status: ERROR ({str(e)[:15]})"
            return False

    def _toggle_material_opacity(self):
        """White Material의 inputs:enable_opacity 토글"""
        try:
            material_paths = ["/ALLEX/Looks/Ivory", "/ALLEX/Looks/Black", "/ALLEX/Looks/DarkGray"]
            
            import omni.usd
            from pxr import Usd, Sdf
            
            # USD Stage 가져오기
            stage = omni.usd.get_context().get_stage()
            if not stage:
                print("❌ USD Stage를 찾을 수 없습니다")
                if hasattr(self._ui_builder, '_white_material_opacity_status_label'):
                    self._ui_builder._white_material_opacity_status_label.text = "White Material Opacity: NO STAGE"
                return False
            
            for material_path in material_paths:
                # Material Prim 가져오기
                material_prim = stage.GetPrimAtPath(material_path)
                shader_prim = None
                for child in material_prim.GetChildren():
                    if child.GetTypeName() == "Shader":
                        shader_prim = child
                        break
                
                opacity_attr = shader_prim.GetAttribute("inputs:enable_opacity")
                new_value = not opacity_attr.Get()

                try:
                    opacity_attr.Set(new_value)
                   
                except Exception as e:
                    print(f"⚠️ {material_path} opacity 변경 실패: {e}")
            
            return True
            
        except Exception as e:
            print(f"❌ Material Opacity 변경 실패: {e}")
            return False

    



    def reset_hand_force_visibility(self):
        """Hand Force Visualization을 보이도록 리셋"""
        try:
            hand_force_prims = VisibilityConfig.HAND_FORCE_PRIMS
            
            import isaacsim.core.utils.prims as prims_utils
            from isaacsim.core.prims import XFormPrim
            
            success_count = 0
            
            for prim_path in hand_force_prims:
                if prims_utils.is_prim_path_valid(prim_path):
                    try:
                        prim = XFormPrim(prim_paths_expr=prim_path)
                        prim.set_visibilities(visibilities=[False])
                        success_count += 1
                    except Exception as e:
                        print(f"⚠️ {prim_path} reset 실패: {e}")
            
            self._ui_builder._hand_force_visible = True
            if hasattr(self._ui_builder, '_hand_force_status_label'):
                self._ui_builder._hand_force_status_label.text = f"Hand Force Viz: VISIBLE ({success_count}/12)"
                            
        except Exception as e:
            print(f"⚠️ Hand Force Visualization reset 실패: {e}")

    def reset_joint_torque_visibility(self):
        """Joint Torque Visualization을 보이도록 리셋"""
        try:
            # set을 list로 변환
            joint_torque_prims = list(VisibilityConfig.JOINT_TORQUE_PRIMS)
            
            import isaacsim.core.utils.prims as prims_utils
            from isaacsim.core.prims import XFormPrim
            
            success_count = 0
            
            for prim_path in joint_torque_prims:
                if prims_utils.is_prim_path_valid(prim_path):
                    try:
                        prim = XFormPrim(prim_paths_expr=prim_path)
                        prim.set_visibilities(visibilities=[True])
                        success_count += 1
                    except Exception as e:
                        print(f"⚠️ {prim_path} reset 실패: {e}")
            
            self._ui_builder._joint_torque_visible = True
            if hasattr(self._ui_builder, '_joint_torque_status_label'):
                self._ui_builder._joint_torque_status_label.text = f"Joint Torque: OFF"
            
            print(f"✅ Joint Torque Visualization reset 완료: ({success_count}/{len(joint_torque_prims)} prims)")
                            
        except Exception as e:
            print(f"⚠️ Joint Torque Visualization reset 실패: {e}")

    def reset_table_can_visibility(self):
        """Table과 Can Visualization과 Collision을 보이도록 리셋"""
        try:
            table_can_prims = VisibilityConfig.TABLE_CAN_PRIMS
            collision_prims = VisibilityConfig.TABLE_CAN_COLLISION_PRIMS
            
            import isaacsim.core.utils.prims as prims_utils
            from isaacsim.core.prims import XFormPrim
            import omni.usd
            from pxr import Usd, UsdGeom, UsdPhysics
            
            # USD Stage 가져오기
            stage = omni.usd.get_context().get_stage()
            if not stage:
                print("❌ USD Stage를 찾을 수 없습니다")
                return False
            
            # 1. Visibility 리셋
            visibility_success_count = 0
            
            for prim_path in table_can_prims:
                if prims_utils.is_prim_path_valid(prim_path):
                    try:
                        prim = XFormPrim(prim_paths_expr=prim_path)
                        prim.set_visibilities(visibilities=[True])
                        visibility_success_count += 1
                    except Exception as e:
                        print(f"⚠️ {prim_path} visibility reset 실패: {e}")
            
            # 2. Collision 리셋 (활성화)
            collision_success_count = 0
            
            for collision_path in collision_prims:
                try:
                    # Collision Prim 가져오기
                    collision_prim = stage.GetPrimAtPath(collision_path)
                    if collision_prim:
                        # PhysicsCollisionAPI 적용
                        collision_api = UsdPhysics.CollisionAPI.Apply(collision_prim)
                        
                        # collisionEnabled 속성 활성화
                        collision_enabled_attr = collision_prim.GetAttribute("physics:collisionEnabled")
                        if collision_enabled_attr:
                            collision_enabled_attr.Set(True)
                            collision_success_count += 1
                            print(f"✅ {collision_path} collision 활성화")
                        else:
                            # 속성이 없으면 새로 생성하고 활성화
                            collision_enabled_attr = collision_prim.CreateAttribute(
                                "physics:collisionEnabled", 
                                Usd.GetDefaultTypeForType(Usd.GetTypeForTypeName("bool"))
                            )
                            collision_enabled_attr.Set(True)
                            collision_success_count += 1
                            print(f"✅ {collision_path} collision 속성 생성 및 활성화")
                    else:
                        print(f"⚠️ {collision_path} collision prim을 찾을 수 없습니다")
                        
                except Exception as e:
                    print(f"⚠️ {collision_path} collision reset 실패: {e}")
            
            # 상태 업데이트
            self._ui_builder._table_can_visible = True
            if hasattr(self._ui_builder, '_table_can_status_label'):
                self._ui_builder._table_can_status_label.text = f"Table/Can: VISIBLE (V:{visibility_success_count}, C:{collision_success_count})"
            
            print(f"✅ Table/Can reset 완료:")
            print(f"   👁️ Visibility: {visibility_success_count}/{len(table_can_prims)} 성공")
            print(f"   💥 Collision: {collision_success_count}/{len(collision_prims)} 성공")
                            
        except Exception as e:
            print(f"⚠️ Table/Can reset 실패: {e}")
            import traceback
            traceback.print_exc()

    def _reset_material_opacity(self):
        """White와 Black Material의 opacity를 비활성화로 리셋"""
        try:
            material_paths = ["/ALLEX/Looks/Ivory", "/ALLEX/Looks/Black", "/ALLEX/Looks/DarkGray"]
            
            import omni.usd
            from pxr import Usd, Sdf
            
            stage = omni.usd.get_context().get_stage()
            
            for material_path in material_paths:
                material_prim = stage.GetPrimAtPath(material_path)
                shader_prim = None
                
                for child in material_prim.GetChildren():
                    if child.GetTypeName() == "Shader":
                        shader_prim = child
                        break
                
                # inputs:enable_opacity 속성 가져오기 또는 생성
                opacity_attr = shader_prim.GetAttribute("inputs:enable_opacity")

                try:
                    # 비활성화로 설정
                    opacity_attr.Set(False)
   
                except Exception as e:
                    print(f"❌ {material_path} reset 실패: {e}")
            
            return True
            
        except Exception as e:
            print(f"⚠️ Material Opacity reset 실패: {e}")
            return False