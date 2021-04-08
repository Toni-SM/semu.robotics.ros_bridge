import carb
import omni.kit.ui
import omni.kit.editor

import omni.add_on.RosControlBridgeSchema as ROSControlSchema

ADD_ROS_CONTROL_FOLLOW_JOINT_TRAJECTORY = "Create/Isaac/ROS Control/FollowJointTrajectory"
ADD_ROS_CONTROL_FOLLOW_JOINT_TRAJECTORY = "Test/FollowJointTrajectory"


class RosControlBridgeMenu:
    def __init__(self):
        self._usd_context = omni.usd.get_context()
        self._menus = []

        editor_menu = omni.kit.ui.get_editor_menu()
        self._menus.append(editor_menu.add_item(ADD_ROS_CONTROL_FOLLOW_JOINT_TRAJECTORY, self._on_scene_menu_click))
        
    def setup_base_prim(self, prim):
        prim.CreateRosNodePrefixAttr("")
        prim.CreateEnabledAttr(True)

    def get_path(self, name):
        selectedPrims = self._usd_context.get_selection().get_selected_prim_paths()
        if len(selectedPrims):
            return omni.kit.utils.get_stage_next_free_path(self._stage, selectedPrims[-1] + name, False)
        return omni.kit.utils.get_stage_next_free_path(self._stage, name, True)

    def add_follow_joint_trajectory(self):

        prim = ROSControlSchema.RosControlFollowJointTrajectory.Define(self._stage, self.get_path("/ROSControl_FollowJointTrajectory"))
        self.setup_base_prim(prim)

        prim.CreateControllerNameAttr("/robot_controller")
        prim.CreateActionNamespaceAttr("/follow_joint_trajectory")
        prim.CreateArticulationPrimRel()

    def _on_scene_menu_click(self, menu, value):
        self._stage = self._usd_context.get_stage()

        if menu == ADD_ROS_CONTROL_FOLLOW_JOINT_TRAJECTORY:
            self.add_follow_joint_trajectory()

    def shutdown(self):
        self._menus = None
