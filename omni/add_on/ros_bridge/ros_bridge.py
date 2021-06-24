import time
import threading
import numpy as np
import cv2

import omni
import omni.kit
import carb
from pxr import Usd
from omni.syntheticdata import sensors

import rospy
import sensor_msgs.msg
import rosgraph

import omni.add_on.RosBridgeSchema as ROSSchema


def acquire_ros_bridge_interface(plugin_name=None, library_path=None):
    return RosBridge()

def release_ros_bridge_interface(bridge):
    bridge.shutdown()


class RosBridge:
    def __init__(self):
        self._components = []

        # omni objects
        self._usd_context = omni.usd.get_context()
        self._timeline = omni.timeline.get_timeline_interface()
        self._viewport_interface = omni.kit.viewport.get_viewport_interface()
        
        # events
        self._event_editor_step = (omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_editor_step_event))
        self._event_timeline = self._timeline.get_timeline_event_stream().create_subscription_to_pop(self._on_timeline_event)
        self._stage_event = self._usd_context.get_stage_event_stream().create_subscription_to_pop(self._on_stage_event)

        # ROS node
        self._ros_node()

    def shutdown(self):
        self._event_editor_step = None
        self._event_timeline = None
        self._stage_event = None
        self._stop_components()

    def _ros_node(self):
        node_name = carb.settings.get_settings().get("/exts/omni.add_on.ros_bridge/nodeName")
        # check ROS master
        try:
            rosgraph.Master("/rostopic").getPid()
        except:
            print("[WARNING] {}: ROS master is not running",format(node_name))
            return False
        # start ROS node
        try:
            rospy.init_node(node_name)
            print("[INFO] {} node started".format(node_name))
        except rospy.ROSException as e:
            print("[ERROR] {}:".format(node_name), e)
            return False
        return True

    def _get_ros_bridge_schemas(self):
        schemas = []
        stage = self._usd_context.get_stage()
        for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath("/")):
            if prim.GetTypeName() == "RosCompressedCamera":
                schemas.append(ROSSchema.RosCompressedCamera(prim))
        return schemas

    def _stop_components(self):
        for component in self._components:
            component.stop()

    def _reload_components(self):
        # stop components
        self._stop_components()
        # load components
        self._components = []
        self._skip_step = True
        for schema in self._get_ros_bridge_schemas():
            if schema.__class__.__name__ == "RosCompressedCamera":
                self._components.append(RosCompressedCamera(self._viewport_interface, self._usd_context, schema))

    def _on_editor_step_event(self, event):
        if self._timeline.is_playing():
            for component in self._components:
                if self._skip_step:
                    self._skip_step = False
                    return
                # start components
                if not component.started:
                    component.start()
                    return
                # step
                component.step(event.payload["dt"])
    
    def _on_timeline_event(self, event):
        # reload components
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._reload_components()
            print("[INFO] RosBridge: components reloaded")
        # stop components
        elif event.type == int(omni.timeline.TimelineEventType.STOP) or event.type == int(omni.timeline.TimelineEventType.PAUSE):
            self._stop_components()
            print("[INFO] RosBridge: components stopped")

    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.OPENED):
            pass


class RosController():
    def __init__(self, usd_context, schema):
        self._usd_context = usd_context
        self._schema = schema
        
        self.started = False
    
    def start(self):
        raise NotImplementedError

    def stop(self):
        print("[INFO] RosController: stopping", self._schema.__class__.__name__)
        self.started = False

    def step(self, dt):
        raise NotImplementedError


class RosCompressedCamera(RosController):
    def __init__(self, viewport_interface, usd_context, schema):
        super().__init__(usd_context, schema)

        self._viewport_interface = viewport_interface
        
        self._period = 1 / 30
        self._gt_sensors = []

        self._pub_rgb = None
        self._image_rgb = sensor_msgs.msg.CompressedImage()
        self._pub_depth = None
        self._image_depth = sensor_msgs.msg.CompressedImage()
        
    def start(self):
        self.started = True
        self._viewport_window = None
        print("[INFO] RosCompressedCamera: starting", self._schema.__class__.__name__)

        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        if not len(relationships):
            print("[WARNING] RosCompressedCamera: empty relationships")
            return

        # get viewport window
        path = relationships[0].GetPrimPath().pathString
        for interface in self._viewport_interface.get_instance_list():
            window = self._viewport_interface.get_viewport_window(interface)
            if path == window.get_active_camera():
                self._viewport_window = window
                break
        
        # TODO: create a new viewport_window if not exits
        # TODO: GetResolutionAttr
        # TODO: GetFrameIdAttr

        # publishers
        queue_size = self._schema.GetQueueSizeAttr().Get()
        # rgb
        if self._schema.GetRgbEnabledAttr().Get():
            self._gt_sensors.append("rgb")
            topic_name = self._schema.GetRgbPubTopicAttr().Get()
            self._pub_rgb = rospy.Publisher(topic_name, sensor_msgs.msg.CompressedImage, queue_size=queue_size)
            print("[INFO] RosCompressedCamera: register rgb:", self._pub_rgb.name)
        # depth
        if self._schema.GetDepthEnabledAttr().Get():
            self._gt_sensors.append("depth")
            topic_name = self._schema.GetDepthPubTopicAttr().Get()
            self._pub_depth = rospy.Publisher(topic_name, sensor_msgs.msg.CompressedImage, queue_size=queue_size)
            print("[INFO] RosCompressedCamera: register depth:", self._pub_depth.name)

        if self._gt_sensors:
            threading.Thread(target=self._publish).start()

    def stop(self):
        self._gt_sensors = []
        if self._pub_rgb is not None:
            print("[INFO] RosCompressedCamera: unregister rgb:", self._pub_rgb.name)
            self._pub_rgb.unregister()
            self._pub_rgb = None
        if self._pub_depth is not None:
            print("[INFO] RosCompressedCamera: unregister depth:", self._pub_depth.name)
            self._pub_depth.unregister()
            self._pub_depth = None
        super().stop()

    def step(self, dt):
        pass
            
    def _publish(self):
        while self._gt_sensors:
            t0 = time.time()

            # publish rgb
            if "rgb" in self._gt_sensors:
                frame = sensors.get_rgb(self._viewport_window)
                self._image_rgb.data = np.array(cv2.imencode('.jpg', cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))[1]).tostring()
                if self._pub_rgb is not None:
                    self._pub_rgb.publish(self._image_rgb)
            # publish depth
            if "depth" in self._gt_sensors:
                frame = sensors.get_depth(self._viewport_window)
                self._image_depth.data = np.array(cv2.imencode('.jpg', cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))[1]).tostring()
                if self._pub_depth is not None:
                    self._pub_depth.publish(self._image_depth)
            
            # compute dt
            dt = self._period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
            




        # relationships = self._schema.GetArticulationPrimRel().GetTargets()
        # if not len(relationships):
        #     print("[WARNING] RosControllerFollowJointTrajectory: empty relationships")
        #     return

        # # check for articulation API
        # stage = self._usd_context.get_stage()
        # path = relationships[0].GetPrimPath().pathString
        # if not stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxArticulationAPI):
        #     print("[WARNING] RosControllerFollowJointTrajectory: prim {} doesn't have PhysxArticulationAPI".format(path))
        #     return
        
        # # get articulation
        # self._ar = self._dci.get_articulation(path)
        # if self._ar == _dynamic_control.INVALID_HANDLE:
        #     print("[WARNING] RosControllerFollowJointTrajectory: prim {}: invalid handle".format(path))
        #     return

        # # get DOF
        # self._dof = {}
        # for i in range(self._dci.get_articulation_dof_count(self._ar)):
        #     dof = self._dci.get_articulation_dof(self._ar, i)
        #     if dof != _dynamic_control.DofType.DOF_NONE:
        #         joint_name = self._dci.get_joint_name(self._dci.get_dof_joint(dof))
        #         self._dof[joint_name] = {"dof": dof, "target": None, "current": None, "error": None, "dt": None}

        # # build action name
        # _action_name = self._schema.GetRosNodePrefixAttr().Get() \
        #                 + self._schema.GetControllerNameAttr().Get() \
        #                 + self._schema.GetActionNamespaceAttr().Get()

        # # start actionlib server
        # self._start_server(_action_name, control_msgs.msg.FollowJointTrajectoryAction)
    