import json
import time
import threading

import omni
import carb
import omni.kit
from pxr import Usd, Gf
from omni.syntheticdata import sensors

try:
    import numpy as np
except ImportError:
    print("numpy not found. attempting to install...")
    omni.kit.pipapi.install("numpy")
    import numpy as np
try:
    import cv2
except ImportError:
    print("opencv-python not found. attempting to install...")
    omni.kit.pipapi.install("opencv-python")
    import cv2

import rospy
import rosgraph
import sensor_msgs.msg

import omni.add_on.RosBridgeSchema as ROSSchema

from .srv import _GetPrims, _GetPrimAttribute, _GetPrimAttributes, _SetPrimAttribute


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
            elif prim.GetTypeName() == "RosAttribute":
                schemas.append(ROSSchema.RosAttribute(prim))
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
            elif schema.__class__.__name__ == "RosAttribute":
                self._components.append(RosAttribute(self._usd_context, schema))

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
        super(RosCompressedCamera, self).__init__(usd_context, schema)

        self._viewport_interface = viewport_interface
        
        self._period = 1 / 30
        self._gt_sensors = []

        self._pub_rgb = None
        self._image_rgb = sensor_msgs.msg.CompressedImage()
        self._image_rgb.format = "jpeg"
        self._pub_depth = None
        self._image_depth = sensor_msgs.msg.CompressedImage()
        self._image_depth.format = "jpeg"
        
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
        
        # TODO: Create a new viewport_window if not exits
        # TODO: GetResolutionAttr

        # frame id
        self._image_rgb.header.frame_id = self._schema.GetFrameIdAttr().Get()
        self._image_depth.header.frame_id = self._schema.GetFrameIdAttr().Get()

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
        super(RosCompressedCamera, self).stop()

    def step(self, dt):
        pass
            
    def _publish(self):
        while self._gt_sensors:
            t0 = time.time()

            # publish rgb
            if "rgb" in self._gt_sensors:
                try:
                    frame = sensors.get_rgb(self._viewport_window)
                except ValueError as e:
                    print("[ERROR] sensors.get_rgb:", e)
                    frame = None
                if frame is not None:
                    self._image_rgb.data = np.array(cv2.imencode('.jpg', cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))[1]).tostring()
                    if self._pub_rgb is not None:
                        self._pub_rgb.publish(self._image_rgb)
            
            # publish depth
            if "depth" in self._gt_sensors:
                try:
                    frame = sensors.get_depth(self._viewport_window)
                except ValueError as e:
                    print("[ERROR] sensors.get_depth:", e)
                    frame = None
                if frame is not None:
                    if np.isfinite(frame).all() and np.max(frame) != 0:
                        frame /= np.max(frame)
                        frame *= 255
                    self._image_depth.data = np.array(cv2.imencode('.jpg', cv2.cvtColor(frame.astype(np.uint8), cv2.COLOR_BGR2RGB))[1]).tostring()
                    if self._pub_depth is not None:
                        self._pub_depth.publish(self._image_depth)
            
            # compute dt
            dt = self._period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
            

class RosAttribute(RosController):
    def __init__(self, usd_context, schema):
        super(RosAttribute, self).__init__(usd_context, schema)

        self._srv_prims = None
        self._srv_attributes = None
        self._srv_getter = None
        self._srv_setter = None

    def _process_setter_request(self, request):
        response = _SetPrimAttribute.SetPrimAttributeResponse()
        response.success = False
        if self._schema.GetEnabledAttr().Get():
            stage = self._usd_context.get_stage()
            # get prim
            if stage.GetPrimAtPath(request.path).IsValid():
                prim = stage.GetPrimAtPath(request.path)
                if request.attribute and prim.HasAttribute(request.attribute):
                    # attribute
                    attribute = prim.GetAttribute(request.attribute)
                    attribute_type = type(attribute.Get()).__name__
                    try:
                        # value
                        value = json.loads(request.value)
                        # parse data
                        if attribute_type in ['Vec2d', 'Vec2f', 'Vec2h', 'Vec2i']:
                            attribute.Set(type(attribute.Get())(value))
                        elif attribute_type in ['Vec3d', 'Vec3f', 'Vec3h', 'Vec3i']:
                            attribute.Set(type(attribute.Get())(value))
                        elif attribute_type in ['Vec4d', 'Vec4f', 'Vec4h', 'Vec4i']:
                            attribute.Set(type(attribute.Get())(value))
                        elif attribute_type in ['Quatd', 'Quatf', 'Quath']:
                            attribute.Set(type(attribute.Get())(*value))
                        elif attribute_type in ['Matrix4d', 'Matrix4f']:
                            attribute.Set(type(attribute.Get())(value))
                        elif attribute_type.startswith('Vec') and attribute_type.endswith('Array'):
                            attribute.Set(type(attribute.Get())(value))
                        elif attribute_type.startswith('Matrix') and attribute_type.endswith('Array'):
                            if attribute_type.endswith("dArray"):
                                attribute.Set(type(attribute.Get())([Gf.Matrix2d(v) for v in value]))
                            elif attribute_type.endswith("fArray"):
                                attribute.Set(type(attribute.Get())([Gf.Matrix2f(v) for v in value]))
                        elif attribute_type.startswith('Quat') and attribute_type.endswith('Array'):
                            if attribute_type.endswith("dArray"):
                                attribute.Set(type(attribute.Get())([Gf.Quatd(*v) for v in value]))
                            elif attribute_type.endswith("fArray"):
                                attribute.Set(type(attribute.Get())([Gf.Quatf(*v) for v in value]))
                            elif attribute_type.endswith("hArray"):
                                attribute.Set(type(attribute.Get())([Gf.Quath(*v) for v in value]))
                        elif attribute_type.endswith('Array'):
                            attribute.Set(type(attribute.Get())(value))
                        elif attribute_type in ['AssetPath']:
                            attribute.Set(type(attribute.Get())(value))
                        elif attribute_type in ['NoneType']:
                            pass
                        else:
                            attribute.Set(type(attribute.Get())(value))
                        response.success = True
                    except Exception as e:
                        print("[ERROR] srv {} request for {} ({}: {}): {}".format(self._srv_setter.resolved_name, request.path, request.attribute, value, e))
                        response.success = False
                        response.message = str(e)
                else:
                    response.message = "Prim has not attribute {}".format(request.attribute)
            else:
                response.message = "Invalid prim ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response
        
    def _process_getter_request(self, request):
        response = _GetPrimAttribute.GetPrimAttributeResponse()
        response.success = False
        if self._schema.GetEnabledAttr().Get():
            stage = self._usd_context.get_stage()
            # get prim
            if stage.GetPrimAtPath(request.path).IsValid():
                prim = stage.GetPrimAtPath(request.path)
                if request.attribute and prim.HasAttribute(request.attribute):
                    attribute = prim.GetAttribute(request.attribute)
                    response.type = type(attribute.Get()).__name__
                    # parse data
                    response.success = True
                    if response.type in ['Vec2d', 'Vec2f', 'Vec2h', 'Vec2i']:
                        data = attribute.Get()
                        response.value = json.dumps([data[i] for i in range(2)])
                    elif response.type in ['Vec3d', 'Vec3f', 'Vec3h', 'Vec3i']:
                        data = attribute.Get()
                        response.value = json.dumps([data[i] for i in range(3)])
                    elif response.type in ['Vec4d', 'Vec4f', 'Vec4h', 'Vec4i']:
                        data = attribute.Get()
                        response.value = json.dumps([data[i] for i in range(4)])
                    elif response.type in ['Quatd', 'Quatf', 'Quath']:
                        data = attribute.Get()
                        response.value = json.dumps([data.real, data.imaginary[0], data.imaginary[1], data.imaginary[2]])                    
                    elif response.type in ['Matrix4d', 'Matrix4f']:
                        data = attribute.Get()
                        response.value = json.dumps([[data.GetRow(i)[j] for j in range(data.dimension[1])] for i in range(data.dimension[0])])
                    elif response.type.startswith('Vec') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[d[i] for i in range(len(d))] for d in data])
                    elif response.type.startswith('Matrix') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[[d.GetRow(i)[j] for j in range(d.dimension[1])] for i in range(d.dimension[0])] for d in data])
                    elif response.type.startswith('Quat') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[d.real, d.imaginary[0], d.imaginary[1], d.imaginary[2]] for d in data])
                    elif response.type.endswith('Array'):
                        try:
                            response.value = json.dumps(list(attribute.Get()))
                        except Exception as e:
                            print("[UNKNOW]", type(attribute.Get()))
                            print("  |-- Please, report a new issue (https://github.com/Toni-SM/omni.add_on.ros_bridge/issues)")
                            response.success = False
                            response.message = "Unknow type {}".format(type(attribute.Get()))
                    elif response.type in ['AssetPath']:
                        response.value = json.dumps(str(attribute.Get().path))
                    else:
                        try:
                            response.value = json.dumps(attribute.Get())
                        except Exception as e:
                            print("[UNKNOW]", type(attribute.Get()), attribute.Get())
                            print("  |-- Please, report a new issue (https://github.com/Toni-SM/omni.add_on.ros_bridge/issues)")
                            response.success = False
                            response.message = "Unknow type {}".format(type(attribute.Get()))
                else:
                    response.message = "Prim has not attribute {}".format(request.attribute)
            else:
                response.message = "Invalid prim ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response

    def _process_attributes_request(self, request):
        response = _GetPrimAttributes.GetPrimAttributesResponse()
        response.success = False
        if self._schema.GetEnabledAttr().Get():
            stage = self._usd_context.get_stage()
            # get prim
            if stage.GetPrimAtPath(request.path).IsValid():
                prim = stage.GetPrimAtPath(request.path)
                for attribute in prim.GetAttributes():
                    if attribute.GetNamespace():
                        response.names.append("{}:{}".format(attribute.GetNamespace(), attribute.GetBaseName()))
                    else:
                        response.names.append(attribute.GetBaseName())
                    response.displays.append(attribute.GetDisplayName())
                    response.types.append(type(attribute.Get()).__name__)
                    response.success = True
            else:
                response.message = "Invalid prim ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response
    
    def _process_prims_request(self, request):
        response = _GetPrims.GetPrimsResponse()
        response.success = False
        if self._schema.GetEnabledAttr().Get():
            stage = self._usd_context.get_stage()
            # get prims
            if not request.path or stage.GetPrimAtPath(request.path).IsValid():
                path = request.path if request.path else "/"
                for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath(path)):
                    response.paths.append(str(prim.GetPath()))
                    response.types.append(prim.GetTypeName())
                    response.success = True
            else:
                response.message = "Invalid search path ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response

    def start(self):
        self.started = True
        print("[INFO] RosAttribute: starting", self._schema.__class__.__name__)

        service_name = self._schema.GetPrimsSrvTopicAttr().Get()
        self._srv_prims = rospy.Service(service_name, _GetPrims.GetPrims, self._process_prims_request)
        print("[INFO] RosAttribute: register srv:", self._srv_prims.resolved_name)

        service_name = self._schema.GetGetAttrSrvTopicAttr().Get()
        self._srv_getter = rospy.Service(service_name, _GetPrimAttribute.GetPrimAttribute, self._process_getter_request)
        print("[INFO] RosAttribute: register srv:", self._srv_getter.resolved_name)

        service_name = self._schema.GetAttributesSrvTopicAttr().Get()
        self._srv_attributes = rospy.Service(service_name, _GetPrimAttributes.GetPrimAttributes, self._process_attributes_request)
        print("[INFO] RosAttribute: register srv:", self._srv_attributes.resolved_name)

        service_name = self._schema.GetSetAttrSrvTopicAttr().Get()
        self._srv_setter = rospy.Service(service_name, _SetPrimAttribute.SetPrimAttribute, self._process_setter_request)
        print("[INFO] RosAttribute: register srv:", self._srv_setter.resolved_name)
        
    def stop(self):
        if self._srv_prims is not None:
            print("[INFO] RosAttribute: unregister srv:", self._srv_prims.resolved_name)
            self._srv_prims.shutdown()
            self._srv_prims = None
        if self._srv_getter is not None:
            print("[INFO] RosAttribute: unregister srv:", self._srv_getter.resolved_name)
            self._srv_getter.shutdown()
            self._srv_getter = None
        if self._srv_attributes is not None:
            print("[INFO] RosAttribute: unregister srv:", self._srv_attributes.resolved_name)
            self._srv_attributes.shutdown()
            self._srv_attributes = None
        if self._srv_setter is not None:
            print("[INFO] RosAttribute: unregister srv:", self._srv_setter.resolved_name)
            self._srv_setter.shutdown()
            self._srv_setter = None
        super(RosAttribute, self).stop()

    def step(self, dt):
        pass
        
            