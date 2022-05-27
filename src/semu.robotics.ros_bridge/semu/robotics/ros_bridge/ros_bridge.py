from typing import List, Any

import json
import asyncio
import threading

import omni
import carb
import omni.kit
from pxr import Usd, Gf, PhysxSchema
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.stage import get_stage_units

import rospy
import rosgraph
import actionlib
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint

import semu.usd.schemas.RosBridgeSchema as ROSSchema
import semu.usd.schemas.RosControlBridgeSchema as ROSControlSchema


# message types
GetPrims = None
GetPrimAttributes = None
GetPrimAttribute = None
SetPrimAttribute = None


def acquire_ros_bridge_interface(ext_id: str = "") -> 'RosBridge':
    """
    Acquire the RosBridge interface

    :param ext_id: The extension id
    :type ext_id: str

    :returns: The RosBridge interface
    :rtype: RosBridge
    """
    global GetPrims, GetPrimAttributes, GetPrimAttribute, SetPrimAttribute

    from add_on_msgs.srv import _GetPrims as get_prims_srv
    from add_on_msgs.srv import _GetPrimAttributes as get_prim_attributes_srv
    from add_on_msgs.srv import _GetPrimAttribute as get_prim_attribute_srv
    from add_on_msgs.srv import _SetPrimAttribute as set_prim_attribute_srv
    
    GetPrims = get_prims_srv
    GetPrimAttributes = get_prim_attributes_srv
    GetPrimAttribute = get_prim_attribute_srv
    SetPrimAttribute = set_prim_attribute_srv

    bridge = RosBridge()
    return bridge

def release_ros_bridge_interface(bridge: 'RosBridge') -> None:
    """
    Release the RosBridge interface
    
    :param bridge: The RosBridge interface
    :type bridge: RosBridge
    """
    bridge.shutdown()


class RosBridge:
    def __init__(self) -> None:
        """Initialize the RosBridge interface
        """
        self._components = []
        self._node_name = carb.settings.get_settings().get("/exts/semu.robotics.ros_bridge/nodeName")

        # omni objects and interfaces
        self._usd_context = omni.usd.get_context()
        self._timeline = omni.timeline.get_timeline_interface()
        self._physx_interface = omni.physx.acquire_physx_interface()
        self._dci = _dynamic_control.acquire_dynamic_control_interface()
        
        # events
        self._update_event = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update_event)
        self._timeline_event = self._timeline.get_timeline_event_stream().create_subscription_to_pop(self._on_timeline_event)
        self._stage_event = self._usd_context.get_stage_event_stream().create_subscription_to_pop(self._on_stage_event)
        self._physx_event = self._physx_interface.subscribe_physics_step_events(self._on_physics_event)

        # ROS node
        self._init_ros_node()

    def shutdown(self) -> None:
        """Shutdown the RosBridge interface
        """
        self._update_event = None
        self._timeline_event = None
        self._stage_event = None

        self._stop_components()

    def _init_ros_node(self) -> None:
        """Initialize the ROS node
        """
        # check ROS master
        try:
            rosgraph.Master("/rostopic").getPid()
        except:
            print("[Warning][semu.robotics.ros_bridge] {}: ROS master is not running".format(self._node_name))
            return False
        # start ROS node
        try:
            rospy.init_node(self._node_name)
            print("[Info][semu.robotics.ros_bridge] {} node started".format(self._node_name))
        except rospy.ROSException as e:
            print("[Error][semu.robotics.ros_bridge] {}: {}".format(self._node_name, e))
            return False
        return True

    def _get_ros_bridge_schemas(self) -> List['ROSSchema.RosBridgeComponent']:
        """Get the ROS bridge schemas in the current stage

        :returns: The ROS bridge schemas
        :rtype: list of RosBridgeComponent
        """
        schemas = []
        stage = self._usd_context.get_stage()
        for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath("/")):
            if prim.GetTypeName() == "RosAttribute":
                schemas.append(ROSSchema.RosAttribute(prim))
            elif prim.GetTypeName() == "RosControlFollowJointTrajectory":
                schemas.append(ROSControlSchema.RosControlFollowJointTrajectory(prim))
            elif prim.GetTypeName() == "RosControlGripperCommand":
                schemas.append(ROSControlSchema.RosControlGripperCommand(prim))
        return schemas

    def _stop_components(self) -> None:
        """Stop all components
        """
        for component in self._components:
            component.stop()

    def _reload_components(self) -> None:
        """Reload all components
        """
        # stop components
        self._stop_components()
        # load components
        self._components = []
        self._skip_update_step = True
        for schema in self._get_ros_bridge_schemas():
            if schema.__class__.__name__ == "RosAttribute":
                self._components.append(RosAttribute(self._usd_context, schema, self._dci))
            elif schema.__class__.__name__ == "RosControlFollowJointTrajectory":
                self._components.append(RosControlFollowJointTrajectory(self._usd_context, schema, self._dci))
            elif schema.__class__.__name__ == "RosControlGripperCommand":
                self._components.append(RosControllerGripperCommand(self._usd_context, schema, self._dci))

    def _on_update_event(self, event: 'carb.events._events.IEvent') -> None:
        """Handle the kit update event

        :param event: Event
        :type event: carb.events._events.IEvent
        """
        if self._timeline.is_playing():
            for component in self._components:
                if self._skip_update_step:
                    self._skip_update_step = False
                    return
                # start components
                if not component.started:
                    component.start()
                    return
                # step
                component.update_step(event.payload["dt"])
    
    def _on_timeline_event(self, event: 'carb.events._events.IEvent') -> None:
        """Handle the timeline event

        :param event: Event
        :type event: carb.events._events.IEvent
        """
        # reload components
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._reload_components()
            print("[Info][semu.robotics.ros_bridge] RosControlBridge: components reloaded")
        # stop components
        elif event.type == int(omni.timeline.TimelineEventType.STOP) or event.type == int(omni.timeline.TimelineEventType.PAUSE):
            self._stop_components()
            print("[Info][semu.robotics.ros_bridge] RosControlBridge: components stopped")

    def _on_stage_event(self, event: 'carb.events._events.IEvent') -> None:
        """Handle the stage event

        :param event: The stage event
        :type event: carb.events._events.IEvent
        """
        pass

    def _on_physics_event(self, step: float) -> None:
        """Handle the physics event

        :param step: The physics step
        :type step: float
        """
        for component in self._components:
            component.physics_step(step)


class RosController():
    def __init__(self, usd_context: 'omni.usd._usd.UsdContext', schema: 'ROSSchema.RosBridgeComponent') -> None:
        """Base class for RosController

        :param usd_context: USD context
        :type usd_context: omni.usd._usd.UsdContext
        :param schema: The ROS bridge schema
        :type schema: ROSSchema.RosBridgeComponent
        """
        self._usd_context = usd_context
        self._schema = schema
        
        self.started = False

    def start(self) -> None:
        """Start the component
        """
        raise NotImplementedError

    def stop(self) -> None:
        """Stop the component
        """
        print("[Info][semu.robotics.ros_bridge] RosController: stopping {}".format(self._schema.__class__.__name__))
        self.started = False

    def update_step(self, dt: float) -> None:
        """Kit update step
        
        :param dt: The delta time
        :type dt: float
        """
        raise NotImplementedError

    def physics_step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        raise NotImplementedError
    

class RosAttribute(RosController):
    def __init__(self, 
                 usd_context: 'omni.usd._usd.UsdContext', 
                 schema: 'ROSSchema.RosBridgeComponent', 
                 dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """RosAttribute interface

        :param usd_context: USD context
        :type usd_context: omni.usd._usd.UsdContext
        :param schema: The ROS bridge schema
        :type schema: ROSSchema.RosAttribute
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(usd_context, schema)

        self._dci = dci

        self._srv_prims = None
        self._srv_attributes = None
        self._srv_getter = None
        self._srv_setter = None

        self._value = None
        self._attribute = None

        self._event = threading.Event()
        self._event.set()

        self.__event_timeout = carb.settings.get_settings().get("/exts/semu.robotics.ros_bridge/eventTimeout")
        self.__set_attribute_using_asyncio = \
            carb.settings.get_settings().get("/exts/semu.robotics.ros_bridge/setAttributeUsingAsyncio")
        print("[Info][semu.robotics.ros_bridge] RosAttribute: asyncio: {}".format(self.__set_attribute_using_asyncio))
        print("[Info][semu.robotics.ros_bridge] RosAttribute: event timeout: {}".format(self.__event_timeout))

    async def _set_attribute(self, attribute: 'pxr.Usd.Attribute', attribute_value: Any) -> None:
        """Set the attribute value using asyncio

        :param attribute: The prim's attribute to set
        :type attribute: pxr.Usd.Attribute
        :param attribute_value: The attribute value
        :type attribute_value: Any
        """
        ret = attribute.Set(attribute_value)

    def _process_setter_request(self, 
                                request: 'SetPrimAttribute.SetPrimAttributeRequest') -> 'SetPrimAttribute.SetPrimAttributeResponse':
        """Process the setter request

        :param request: The service request
        :type request: SetPrimAttribute.SetPrimAttributeRequest

        :return: The service response
        :rtype: SetPrimAttribute.SetPrimAttributeResponse
        """
        response = SetPrimAttribute.SetPrimAttributeResponse()
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
                    # value
                    try:
                        value = json.loads(request.value)
                        attribute_value = None
                    except json.JSONDecodeError:
                        print("[Error][semu.robotics.ros_bridge] RosAttribute: invalid value: {}".format(request.value))
                        response.success = False
                        response.message = "Invalid value '{}'".format(request.value)
                        return response
                    # parse data
                    try:
                        if attribute_type in ['Vec2d', 'Vec2f', 'Vec2h', 'Vec2i']:
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type in ['Vec3d', 'Vec3f', 'Vec3h', 'Vec3i']:
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type in ['Vec4d', 'Vec4f', 'Vec4h', 'Vec4i']:
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type in ['Quatd', 'Quatf', 'Quath']:
                            attribute_value = type(attribute.Get())(*value)
                        elif attribute_type in ['Matrix4d', 'Matrix4f']:
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type.startswith('Vec') and attribute_type.endswith('Array'):
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type.startswith('Matrix') and attribute_type.endswith('Array'):
                            if attribute_type.endswith("dArray"):
                                attribute_value = type(attribute.Get())([Gf.Matrix2d(v) for v in value])
                            elif attribute_type.endswith("fArray"):
                                attribute_value = type(attribute.Get())([Gf.Matrix2f(v) for v in value])
                        elif attribute_type.startswith('Quat') and attribute_type.endswith('Array'):
                            if attribute_type.endswith("dArray"):
                                attribute_value = type(attribute.Get())([Gf.Quatd(*v) for v in value])
                            elif attribute_type.endswith("fArray"):
                                attribute_value = type(attribute.Get())([Gf.Quatf(*v) for v in value]) 
                            elif attribute_type.endswith("hArray"):
                                attribute_value = type(attribute.Get())([Gf.Quath(*v) for v in value])
                        elif attribute_type.endswith('Array'):
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type in ['AssetPath']:
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type in ['NoneType']:
                            pass
                        else:
                            attribute_value = type(attribute.Get())(value)
                        
                        # set attribute
                        if attribute_value is not None:
                            # set attribute usign asyncio
                            if self.__set_attribute_using_asyncio:
                                try:
                                    loop = asyncio.get_event_loop()
                                except:
                                    loop = asyncio.new_event_loop()
                                asyncio.set_event_loop(loop)
                                future = asyncio.ensure_future(self._set_attribute(attribute, attribute_value))
                                loop.run_until_complete(future)
                                response.success = True
                            # set attribute in the physics event
                            else:
                                self._attribute = attribute
                                self._value = attribute_value

                                self._event.clear()
                                response.success = self._event.wait(self.__event_timeout)
                                if not response.success:
                                    response.message = "The timeout ({} s) for setting the attribute value has been reached" \
                                        .format(self.__event_timeout)

                    except Exception as e:
                        print("[Error][semu.robotics.ros_bridge] RosAttribute: srv {} request for {} ({}: {}): {}" \
                            .format(self._srv_setter.resolved_name, request.path, request.attribute, value, e))
                        response.success = False
                        response.message = str(e)
                else:
                    response.message = "Prim has not attribute {}".format(request.attribute)
            else:
                response.message = "Invalid prim ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response
        
    def _process_getter_request(self, 
                                request: 'GetPrimAttribute.GetPrimAttributeRequest') -> 'GetPrimAttribute.GetPrimAttributeResponse':
        """Process the getter request

        :param request: The service request
        :type request: GetPrimAttribute.GetPrimAttributeRequest

        :return: The service response
        :rtype: GetPrimAttribute.GetPrimAttributeResponse
        """
        response = GetPrimAttribute.GetPrimAttributeResponse()
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
                        response.value = json.dumps([[data.GetRow(i)[j] for j in range(data.dimension[1])] \
                            for i in range(data.dimension[0])])
                    elif response.type.startswith('Vec') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[d[i] for i in range(len(d))] for d in data])
                    elif response.type.startswith('Matrix') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[[d.GetRow(i)[j] for j in range(d.dimension[1])] \
                            for i in range(d.dimension[0])] for d in data])
                    elif response.type.startswith('Quat') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[d.real, d.imaginary[0], d.imaginary[1], d.imaginary[2]] for d in data])
                    elif response.type.endswith('Array'):
                        try:
                            response.value = json.dumps(list(attribute.Get()))
                        except Exception as e:
                            print("[Warning][semu.robotics.ros_bridge] RosAttribute: Unknow attribute type {}" \
                                .format(type(attribute.Get())))
                            print("  |-- Please, report a new issue (https://github.com/Toni-SM/semu.robotics.ros_bridge/issues)")
                            response.success = False
                            response.message = "Unknow type {}".format(type(attribute.Get()))
                    elif response.type in ['AssetPath']:
                        response.value = json.dumps(str(attribute.Get().path))
                    else:
                        try:
                            response.value = json.dumps(attribute.Get())
                        except Exception as e:
                            print("[Warning][semu.robotics.ros_bridge] RosAttribute: Unknow {}: {}" \
                                .format(type(attribute.Get()), attribute.Get()))
                            print("  |-- Please, report a new issue (https://github.com/Toni-SM/semu.robotics.ros_bridge/issues)")
                            response.success = False
                            response.message = "Unknow type {}".format(type(attribute.Get()))
                else:
                    response.message = "Prim has not attribute {}".format(request.attribute)
            else:
                response.message = "Invalid prim ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response

    def _process_attributes_request(self, 
                                    request: 'GetPrimAttributes.GetPrimAttributesRequest') -> 'GetPrimAttributes.GetPrimAttributesResponse':
        """Process the 'get all attributes' request

        :param request: The service request
        :type request: GetPrimAttributes.GetPrimAttributesRequest

        :return: The service response
        :rtype: GetPrimAttributes.GetPrimAttributesResponse
        """
        response = GetPrimAttributes.GetPrimAttributesResponse()
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
    
    def _process_prims_request(self, request: 'GetPrims.GetPrimsRequest') -> 'GetPrims.GetPrimsResponse':
        """Process the 'get all prims' request

        :param request: The service request
        :type request: GetPrims.GetPrimsRequest

        :return: The service response
        :rtype: GetPrims.GetPrimsResponse
        """
        response = GetPrims.GetPrimsResponse()
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

    def start(self) -> None:
        """Start the services
        """
        print("[Info][semu.robotics.ros_bridge] RosAttribute: starting {}".format(self._schema.__class__.__name__))

        service_name = self._schema.GetPrimsSrvTopicAttr().Get()
        self._srv_prims = rospy.Service(service_name, GetPrims.GetPrims, self._process_prims_request)
        print("[Info][semu.robotics.ros_bridge] RosAttribute: register srv: {}".format(self._srv_prims.resolved_name))

        service_name = self._schema.GetGetAttrSrvTopicAttr().Get()
        self._srv_getter = rospy.Service(service_name, GetPrimAttribute.GetPrimAttribute, self._process_getter_request)
        print("[Info][semu.robotics.ros_bridge] RosAttribute: register srv: {}".format(self._srv_getter.resolved_name))

        service_name = self._schema.GetAttributesSrvTopicAttr().Get()
        self._srv_attributes = rospy.Service(service_name, GetPrimAttributes.GetPrimAttributes, self._process_attributes_request)
        print("[Info][semu.robotics.ros_bridge] RosAttribute: register srv: {}".format(self._srv_attributes.resolved_name))

        service_name = self._schema.GetSetAttrSrvTopicAttr().Get()
        self._srv_setter = rospy.Service(service_name, SetPrimAttribute.SetPrimAttribute, self._process_setter_request)
        print("[Info][semu.robotics.ros_bridge] RosAttribute: register srv: {}".format(self._srv_setter.resolved_name))
        
        self.started = True

    def stop(self) -> None:
        """Stop the services
        """
        if self._srv_prims is not None:
            print("[Info][semu.robotics.ros_bridge] RosAttribute: unregister srv: {}".format(self._srv_prims.resolved_name))
            self._srv_prims.shutdown()
            self._srv_prims = None
        if self._srv_getter is not None:
            print("[Info][semu.robotics.ros_bridge] RosAttribute: unregister srv: {}".format(self._srv_getter.resolved_name))
            self._srv_getter.shutdown()
            self._srv_getter = None
        if self._srv_attributes is not None:
            print("[Info][semu.robotics.ros_bridge] RosAttribute: unregister srv: {}".format(self._srv_attributes.resolved_name))
            self._srv_attributes.shutdown()
            self._srv_attributes = None
        if self._srv_setter is not None:
            print("[Info][semu.robotics.ros_bridge] RosAttribute: unregister srv: {}".format(self._srv_setter.resolved_name))
            self._srv_setter.shutdown()
            self._srv_setter = None
        super().stop()

    def update_step(self, dt: float) -> None:
        """Kit update step

        :param dt: The delta time
        :type dt: float
        """
        pass

    def physics_step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        if self.__set_attribute_using_asyncio:
            return
        if self._dci.is_simulating():
            if not self._event.is_set():
                if self._attribute is not None:
                    ret = self._attribute.Set(self._value)
                self._event.set()
        

class RosControlFollowJointTrajectory(RosController):
    def __init__(self, 
                 usd_context: 'omni.usd._usd.UsdContext', 
                 schema: 'ROSSchema.RosBridgeComponent', 
                 dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """FollowJointTrajectory interface
        
        :param usd_context: The USD context
        :type usd_context: omni.usd._usd.UsdContext
        :param schema: The schema
        :type schema: ROSSchema.RosBridgeComponent
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(usd_context, schema)
        
        self._dci = dci

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._action_server = None

        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_point_index = 1

        # feedback / result
        self._action_result_message = control_msgs.msg.FollowJointTrajectoryResult()
        self._action_feedback_message = control_msgs.msg.FollowJointTrajectoryFeedback()
    
    def start(self) -> None:
        """Start the action server
        """
        print("[Info][semu.robotics.ros_bridge] RosControlFollowJointTrajectory: starting {}" \
            .format(self._schema.__class__.__name__))

        # get attributes and relationships
        action_namespace = self._schema.GetActionNamespaceAttr().Get()
        controller_name = self._schema.GetControllerNameAttr().Get()

        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        if not len(relationships):
            print("[Warning][semu.robotics.ros_bridge] RosControlFollowJointTrajectory: empty relationships")
            return

        # check for articulation API
        stage = self._usd_context.get_stage()
        path = relationships[0].GetPrimPath().pathString
        if not stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxArticulationAPI):
            print("[Warning][semu.robotics.ros_bridge] RosControlFollowJointTrajectory: {} doesn't have PhysxArticulationAPI".format(path))
            return
        
        # start action server
        self._shutdown_action_server()
        self._action_server = actionlib.ActionServer(controller_name + action_namespace, 
                                                     control_msgs.msg.FollowJointTrajectoryAction,
                                                     goal_cb=self._on_goal,
                                                     cancel_cb=self._on_cancel,
                                                     auto_start=False)
        try:
            self._action_server.start()
        except ConnectionRefusedError:
            print("[Error][semu.robotics.ros_bridge] RosControlFollowJointTrajectory: action server {} not started" \
                .format(controller_name + action_namespace))
            return
        print("[Info][semu.robotics.ros_bridge] RosControlFollowJointTrajectory: register action {}" \
            .format(controller_name + action_namespace))

        self.started = True
    
    def stop(self) -> None:
        """Stop the action server
        """
        super().stop()
        self._articulation = _dynamic_control.INVALID_HANDLE
        self._shutdown_action_server()
        self._action_goal_handle = None
        self._action_goal = None

    def _shutdown_action_server(self) -> None:
        """Shutdown the action server
        """
        # omni.isaac.ros_bridge/noetic/lib/python3/dist-packages/actionlib/action_server.py
        print("[Info][semu.robotics.ros_bridge] RosControlFollowJointTrajectory: destroy action server: {}" \
            .format(self._schema.GetPrim().GetPath()))
        if self._action_server:
            if self._action_server.started:
                self._action_server.started = False

                self._action_server.status_pub.unregister()
                self._action_server.result_pub.unregister()
                self._action_server.feedback_pub.unregister()

                self._action_server.goal_sub.unregister()
                self._action_server.cancel_sub.unregister()

            del self._action_server
            self._action_server = None

    def _init_articulation(self) -> None:
        """Initialize the articulation and register joints
        """
        # get articulation
        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        path = relationships[0].GetPrimPath().pathString
        self._articulation = self._dci.get_articulation(path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[Warning][semu.robotics.ros_bridge] RosControlFollowJointTrajectory: {} is not an articulation".format(path))
            return
        
        dof_props = self._dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        for i in range(self._dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self._dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                dof_name = self._dci.get_dof_name(dof_ptr)
                if dof_name not in self._joints:
                    _joint = self._dci.find_articulation_joint(self._articulation, dof_name)
                    self._joints[dof_name] = {"joint": _joint,
                                              "type": self._dci.get_joint_type(_joint),
                                              "dof": self._dci.find_articulation_dof(self._articulation, dof_name), 
                                              "lower": lower_limits[i], 
                                              "upper": upper_limits[i], 
                                              "has_limits": has_limits[i]}

        if not self._joints:
            print("[Warning][semu.robotics.ros_bridge] RosControlFollowJointTrajectory: no joints found in {}".format(path))
            self.started = False

    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float
        """
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self._dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def _get_joint_position(self, name: str) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str

        :return: The current position of the joint
        :rtype: float
        """
        position = self._dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            return position * get_stage_units()
        return position

    def _on_goal(self, goal_handle: 'actionlib.ServerGoalHandle') -> None:
        """Callback function for handling new goal requests

        :param goal_handle: The goal handle
        :type goal_handle: actionlib.ServerGoalHandle
        """
        goal = goal_handle.get_goal()

        # reject if joints don't match
        for name in goal.trajectory.joint_names:
            if name not in self._joints:
                print("[Warning][semu.robotics.ros_bridge] RosControlFollowJointTrajectory: joints don't match ({} not in {})" \
                    .format(name, list(self._joints.keys())))
                self._action_result_message.error_code = self._action_result_message.INVALID_JOINTS
                goal_handle.set_rejected(self._action_result_message, "")
                return

        # reject if there is an active goal
        if self._action_goal is not None:
            print("[Warning][semu.robotics.ros_bridge] RosControlFollowJointTrajectory: multiple goals not supported")
            self._action_result_message.error_code = self._action_result_message.INVALID_GOAL
            goal_handle.set_rejected(self._action_result_message, "")
            return

        # check initial position
        if goal.trajectory.points[0].time_from_start.to_sec():
            initial_point = JointTrajectoryPoint(positions=[self._get_joint_position(name) for name in goal.trajectory.joint_names],
                                                 time_from_start=rospy.Duration())
            goal.trajectory.points.insert(0, initial_point)
        
        # store goal data
        self._action_goal = goal
        self._action_goal_handle = goal_handle
        self._action_point_index = 1
        self._action_start_time = rospy.get_time()
        self._action_feedback_message.joint_names = list(goal.trajectory.joint_names)

        goal_handle.set_accepted()

    def _on_cancel(self, goal_handle: 'actionlib.ServerGoalHandle') -> None:
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: actionlib.ServerGoalHandle
        """
        if self._action_goal is None:
            goal_handle.set_rejected()
            return
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        goal_handle.set_canceled()

    def update_step(self, dt: float) -> None:
        """Kit update step

        :param dt: The delta time
        :type dt: float
        """
        pass

    def physics_step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # update articulation
        if self._action_goal is not None and self._action_goal_handle is not None:
            # end of trajectory
            if self._action_point_index >= len(self._action_goal.trajectory.points):
                self._action_goal = None
                self._action_result_message.error_code = self._action_result_message.SUCCESSFUL
                if self._action_goal_handle is not None:
                    self._action_goal_handle.set_succeeded(self._action_result_message)
                    self._action_goal_handle = None
                return
            
            previous_point = self._action_goal.trajectory.points[self._action_point_index - 1]
            current_point = self._action_goal.trajectory.points[self._action_point_index]
            time_passed = rospy.get_time() - self._action_start_time

            # set target using linear interpolation
            if time_passed <= current_point.time_from_start.to_sec():
                ratio = (time_passed - previous_point.time_from_start.to_sec()) \
                      / (current_point.time_from_start.to_sec() - previous_point.time_from_start.to_sec())
                self._dci.wake_up_articulation(self._articulation)
                for i, name in enumerate(self._action_goal.trajectory.joint_names):
                    side = -1 if current_point.positions[i] < previous_point.positions[i] else 1
                    target_position = previous_point.positions[i] \
                                    + side * ratio * abs(current_point.positions[i] - previous_point.positions[i])
                    self._set_joint_position(name, target_position)
            # send feedback
            else:
                self._action_point_index += 1
                self._action_feedback_message.actual.positions = [self._get_joint_position(name) \
                    for name in self._action_goal.trajectory.joint_names]
                self._action_feedback_message.actual.time_from_start = rospy.Duration.from_sec(time_passed)
                if self._action_goal_handle is not None:
                    self._action_goal_handle.publish_feedback(self._action_feedback_message)


class RosControllerGripperCommand(RosController):
    def __init__(self, 
                 usd_context: 'omni.usd._usd.UsdContext', 
                 schema: 'ROSSchema.RosBridgeComponent', 
                 dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """GripperCommand interface

        :param usd_context: The USD context
        :type usd_context: omni.usd._usd.UsdContext
        :param schema: The ROS bridge schema
        :type schema: ROSSchema.RosBridgeComponent
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(usd_context, schema)
        
        self._dci = dci

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._action_server = None

        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        # TODO: add to schema?
        self._action_timeout = 10.0
        self._action_position_threshold = 0.001
        self._action_previous_position_sum = float("inf")

        # feedback / result
        self._action_result_message = control_msgs.msg.GripperCommandResult()
        self._action_feedback_message = control_msgs.msg.GripperCommandFeedback()
    
    def start(self) -> None:
        """Start the action server
        """
        print("[Info][semu.robotics.ros_bridge] RosControllerGripperCommand: starting {}" \
            .format(self._schema.__class__.__name__))

        # get attributes and relationships
        action_namespace = self._schema.GetActionNamespaceAttr().Get()
        controller_name = self._schema.GetControllerNameAttr().Get()

        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        if not len(relationships):
            print("[Warning][semu.robotics.ros_bridge] RosControllerGripperCommand: empty relationships")
            return
        elif len(relationships) == 1:
            print("[Warning][semu.robotics.ros_bridge] RosControllerGripperCommand: relationship is not a group")
            return

        # check for articulation API
        stage = self._usd_context.get_stage()
        path = relationships[0].GetPrimPath().pathString
        if not stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxArticulationAPI):
            print("[Warning][semu.robotics.ros_bridge] RosControllerGripperCommand: {} doesn't have PhysxArticulationAPI".format(path))
            return
        
        # start action server
        self._shutdown_action_server()
        self._action_server = actionlib.ActionServer(controller_name + action_namespace, 
                                                     control_msgs.msg.GripperCommandAction,
                                                     goal_cb=self._on_goal,
                                                     cancel_cb=self._on_cancel,
                                                     auto_start=False)
        try:
            self._action_server.start()
        except ConnectionRefusedError:
            print("[Error][semu.robotics.ros_bridge] RosControllerGripperCommand: action server {} not started" \
                .format(controller_name + action_namespace))
            return
        print("[Info][semu.robotics.ros_bridge] RosControllerGripperCommand: register action {}" \
            .format(controller_name + action_namespace))

        self.started = True

    def stop(self) -> None:
        """Stop the action server
        """
        super().stop()
        self._articulation = _dynamic_control.INVALID_HANDLE
        self._shutdown_action_server()
        self._action_goal_handle = None
        self._action_goal = None

    def _shutdown_action_server(self) -> None:
        """Shutdown the action server
        """
        # omni.isaac.ros_bridge/noetic/lib/python3/dist-packages/actionlib/action_server.py
        print("[Info][semu.robotics.ros_bridge] RosControllerGripperCommand: destroy action server {}" \
            .format(self._schema.GetPrim().GetPath()))
        if self._action_server:
            if self._action_server.started:
                self._action_server.started = False

                self._action_server.status_pub.unregister()
                self._action_server.result_pub.unregister()
                self._action_server.feedback_pub.unregister()

                self._action_server.goal_sub.unregister()
                self._action_server.cancel_sub.unregister()

            del self._action_server
            self._action_server = None

    def _init_articulation(self) -> None:
        """Initialize the articulation and register joints
        """
        # get articulation
        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        path = relationships[0].GetPrimPath().pathString
        self._articulation = self._dci.get_articulation(path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[Warning][semu.robotics.ros_bridge] RosControllerGripperCommand: {} is not an articulation".format(path))
            return
        
        dof_props = self._dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        # TODO: move to another relationship in the schema
        paths = [relationship.GetPrimPath().pathString for relationship in relationships[1:]]

        for i in range(self._dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self._dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                # add only required joints
                if self._dci.get_dof_path(dof_ptr) in paths:
                    dof_name = self._dci.get_dof_name(dof_ptr)
                    if dof_name not in self._joints:
                        _joint = self._dci.find_articulation_joint(self._articulation, dof_name)
                        self._joints[dof_name] = {"joint": _joint,
                                                  "type": self._dci.get_joint_type(_joint),
                                                  "dof": self._dci.find_articulation_dof(self._articulation, dof_name), 
                                                  "lower": lower_limits[i], 
                                                  "upper": upper_limits[i], 
                                                  "has_limits": has_limits[i]}

        if not self._joints:
            print("[Warning][semu.robotics.ros_bridge] RosControllerGripperCommand: no joints found in {}".format(path))
            self.started = False
    
    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float
        """
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self._dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def _get_joint_position(self, name: str) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str

        :return: The current position of the joint
        :rtype: float
        """
        position = self._dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            return position * get_stage_units()
        return position

    def _on_goal(self, goal_handle: 'actionlib.ServerGoalHandle') -> None:
        """Callback function for handling new goal requests

        :param goal_handle: The goal handle
        :type goal_handle: actionlib.ServerGoalHandle
        """
        goal = goal_handle.get_goal()

        # reject if there is an active goal
        if self._action_goal is not None:
            print("[Warning][semu.robotics.ros_bridge] RosControllerGripperCommand: multiple goals not supported")
            goal_handle.set_rejected()
            return

        # store goal data
        self._action_goal = goal
        self._action_goal_handle = goal_handle
        self._action_start_time = rospy.get_time()
        self._action_previous_position_sum = float("inf")

        goal_handle.set_accepted()

    def _on_cancel(self, goal_handle: 'actionlib.ServerGoalHandle') -> None:
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: actionlib.ServerGoalHandle
        """
        if self._action_goal is None:
            goal_handle.set_rejected()
            return
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_previous_position_sum = float("inf")
        goal_handle.set_canceled()

    def update_step(self, dt: float) -> None:
        """Kit update step

        :param dt: The delta time
        :type dt: float
        """
        pass

    def physics_step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # update articulation
        if self._action_goal is not None and self._action_goal_handle is not None:
            target_position = self._action_goal.command.position
            # set target
            self._dci.wake_up_articulation(self._articulation)
            for name in self._joints:
                self._set_joint_position(name, target_position)
            # end (position reached)
            position = 0
            current_position_sum = 0
            position_reached = True
            for name in self._joints:
                position = self._get_joint_position(name)
                current_position_sum += position
                if abs(position - target_position) > self._action_position_threshold:
                    position_reached = False
                    break
            if position_reached:
                self._action_goal = None
                self._action_result_message.position = position
                self._action_result_message.stalled = False
                self._action_result_message.reached_goal = True
                if self._action_goal_handle is not None:
                    self._action_goal_handle.set_succeeded(self._action_result_message)
                    self._action_goal_handle = None
                return
            # end (stalled)
            if abs(current_position_sum - self._action_previous_position_sum) < 1e-6:
                self._action_goal = None
                self._action_result_message.position = position
                self._action_result_message.stalled = True
                self._action_result_message.reached_goal = False
                if self._action_goal_handle is not None:
                    self._action_goal_handle.set_succeeded(self._action_result_message)
                    self._action_goal_handle = None
                return
            self._action_previous_position_sum = current_position_sum
            # end (timeout)
            time_passed = rospy.get_time() - self._action_start_time
            if time_passed >= self._action_timeout:
                self._action_goal = None
                if self._action_goal_handle is not None:
                    self._action_goal_handle.set_aborted()
                    self._action_goal_handle = None
            # TODO: send feedback
            # self._action_goal_handle.publish_feedback(self._action_feedback_message)
