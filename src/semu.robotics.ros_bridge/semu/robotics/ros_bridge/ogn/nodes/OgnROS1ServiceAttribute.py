from typing import Any

import json
import asyncio
import threading

import omni
import carb
from pxr import Usd, Gf
from omni.isaac.dynamic_control import _dynamic_control

import rospy


class InternalState:
    def __init__(self):
        """Internal state for the ROS1 Attribute node
        """
        self.initialized = False

        self.dci = None
        self.usd_context = None
        self.timeline_event = None

        self.GetPrims = None
        self.GetPrimAttributes = None
        self.GetPrimAttribute = None
        self.SetPrimAttribute = None

        self.srv_prims = None
        self.srv_attributes = None
        self.srv_getter = None
        self.srv_setter = None

        self._event = threading.Event()
        self._event.set()

        self.__event_timeout = carb.settings.get_settings().get("/exts/semu.robotics.ros_bridge/eventTimeout")
        self.__set_attribute_using_asyncio = \
            carb.settings.get_settings().get("/exts/semu.robotics.ros_bridge/setAttributeUsingAsyncio")
        print("[Info][semu.robotics.ros_bridge] ROS1 Attribute: asyncio: {}".format(self.__set_attribute_using_asyncio))
        print("[Info][semu.robotics.ros_bridge] ROS1 Attribute: event timeout: {}".format(self.__event_timeout))

    def on_timeline_event(self, event: 'carb.events._events.IEvent') -> None:
        """Handle the timeline event

        :param event: Event
        :type event: carb.events._events.IEvent
        """
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            pass
        elif event.type == int(omni.timeline.TimelineEventType.PAUSE):
            pass
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self.initialized = False
            self.shutdown_services()

    def shutdown_services(self) -> None:
        """Shutdown the services
        """
        if self.srv_prims is not None:
            print("[Info][semu.robotics.ros_bridge] RosAttribute: unregister srv: {}".format(self.srv_prims.resolved_name))
            self.srv_prims.shutdown()
            self.srv_prims = None
        if self.srv_getter is not None:
            print("[Info][semu.robotics.ros_bridge] RosAttribute: unregister srv: {}".format(self.srv_getter.resolved_name))
            self.srv_getter.shutdown()
            self.srv_getter = None
        if self.srv_attributes is not None:
            print("[Info][semu.robotics.ros_bridge] RosAttribute: unregister srv: {}".format(self.srv_attributes.resolved_name))
            self.srv_attributes.shutdown()
            self.srv_attributes = None
        if self.srv_setter is not None:
            print("[Info][semu.robotics.ros_bridge] RosAttribute: unregister srv: {}".format(self.srv_setter.resolved_name))
            self.srv_setter.shutdown()
            self.srv_setter = None

    async def _set_attribute(self, attribute: 'pxr.Usd.Attribute', attribute_value: Any) -> None:
        """Set the attribute value using asyncio

        :param attribute: The prim's attribute to set
        :type attribute: pxr.Usd.Attribute
        :param attribute_value: The attribute value
        :type attribute_value: Any
        """
        ret = attribute.Set(attribute_value)

    def process_setter_request(self, request: 'SetPrimAttribute.SetPrimAttributeRequest') -> 'SetPrimAttribute.SetPrimAttributeResponse':
        """Process the setter request

        :param request: The service request
        :type request: SetPrimAttribute.SetPrimAttributeRequest

        :return: The service response
        :rtype: SetPrimAttribute.SetPrimAttributeResponse
        """
        response = self.SetPrimAttribute.SetPrimAttributeResponse()
        response.success = False
        stage = self.usd_context.get_stage()
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
                    print("[Error][semu.robotics.ros_bridge] ROS1 Attribute: invalid value: {}".format(request.value))
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
                    print("[Error][semu.robotics.ros_bridge] ROS1 Attribute: srv {} request for {} ({}: {}): {}" \
                        .format(self.srv_setter.resolved_name, request.path, request.attribute, value, e))
                    response.success = False
                    response.message = str(e)
            else:
                response.message = "Prim has not attribute {}".format(request.attribute)
        else:
            response.message = "Invalid prim ({})".format(request.path)
        return response
        
    def process_getter_request(self, request: 'GetPrimAttribute.GetPrimAttributeRequest') -> 'GetPrimAttribute.GetPrimAttributeResponse':
        """Process the getter request

        :param request: The service request
        :type request: GetPrimAttribute.GetPrimAttributeRequest

        :return: The service response
        :rtype: GetPrimAttribute.GetPrimAttributeResponse
        """
        response = self.GetPrimAttribute.GetPrimAttributeResponse()
        response.success = False
        stage = self.usd_context.get_stage()
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
                        print("[Warning][semu.robotics.ros_bridge] ROS1 Attribute: Unknow attribute type {}" \
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
                        print("[Warning][semu.robotics.ros_bridge] ROS1 Attribute: Unknow {}: {}" \
                            .format(type(attribute.Get()), attribute.Get()))
                        print("  |-- Please, report a new issue (https://github.com/Toni-SM/semu.robotics.ros_bridge/issues)")
                        response.success = False
                        response.message = "Unknow type {}".format(type(attribute.Get()))
            else:
                response.message = "Prim has not attribute {}".format(request.attribute)
        else:
            response.message = "Invalid prim ({})".format(request.path)
        return response

    def process_attributes_request(self, request: 'GetPrimAttributes.GetPrimAttributesRequest') -> 'GetPrimAttributes.GetPrimAttributesResponse':
        """Process the 'get all attributes' request

        :param request: The service request
        :type request: GetPrimAttributes.GetPrimAttributesRequest

        :return: The service response
        :rtype: GetPrimAttributes.GetPrimAttributesResponse
        """
        response = self.GetPrimAttributes.GetPrimAttributesResponse()
        response.success = False
        stage = self.usd_context.get_stage()
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
        return response
    
    def process_prims_request(self, request: 'GetPrims.GetPrimsRequest') -> 'GetPrims.GetPrimsResponse':
        """Process the 'get all prims' request

        :param request: The service request
        :type request: GetPrims.GetPrimsRequest

        :return: The service response
        :rtype: GetPrims.GetPrimsResponse
        """
        response = self.GetPrims.GetPrimsResponse()
        response.success = False
        stage = self.usd_context.get_stage()
        # get prims
        if not request.path or stage.GetPrimAtPath(request.path).IsValid():
            path = request.path if request.path else "/"
            for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath(path)):
                response.paths.append(str(prim.GetPath()))
                response.types.append(prim.GetTypeName())
                response.success = True
        else:
            response.message = "Invalid search path ({})".format(request.path)
        return response

    def step(self, dt: float) -> None:
        """Update step

        :param dt: Delta time
        :type dt: float
        """
        if not self.initialized:
            return
        if self.__set_attribute_using_asyncio:
            return
        if self.dci.is_simulating():
            if not self._event.is_set():
                if self._attribute is not None:
                    ret = self._attribute.Set(self._value)
                self._event.set()


class OgnROS1ServiceAttribute:
    """This node provides the services to list, read and write prim's attributes
    """
    @staticmethod
    def initialize(graph_context, node):
        pass

    @staticmethod
    def internal_state() -> InternalState:
        return InternalState()

    @staticmethod
    def compute(db) -> bool:
        if db.internal_state.initialized:
            db.internal_state.step(0)
        else:
            try:
                db.internal_state.usd_context = omni.usd.get_context()
                db.internal_state.dci = _dynamic_control.acquire_dynamic_control_interface()

                if db.internal_state.timeline_event is None:
                    timeline = omni.timeline.get_timeline_interface()
                    db.internal_state.timeline_event = timeline.get_timeline_event_stream() \
                        .create_subscription_to_pop(db.internal_state.on_timeline_event)

                def get_service_name(namespace, name):
                    service_namespace = namespace if namespace.startswith("/") else "/" + namespace
                    service_name = name if name.startswith("/") else "/" + name
                    return service_namespace if namespace else "" + service_name

                # load service definitions
                from add_on_msgs.srv import _GetPrims
                from add_on_msgs.srv import _GetPrimAttributes
                from add_on_msgs.srv import _GetPrimAttribute
                from add_on_msgs.srv import _SetPrimAttribute
                
                db.internal_state.GetPrims = _GetPrims
                db.internal_state.GetPrimAttributes = _GetPrimAttributes
                db.internal_state.GetPrimAttribute = _GetPrimAttribute
                db.internal_state.SetPrimAttribute = _SetPrimAttribute

                # create services
                db.internal_state.shutdown_services()

                service_name = get_service_name(db.inputs.nodeNamespace, db.inputs.primsServiceName)
                db.internal_state.srv_prims = rospy.Service(service_name, 
                                                            db.internal_state.GetPrims.GetPrims, 
                                                            db.internal_state.process_prims_request)
                print("[Info][semu.robotics.ros_bridge] ROS1 Attribute: register srv: {}".format(db.internal_state.srv_prims.resolved_name))

                service_name = get_service_name(db.inputs.nodeNamespace, db.inputs.getAttributesServiceName)
                db.internal_state.srv_attributes = rospy.Service(service_name, 
                                                                 db.internal_state.GetPrimAttributes.GetPrimAttributes, 
                                                                 db.internal_state.process_attributes_request)
                print("[Info][semu.robotics.ros_bridge] ROS1 Attribute: register srv: {}".format(db.internal_state.srv_attributes.resolved_name))

                service_name = get_service_name(db.inputs.nodeNamespace, db.inputs.getAttributeServiceName)
                db.internal_state.srv_getter = rospy.Service(service_name, 
                                                             db.internal_state.GetPrimAttribute.GetPrimAttribute, 
                                                             db.internal_state.process_getter_request)
                print("[Info][semu.robotics.ros_bridge] ROS1 Attribute: register srv: {}".format(db.internal_state.srv_getter.resolved_name))

                service_name = get_service_name(db.inputs.nodeNamespace, db.inputs.setAttributeServiceName)
                db.internal_state.srv_setter = rospy.Service(service_name, 
                                                             db.internal_state.SetPrimAttribute.SetPrimAttribute, 
                                                             db.internal_state.process_setter_request)
                print("[Info][semu.robotics.ros_bridge] ROS1 Attribute: register srv: {}".format(db.internal_state.srv_setter.resolved_name))
                
            except Exception as error:
                print("[Error][semu.robotics.ros_bridge] ROS1 Attribute: error: {}".format(error))
                db.log_error(str(error))
                db.internal_state.initialized = False
                return False

            db.internal_state.initialized = True

        return True
