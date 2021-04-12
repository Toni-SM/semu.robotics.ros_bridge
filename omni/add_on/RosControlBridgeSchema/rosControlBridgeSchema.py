from pxr import Sdf
import omni.isaac.RosBridgeSchema as ROSSchema


class RosControlFollowJointTrajectory(ROSSchema.RosBridgeComponent):
    def __init__(self, prim):
        # RosBridgeComponent:
        # __init__(_object*, pxrInternal_v0_19__pxrReserved__::UsdSchemaBase schemaObj)
        # __init__(_object*, pxrInternal_v0_19__pxrReserved__::UsdPrim prim)
        # __init__(_object*)
        super().__init__(prim)
        
    @staticmethod
    def Define(stage, path):
        prim = stage.DefinePrim(path, "RosControlFollowJointTrajectory")
        return RosControlFollowJointTrajectory(prim)

    def CreateArticulationPrimRel(self):
        self.GetPrim().CreateRelationship("articulationPrim")

    def CreateControllerNameAttr(self, value):
        self.GetPrim().CreateAttribute("controllerName", Sdf.ValueTypeNames.String, False).Set(value)

    def CreateActionNamespaceAttr(self, value):
        self.GetPrim().CreateAttribute("actionNamespace", Sdf.ValueTypeNames.String, False).Set(value)

    @staticmethod
    def Get(stage, path):
        # Get(pxrInternal_v0_19__pxrReserved__::TfWeakPtr<pxrInternal_v0_19__pxrReserved__::UsdStage> stage, pxrInternal_v0_19__pxrReserved__::SdfPath path)
        prim = stage.GetPrimAtPath(path)
        return RosControlFollowJointTrajectory(prim)

    def GetArticulationPrimRel(self):
        return self.GetPrim().GetRelationship("articulationPrim")

    def GetControllerNameAttr(self):
        return self.GetPrim().GetAttribute("controllerName")

    def GetActionNamespaceAttr(self):
        return self.GetPrim().GetAttribute("actionNamespace")

    @staticmethod
    def GetSchemaAttributeNames(includeInherited=True):
        names = []
        if includeInherited: 
            names = ROSSchema.RosBridgeComponent.GetSchemaAttributeNames(includeInherited)
        return names + ["controllerName", "actionNamespace"]


class RosControlGripperCommand(ROSSchema.RosBridgeComponent):
    def __init__(self, prim):
        # RosBridgeComponent:
        # __init__(_object*, pxrInternal_v0_19__pxrReserved__::UsdSchemaBase schemaObj)
        # __init__(_object*, pxrInternal_v0_19__pxrReserved__::UsdPrim prim)
        # __init__(_object*)
        super().__init__(prim)
        
    @staticmethod
    def Define(stage, path):
        prim = stage.DefinePrim(path, "RosControlGripperCommand")
        return RosControlGripperCommand(prim)

    def CreateArticulationPrimRel(self):
        self.GetPrim().CreateRelationship("articulationPrim")

    def CreateControllerNameAttr(self, value):
        self.GetPrim().CreateAttribute("controllerName", Sdf.ValueTypeNames.String, False).Set(value)

    def CreateActionNamespaceAttr(self, value):
        self.GetPrim().CreateAttribute("actionNamespace", Sdf.ValueTypeNames.String, False).Set(value)

    @staticmethod
    def Get(stage, path):
        # Get(pxrInternal_v0_19__pxrReserved__::TfWeakPtr<pxrInternal_v0_19__pxrReserved__::UsdStage> stage, pxrInternal_v0_19__pxrReserved__::SdfPath path)
        prim = stage.GetPrimAtPath(path)
        return RosControlGripperCommand(prim)

    def GetArticulationPrimRel(self):
        return self.GetPrim().GetRelationship("articulationPrim")

    def GetControllerNameAttr(self):
        return self.GetPrim().GetAttribute("controllerName")

    def GetActionNamespaceAttr(self):
        return self.GetPrim().GetAttribute("actionNamespace")

    @staticmethod
    def GetSchemaAttributeNames(includeInherited=True):
        names = []
        if includeInherited: 
            names = ROSSchema.RosBridgeComponent.GetSchemaAttributeNames(includeInherited)
        return names + ["controllerName", "actionNamespace"]
