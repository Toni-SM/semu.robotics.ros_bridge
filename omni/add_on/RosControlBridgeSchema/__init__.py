from pxr import Tf

from . import _rosControlBridgeSchema

Tf.PrepareModule(_rosControlBridgeSchema, locals())
del Tf
