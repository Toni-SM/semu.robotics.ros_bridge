# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import semu.robotics.ros_bridge

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestROSBridge(omni.kit.test.AsyncTestCaseFailOnLogError):
    # Before running each test
    async def setUp(self):
        pass

    # After running each test
    async def tearDown(self):
        pass

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_ros_bridge(self):
        print("test_ros_bridge - TODO")
        pass
        