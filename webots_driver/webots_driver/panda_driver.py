from std_msgs.msg import Float32
import rclpy
import rclpy.node
import signal
import os

class PandaDriver():
    # Is needed even if it does nothing
    def init(self, webots_node, properties):
        # Unfortunately, we cannot get an instance of the parent ROS node.
        # However, we can create a new one.
        rclpy.init(args=None)
        self._node = rclpy.node.Node('panda_driver')

