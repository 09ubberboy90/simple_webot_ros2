# Compared to the typical `from controller import Node` we added parent modules.
from webots_ros2_driver_webots.controller import Node
from std_msgs.msg import Float32
import rclpy
import rclpy.node


class PandaDriver():
    # The `init` method is called only once the driver is initialized.
    # You will always get two arguments in the `init` method.
    # - The `webots_node` argument contains a reference on a Supervisor instance.
    # - The `properties` argument is a dictionary created from the XML tags.
    def init(self, webots_node, properties):
        # Unfortunately, we cannot get an instance of the parent ROS node.
        # However, we can create a new one.
        rclpy.init(args=None)
        self._node = rclpy.node.Node('panda_driver')

        self._robot = webots_node.robot
        self._publisher = self._node.create_publisher(Float32, 'clock', 1)

        # self._node.get_logger().info(f'properties: {properties}')

        # self._node.get_logger().info(f'basic timestep: {int(webots_node.robot.getBasicTimeStep())}')
        # self._node.get_logger().info(f'robot name: {webots_node.robot.getName()}')
        # self._node.get_logger().info(f'is robot? {webots_node.robot.getType() == Node.ROBOT}')



    # The `step` method is called at every step.
    def step(self):
        self._publisher.publish(Float32(data=self._robot.getTime()))
