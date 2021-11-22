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
        # This will print the parameter from the URDF file.
        #
        #     `{ 'parameterExample': 'someValue' }`
        #
        print('properties:', properties)

        # The robot property is a reference to a Supervisor instance.
        # It allows you to access the standard Webots API.
        # See: https://cyberbotics.com/doc/reference/supervisor
        print('basic timestep:', int(webots_node.robot.getBasicTimeStep()))
        print('robot name:', webots_node.robot.getName())
        print('is robot?', webots_node.robot.getType() == Node.ROBOT)

        self.__robot = webots_node.robot

        # Unfortunately, we cannot get an instance of the parent ROS node.
        # However, we can create a new one.
        rclpy.init(args=None)
        self.__node = rclpy.node.Node('plugin_node_example')
        self.__publisher = self.__node.create_publisher(Float32, 'custom_time', 1)

    # The `step` method is called at every step.
    def step(self):
        self.__publisher.publish(Float32(data=self.__robot.getTime()))
