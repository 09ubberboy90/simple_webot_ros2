from std_msgs.msg import Float32
import rclpy
import rclpy.node
import signal
import os

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
        dir_path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(dir_path, "..", "pic.jpg")
        signal.signal(signal.SIGINT, lambda sig, frame: self.on_kill())
        signal.signal(signal.SIGINT, lambda sig, frame: self.on_kill())

        # self._node.get_logger().info(f'properties: {properties}')

        # self._node.get_logger().info(f'basic timestep: {int(webots_node.robot.getBasicTimeStep())}')
        # self._node.get_logger().info(f'robot name: {webots_node.robot.getName()}')
        # self._node.get_logger().info(f'is robot? {webots_node.robot.getType() == Node.ROBOT}')



    # The `step` method is called at every step.
    def step(self):
        self._publisher.publish(Float32(data=self._robot.getTime()))

    def on_kill(self):
        print("Killing")
        dir_path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(dir_path, "..", "pic.jpg")
        self._robot.exportImage(os.path.join(dir_path, path), 100)
        print(self._robot.getTime())