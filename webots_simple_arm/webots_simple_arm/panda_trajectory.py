# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
import os
from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_core.trajectory_follower import TrajectoryFollower
from webots_ros2_core.webots.controller import Node
from sensor_msgs.msg import JointState

MAX_REALTIME_FACTOR = 20

class WebotsRoboticArmNode(WebotsNode):
    """
    Extends WebotsNode to allow easy integration with robotic arms.

    Args:
        name (WebotsNode): Webots Robot node.
        args (dict): Arguments passed to ROS2 base node.
        prefix (str): Prefix passed to JointStatePublisher.
    """

    def __init__(self, args):
        super().__init__("webots_controller_node", args)
        self.__frame_id = 'joint_states'
        self.__joint_prefix = ''

        self.__sensors = []
        self.__timestep = int(self.robot.getBasicTimeStep())
        self.__last_joint_states = None
        self.__previous_time = 0
        self.__previous_position = []
        self.__joint_names = []

        self.declare_parameter("controller_name", 'panda_arm_controller')
        self.declare_parameter('prefix', '')

        self.param_controller_name = self.get_parameter("controller_name")
        self.param_prefix = self.get_parameter('prefix')
        self.__timer = self.create_timer((1 / MAX_REALTIME_FACTOR) * 1e-3 * self.timestep, self.__timer_callback)

        for i in range(self.robot.getNumberOfDevices()):
            device = self.robot.getDeviceByIndex(i)
            if device.getNodeType() == Node.POSITION_SENSOR:
                motor = device.getMotor()
                name = motor.getName() if motor is not None else device.getName()
                self.__joint_names.append(name)
                self.__sensors.append(device)
                self.__previous_position.append(0)
                device.enable(self.__timestep)
        self.__publisher = self.create_publisher(JointState, 'joint_states', 1)

        # os.environ['WEBOTS_ROBOT_NAME'] = self.param_robot_name
        self.__trajectory_follower = TrajectoryFollower(self.robot, self, self.param_prefix.value, self.param_controller_name.value)

        self.get_logger().info('Initializing robotic arm node with prefix = "%s"' %
                               self.param_prefix.value)
    def __timer_callback(self):
        self.publish()

    def publish(self):
        """Publish the 'joint_states' topic with up to date value."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.__frame_id
        msg.name = [s + self.__joint_prefix for s in self.__joint_names]
        msg.position = []
        time_difference = self.robot.getTime() - self.__previous_time
        for i in range(len(self.__sensors)):
            value = self.__sensors[i].getValue()
            msg.position.append(value)
            msg.velocity.append((value - self.__previous_position[i]) /
                                time_difference if time_difference > 0 else 0.0)
            self.__previous_position[i] = value
        msg.effort = [0.0] * 6
        self.__publisher.publish(msg)
        self.__last_joint_states = msg
        self.__previous_time = self.robot.getTime()

def main(args=None):
    rclpy.init(args=args)

    robotic_arm = WebotsRoboticArmNode(args=args)
    # robotic_arm.start_device_manager()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = rclpy.executors.MultiThreadedExecutor()

    rclpy.spin(robotic_arm)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
