#BSD 3-Clause License
#
#Copyright (c) 2021, Florent Audonnet
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
#3. Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rclpy
import os, sys
from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_core.utils import append_webots_python_lib_to_path
from webots_ros2_core.trajectory_follower import TrajectoryFollower
from sensor_msgs.msg import JointState
try:
    append_webots_python_lib_to_path()
    from controller import Node
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e

class PandaNode(WebotsNode):
    def __init__(self, args=None):
        super().__init__("panda",args)
        os.environ['WEBOTS_ROBOT_NAME'] = "panda"
        self.trajectoryFollower = TrajectoryFollower(self.robot, self, joint_prefix="")
        self.nodes = {}
        for i in range(self.robot.getNumberOfDevices()):
            device = self.robot.getDeviceByIndex(i)
            if device.getNodeType() == Node.POSITION_SENSOR:
                motor = device.getMotor()
                name = motor.getName() if motor is not None else device.getName()
                self.nodes[name] = motor
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)

    def listener_callback(self, msg:JointState):
        for idx, name in enumerate(msg.name):
            self.nodes[name].setPosition(msg.position[idx])

def main(args=None):
    rclpy.init(args=args)

    panda_controller = PandaNode(args=args)
    panda_controller.start_device_manager()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = rclpy.executors.MultiThreadedExecutor()

    rclpy.spin(panda_controller, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()