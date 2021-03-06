import rclpy
import os
import sys
from webots_ros2_core.webots_node import WebotsNode
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import GetEntityState, GetModelList
from gazebo_msgs.msg import EntityState
import pyquaternion 
from rosgraph_msgs.msg import Clock
from rclpy.time import Time

class SpawnerNode(WebotsNode):
    def __init__(self, args=None):
        super().__init__("spawner", args)

        self.package_dir = get_package_share_directory('webots_driver')
        self.children = self.robot.getRoot().getField("children")
        self.entity = self.create_service(
            GetEntityState, 'get_entity_state', self.get_entity_state)
        self.model = self.create_service(
            GetModelList, 'get_model_list', self.get_model_list)
        self.publisher_ = self.create_publisher(Clock, 'clock', 10)
        timer_period = self.robot.getBasicTimeStep()/1000 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.objs = {}
        # self.robot.simulationSetMode(self.robot.SIMULATION_MODE_FAST)
        self.spawn_obj("worlds/Table.wbo", rotation = [1,0,0,1.57], position=[0.6, 0, -0.2])
        # for i in range(-5,6):
        #     for j in range(-5,6):
        #         print(f"Spawned at {i*0.5} {j*0.5}")
        #         self.spawn_obj("worlds/Cube.wbo", position = [i, 0, j])
        for x in range(2, 5):
            for y in range(-3, 4):
                self.spawn_obj("worlds/Cube.wbo", [x/10, y/10, 0.45])

    def timer_callback(self):
        msg = Clock()
        time = self.robot.getTime()
        msg.clock = Time(nanoseconds=time*1e9).to_msg()
        self.publisher_.publish(msg)

    def spawn_obj(self, path, position=[0, 0, 0], rotation = [0,1,0,0]):
        self.children.importMFNode(0, os.path.join(self.package_dir, path))
        obj = self.children.getMFNode(0)
        obj.getField("translation").setSFVec3f(position)
        obj.getField("rotation").setSFRotation(rotation)
        self.objs[obj.getField("name").getSFString()] = obj

    def get_model_list(self, request: GetModelList.Request, response: GetModelList.Response):
        response.model_names = list(self.objs.keys())
        response.success = True
        return response

    def get_entity_state(self, request: GetEntityState.Request, response: GetEntityState.Response):
        obj = self.objs.get(request.name)
        success = True
        if obj is None:
            response.success = False
            return response
        state = EntityState()
        state.name = request.name
        pose = Pose()
        try:    
            pose.position = self.get_postion(obj)
            pose.orientation = self.get_rotation(obj)
        except: # object got deleted
            success = False
        finally:    
            state.pose = pose
            response.state = state
            response.success = success
        return response

    def get_postion(self, obj):
        position = Point()
        obj_pose = obj.getField("translation").getSFVec3f()
        position.x = obj_pose[0]
        position.y = obj_pose[1]
        position.z = obj_pose[2]
        return position

    def get_rotation(self, obj):
        rotation = Quaternion()
        obj_rot = obj.getField("rotation").getSFRotation()
        quat = pyquaternion.Quaternion(axis=obj_rot[:3], angle=obj_rot[3])
        rotation.x = float(quat.x)
        rotation.y = float(quat.y)
        rotation.z = float(quat.z)
        rotation.w = float(quat.w)
        return rotation


def main(args=None):
    rclpy.init(args=args)
    os.environ['WEBOTS_ROBOT_NAME'] = "spawner"

    spawner = SpawnerNode(args=args)

    rclpy.spin(spawner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
