//BSD 3-Clause License
//
//Copyright (c) 2021, Florent Audonnet
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//1. Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//2. Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
//3. Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "moveit.hpp"
using std::placeholders::_1;
using namespace Eigen;
using namespace std::chrono_literals;

enum gripper_state
{
    opened = 35,
    closed = 0
};

bool wait_for_exec(moveit::planning_interface::MoveGroupInterface *move_group, std::shared_ptr<sim_action_server::ActionServer> server)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for (int i = 0; i < 10; i++)
    {
        // 10 tries to plan otherwise give up
        bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Plan %d %s", i, success ? "SUCCEEDED" : "FAILED");

        if (success)
        {
            std::thread([move_group, plan]() { move_group->execute(plan); }).detach();
            //move_group->asyncExecute(plan);
            return server->execute_plan(plan.trajectory_.joint_trajectory);
        }
    }
    auto pose = move_group->getPoseTarget().pose.position;
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to find a valid path to %f, %f, %f", pose.x, pose.y, pose.z);
    throw "Couldn't plan a path";
    return false;
}

bool change_gripper(moveit::planning_interface::MoveGroupInterface *hand_move_group, std::shared_ptr<sim_action_server::ActionServer> server, gripper_state state)
{
    auto current_state = hand_move_group->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(current_state->getJointModelGroup("hand"), joint_group_positions);
    float pose = (float)state / 1000;
    joint_group_positions[0] = pose;
    joint_group_positions[1] = pose;
    hand_move_group->setJointValueTarget(joint_group_positions);
    return wait_for_exec(hand_move_group, server);
}

bool goto_pose(moveit::planning_interface::MoveGroupInterface *move_group, std::shared_ptr<sim_action_server::ActionServer> server, geometry_msgs::msg::Pose pose)
{
    move_group->setPoseTarget(pose);
    return wait_for_exec(move_group, server);
}
bool goto_joint_pose(moveit::planning_interface::MoveGroupInterface *move_group, std::shared_ptr<sim_action_server::ActionServer> server, sensor_msgs::msg::JointState joints)
{
    move_group->setJointValueTarget(joints);
    return wait_for_exec(move_group, server);
}
class SetParam : public rclcpp::Node
{
public:
    SetParam() : Node("set_param")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("stop_updating_obj", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&SetParam::timer_callback, this));
    };
    void set_param(bool new_param)
    {
        param = new_param;
    }

private:
    bool param = false;
    void timer_callback()
    {
        auto message = std_msgs::msg::Bool();
        message.data = param;
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    auto parameter_server = std::make_shared<SetParam>();

    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(parameter_server);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface hand_move_group(move_group_node, "hand");
    move_group.allowReplanning(true);
    move_group.setNumPlanningAttempts(10);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface("");

    std::string action_node_name;
    bool gazebo;
    if (!move_group_node->get_parameter("gazebo", gazebo))
    {
        // In case the parameter was not created use default
        gazebo = false;
    }
    bool use_spawn_obj;
    if (!move_group_node->get_parameter("use_spawn_obj", use_spawn_obj))
    {
        // In case the parameter was not created use default
        use_spawn_obj = false;
    }
    if (!move_group_node->get_parameter("action_node_name", action_node_name))
    {
        // In case the parameter was not created use default
        action_node_name = "/follow_joint_trajectory";
    }
    // auto node_name = action_node_name;
    // std::replace(node_name.begin(),node_name.end(), '/', '_');
    // auto server = std::make_shared<sim_action_server::ActionServer>(node_name.substr(1)+"_node", action_node_name);
    std::shared_ptr<sim_action_server::ActionServer> hand_server;

    auto server = std::make_shared<sim_action_server::ActionServer>("trajectory_control", action_node_name);
    if (gazebo)
    {
        hand_server = std::make_shared<sim_action_server::ActionServer>("trajectory_control", "/hand_controller/follow_joint_trajectory");
    }

    // Planning to a Pose goal

    if (use_spawn_obj)
    {

        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);
        hand_move_group.setMaxVelocityScalingFactor(1.0);
        hand_move_group.setMaxAccelerationScalingFactor(1.0);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening Hand");
        if (gazebo)
        {
            change_gripper(&hand_move_group, hand_server, gripper_state::opened);
            change_gripper(&hand_move_group, hand_server, gripper_state::closed); // fix gripper slightly to left
            change_gripper(&hand_move_group, hand_server, gripper_state::opened);
        }
        else
        {
            change_gripper(&hand_move_group, server, gripper_state::opened);
        }

        // Move above object
        std::vector<std::string> targets = {"target"};
        auto collision_objects = planning_scene_interface.getObjects(targets);
        auto collision_object = collision_objects["target"];
        auto pose = collision_object.primitive_poses[0];
        Quaternionf q = AngleAxisf(3.14, Vector3f::UnitX()) * AngleAxisf(0, Vector3f::UnitY()) * AngleAxisf(0.785, Vector3f::UnitZ());

        if (gazebo)
        {
            pose.position.z += 0.13;
        }
        else
        {
            pose.position.z += 0.1;
        }
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to object pose");
        pose.position.z += 0.1; // approach
        goto_pose(&move_group, server, pose);
        pose.position.z -= 0.1; 
        goto_pose(&move_group, server, pose);

        parameter_server->set_param(true);
        collision_object.operation = collision_object.REMOVE;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        planning_scene_interface.applyCollisionObject(collision_object);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        hand_move_group.setMaxVelocityScalingFactor(0.1);
        hand_move_group.setMaxAccelerationScalingFactor(0.1);

        // Close gripper
        change_gripper(&hand_move_group, server, gripper_state::closed);

        // Move above object again
        pose.position.z += 0.1;
        goto_pose(&move_group, server, pose);

        sensor_msgs::msg::JointState joints;
        joints.name = {"panda_joint1",
                   "panda_joint2",
                   "panda_joint3",
                   "panda_joint4",
                   "panda_joint5",
                   "panda_joint6",
                   "panda_joint7"};
        joints.position= {0.0,
                           -1.75,
                           0.0,
                           -0.1,
                           0.0,
                           3.6,
                           0.8};
        goto_joint_pose(&move_group, server, joints);

        // Arm trajectory
        joints.position= {0.0,
                           0.9,
                           0.0,
                           -1.1,
                           0.0,
                           1.9,
                           0.8};

        move_group.setJointValueTarget(joints);
        move_group.setMaxVelocityScalingFactor(1.0);
        move_group.setMaxAccelerationScalingFactor(1.0);

        auto current_state = hand_move_group.getCurrentState();
        float gripper_pose = (float)gripper_state::opened / 1000;
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(current_state->getJointModelGroup("hand"), joint_group_positions);
        joint_group_positions[0] = gripper_pose;
        joint_group_positions[1] = gripper_pose;
        hand_move_group.setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        for (int i = 0; i < 10; i++)
        {
            // 10 tries to plan otherwise give up
            bool success = (hand_move_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Plan %d %s", i, success ? "SUCCEEDED" : "FAILED");

            if (success)
            {        
                break;
            }
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

        for (int i = 0; i < 10; i++)
        {
            // 10 tries to plan otherwise give up
            bool success = (move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Plan %d %s", i, success ? "SUCCEEDED" : "FAILED");

            if (success)
            {
                break;
            }
        }

        auto gripper_traj = gripper_plan.trajectory_.joint_trajectory;
        auto arm_traj = &arm_plan.trajectory_.joint_trajectory;

        // Merge hand opening into arm trajectory, such that it is timed for release (at 50%)
        auto release_index = round(0.5*arm_traj->points.size());
        for (auto finger_joint : gripper_traj.joint_names)
        {
            arm_traj->joint_names.push_back(finger_joint);
        }

        while (arm_traj->points[release_index].effort.size() < 9){
            arm_traj->points[release_index].effort.push_back(0.0);
        }
        for (int j = 0; j < arm_traj->points.size(); j++)
        {
            for(int i = 0; i < 2; i++)
            { 
                if (j > release_index)
                {
                    arm_traj->points[j].positions.push_back(
                        gripper_traj.points[gripper_traj.points.size()-1].positions[i]);
                    arm_traj->points[j].velocities.push_back(
                        gripper_traj.points[gripper_traj.points.size()-1].velocities[i]);
                    arm_traj->points[j].accelerations.push_back(
                        gripper_traj.points[gripper_traj.points.size()-1].accelerations[i]);
                }
                else
                {
                    arm_traj->points[j].positions.push_back(
                        gripper_traj.points[0].positions[i]);
                    arm_traj->points[j].velocities.push_back(
                        gripper_traj.points[0].velocities[i]);
                    arm_traj->points[j].accelerations.push_back(
                        gripper_traj.points[0].accelerations[i]);
                }
            }
        }
        // for (int j = 0; j < gripper_traj.points.size(); j++)
        // {
        //     for(int i = 0; i < 2; i++)
        //     { 
        //         arm_traj->points[release_index+j].positions.push_back(
        //             gripper_traj.points[j].positions[i]);
        //         arm_traj->points[release_index+j].velocities.push_back(
        //             gripper_traj.points[j].velocities[i]);
        //         arm_traj->points[release_index+j].accelerations.push_back(
        //             gripper_traj.points[j].accelerations[i]);
        //     }
        // }

        std::thread([&move_group, arm_plan]() { move_group.execute(arm_plan); }).detach();
        //move_group->asyncExecute(plan);
        server->execute_plan(arm_plan.trajectory_.joint_trajectory);

        // Move to default position
        joints.position = {0.0,
                           0.0,
                           0.0,
                           -1.57,
                           0.0,
                           1.57,
                           0.79};

        parameter_server->set_param(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        goto_joint_pose(&move_group, server, joints);
        collision_objects = planning_scene_interface.getObjects(targets);
        collision_object = collision_objects["target"];
        auto new_pose = collision_object.primitive_poses[0];
        
        if ((new_pose.position.x < pose.position.x+0.2) || (pose.position.y + 0.1 < new_pose.position.y) || (pose.position.y - 0.1 > new_pose.position.y))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cube is not in bound");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task completed Succesfully");
        }
    }
    rclcpp::shutdown();
    return 0;
}