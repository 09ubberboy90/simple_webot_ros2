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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>

using std::placeholders::_1;

using namespace Eigen;
using namespace std::chrono_literals;

enum gripper_state
{
    opened = 35,
    closed = 0
};

bool wait_for_exec(moveit::planning_interface::MoveGroupInterface *move_group)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for (int i = 0; i < 10; i++)
    {
        // 10 tries to plan otherwise give up
        bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Plan %d %s", i, success ? "SUCCEEDED" : "FAILED");

        if (success)
        {
            move_group->execute(plan);
            return true;
        }
    }
    auto pose = move_group->getPoseTarget().pose.position;
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to find a valid path to %f, %f, %f", pose.x, pose.y, pose.z);
    throw "Couldn't plan a path";
    return false;
}

bool change_gripper(moveit::planning_interface::MoveGroupInterface *hand_move_group, gripper_state state)
{
    auto current_state = hand_move_group->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(current_state->getJointModelGroup("hand"), joint_group_positions);
    float pose = (float)state / 1000;
    joint_group_positions[0] = pose;
    joint_group_positions[1] = pose;
    hand_move_group->setGoalPositionTolerance(0.04);
    hand_move_group->setJointValueTarget(joint_group_positions);
    return wait_for_exec(hand_move_group);
}

bool goto_pose(moveit::planning_interface::MoveGroupInterface *move_group, geometry_msgs::msg::Pose pose)
{
    move_group->setPoseTarget(pose);
    return wait_for_exec(move_group);
}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("panda_group_interface", node_options);

    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    // executor.add_node(parameter_server);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface hand_move_group(move_group_node, "hand");

    planning_scene_monitor::PlanningSceneMonitor planning_scene_monitor(move_group_node, "robot_description");


    move_group.allowReplanning(true);
    move_group.setNumPlanningAttempts(10);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    bool gazebo;
    if (!move_group_node->get_parameter("gazebo", gazebo))
    {
        // In case the parameter was not created use default
        gazebo = false;
    }
    auto start_pose = move_group.getCurrentPose().pose;
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening Hand");
    // if (gazebo)
    // {
    //     change_gripper(&hand_move_group, hand_server, gripper_state::opened);
    //     change_gripper(&hand_move_group, hand_server, gripper_state::closed); // fix gripper slightly to left
    //     change_gripper(&hand_move_group, hand_server, gripper_state::opened);
    //     move_group.setMaxVelocityScalingFactor(0.5);
    //     move_group.setMaxAccelerationScalingFactor(0.5);

    // }
    // else
    // {
    change_gripper(&hand_move_group, gripper_state::opened);
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);

    // }


    std::string target = "target";
    
    auto collision_object = planning_scene_interface.getObjects({target})[target];

    auto pose = collision_object.primitive_poses[0];
    Quaternionf q = AngleAxisf(3.14, Vector3f::UnitX()) * AngleAxisf(0, Vector3f::UnitY()) * AngleAxisf(0.785, Vector3f::UnitZ());

    if (gazebo)
    {
        pose.position.z += 0.135;
        // pose.position.y += 0.005;
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
    goto_pose(&move_group, pose);
    pose.position.z -= 0.1; 
    goto_pose(&move_group, pose);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Closing hand");
    // parameter_server->set_param(true);

    auto acm = planning_scene_monitor.getPlanningScene()->getAllowedCollisionMatrix();
    acm.setDefaultEntry(target, true);

    collision_object.operation = collision_object.REMOVE;
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    planning_scene_interface.applyCollisionObject(collision_object);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    hand_move_group.attachObject(target);
    hand_move_group.setMaxVelocityScalingFactor(0.1);
    hand_move_group.setMaxAccelerationScalingFactor(0.1);

    if (gazebo)
    {
        change_gripper(&hand_move_group, gripper_state::closed);
    }
    else
    {
        change_gripper(&hand_move_group, gripper_state::closed);
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lifting");
    pose.position.z += 0.1;
    goto_pose(&move_group, pose);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to target pose");
    pose.position.x += 0.2;
    goto_pose(&move_group, pose);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lowering");
    pose.position.z -= 0.1;
    goto_pose(&move_group, pose);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening Hand");
    if (gazebo)
    {
        change_gripper(&hand_move_group, gripper_state::opened);
    }
    else
    {
        change_gripper(&hand_move_group, gripper_state::opened);
    }

    collision_object = planning_scene_interface.getAttachedObjects({target})[target].object;
    collision_object.operation = collision_object.ADD;
    planning_scene_interface.applyCollisionObject(collision_object);

    move_group.detachObject("target");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to start pose");
    goto_pose(&move_group, start_pose);
    collision_object = planning_scene_interface.getObjects({target})[target];
    auto new_pose = collision_object.primitive_poses[0];

    if ((new_pose.position.x < pose.position.x - 0.05) || (pose.position.x + 0.05 < new_pose.position.x))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cube is not in bound");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task completed Succesfully");
    }
    rclcpp::shutdown();
    return 0;
}