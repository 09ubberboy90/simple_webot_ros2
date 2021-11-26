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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"
#include "gazebo_msgs/msg/model_state.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

void namer(std::shared_ptr<gazebo_msgs::srv::GetModelList_Request>, std::string);
void namer(std::shared_ptr<gazebo_msgs::srv::GetEntityState_Request>, std::string);

void result_handler(std::shared_ptr<rclcpp::Node>,std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetModelList_Response>>, gazebo_msgs::msg::ModelStates *);
void result_handler(std::shared_ptr<rclcpp::Node>,std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetEntityState_Response>>, gazebo_msgs::msg::ModelStates *);

int set_service(std::shared_ptr<rclcpp::Node>, rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr, bool);
void set_bool(const std::shared_ptr<std_srvs::srv::SetBool::Request>, std::shared_ptr<std_srvs::srv::SetBool::Response>, bool *);


template <typename MessageType>
int service_caller(std::shared_ptr<rclcpp::Node>, std::string, gazebo_msgs::msg::ModelStates *, std::string arg="");

template <typename MessageType>
int service_caller(std::shared_ptr<rclcpp::Node> node, std::string srv_name, gazebo_msgs::msg::ModelStates *states, std::string arg)
{
    typename rclcpp::Client<MessageType>::SharedPtr model_list_cl =
        node->create_client<MessageType>(srv_name);

    auto request = std::make_shared<typename MessageType::Request>();
    if (!arg.empty())
    {
        namer(request, arg);
    }
    std::chrono::seconds timeout(1);
    while (!model_list_cl->wait_for_service(timeout))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    auto result = model_list_cl->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        result_handler(node, result, states);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
        return 0;
    }
    return 1;

}

