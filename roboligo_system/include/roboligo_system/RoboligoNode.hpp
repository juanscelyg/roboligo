//  Copyright 2026 Juan S. Cely G.

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      https://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef ROBOLIGO_SYSTEM__ROBOLIGONODE_HPP_
#define ROBOLIGO_SYSTEM__ROBOLIGONODE_HPP_

#include <functional>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "roboligo_interfaces/srv/roboligo_string.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>

#include "roboligo_common/connector/ConnectorBase.hpp"

#include "roboligo_common/classification/ClassificationBase.hpp"

#include "roboligo_states/StatesNode.hpp"
#include "roboligo_connector/ConnectorNode.hpp"

#include "roboligo_common/types/RobotState.hpp"


namespace roboligo
{
    struct RoboligoNodeInfo
    {
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr; ///< Shared pointer to the managed lifecycle node.
        rclcpp::CallbackGroup::SharedPtr cbg;
    };

    class RoboligoNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(RoboligoNode)
        using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        explicit RoboligoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        ~RoboligoNode();

        CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

        CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

        CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

        CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

        CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

        CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

        std::map<std::string, RoboligoNodeInfo> get_roboligo_system_nodes();

        rclcpp::Service<roboligo_interfaces::srv::RoboligoString>::SharedPtr input_command;

        void roboligo_init();

        void roboligo_cycle();

        void preShutdown();

        void input_callback(const std::shared_ptr<roboligo_interfaces::srv::RoboligoString::Request> request,
            std::shared_ptr<roboligo_interfaces::srv::RoboligoString::Response> response);

    private:

        StatesNode::SharedPtr states_node_ = StatesNode::make_shared();

        ConnectorNode::SharedPtr connector_node_ = ConnectorNode::make_shared();

        std::shared_ptr<RobotState> robot_state_ = std::make_shared<RobotState>(); ///< Shared pointer roboligo robot object

        std::string input_topic_{"/cmd_vel"};

        std::string robot_name_;

        bool stamped_{false};

    };

} // namespace roboligo

#endif // ROBOLIGO_SYSTEM__ROBOLIGONODE_HPP_