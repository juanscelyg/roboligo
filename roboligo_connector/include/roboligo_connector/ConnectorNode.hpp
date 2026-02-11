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

#ifndef ROBOLIGO_CONNECTOR__CONNECTORNODE_HPP_
#define ROBOLIGO_CONNECTOR__CONNECTORNODE_HPP_


#include "pluginlib/class_loader.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "roboligo_common/connector/ConnectorBase.hpp"
#include "roboligo_common/types/RobotState.hpp"

namespace roboligo
{

    class ConnectorNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:

    RCLCPP_SMART_PTR_DEFINITIONS(ConnectorNode)

    using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit ConnectorNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    virtual ~ConnectorNode();

    CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

    bool init(std::shared_ptr<RobotState> robot_state);

    void reset_connector(void);

    bool cycle(std::shared_ptr<RobotState> robot_state);

    private:

    std::shared_ptr<ConnectorBase> connector_ {nullptr};

    const std::shared_ptr<const RobotState> robot_state_;

    std::shared_ptr<pluginlib::ClassLoader<roboligo::ConnectorBase>> connector_loader_;
    };

}  // namespace roboligo

#endif  // ROBOLIGO_CONNECTOR__CONNECTORNODE_HPP_
