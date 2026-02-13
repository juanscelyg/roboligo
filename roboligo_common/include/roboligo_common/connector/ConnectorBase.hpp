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

#ifndef ROBOLIGO_COMMON_CLASSIFICATION__CONNECTORBASE_HPP_
#define ROBOLIGO_COMMON_CLASSIFICATION__CONNECTORBASE_HPP_

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "roboligo_common/types/PluginBase.hpp"
#include "roboligo_common/connector/Sensor.hpp"
#include "roboligo_common/connector/Commander.hpp"
#include "roboligo_common/types/RobotState.hpp"

namespace roboligo 
{
    /**
    * @class ConnectorBase
    * @brief Base class for connector plugins that manage robot state transitions.
    * 
    * ConnectorBase provides a plugin interface for handling robot state initialization,
    * updates, and cleanup. Derived classes should override the virtual methods to
    * implement specific connector behavior.
    */
    class ConnectorBase : public PluginBase
    {
    public:
        /**
        * @brief Default constructor.
        */
        ConnectorBase() = default;

        /**
        * @brief Virtual destructor.
        */
        virtual ~ConnectorBase() = default;

        /**
        * @brief Initialize the connector with a lifecycle node.
        * 
        * @param parent_node Shared pointer to the lifecycle node that owns this plugin.
        * @param plugin_name Name identifier for this connector plugin.
        */
        virtual void initialize(
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
            const std::string & plugin_name);

        /**
        * @brief Set initial robot state.
        * 
        * Called during the configuration phase to establish the initial robot state.
        * 
        * @param robot_state Reference to the robot state to configure.
        * @return true if set operation succeeded, false otherwise.
        */
        virtual bool set(RobotState & robot_state);

        /**
        * @brief Update robot state.
        * 
        * Called during the active phase to update the robot state.
        * 
        * @param robot_state Reference to the robot state to update.
        * @return true if update operation succeeded, false otherwise.
        */
        virtual bool update(RobotState & robot_state);

        /**
        * @brief Exit and cleanup robot state.
        * 
        * Called during shutdown to finalize the robot state.
        * 
        * @param robot_state Reference to the robot state to finalize.
        * @return true if exit operation succeeded, false otherwise.
        */
        virtual bool exit(RobotState & robot_state);

    protected:
        /**
        * @brief Hook called during set operation.
        * 
        * Override this method to implement custom set behavior.
        * 
        * @param robot_state Reference to the robot state.
        */
        virtual void on_set([[maybe_unused]]RobotState & robot_state){}

        /**
        * @brief Hook called during update operation.
        * 
        * Override this method to implement custom update behavior.
        * 
        * @param robot_state Reference to the robot state.
        */
        virtual void on_update([[maybe_unused]]RobotState & robot_state){}

        /**
        * @brief Hook called during exit operation.
        * 
        * Override this method to implement custom cleanup behavior.
        * 
        * @param robot_state Reference to the robot state.
        */
        virtual void on_exit([[maybe_unused]]RobotState & robot_state){}

    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CLASSIFICATION__CONNECTORBASE_HPP_