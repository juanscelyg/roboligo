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

#ifndef ROBOLIGO_COMMON_CLASSIFICATION__CLASSIFICATIONBASE_HPP_
#define ROBOLIGO_COMMON_CLASSIFICATION__CLASSIFICATIONBASE_HPP_

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "roboligo_common/types/PluginBase.hpp"
#include "roboligo_common/classification/Mode.hpp"
#include "roboligo_common/classification/Trigger.hpp"
#include "roboligo_common/types/RobotState.hpp"

namespace roboligo 
{
    /**
    * @class ClassificationBase
    * @brief Base class for classification plugins in the Roboligo system.
    * 
    * ClassificationBase provides an abstract interface for implementing classification
    * functionality within the robot control framework. It inherits from PluginBase and
    * follows a plugin architecture pattern, allowing extensibility through derived classes.
    */
    class ClassificationBase : public PluginBase
    {
    public:
        /**
        * @brief constructor
        */
        ClassificationBase() = default;

        /**
        * @brief Destructor
        */
        virtual ~ClassificationBase() = default;

        /**
        * @brief Initializes the classification plugin.
        * @param parent_node Shared pointer to the lifecycle node managing this plugin.
        * @param plugin_name Name identifier for this plugin instance.
        */
        virtual void initialize(
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
            const std::string & plugin_name);

        /**
        * @brief Sets the robot state classification.
        * @param robot_state Reference to the robot state to classify.
        * @return True if the operation succeeded, false otherwise.
        */
        virtual bool set(RobotState & robot_state);

        /**
        * @brief Updates the robot state classification.
        * @param robot_state Reference to the robot state to update.
        * @return True if the operation succeeded, false otherwise.
        */
        virtual bool update(RobotState & robot_state);

    protected:
        /**
        * @brief Called when setting robot state (override in derived classes).
        * @param robot_state Reference to the robot state being set.
        */
        virtual void on_set([[maybe_unused]]RobotState & robot_state){}

        /**
        * @brief Called when updating robot state (override in derived classes).
        * @param robot_state Reference to the robot state being updated.
        */
        virtual void on_update([[maybe_unused]]RobotState & robot_state){}

    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CLASSIFICATION__CLASSIFICATIONBASE_HPP_