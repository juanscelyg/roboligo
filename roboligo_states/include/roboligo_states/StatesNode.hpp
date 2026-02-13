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

#ifndef ROBOLIGO_STATES__STATESNODE_HPP_
#define ROBOLIGO_STATES__STATESNODE_HPP_

#include "pluginlib/class_loader.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "roboligo_common/classification/ClassificationBase.hpp"
#include "roboligo_common/types/RobotState.hpp"

namespace roboligo
{
    /**
    * @class StatesNode
    * @brief A lifecycle node that manages the configuration robot.
    * 
    * This class managments the modes and triggers for each robot type
    */
    class StatesNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:

        RCLCPP_SMART_PTR_DEFINITIONS(StatesNode)
        using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        /**
        * @brief Constructor for StatesNode
        * @param NodeOptions to configurate lifecycle node
        */
        explicit StatesNode(
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        /**
        * @brief Destructor
        */
        ~StatesNode();

        /**
        * @brief Instructions on_configure state
        * @param state lifecycle
        * @return CallbackReturnT defined in this class
        */
        CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

        /**
        * @brief Instructions on_active state
        * @param state lifecycle
        * @return CallbackReturnT defined in this class
        */
        CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

        /**
        * @brief Instructions on_deactivate state
        * @param state lifecycle
        * @return CallbackReturnT defined in this class
        */
        CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

        /**
        * @brief Instructions on_cleanup state
        * @param state lifecycle
        * @return CallbackReturnT defined in this class
        */
        CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

        /**
        * @brief Instructions on_shutdown state
        * @param state lifecycle
        * @return CallbackReturnT defined in this class
        */
        CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

        /**
        * @brief Instructions on_error state
        * @param state lifecycle
        * @return CallbackReturnT defined in this class
        */
        CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

        /**
        * @brief Method called at initialize and configurate settings 
        * @param robot state object
        * @return bool success result
        */
        bool init(std::shared_ptr<RobotState> robot_state);

        /**
        * @brief Method used for each iteration 
        * @param robot state object
        * @return bool success result
        */
        bool cycle(std::shared_ptr<RobotState> robot_state);

        /**
        * @brief When the node is in shutdowning, this method is called
        */
        void reset_classification(void);

    private:

        std::shared_ptr<ClassificationBase> classification_ {nullptr}; ///< ClassificationBase instance

        const std::shared_ptr<const RobotState> robot_state_; ///< Robot State object to manage data

        std::unique_ptr<pluginlib::ClassLoader<roboligo::ClassificationBase>> classification_loader_; ///< Class Loader for plugin
    };

}  // namespace roboligo

#endif  // ROBOLIGO_STATES__STATESNODE_HPP_
