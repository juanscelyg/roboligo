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

#ifndef ROBOLIGO_COMMON_TYPES__PLUGINBASE_HPP_
#define ROBOLIGO_COMMON_TYPES__PLUGINBASE_HPP_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace roboligo
{
    /**
     * @class PluginBase
     * @brief Base class for plugin implementations with lifecycle node integration.
     * 
     * Provides common functionality for plugins including initialization,
     * node management, and plugin identification within a ROS2 lifecycle framework.
     */
    class PluginBase
    {
    public:
        /**
        * @brief Default constructor.
        */
        PluginBase() = default;

        /**
        * @brief Virtual destructor for proper cleanup of derived classes.
        */
        virtual ~PluginBase() = default;

        /**
        * @brief Initializes the plugin with a parent lifecycle node and plugin name.
        * 
        * @param parent_node Shared pointer to the parent lifecycle node.
        * @param plugin_name Name identifier for this plugin instance.
        */
        virtual void initialize(
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
            const std::string & plugin_name);

        /**
        * @brief Called during plugin initialization. Override in derived classes for custom setup.
        */
        virtual void on_initialize() {}

        /**
        * @brief Retrieves the parent lifecycle node.
        * 
        * @return Shared pointer to the parent lifecycle node.
        * @nodiscard The returned value should not be ignored.
        */
        [[nodiscard]] std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const;

        /**
        * @brief Retrieves the plugin name.
        * 
        * @return Const reference to the plugin name string.
        * @nodiscard The returned value should not be ignored.
        */
        [[nodiscard]] const std::string & get_plugin_name() const;

    private:

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_ {nullptr}; ///< Parent lifecycle node reference

        std::string plugin_name_; ///< Plugin name identifier

        float frequency_{10.0}; ///< Plugin execution frequency in Hz
    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_TYPES__PLUGINBASE_HPP_