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
    class PluginBase
    {
        public:

        PluginBase() = default;

        virtual ~PluginBase() = default;

        virtual void initialize(
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
            const std::string & plugin_name);

        virtual void on_initialize() {}

        [[nodiscard]] std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const;

        [[nodiscard]] const std::string & get_plugin_name() const;

        private:

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_ {nullptr};

        std::string plugin_name_;

        float frequency_{10.0};
    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_TYPES__PLUGINBASE_HPP_