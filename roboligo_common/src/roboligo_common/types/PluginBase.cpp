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

#include "roboligo_common/types/PluginBase.hpp"

namespace roboligo
{
    void 
    PluginBase::initialize(
        const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
        const std::string & plugin_name)
    {
        parent_node_ = parent_node;
        plugin_name_ = plugin_name;

        parent_node_->declare_parameter(plugin_name + ".freq", frequency_);
        parent_node_->get_parameter(plugin_name + ".freq", frequency_);

        on_initialize();
    }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> 
    PluginBase::get_node() const
    {
        return parent_node_;
    }

    const std::string & 
    PluginBase::get_plugin_name() const
    {
        return plugin_name_;
    }
}// namespace roboligo