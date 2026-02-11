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

#include "roboligo_common/classification/ClassificationBase.hpp"

namespace roboligo
{
    void 
    ClassificationBase::initialize(
        const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
        const std::string & plugin_name)
    {
        auto node = parent_node;

        // Publishers

        // Parameters

        PluginBase::initialize(parent_node, plugin_name);
    }

    bool
    ClassificationBase::set(RobotState & robot_state)
    {
        on_set(robot_state);
        return true;
    }

    bool
    ClassificationBase::update(RobotState & robot_state)
    {
        on_update(robot_state);
        return true;
    }

} // namespace roboligo