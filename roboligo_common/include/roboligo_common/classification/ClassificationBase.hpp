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
    class ClassificationBase : public PluginBase
    {
        public:
        ClassificationBase() = default;
        virtual ~ClassificationBase() = default;

        virtual void initialize(
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
            const std::string & plugin_name);

        virtual bool set(RobotState & robot_state);

        virtual bool update(RobotState & robot_state);

        protected:
        virtual void on_set([[maybe_unused]]RobotState & robot_state){}

        virtual void on_update([[maybe_unused]]RobotState & robot_state){}

    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CLASSIFICATION__CLASSIFICATIONBASE_HPP_