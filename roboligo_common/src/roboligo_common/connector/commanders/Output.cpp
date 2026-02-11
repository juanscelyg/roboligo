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

#include "roboligo_common/connector/commanders/Output.hpp"

namespace roboligo
{
    Output::Output(std::string new_name, std::string new_topic)
        : Commander(new_name, new_topic)
    {
        set_mode(interfaces::modes::PUBLISHER);
        set_interface(name, topic_name);
    }

    void 
    Output::set_data(const geometry_msgs::msg::Twist* msg)
    {
        twist.data = std::make_shared<geometry_msgs::msg::Twist>(*msg);
    }

    void 
    Output::set_data_stamped(const geometry_msgs::msg::TwistStamped* msg)
    {
        twist_stamped.data = std::make_shared<geometry_msgs::msg::TwistStamped>(*msg);
    }
}// namespace roboligo