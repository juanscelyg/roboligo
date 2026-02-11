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

#ifndef ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__INPUT_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__INPUT_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "roboligo_common/connector/Commander.hpp"

namespace roboligo
{
    struct RoboligoInputTwist{
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
        geometry_msgs::msg::Twist::SharedPtr data;
    };

    struct RoboligoInputTwistStamped{
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber;
        geometry_msgs::msg::TwistStamped::SharedPtr data;
    };

    class Input : public Commander
    {
    public:
        Input(std::string new_name, std::string new_topic);

        virtual ~Input() = default;

        RoboligoInputTwist twist;

        RoboligoInputTwistStamped twist_stamped;
    
        void callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        
        void callback_stamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__INPUT_HPP_
