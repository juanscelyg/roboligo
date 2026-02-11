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

#ifndef ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__OUTPUT_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__OUTPUT_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>

#include "roboligo_common/connector/Commander.hpp"

namespace roboligo
{
    struct RoboligoOutputTwist{
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        geometry_msgs::msg::Twist::SharedPtr data;
    };

    struct RoboligoOutputTwistStamped{
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher;
        geometry_msgs::msg::TwistStamped::SharedPtr data;
    };

    struct RoboligoOutputPositionTarget{
        rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr publisher;
        mavros_msgs::msg::PositionTarget::SharedPtr data;
        int coordinate_frame = 8;
        int type_mask = 1991; //b011111000111
    };

    class Output : public Commander
    {
    public:
        Output(std::string new_name, std::string new_topic);

        virtual ~Output() = default;

        RoboligoOutputTwist twist;

        RoboligoOutputTwistStamped twist_stamped;

        RoboligoOutputPositionTarget position_target;

        void set_data(const geometry_msgs::msg::Twist* msg);

        void set_data_stamped(const geometry_msgs::msg::TwistStamped* msg);
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__OUTPUT_HPP_
