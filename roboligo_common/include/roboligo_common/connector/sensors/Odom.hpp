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

#ifndef ROBOLIGO_COMMON_CONNECTOR_SENSORS__ODOM_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_SENSORS__ODOM_HPP_

#include <nav_msgs/msg/odometry.hpp>

#include "roboligo_common/connector/Sensor.hpp"

namespace roboligo
{
    class Odom : public Sensor
    {
    public:
        Odom(std::string new_name, std::string new_topic)
            : Sensor(new_name, new_topic){}

        virtual ~Odom() = default;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;  

        nav_msgs::msg::Odometry::SharedPtr data;

        void callback(nav_msgs::msg::Odometry::SharedPtr msg);
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_SENSORS__ODOM_HPP_