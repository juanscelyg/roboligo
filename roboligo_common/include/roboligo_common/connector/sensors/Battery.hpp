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

#ifndef ROBOLIGO_COMMON_CONNECTOR_SENSORS__BATTERY_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_SENSORS__BATTERY_HPP_

#include <sensor_msgs/msg/battery_state.hpp>

#include "roboligo_common/connector/Sensor.hpp"

namespace roboligo
{
    class Battery : public Sensor
    {
    public:
        Battery(std::string new_name, std::string new_topic)
            : Sensor(new_name, new_topic){}

        virtual ~Battery() = default;

        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscriber;  

        sensor_msgs::msg::BatteryState::SharedPtr data;

        void callback(sensor_msgs::msg::BatteryState::SharedPtr msg);
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_SENSORS__BATTERY_HPP_