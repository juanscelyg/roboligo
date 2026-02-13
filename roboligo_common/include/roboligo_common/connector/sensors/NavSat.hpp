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

#ifndef ROBOLIGO_COMMON_CONNECTOR_SENSORS__NAVSAT_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_SENSORS__NAVSAT_HPP_

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "roboligo_common/connector/Sensor.hpp"

namespace roboligo
{
    /**
     * @class NavSat
     * @brief A sensor class for handling GNSS/GPS navigation satellite data.
     * 
     * Inherits from Sensor and manages subscriptions to NavSatFix messages,
     * typically containing latitude, longitude, and altitude information.
     */
    class NavSat : public Sensor
    {
    public:
        /**
        * @brief Constructor for NavSat sensor.
        * @param new_name The name identifier for this sensor instance.
        * @param new_topic The ROS 2 topic to subscribe to for NavSatFix messages.
        */
        NavSat(std::string new_name, std::string new_topic)
            : Sensor(new_name, new_topic){}

        /**
        * @brief Virtual destructor.
        */
        virtual ~NavSat() = default;

        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber;  ///< ROS 2 subscription to NavSatFix messages

        sensor_msgs::msg::NavSatFix::SharedPtr data; ///< Pointer to the most recent NavSatFix data received.
        /**
        * @brief Callback function invoked when new NavSatFix data is received.
        * @param msg Shared pointer to the received NavSatFix message.
        */
        void callback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_SENSORS__NAVSAT_HPP_