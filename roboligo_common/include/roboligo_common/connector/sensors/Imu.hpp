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

#ifndef ROBOLIGO_COMMON_CONNECTOR_SENSORS__IMU_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_SENSORS__IMU_HPP_

#include <sensor_msgs/msg/imu.hpp>

#include "roboligo_common/connector/Sensor.hpp"

namespace roboligo
{
    /**
     * @class Imu
     * @brief IMU sensor connector that subscribes to sensor messages and manages IMU data.
     * 
     * Provides an interface for receiving and storing IMU (Inertial Measurement Unit) data
     * from ROS 2 topics.
     */
    class Imu : public Sensor
    {
    public:
        /**
        * @brief Constructor for Imu sensor.
        * @param new_name Identifier name for the sensor.
        * @param new_topic ROS 2 topic name to subscribe to.
        */
        Imu(std::string new_name, std::string new_topic)
            : Sensor(new_name, new_topic){}
        /**
        * @brief Virtual destructor.
        */
        virtual ~Imu() = default;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber;  ///< ROS 2 subscription handle for IMU messages

        sensor_msgs::msg::Imu::SharedPtr data; ///< Latest IMU data received from the subscriber
        /**
        * @brief Callback function triggered when new IMU message is received.
        * @param msg Shared pointer to the received IMU message.
        */
        void callback(sensor_msgs::msg::Imu::SharedPtr msg);
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_SENSORS__IMU_HPP_
