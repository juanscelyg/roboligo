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

#ifndef ROBOLIGO_COMMON_CONNECTOR__SENSOR_HPP_
#define ROBOLIGO_COMMON_CONNECTOR__SENSOR_HPP_

#include "roboligo_common/types/Interface.hpp"
#include "roboligo_common/types/Linker.hpp"

namespace roboligo
{
    /**
    * @class Sensor
    * @brief A sensor class that extends Linker functionality for subscribing to sensor data.
    * 
    * This class provides the base implementation for sensor connections in the roboligo system.
    * It handles subscription to sensor topics and manages sensor-specific initialization.
    */
    class Sensor : public Linker
    {
        public:
            /**
            * @brief Constructor for Sensor.
            * @param new_name The name identifier for the sensor.
            * @param new_topic The ROS topic name to subscribe to.
            */
            Sensor(std::string new_name, std::string new_topic);

            /**
            * @brief Virtual destructor.
            */
            virtual ~Sensor() = default;

            /**
            * @brief Initializes the sensor with a name and topic.
            * @param new_name The name identifier for the sensor.
            * @param new_topic The ROS topic name to subscribe to.
            */
            void init(std::string new_name, std::string new_topic);

        protected:

            /**
            * @brief Protected initialization method.
            * @param new_name The name identifier for the sensor.
            * @param new_topic The ROS topic name to subscribe to.
            */
            void initialize(std::string new_name, std::string new_topic);
            
            interfaces::modes mode_{interfaces::modes::SUBSCRIBER}; ///< The connection mode set to SUBSCRIBER for receiving sensor data
            
    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR__SENSOR_HPP_