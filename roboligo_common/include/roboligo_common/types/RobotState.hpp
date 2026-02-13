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

#ifndef ROBOLIGO_COMMON__ROBOT_HPP_
#define ROBOLIGO_COMMON__ROBOT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "roboligo_common/connector/sensors/Imu.hpp"
#include "roboligo_common/connector/sensors/NavSat.hpp"
#include "roboligo_common/connector/sensors/Battery.hpp"
#include "roboligo_common/connector/sensors/Odom.hpp"

#include "roboligo_common/connector/commanders/Input.hpp"
#include "roboligo_common/connector/commanders/Output.hpp"

#include "roboligo_common/classification/Trigger.hpp"
#include "roboligo_common/classification/Mode.hpp"

namespace roboligo
{
    /**
    * @class RobotState
    * @brief Represents the state and configuration of a robot.
    * 
    * This class manages robot properties including name, operational modes, triggers,
    * and various sensor/actuator components. It tracks availability, simulation status,
    * and time-stamping capabilities.
    */
    class RobotState
    {
    public:
        /**
        * @brief Default constructor.
        */
        RobotState() = default;

        /**
        * @brief Virtual destructor.
        */
        virtual ~RobotState() = default;

        /**
        * @brief Gets the robot name.
        * @return The robot's name as a string.
        */
        std::string get_name(void);

        /**
        * @brief Sets the robot name.
        * @param new_name The new name to assign.
        */
        void set_name(std::string new_name);

        /**
        * @brief Sets the available operational modes.
        * @param modes Vector of Mode objects.
        */
        void set_modes(std::vector<Mode> modes);

        /**
        * @brief Sets the available triggers.
        * @param triggers Vector of Trigger objects.
        */
        void set_triggers(std::vector<Trigger> triggers);

        /**
        * @brief Gets all available modes.
        * @return Vector of Mode objects.
        */
        std::vector<Mode> get_modes(void);

        /**
        * @brief Gets all available triggers.
        * @return Vector of Trigger objects.
        */
        std::vector<Trigger> get_triggers(void);

        /**
        * @brief Retrieves a trigger by name.
        * @param trigger_name The name of the trigger to find.
        * @return Pointer to the Trigger object, or nullptr if not found.
        */
        Trigger* get_trigger(const std::string & trigger_name);

        /**
        * @brief Retrieves a mode by name.
        * @param mode_name The name of the mode to find.
        * @return Pointer to the Mode object, or nullptr if not found.
        */
        Mode* get_mode(const std::string & mode_name);

        /**
        * @brief Checks if the robot is available.
        * @return true if available, false otherwise.
        */
        bool is_available(void);

        /**
        * @brief Sets the robot availability status.
        * @param new_state true to mark as available, false otherwise.
        */
        void set_available(bool new_state);

        /**
        * @brief Checks if the robot is in simulation mode.
        * @return true if in simulation, false otherwise.
        */
        bool is_simulation(void);

        /**
        * @brief Sets the simulation mode status.
        * @param new_state true to enable simulation mode.
        */
        void set_simulation(bool new_state);

        /**
        * @brief Checks if the robot uses time stamping.
        * @return true if stamped, false otherwise.
        */
        bool is_stamped(void);

        /**
        * @brief Enables or disables time stamping.
        * @param new_state true to enable stamping.
        */
        void set_stamp(bool new_state);

        /**
        * @brief Returns a string representation of all triggers.
        * @return Formatted string containing trigger information.
        */
        std::string show_triggers(void);

        /**
        * @brief Returns a string representation of all modes.
        * @return Formatted string containing mode information.
        */
        std::string show_modes(void);

        /**
        * @brief Displays complete robot state information.
        */
        void show(void);

        std::shared_ptr<roboligo::Imu> imu = std::make_shared<roboligo::Imu>( 
            "imu", "imu_topic"); ///< Inertial Measurement Unit sensor component

        std::shared_ptr<roboligo::NavSat> gps = std::make_shared<roboligo::NavSat>( 
            "gps", "gps_topic"); ///< Global Navigation Satellite System (GPS) sensor

        std::shared_ptr<roboligo::Battery> battery = std::make_shared<roboligo::Battery>( 
            "battery", "battery_topic"); ///< Battery status monitor component

        std::shared_ptr<roboligo::Odom> odom = std::make_shared<roboligo::Odom>( 
            "odom", "odom_topic"); ///< Odometry sensor component

        std::shared_ptr<roboligo::Output> output = std::make_shared<roboligo::Output>( 
            "output", "output_topic"); ///< Output actuator component

        std::shared_ptr<roboligo::Input> input = std::make_shared<roboligo::Input>( 
            "input", "input_topic"); ///< Input sensor component

        std::shared_ptr<roboligo::Output> position_target = std::make_shared<roboligo::Output>( 
            "position_target", "output_topic"); ///< Position target output component

    private:

        std::string name_{"default_robot"}; ///< roboligo robot name 

        bool simulation_{false}; ///< roboligo robot simulation flag

        std::vector<Mode> modes_; ///< Collection of operational modes

        std::vector<Trigger> triggers_; ///< Collection of trigger events

        bool available_{false}; ///< Indicates if robot is operational

        bool stamped_{false}; ///< Indicates if robot data includes timestamps

    };
}

#endif // ROBOLIGO_COMMON__ROBOT_HPP_