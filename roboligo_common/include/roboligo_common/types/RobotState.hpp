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
    class RobotState
    {
    public:
        RobotState() = default;

        virtual ~RobotState() = default;

        std::string get_name(void);

        void set_name(std::string new_name);

        void set_modes(std::vector<Mode> modes);

        void set_triggers(std::vector<Trigger> triggers);

        std::vector<Mode> get_modes(void);

        std::vector<Trigger> get_triggers(void);

        Trigger* get_trigger(const std::string & trigger_name);

        Mode* get_mode(const std::string & mode_name);

        bool is_available(void);

        void set_available(bool new_state);

        bool is_simulation(void);

        void set_simulation(bool new_state);

        bool is_stamped(void);

        void set_stamp(bool new_state);

        std::string show_triggers(void);

        std::string show_modes(void);

        void show(void);

        std::shared_ptr<roboligo::Imu> imu = std::make_shared<roboligo::Imu>( "imu", "imu_topic");

        std::shared_ptr<roboligo::NavSat> gps = std::make_shared<roboligo::NavSat>( "gps", "gps_topic");

        std::shared_ptr<roboligo::Battery> battery = std::make_shared<roboligo::Battery>( "battery", "battery_topic");

        std::shared_ptr<roboligo::Odom> odom = std::make_shared<roboligo::Odom>( "odom", "odom_topic");

        std::shared_ptr<roboligo::Output> output = std::make_shared<roboligo::Output>( "output", "output_topic");

        std::shared_ptr<roboligo::Input> input = std::make_shared<roboligo::Input>( "input", "input_topic");

        std::shared_ptr<roboligo::Output> position_target = std::make_shared<roboligo::Output>( "position_target", "output_topic");

    private:

        std::string name_{"default_robot"}; ///< roboligo robot name 

        bool simulation_{false}; ///< roboligo robot simulation flag

        std::vector<Mode> modes_;

        std::vector<Trigger> triggers_;

        bool available_{false};

        bool stamped_{false};

    };
}

#endif // ROBOLIGO_COMMON__ROBOT_HPP_