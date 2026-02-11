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

#include "roboligo_common/types/RobotState.hpp"

namespace roboligo
{

    std::string
    RobotState::get_name()
    {
        return name_;
    }

    void
    RobotState::set_name(std::string new_name)
    {
        name_ = new_name;
    }

    void 
    RobotState::set_modes(std::vector<Mode> modes)
    {
        modes_ = std::move(modes);
    }

    std::vector<Mode> 
    RobotState::get_modes()
    {
        std::vector<Mode> modes = std::move(modes_);
        return modes;
    }

    Mode* 
    RobotState::get_mode(const std::string & mode_name)
    {
        for (auto & mode : modes_) {
            if (mode.get_name() == mode_name) {
                return &mode;
            }
        }
        return nullptr;
    }

    void 
    RobotState::set_triggers(std::vector<Trigger> triggers)
    {
        triggers_ = std::move(triggers);
    }

    std::vector<Trigger> 
    RobotState::get_triggers()
    {
        std::vector<Trigger> triggers = triggers_;
        return triggers;
    }

    Trigger* 
    RobotState::get_trigger(const std::string & trigger_name)
    {
        for (auto & trigger : triggers_) {
            if (trigger.get_name() == trigger_name) {
                return &trigger;
            }
        }
        return nullptr;
    }

    bool
    RobotState::is_available()
    {
        return available_;
    }

    void
    RobotState::set_available(bool new_state)
    {
        available_ = new_state;
    }

    bool 
    RobotState::is_simulation(void)
    {
        return simulation_;
    }

    void 
    RobotState::set_simulation(bool new_state)
    {
        simulation_ = new_state;
    }

    bool 
    RobotState::is_stamped(void)
    {
        return stamped_;
    }

    void 
    RobotState::set_stamp(bool new_state)
    {
        stamped_ = new_state;
    }

    std::string
    RobotState::show_triggers(void)
    {
        std::string separator{"\n\t\t"};
        std::string triggers{separator};
        for (auto & trigger : triggers_) {
            triggers += trigger.get_name() + " : "  + (trigger.is_available() ? "On" : "Off") + " | " + separator;
        }
        return triggers;
    }

    std::string
    RobotState::show_modes(void)
    {
        std::string modes{""};
        for (auto & mode : modes_) {
            modes += mode.get_name() + " ";
        }
        return modes;
    }

    void 
    RobotState::show(void)
    {
        std::cout << "------ :: Roboligo State :: ------" << std::endl;
        std::cout <<" \t -- name: " << get_name().c_str() << std::endl;
        std::cout <<" \t -- available: " << (is_available()  == 1 ? "On" : "Off") << std::endl;
        std::cout <<" \t -- simulation: " << (is_simulation() == 1 ? "On" : "Off") << std::endl;
        std::cout <<" \t -- modes: " << show_modes().c_str() << std::endl;
        std::cout <<" \t -- triggers: " << show_triggers().c_str() << std::endl;
        std::cout << "------------------------------------" << std::endl;
    }

} // namespace roboligo