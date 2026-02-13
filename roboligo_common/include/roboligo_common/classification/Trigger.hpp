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

#ifndef ROBOLIGO_COMMON_CLASSIFICATION__TRIGGER_HPP_
#define ROBOLIGO_COMMON_CLASSIFICATION__TRIGGER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <iostream>

#include "roboligo_common/classification/Mode.hpp"

namespace roboligo 
{
    /**
    * @class Trigger
    * @brief Represents a trigger that can transition between different modes in a state machine.
    * 
    * A Trigger encapsulates the logic for transitioning from one or more previous modes
    * to one or more next modes. It supports callbacks and maintains availability status.
    */
    class Trigger
    {
    public:
        /**
        * @brief Constructor for Trigger.
        * @param name The name identifier of the trigger.
        * @param value The value associated with the trigger.
        * @param previous Vector of modes that precede this trigger.
        * @param next Vector of modes that follow this trigger.
        */
        Trigger(std::string name, std::string value, std::vector<Mode> previous, std::vector<Mode> next)
            :name_(name), value_(value), previous_(std::move(previous)), next_(std::move(next)) {}

        /**
        * @brief Destructor
        */
        virtual ~Trigger() = default;

        /**
        * @brief Gets the name of the trigger.
        * @return The trigger's name as a string.
        */
        std::string get_name(void);

        /**
        * @brief Gets the value of the trigger.
        * @return The trigger's value as a string.
        */
        std::string get_value(void);

        /**
        * @brief Sets a new name for the trigger.
        * @param new_name The new name to assign.
        */
        void set_name(std::string new_name);

        /**
        * @brief Sets a new value for the trigger.
        * @param new_value The new value to assign.
        */
        void set_value(std::string new_value);

        /**
        * @brief Sets the previous modes for this trigger.
        * @param modes Vector of modes to set as previous.
        */
        void set_previous_modes(std::vector<Mode> modes);

        /**
        * @brief Sets the next modes for this trigger.
        * @param modes Vector of modes to set as next.
        */
        void set_next_modes(std::vector<Mode> modes);

        /**
        * @brief Checks if the trigger is available for execution.
        * @return True if the trigger is available, false otherwise.
        */
        bool is_available(void);

        /**
        * @brief Registers a callback function to be executed when the trigger fires.
        * @param callback The callback function to register.
        */
        void register_callback(const std::function<void()>& callback);

        /**
        * @brief Executes the trigger and invokes its registered callback.
        */
        void execute(void);

        std::function<void()> cb; ///< Callback function

    protected:

        std::string name_; ///< Trigger name
        
        std::string value_; ///< Value for the trigger

        std::vector<Mode> previous_; ///< Vector of previous modes

        std::vector<Mode> next_; ///< Vector of next modes

        bool available_{false}; ///< If trigger is available

    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CLASSIFICATION__TRIGGER_HPP_