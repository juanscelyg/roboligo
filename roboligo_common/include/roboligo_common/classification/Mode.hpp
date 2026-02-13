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

#ifndef ROBOLIGO_COMMON_CLASSIFICATION__MODE_HPP_
#define ROBOLIGO_COMMON_CLASSIFICATION__MODE_HPP_

#include <string>

namespace roboligo
{
    /**
    * @class Mode
    * @brief Represents a configurable mode with name, value, and active status.
    * 
    * This class encapsulates mode information including a name identifier,
    * a value representation, and an active status flag. It provides methods
    * to get and set these properties.
    */
    class Mode
    {
    public:
        /**
        * @brief Constructs a Mode object with the specified name and value.
        * @param name The mode name identifier
        * @param value The mode value representation
        */
        Mode(std::string name, std::string value)
            :name_(name), value_(value) {}

        /**
        * @brief Virtual destructor for proper cleanup of derived classes.
        */
        virtual ~Mode() = default;

        /**
        * @brief Gets the mode name.
        * @return The name of the mode
        */
        std::string get_name(void);

        /**
        * @brief Gets the mode value.
        * @return The value associated with the mode
        */
        std::string get_value(void);

        /**
        * @brief Sets the mode name.
        * @param new_name The new mode name
        */
        void set_name(std::string new_name);

        /**
        * @brief Sets the mode value.
        * @param new_value The new mode value
        */
        void set_value(std::string new_value);

        /**
        * @brief Checks if the mode is active.
        * @return True if the mode is active, false otherwise
        */
        bool is_ative(void);

        /**
        * @brief Sets the active status of the mode.
        * @param status The new active status
        */
        void set_active(bool status);

    protected:
        std::string name_{"base_mode"};      ///< Mode name identifier
        std::string value_{"MODE"};          ///< Mode value representation
        bool active_{false};                 ///< Active status flag
    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CLASSIFICATION__MODE_HPP_