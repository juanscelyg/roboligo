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

#ifndef ROBOLIGO_COMMON_TYPES__INTERFACE_HPP_
#define ROBOLIGO_COMMON_TYPES__INTERFACE_HPP_

#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

namespace roboligo 
{
    /**
    * @namespace interfaces
    * @brief Namespace containing interface management classes and types.
    */
    namespace interfaces 
    {
        /**
        * @enum types
        * @brief Enumeration for interface communication types.
        * @var types::MSG Message-based interface
        * @var types::SRV Service-based interface
        */
        enum class types 
        { 
            MSG, 
            SRV 
        };

        /**
        * @enum modes
        * @brief Enumeration for interface operational modes.
        * @var modes::PUBLISHER Publishes data
        * @var modes::SUBSCRIBER Subscribes to data
        * @var modes::CALLER Calls a service
        */
        enum class modes 
        { 
            PUBLISHER, 
            SUBSCRIBER, 
            CALLER 
        };

        /**
        * @struct type
        * @brief Defines interface configuration structure.
        * @member types type The communication type (MSG or SRV)
        * @member modes mode The operational mode (PUBLISHER, SUBSCRIBER, CALLER)
        * @member std::string value The interface identifier (e.g., topic name)
        */
        struct type {
            types type;             ///< Type MSG or SRV
            modes mode;             ///< Interface mode PUBLISHER, SUBSCRIBER, CALLER
            std::string value = ""; ///< Interface value as topic name (i. e. /prueba)
        };

        /**
        * @class Interface
        * @brief Represents a configurable interface with name, type, mode, and value.
        * 
        * This class manages interface properties including its name, operational mode,
        * and current value. It provides methods to access and modify interface state.
        */
        class Interface {
        public:
            /**
            * @brief Constructs an Interface with the specified parameters.
            * @param name The interface name identifier.
            * @param mode The operational mode of the interface.
            * @param value The initial value of the interface.
            */
            Interface(std::string name, modes mode, std::string value);   

            virtual ~Interface() = default;

            /**
            * @brief Retrieves the interface name.
            * @return The name of the interface as a string.
            */
            std::string get_name(void);

            /**
            * @brief Retrieves the interface type.
            * @return The type of the interface.
            */
            types get_type(void);

            /**
            * @brief Retrieves the interface mode.
            * @return The operational mode of the interface.
            */
            modes get_mode(void);
           
            /**
            * @brief Retrieves the current value of the interface.
            * @return The interface value as a string.
            */
            std::string get_value(void);

            /**
            * @brief Updates the interface value.
            * @param value The new value to set.
            */
            void set_value(std::string value);

            /**
            * @brief Displays the interface information.
            * Outputs current interface properties to the standard output.
            */
            void show();

        private:
            std::string name_{};    ///< Interface name
            type type_{};           ///< Interface type
            modes mode_{};          ///< Interface mode
        };

    } // namespace interfaces
} // namespace roboligo

#endif // ROBOLIGO_COMMON_TYPES__INTERFACE_HPP_
