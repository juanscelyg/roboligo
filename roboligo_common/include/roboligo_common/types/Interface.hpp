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
    namespace interfaces 
    {
        enum class types 
        { 
            MSG, 
            SRV 
        };
        
        enum class modes 
        { 
            PUBLISHER, 
            SUBSCRIBER, 
            CALLER 
        };

        struct type {
            types type;             ///< Type MSG or SRV
            modes mode;             ///< Interface mode PUBLISHER, SUBSCRIBER, CALLER
            std::string value = ""; ///< Interface value as topic name (i. e. /prueba)
        };

        class Interface {
        public:
            Interface(std::string name, modes mode, std::string value);   

            virtual ~Interface() = default;

            std::string get_name(void);

            types get_type(void);

            modes get_mode(void);
           
            std::string get_value(void);

            void set_value(std::string value);

            void show();

        private:
            std::string name_{};    ///< Interface name
            type type_{};           ///< Interface type
            modes mode_{};          ///< Interface mode
        };

    } // namespace interfaces
} // namespace roboligo

#endif // ROBOLIGO_COMMON_TYPES__INTERFACE_HPP_
