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

#ifndef ROBOLIGO_COMMON_CONNECTOR__COMMANDER_HPP_
#define ROBOLIGO_COMMON_CONNECTOR__COMMANDER_HPP_

#include "roboligo_common/types/Linker.hpp"

namespace roboligo
{
    /**
    * @class Commander
    * @brief Base class for commanding robotic operations through message publishing.
    * 
    * Inherits from Linker to provide ROS communication functionality.
    * Manages stamped message publishing with optional timestamp inclusion.
    */
    class Commander : public Linker
    {
    public:
        /**
        * @brief Constructor for Commander.
        * @param new_name The name identifier for the commander instance.
        * @param new_value The initial value or topic associated with the commander.
        */
        Commander(std::string new_name, std::string new_value);

        /**
        * @brief Virtual destructor.
        */
        virtual ~Commander() = default;

        /**
        * @brief Enable or disable timestamp stamping for messages.
        * @param stamped True to include timestamps, false otherwise.
        */
        void set_stamp(bool stamped);

        /**
        * @brief Check if timestamp stamping is enabled.
        * @return True if stamping is enabled, false otherwise.
        */
        bool is_stamped(void);

        /**
        * @brief Initialize the commander with name and value.
        * @param new_name The name identifier for the commander instance.
        * @param new_value The initial value or topic associated with the commander.
        */
        void init(std::string new_name, std::string new_value);
        
    protected:
        /**
        * @brief Protected initialization method.
        * @param new_name The name identifier for the commander instance.
        * @param new_topic The ROS topic for publishing messages.
        */
        void initialize(std::string new_name, std::string new_topic);
        
        bool use_stamp_{true}; ///< Flag indicating whether to include timestamps in published messages
    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR__COMMANDER_HPP_