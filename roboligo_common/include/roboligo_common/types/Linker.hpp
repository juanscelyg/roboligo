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

#ifndef ROBOLIGO_COMMON_TYPES__LINK_HPP_
#define ROBOLIGO_COMMON_TYPES__LINK_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "roboligo_common/types/Interface.hpp" 

namespace roboligo
{
    /**
    * @enum Sense
    * @brief Defines the direction of a linker connection
    * @var Sense::INPUT - Linker receives data
    * @var Sense::OUTPUT - Linker sends data
    */
    enum class Sense 
    { 
        INPUT, 
        OUTPUT 
    };

    /**
    * @class Linker
    * @brief Manages the configuration and state of interface connections
    * 
    * Handles linking between components through topics and services,
    * including mode management, availability, and configuration status.
    */
    class Linker
    {
    public:
        /**
        * @brief Default constructor
        */
        Linker(){}

        /**
        * @brief Virtual destructor
        */
        virtual ~Linker() = default;

        /**
        * @brief Gets the current mode of the linker
        * @return Current interface mode
        */
        interfaces::modes get_mode();

        /**
        * @brief Sets the linker mode
        * @param new_mode The mode to set
        */
        void set_mode(interfaces::modes new_mode);

        /**
        * @brief Gets the linker name
        * @return Name string
        */
        std::string get_name();

        /**
        * @brief Sets the linker name
        * @param new_name Name to assign
        */
        void set_name(std::string new_name);

        /**
        * @brief Gets the topic name
        * @return Topic name string
        */
        std::string get_topic();

        /**
        * @brief Sets the topic name
        * @param new_name Topic name to assign
        */
        void set_topic(std::string new_name);

        /**
        * @brief Sets the service name
        * @param new_name Service name to assign
        */
        void set_service(std::string new_name);

        /**
        * @brief Configures the interface with name and topic
        * @param name_ Interface name
        * @param topic_name_ Topic name
        */
        void set_interface(std::string name_, std::string topic_name_);

        /**
        * @brief Checks if the linker is available
        * @return True if available, false otherwise
        */
        bool is_available(void);

        /**
        * @brief Checks if the linker is configured
        * @return True if configured, false otherwise
        */
        bool is_configured(void);

        /**
        * @brief Sets the availability state
        * @param new_state Availability state
        */
        void set_available(bool new_state);

        /**
        * @brief Sets the configuration state
        * @param new_state Configuration state
        */
        void set_configured(bool new_state);

        std::string name{"linker"}; ///< Linker identifier name

        std::string topic_name{"/interface"}; ///< Associated topic name

        std::shared_ptr<interfaces::Interface> interface; ///< Shared pointer to the interface object

    protected:
        interfaces::modes mode_; ///< Current operating mode

        bool available_{false}; ///< Availability flag
        
        bool configured_{false}; ///< Configuration state flag
    };
} // namespace roboligo

#endif // ROBOLIGO_COMMON_TYPES__LINK_HPP_
