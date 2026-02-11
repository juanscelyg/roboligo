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
    enum class Sense 
    { 
        INPUT, 
        OUTPUT 
    };

    class Linker
    {
    public:
        Linker(){}

        virtual ~Linker() = default;

        interfaces::modes get_mode();

        void set_mode(interfaces::modes new_mode);

        std::string get_name();

        void set_name(std::string new_name);

        std::string get_topic();

        void set_topic(std::string new_name);

        void set_service(std::string new_name);

        void set_interface(std::string name_, std::string topic_name_);

        bool is_available(void);

        bool is_configured(void);

        void set_available(bool new_state);

        void set_configured(bool new_state);

        std::string name{"linker"};

        std::string topic_name{"/interface"};

        std::shared_ptr<interfaces::Interface> interface;

    protected:
        interfaces::modes mode_;

        bool available_{false};
        
        bool configured_{false};
    };
} // namespace roboligo

#endif // ROBOLIGO_COMMON_TYPES__LINK_HPP_
