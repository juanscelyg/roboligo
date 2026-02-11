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

#include "roboligo_common/connector/commanders/Service.hpp"

namespace roboligo
{
    Service::Service(std::string new_name, std::string new_value)
        : Commander(new_name, new_value)
    {
        set_mode(interfaces::modes::CALLER);
        set_interface(name, topic_name);
    }

    // (ToDo) Implementation

}// namespace roboligo