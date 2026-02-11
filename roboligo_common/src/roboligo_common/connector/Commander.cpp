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

#include "roboligo_common/connector/Commander.hpp"

namespace roboligo 
{
    Commander::Commander(std::string new_name, std::string new_value)
    {
        initialize(new_name, new_value);
    }

    void
    Commander::initialize(std::string new_name, std::string new_value)
    {
        set_name(new_name);
        set_topic(new_value);
    }

    void
    Commander::init(std::string new_name, std::string new_value)
    {
        initialize(new_name, new_value);
        set_configured(true);
    }

    void 
    Commander::set_stamp(bool stamped)
    {
        use_stamp_ = stamped;
    }

    bool 
    Commander::is_stamped(void)
    {
        return use_stamp_;
    }

} // namespace roboligo