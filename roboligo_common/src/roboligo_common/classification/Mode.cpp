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

#include "roboligo_common/classification/Mode.hpp"

namespace roboligo
{
    std::string 
    Mode::get_name(void)
    {
        return name_;
    }

    std::string 
    Mode::get_value(void)
    {
        return value_;
    }

    void 
    Mode::set_name(std::string new_name)
    {
        name_ = new_name;
    }

    void 
    Mode::set_value(std::string new_value)
    {
        value_ = new_value;
    }

    bool 
    Mode::is_ative(void){
        return active_;
    }

    void 
    Mode::set_active(bool status)
    {
        active_ = status;
    }
} // namespace roboligo