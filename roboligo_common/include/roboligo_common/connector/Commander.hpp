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
    class Commander : public Linker
    {
    public:
        Commander(std::string new_name, std::string new_value);

        virtual ~Commander() = default;

        void set_stamp(bool stamped);

        bool is_stamped(void);

        void init(std::string new_name, std::string new_value);
        
    protected:
        void initialize(std::string new_name, std::string new_topic);
        
        bool use_stamp_{true};
    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR__COMMANDER_HPP_