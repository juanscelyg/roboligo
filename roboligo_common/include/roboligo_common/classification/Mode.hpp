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
    class Mode
    {
        public:
        Mode(std::string name, std::string value)
            :name_(name), value_(value) {}

        virtual ~Mode() = default;

        std::string get_name(void);

        std::string get_value(void);

        void set_name(std::string new_name);

        void set_value(std::string new_value);

        bool is_ative(void);

        void set_active(bool status);

        protected:

        std::string name_{"base_mode"};

        std::string value_{"MODE"};

        bool active_{false};

    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CLASSIFICATION__MODE_HPP_