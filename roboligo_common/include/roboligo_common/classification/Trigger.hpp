#ifndef ROBOLIGO_COMMON_CLASSIFICATION__TRIGGER_HPP_
#define ROBOLIGO_COMMON_CLASSIFICATION__TRIGGER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <iostream>

#include "roboligo_common/classification/Mode.hpp"

namespace roboligo 
{
    class Trigger
    {
        public:
        Trigger(std::string name, std::string value, std::vector<Mode> previous, std::vector<Mode> next)
            :name_(name), value_(value), previous_(std::move(previous)), next_(std::move(next)) {}

        virtual ~Trigger() = default;

        std::string get_name(void);

        std::string get_value(void);

        void set_name(std::string new_name);

        void set_value(std::string new_value);

        void set_previous_modes(std::vector<Mode> modes);

        void set_next_modes(std::vector<Mode> modes);

        bool is_available(void);

        void register_callback(const std::function<void()>& callback);

        void execute(void);

        std::function<void()> cb;

        protected:

        std::string name_;
        
        std::string value_;

        std::vector<Mode> previous_;

        std::vector<Mode> next_;

        bool available_{false};

    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CLASSIFICATION__TRIGGER_HPP_