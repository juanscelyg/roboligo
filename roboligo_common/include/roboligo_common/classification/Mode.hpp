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