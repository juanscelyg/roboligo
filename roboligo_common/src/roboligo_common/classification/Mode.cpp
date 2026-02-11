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