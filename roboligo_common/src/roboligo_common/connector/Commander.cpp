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