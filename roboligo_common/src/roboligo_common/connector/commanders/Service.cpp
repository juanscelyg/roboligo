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