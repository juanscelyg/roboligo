#include "roboligo_common/connector/Sensor.hpp"

namespace roboligo
{
    Sensor::Sensor(std::string new_name, std::string new_topic)
    {
        initialize(new_name, new_topic);
    }

    void
    Sensor::initialize(std::string new_name, std::string new_topic)
    {
        set_name(new_name);
        set_topic(new_topic); 
        set_interface(name, topic_name);
    }

    void
    Sensor::init(std::string new_name, std::string new_topic)
    {
        initialize(new_name, new_topic);
        set_configured(true);
    }

} // namespace roboligo