#include "roboligo_common/connector/sensors/Battery.hpp"

namespace roboligo
{
    void 
    Battery::callback(sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        set_available(true);
        data = msg;
    }

} // namespace roboligo