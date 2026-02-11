#include "roboligo_common/connector/sensors/Imu.hpp"

namespace roboligo
{
    void 
    Imu::callback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        set_available(true);
        data = msg;
    }

} // namespace roboligo