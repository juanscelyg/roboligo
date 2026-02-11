#include "roboligo_common/connector/sensors/NavSat.hpp"

namespace roboligo
{
    void 
    NavSat::callback(sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        set_available(true);
        data = msg;
    }

} // namespace roboligo