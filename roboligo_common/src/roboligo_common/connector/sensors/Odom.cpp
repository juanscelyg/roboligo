#include "roboligo_common/connector/sensors/Odom.hpp"

namespace roboligo
{
    void 
    Odom::callback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        set_available(true);
        data = msg;
    }

} // namespace roboligo