#ifndef ROBOLIGO_COMMON_CONNECTOR_SENSORS__NAVSAT_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_SENSORS__NAVSAT_HPP_

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "roboligo_common/connector/Sensor.hpp"

namespace roboligo
{
    class NavSat : public Sensor
    {
    public:
        NavSat(std::string new_name, std::string new_topic)
            : Sensor(new_name, new_topic){}

        virtual ~NavSat() = default;

        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber;  

        sensor_msgs::msg::NavSatFix::SharedPtr data;

        void callback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_SENSORS__NAVSAT_HPP_