#ifndef ROBOLIGO_COMMON_CONNECTOR_SENSORS__ODOM_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_SENSORS__ODOM_HPP_

#include <nav_msgs/msg/odometry.hpp>

#include "roboligo_common/connector/Sensor.hpp"

namespace roboligo
{
    class Odom : public Sensor
    {
    public:
        Odom(std::string new_name, std::string new_topic)
            : Sensor(new_name, new_topic){}

        virtual ~Odom() = default;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;  

        nav_msgs::msg::Odometry::SharedPtr data;

        void callback(nav_msgs::msg::Odometry::SharedPtr msg);
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_SENSORS__ODOM_HPP_