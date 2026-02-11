#ifndef ROBOLIGO_COMMON_CONNECTOR_SENSORS__IMU_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_SENSORS__IMU_HPP_

#include <sensor_msgs/msg/imu.hpp>

#include "roboligo_common/connector/Sensor.hpp"

namespace roboligo
{
    class Imu : public Sensor
    {
    public:
        Imu(std::string new_name, std::string new_topic)
            : Sensor(new_name, new_topic){}

        virtual ~Imu() = default;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber;  

        sensor_msgs::msg::Imu::SharedPtr data;

        void callback(sensor_msgs::msg::Imu::SharedPtr msg);
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_SENSORS__IMU_HPP_
