#ifndef ROBOLIGO_COMMON_CONNECTOR_SENSORS__BATTERY_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_SENSORS__BATTERY_HPP_

#include <sensor_msgs/msg/battery_state.hpp>

#include "roboligo_common/connector/Sensor.hpp"

namespace roboligo
{
    class Battery : public Sensor
    {
    public:
        Battery(std::string new_name, std::string new_topic)
            : Sensor(new_name, new_topic){}

        virtual ~Battery() = default;

        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscriber;  

        sensor_msgs::msg::BatteryState::SharedPtr data;

        void callback(sensor_msgs::msg::BatteryState::SharedPtr msg);
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_SENSORS__BATTERY_HPP_