#ifndef ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__OUTPUT_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__OUTPUT_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "roboligo_common/connector/Commander.hpp"

namespace roboligo
{
    struct RoboligoOutputTwist{
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        geometry_msgs::msg::Twist::UniquePtr data;
    };

    struct RoboligoOutputTwistStamped{
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher;
        geometry_msgs::msg::TwistStamped::UniquePtr data;
    };

    class Output : public Commander
    {
    public:
        Output(std::string new_name, std::string new_topic);

        virtual ~Output() = default;

        RoboligoOutputTwist twist;

        RoboligoOutputTwistStamped twist_stamped;

        void set_data(const geometry_msgs::msg::Twist* msg);

        void set_data_stamped(const geometry_msgs::msg::TwistStamped* msg);
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__OUTPUT_HPP_
