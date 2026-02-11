#ifndef ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__INPUT_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__INPUT_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "roboligo_common/connector/Commander.hpp"

namespace roboligo
{
    struct RoboligoInputTwist{
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
        geometry_msgs::msg::Twist::SharedPtr data;
    };

    struct RoboligoInputTwistStamped{
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber;
        geometry_msgs::msg::TwistStamped::SharedPtr data;
    };

    class Input : public Commander
    {
    public:
        Input(std::string new_name, std::string new_topic);

        virtual ~Input() = default;

        RoboligoInputTwist twist;

        RoboligoInputTwistStamped twist_stamped;
    
        void callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        
        void callback_stamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__INPUT_HPP_
