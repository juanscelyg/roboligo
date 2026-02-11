#include "roboligo_common/connector/commanders/Output.hpp"

namespace roboligo
{
    Output::Output(std::string new_name, std::string new_topic)
        : Commander(new_name, new_topic)
    {
        set_mode(interfaces::modes::PUBLISHER);
        set_interface(name, topic_name);
    }

    void 
    Output::set_data(const geometry_msgs::msg::Twist* msg)
    {
        twist.data = std::make_unique<geometry_msgs::msg::Twist>(*msg);
    }

    void 
    Output::set_data_stamped(const geometry_msgs::msg::TwistStamped* msg)
    {
        twist_stamped.data = std::make_unique<geometry_msgs::msg::TwistStamped>(*msg);
    }
}// namespace roboligo