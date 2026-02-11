#include "roboligo_common/connector/commanders/Input.hpp"

namespace roboligo
{
    Input::Input(std::string new_name, std::string new_topic)
        : Commander(new_name, new_topic)
    {
        set_mode(interfaces::modes::SUBSCRIBER);
        set_interface(name, topic_name);
    }

    void 
    Input::callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        set_available(true);
        twist.data = msg;
    }

    void 
    Input::callback_stamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        set_available(true);
        twist_stamped.data = msg;
    }

}// namespace roboligo