#include "roboligo_robot_drone/RoboligoRobotDrone.hpp"

namespace roboligo
{
    void 
    RoboligoRobotDrone::on_initialize()
    {
        auto node = get_node();
        const auto & plugin_name = get_plugin_name();
        //Parameters
        node->declare_parameter<bool>(plugin_name + ".verbose", verbose_);
        node->get_parameter<bool>(plugin_name + ".verbose", verbose_);

        //Publishers

        set_modes();
        set_triggers();

    }

    void 
    RoboligoRobotDrone::set_modes()
    {
        modes_ = {
            standby,
            offboard
        };
    }

    void 
    RoboligoRobotDrone::set_triggers()
    {
        triggers_ = {
            arming,
            takeoff,
            landing,
            offboarding,
            disarming,
            standingby
        };
    }

    void 
    RoboligoRobotDrone::on_set(RobotState & robot_state)
    {
        auto node = get_node();
        RCLCPP_INFO(node->get_logger(), "Robot Drone --> Starting On set");
        robot_state.set_available(true);
        robot_state.set_simulation(true);
        robot_state.set_modes(modes_);
        robot_state.set_triggers(triggers_);
        RCLCPP_INFO(node->get_logger(), "Robot Drone --> Finishing On set");
    }

} // namespace roboligo

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(roboligo::RoboligoRobotDrone, roboligo::ClassificationBase);
