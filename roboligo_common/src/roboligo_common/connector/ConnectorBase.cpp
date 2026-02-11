#include "roboligo_common/connector/ConnectorBase.hpp"

namespace roboligo
{
    void 
    ConnectorBase::initialize(
        const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
        const std::string & plugin_name)
    {
        auto node = parent_node;

        // Publishers

        // Parameters

        PluginBase::initialize(parent_node, plugin_name);
    }

    bool 
    ConnectorBase::set(RobotState & robot_state)
    {
        on_set(robot_state);
        return true;
    }

    bool
    ConnectorBase::update(RobotState & robot_state)
    {
        on_update(robot_state);
        return true;
    }

    bool
    ConnectorBase::exit(RobotState & robot_state)
    {
        on_exit(robot_state);
        return true;
    }

} // namespace roboligo