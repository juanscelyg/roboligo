#include "roboligo_common/classification/ClassificationBase.hpp"

namespace roboligo
{
    void 
    ClassificationBase::initialize(
        const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
        const std::string & plugin_name)
    {
        auto node = parent_node;

        // Publishers

        // Parameters

        PluginBase::initialize(parent_node, plugin_name);
    }

    bool
    ClassificationBase::set(RobotState & robot_state)
    {
        on_set(robot_state);
        return true;
    }

    bool
    ClassificationBase::update(RobotState & robot_state)
    {
        on_update(robot_state);
        return true;
    }

} // namespace roboligo