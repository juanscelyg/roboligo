#include "roboligo_common/types/PluginBase.hpp"

namespace roboligo
{
    void 
    PluginBase::initialize(
        const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
        const std::string & plugin_name)
    {
        parent_node_ = parent_node;
        plugin_name_ = plugin_name;

        parent_node_->declare_parameter(plugin_name + ".freq", frequency_);
        parent_node_->get_parameter(plugin_name + ".freq", frequency_);

        on_initialize();
    }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> 
    PluginBase::get_node() const
    {
        return parent_node_;
    }

    const std::string & 
    PluginBase::get_plugin_name() const
    {
        return plugin_name_;
    }
}// namespace roboligo