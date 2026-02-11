#ifndef ROBOLIGO_CONNECTOR__CONNECTORNODE_HPP_
#define ROBOLIGO_CONNECTOR__CONNECTORNODE_HPP_


#include "pluginlib/class_loader.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "roboligo_common/connector/ConnectorBase.hpp"
#include "roboligo_common/types/RobotState.hpp"

namespace roboligo
{

    class ConnectorNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:

    RCLCPP_SMART_PTR_DEFINITIONS(ConnectorNode)

    using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit ConnectorNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    virtual ~ConnectorNode();

    CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

    bool init(std::shared_ptr<RobotState> robot_state);

    void reset_connector(void);

    bool cycle(std::shared_ptr<RobotState> robot_state);

    private:

    std::shared_ptr<ConnectorBase> connector_ {nullptr};

    const std::shared_ptr<const RobotState> robot_state_;

    std::shared_ptr<pluginlib::ClassLoader<roboligo::ConnectorBase>> connector_loader_;
    };

}  // namespace roboligo

#endif  // ROBOLIGO_CONNECTOR__CONNECTORNODE_HPP_
