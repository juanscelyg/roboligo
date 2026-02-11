#ifndef ROBOLIGO_STATES__STATESNODE_HPP_
#define ROBOLIGO_STATES__STATESNODE_HPP_

#include "pluginlib/class_loader.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "roboligo_common/classification/ClassificationBase.hpp"
#include "roboligo_common/types/RobotState.hpp"

namespace roboligo
{

    class StatesNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:

    RCLCPP_SMART_PTR_DEFINITIONS(StatesNode)

    using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit StatesNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~StatesNode();

    CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

    CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

    bool init(std::shared_ptr<RobotState> robot_state);

    bool cycle(std::shared_ptr<RobotState> robot_state);

    void reset_classification(void);

    private:

    std::shared_ptr<ClassificationBase> classification_ {nullptr};

    const std::shared_ptr<const RobotState> robot_state_;

    std::unique_ptr<pluginlib::ClassLoader<roboligo::ClassificationBase>> classification_loader_;
    };

}  // namespace roboligo

#endif  // ROBOLIGO_STATES__STATESNODE_HPP_
