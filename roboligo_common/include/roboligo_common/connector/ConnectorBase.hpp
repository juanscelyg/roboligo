#ifndef ROBOLIGO_COMMON_CLASSIFICATION__CONNECTORBASE_HPP_
#define ROBOLIGO_COMMON_CLASSIFICATION__CONNECTORBASE_HPP_

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "roboligo_common/types/PluginBase.hpp"
#include "roboligo_common/connector/Sensor.hpp"
#include "roboligo_common/connector/Commander.hpp"
#include "roboligo_common/types/RobotState.hpp"

namespace roboligo 
{
    class ConnectorBase : public PluginBase
    {
        public:
        ConnectorBase() = default;
        virtual ~ConnectorBase() = default;

        virtual void initialize(
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
            const std::string & plugin_name);

        virtual bool set(RobotState & robot_state);

        virtual bool update(RobotState & robot_state);

        virtual bool exit(RobotState & robot_state);

        protected:

        virtual void on_set([[maybe_unused]]RobotState & robot_state){}

        virtual void on_update([[maybe_unused]]RobotState & robot_state){}

        virtual void on_exit([[maybe_unused]]RobotState & robot_state){}

    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CLASSIFICATION__CONNECTORBASE_HPP_