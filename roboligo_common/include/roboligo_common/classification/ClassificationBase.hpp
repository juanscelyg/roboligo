#ifndef ROBOLIGO_COMMON_CLASSIFICATION__CLASSIFICATIONBASE_HPP_
#define ROBOLIGO_COMMON_CLASSIFICATION__CLASSIFICATIONBASE_HPP_

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "roboligo_common/types/PluginBase.hpp"
#include "roboligo_common/classification/Mode.hpp"
#include "roboligo_common/classification/Trigger.hpp"
#include "roboligo_common/types/RobotState.hpp"

namespace roboligo 
{
    class ClassificationBase : public PluginBase
    {
        public:
        ClassificationBase() = default;
        virtual ~ClassificationBase() = default;

        virtual void initialize(
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
            const std::string & plugin_name);

        virtual bool set(RobotState & robot_state);

        virtual bool update(RobotState & robot_state);

        protected:
        virtual void on_set([[maybe_unused]]RobotState & robot_state){}

        virtual void on_update([[maybe_unused]]RobotState & robot_state){}

    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CLASSIFICATION__CLASSIFICATIONBASE_HPP_