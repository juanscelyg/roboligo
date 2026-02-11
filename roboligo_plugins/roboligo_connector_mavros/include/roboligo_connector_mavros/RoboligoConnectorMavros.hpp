#ifndef ROBOLIGO_CONNECTOR_MAVROS__ROBOLIGOCONNECTORMAVROS_HPP_
#define ROBOLIGO_CONNECTOR_MAVROS__ROBOLIGOCONNECTORMAVROS_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"

#include "mavros_msgs/srv/param_set_v2.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include "roboligo_common/connector/ConnectorBase.hpp"

#include "roboligo_common/connector/commanders/Service.hpp"

#include "roboligo_common/types/RobotState.hpp"

namespace roboligo
{
    class RoboligoConnectorMavros : public ConnectorBase
    {
        public:
            RoboligoConnectorMavros() {}

            ~RoboligoConnectorMavros() = default;

            void on_initialize() override;

            void on_set(RobotState & robot_state) override;

            void on_update(RobotState & robot_state) override;

            void on_exit([[maybe_unused]]RobotState & robot_state){};

            void disarming_callback(void);

            void arming_callback(void);

            void takingoff_callback(void);

            void landing_callback(void);

            void offboarding_callback(void);

            void standingby_callback(void);

            void set_data_loss_exception(int value);

            void set_data_loss_offboard_time(double time);

            void set_data_loss_action(int value);

            void set_time_disarm_preflight(double time);

            void set_rcl_loss_exception(int value);

            std::shared_ptr<roboligo::Service> disarming_interface;

            std::shared_ptr<roboligo::Service> arming_interface;

            std::shared_ptr<roboligo::Service> takingoff_interface;

            std::shared_ptr<roboligo::Service> landing_interface;

            std::shared_ptr<roboligo::Service> params_interface;

            std::shared_ptr<roboligo::Service> offboarding_interface;

            std::shared_ptr<roboligo::Service> standingby_interface;

            rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr params_change;

            rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming;

            rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr disarming;

            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr takingoff;

            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr landing;

            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr offboarding;

            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr standingby;

            std::string takingoff_value;

            std::string landing_value;

            std::string offboarding_value;

            std::string standingby_value;

        protected:
            bool verbose_{false}; 

    };
} // namespace roboligo
#endif // ROBOLIGO_CONNECTOR_MAVROS__ROBOLIGOCONNECTORMAVROS_HPP_