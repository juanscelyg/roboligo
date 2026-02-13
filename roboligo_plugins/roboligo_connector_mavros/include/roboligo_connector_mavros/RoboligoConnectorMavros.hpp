//  Copyright 2026 Juan S. Cely G.

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      https://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef ROBOLIGO_CONNECTOR_MAVROS__ROBOLIGOCONNECTORMAVROS_HPP_
#define ROBOLIGO_CONNECTOR_MAVROS__ROBOLIGOCONNECTORMAVROS_HPP_

#include <chrono>
#include <vector>

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
    /**
    * @class RoboligoConnectorMavros
    * @brief Connector class that interfaces with MAVROS for drone control and management.
    * 
    * This class extends ConnectorBase and provides functionality to manage drone operations
    * including arming, disarming, takeoff, landing, and offboarding through MAVROS services.
    * It handles communication with the autopilot via ROS 2 services and manages various
    * drone states and callbacks.
    */
    class RoboligoConnectorMavros : public ConnectorBase
    {
        public:
            /**
            * @fn RoboligoConnectorMavros()
            * @brief Default constructor for RoboligoConnectorMavros.
            */
            RoboligoConnectorMavros() {}

            /**
            * @fn ~RoboligoConnectorMavros()
            * @brief Default destructor for RoboligoConnectorMavros.
            */
            ~RoboligoConnectorMavros() = default;

            /**
            * @fn void on_initialize() override
            * @brief Initializes the connector and sets up all service clients and interfaces.
            */
            void on_initialize() override;

            /**
            * @fn void on_set(RobotState & robot_state) override
            * @brief Sets the robot state and applies necessary configurations.
            * @param robot_state Reference to the robot state object to be set.
            */
            void on_set(RobotState & robot_state) override;

            /**
            * @fn void on_update(RobotState & robot_state) override
            * @brief Updates the robot state with current system information.
            * @param robot_state Reference to the robot state object to be updated.
            */
            void on_update(RobotState & robot_state) override;

            /**
            * @fn void on_exit(RobotState & robot_state)
            * @brief Handles cleanup when the connector exits.
            * @param robot_state Reference to the robot state object.
            */
            void on_exit([[maybe_unused]]RobotState & robot_state){};

            /**
            * @fn void disarming_callback()
            * @brief Callback function triggered when disarming is requested.
            */
            void disarming_callback(void);

            /**
            * @fn void arming_callback()
            * @brief Callback function triggered when arming is requested.
            */
            void arming_callback(void);

            /**
            * @fn void takingoff_callback()
            * @brief Callback function triggered when takeoff is initiated.
            */
            void takingoff_callback(void);

            /**
            * @fn void landing_callback()
            * @brief Callback function triggered when landing is initiated.
            */
            void landing_callback(void);

            /**
            * @fn void offboarding_callback()
            * @brief Callback function triggered when offboarding is requested.
            */
            void offboarding_callback(void);

            /**
            * @fn void standingby_callback()
            * @brief Callback function triggered when standby mode is activated.
            */
            void standingby_callback(void);

            /**
            * @fn void set_data_loss_exception(int value)
            * @brief Sets the data loss exception behavior.
            * @param value Configuration value for data loss exception handling.
            */
            void set_data_loss_exception(int value);

            /**
            * @fn void set_data_loss_offboard_time(double time)
            * @brief Sets the time threshold for offboard action on data loss.
            * @param time Time in seconds before triggering offboard action.
            */
            void set_data_loss_offboard_time(double time);

            /**
            * @fn void set_data_loss_action(int value)
            * @brief Sets the action to perform when data loss is detected.
            * @param value Action type identifier.
            */
            void set_data_loss_action(int value);

            /**
            * @fn void set_time_disarm_preflight(double time)
            * @brief Sets the preflight disarm timeout.
            * @param time Time in seconds for preflight disarm.
            */
            void set_time_disarm_preflight(double time);

            /**
            * @fn void set_rcl_loss_exception(int value)
            * @brief Sets the RCL (RC Link) loss exception behavior.
            * @param value Configuration value for RCL loss exception handling.
            */
            void set_rcl_loss_exception(int value);

            /**
            * @var disarming_interface
            * @brief Roboligo service interface for disarming operations.
            */
            std::shared_ptr<roboligo::Service> disarming_interface;

            /**
            * @var arming_interface
            * @brief Roboligo service interface for arming operations.
            */
            std::shared_ptr<roboligo::Service> arming_interface;

            /**
            * @var takingoff_interface
            * @brief Roboligo service interface for takeoff operations.
            */
            std::shared_ptr<roboligo::Service> takingoff_interface;

            /**
            * @var landing_interface
            * @brief Roboligo service interface for landing operations.
            */
            std::shared_ptr<roboligo::Service> landing_interface;

            /**
            * @var params_interface
            * @brief Roboligo service interface for parameter management.
            */
            std::shared_ptr<roboligo::Service> params_interface;

            /**
            * @var offboarding_interface
            * @brief Roboligo service interface for offboarding operations.
            */
            std::shared_ptr<roboligo::Service> offboarding_interface;

            /**
            * @var standingby_interface
            * @brief Roboligo service interface for standby mode operations.
            */
            std::shared_ptr<roboligo::Service> standingby_interface;

            rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr params_change; ///< ROS 2 service client for changing MAVROS parameters

            rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming; ///< ROS 2 service client for arming the drone

            rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr disarming; ///< ROS 2 service client for disarming the drone

            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr takingoff; ///< ROS 2 service client for setting takeoff flight mode

            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr landing; ///< ROS 2 service client for setting landing flight mode

            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr offboarding; ///< ROS 2 service client for setting offboard flight mode

            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr standingby; ///< ROS 2 service client for setting standby flight mode

            std::string takingoff_value; ///< String identifier for takeoff flight mode

            std::string landing_value; ///< String identifier for landing flight mode

            std::string offboarding_value; ///< String identifier for offboard flight mode

            std::string standingby_value; ///< String identifier for standby flight mode

        protected:
            bool verbose_{false}; ///< Flag to enable verbose logging output. Default is false

    };
} // namespace roboligo
#endif // ROBOLIGO_CONNECTOR_MAVROS__ROBOLIGOCONNECTORMAVROS_HPP_