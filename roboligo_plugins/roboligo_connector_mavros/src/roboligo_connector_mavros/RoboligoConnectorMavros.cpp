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

#include "roboligo_connector_mavros/RoboligoConnectorMavros.hpp"

using namespace std::chrono_literals;

namespace roboligo
{
    void 
    RoboligoConnectorMavros::on_initialize()
    {
        auto node = get_node();
        const auto & plugin_name = get_plugin_name();
        // Parameters
        node->declare_parameter<bool>(plugin_name + ".verbose", verbose_);

        node->get_parameter<bool>(plugin_name + ".verbose", verbose_);

        // Service

        arming_interface = std::make_shared<roboligo::Service>("arming",
            "/mavros/cmd/arming");

        disarming_interface = std::make_shared<roboligo::Service>("disarming",
            "/mavros/cmd/arming");
        
        takingoff_interface = std::make_shared<roboligo::Service>("takingoff",
            "/mavros/set_mode");

        landing_interface = std::make_shared<roboligo::Service>("landing",
            "/mavros/set_mode");

        offboarding_interface = std::make_shared<roboligo::Service>("offboarding",
            "/mavros/set_mode");

        standingby_interface = std::make_shared<roboligo::Service>("standingby",
            "/mavros/set_mode");

        params_interface = std::make_shared<roboligo::Service>("params",
            "/mavros/param/set");

        //services
        arming = node->create_client<mavros_msgs::srv::CommandBool>(
            arming_interface->get_topic());

        disarming = node->create_client<mavros_msgs::srv::CommandBool>(
            arming_interface->get_topic());

        takingoff = node->create_client<mavros_msgs::srv::SetMode>(
            takingoff_interface->get_topic());

        landing = node->create_client<mavros_msgs::srv::SetMode>(
            landing_interface->get_topic());

        offboarding = node->create_client<mavros_msgs::srv::SetMode>(
            offboarding_interface->get_topic());

        standingby = node->create_client<mavros_msgs::srv::SetMode>(
            standingby_interface->get_topic());

        params_change = node->create_client<mavros_msgs::srv::ParamSetV2>(
            params_interface->get_topic());

    }    
    
    void
    RoboligoConnectorMavros::on_set(RobotState & robot_state)
    {
        auto node = get_node();
        RCLCPP_INFO(node->get_logger(), "Mavros Connector --> Starting On set");
                
        takingoff_value = robot_state.get_trigger("takeoff")->get_value();
        landing_value = robot_state.get_trigger("landing")->get_value();
        offboarding_value = robot_state.get_trigger("offboard")->get_value();
        standingby_value = robot_state.get_trigger("standby")->get_value();
        RCLCPP_INFO(node->get_logger(), "Mavros Connector takeoff set--> %s", takingoff_value.c_str());
        RCLCPP_INFO(node->get_logger(), "Mavros Connector landing set --> %s", landing_value.c_str());
        RCLCPP_INFO(node->get_logger(), "Mavros Connector offboard set --> %s", offboarding_value.c_str());
        RCLCPP_INFO(node->get_logger(), "Mavros Connector StandBy set --> %s", standingby_value.c_str());

        std::function<void()> arming_callback = std::bind(&RoboligoConnectorMavros::arming_callback, this);
        robot_state.get_trigger("arming")->register_callback(arming_callback);

        std::function<void()> disarming_callback = std::bind(&RoboligoConnectorMavros::disarming_callback, this);
        robot_state.get_trigger("disarming")->register_callback(disarming_callback);

        std::function<void()> takingoff_callback = std::bind(&RoboligoConnectorMavros::takingoff_callback, this);
        robot_state.get_trigger("takeoff")->register_callback(takingoff_callback);

        std::function<void()> landing_callback = std::bind(&RoboligoConnectorMavros::landing_callback, this);
        robot_state.get_trigger("landing")->register_callback(landing_callback);

        std::function<void()> offboarding_callback = std::bind(&RoboligoConnectorMavros::offboarding_callback, this);
        robot_state.get_trigger("offboard")->register_callback(offboarding_callback);
        
        std::function<void()> standingby_callback = std::bind(&RoboligoConnectorMavros::standingby_callback, this);
        robot_state.get_trigger("standby")->register_callback(standingby_callback);
        
        robot_state.imu->init("imu", "/mavros/imu/data");
        robot_state.gps->init("gps", "/mavros/global_position/global");
        robot_state.battery->init("baterry", "/mavros/battery");
        robot_state.odom->init("odom", "/mavros/local_position/odom");
        robot_state.position_target->init("output_position_target", "/mavros/setpoint_raw/local");
        robot_state.input->init("input", "/cmd_vel");
        robot_state.input->set_stamp(robot_state.is_stamped());

        set_data_loss_exception(7);
        set_rcl_loss_exception(2);
        set_data_loss_action(0);
        set_time_disarm_preflight(60.0);
        set_data_loss_offboard_time(30.0);

        RCLCPP_INFO(node->get_logger(), "Mavros Connector --> Finishing On set");
    }
    
    void 
    RoboligoConnectorMavros::on_update(RobotState & robot_state)
    {
        auto node = get_node();
        // RCLCPP_INFO_STREAM(node->get_logger(), "Available: " << robot_state.is_available() <<
        //                                         "Available input: " << robot_state.input->is_available() <<
        //                                         "Position targe configured: " << robot_state.position_target->is_configured());
        if(robot_state.is_available() && robot_state.input->is_available() && robot_state.position_target->is_configured())
        {     
            auto msg = mavros_msgs::msg::PositionTarget();      
            msg.coordinate_frame = robot_state.position_target->position_target.coordinate_frame;
            msg.type_mask = robot_state.position_target->position_target.type_mask;

            geometry_msgs::msg::Vector3 velocity;
            float yaw_rate;

            if(robot_state.input->is_stamped()){
                msg.header.stamp = robot_state.input->twist_stamped.data->header.stamp;

                velocity.x = robot_state.input->twist_stamped.data->twist.linear.x;
                velocity.y = robot_state.input->twist_stamped.data->twist.linear.y;
                velocity.z = robot_state.input->twist_stamped.data->twist.linear.z;

                yaw_rate = robot_state.input->twist_stamped.data->twist.angular.z;


            } else {
                msg.header.stamp = node->get_clock()->now();

                velocity.x = robot_state.input->twist.data->linear.x;
                velocity.y = robot_state.input->twist.data->linear.y;
                velocity.z = robot_state.input->twist.data->linear.z;

                yaw_rate = robot_state.input->twist.data->angular.z;

            }
            msg.velocity = velocity;
            msg.yaw_rate = yaw_rate;
            robot_state.position_target->position_target.publisher->publish(msg);

        }
    }

    void
    RoboligoConnectorMavros::arming_callback()
    {
        auto node = get_node();
        RCLCPP_INFO_STREAM(node->get_logger(), "Arming was required"); 
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
        while (!arming->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << arming_interface->get_topic() <<" not available, waiting again..."); 
        }
        auto result = arming->async_send_request(request);
    }

    void
    RoboligoConnectorMavros::disarming_callback()
    {
        auto node = get_node();
        RCLCPP_INFO_STREAM(node->get_logger(), "Disarming was required"); 
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = false;
        while (!disarming->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << disarming_interface->get_topic() <<" not available, waiting again..."); 
        }
        auto result = disarming->async_send_request(request);
    }

    void
    RoboligoConnectorMavros::takingoff_callback()
    {
        auto node = get_node();
        RCLCPP_INFO_STREAM(node->get_logger(), "Taking off was required"); 
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = takingoff_value;
        while (!takingoff->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << takingoff_interface->get_topic() <<" not available, waiting again..."); 
        }
        auto result = takingoff->async_send_request(request);
    }

    void
    RoboligoConnectorMavros::landing_callback()
    {
        auto node = get_node();
        RCLCPP_INFO_STREAM(node->get_logger(), "Landing was required"); 
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = landing_value;

        while (!landing->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << landing_interface->get_topic() <<" not available, waiting again..."); 
        }
        auto result = landing->async_send_request(request);
    }

    void
    RoboligoConnectorMavros::offboarding_callback()
    {
        auto node = get_node();
        RCLCPP_INFO_STREAM(node->get_logger(), "Offboard mode was required"); 
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = offboarding_value;

        while (!offboarding->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << offboarding_interface->get_topic() <<" not available, waiting again..."); 
        }
        auto result = offboarding->async_send_request(request);
    }

    void
    RoboligoConnectorMavros::standingby_callback()
    {
        auto node = get_node();
        RCLCPP_INFO_STREAM(node->get_logger(), "StandBy mode was required"); 
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = standingby_value;

        while (!standingby->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << standingby_interface->get_topic() <<" not available, waiting again..."); 
        }
        auto result = standingby->async_send_request(request);
    }

    void
    RoboligoConnectorMavros::set_data_loss_exception(int value)
    {
        auto node = get_node();
        RCLCPP_INFO_STREAM(node->get_logger(), "DLL Exception will be change."); 
        auto request = std::make_shared<mavros_msgs::srv::ParamSetV2_Request>();

        request->force_set = true;
        request->param_id = "COM_DLL_EXCEPT";
        request->value.type = 2;         
        request->value.integer_value = value; 

        while (!params_change->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << params_interface->get_topic() <<" not available, waiting again..."); 
        }
        auto result = params_change->async_send_request(request);
    }

    void
    RoboligoConnectorMavros::set_data_loss_offboard_time(double time)
    {
        auto node = get_node();
        RCLCPP_INFO_STREAM(node->get_logger(), "DLL Time in Offboard mode will be change."); 
        auto request = std::make_shared<mavros_msgs::srv::ParamSetV2_Request>();

        request->force_set = true;
        request->param_id = "COM_OF_LOSS_T";
        request->value.type = 3;         
        request->value.double_value = time; 

        while (!params_change->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << params_interface->get_topic() <<" not available, waiting again..."); 
        }
        auto result = params_change->async_send_request(request);
        
    }

    void
    RoboligoConnectorMavros::set_data_loss_action(int value)
    {
        auto node = get_node();
        RCLCPP_INFO_STREAM(node->get_logger(), "DLL Action will be change."); 
        auto request = std::make_shared<mavros_msgs::srv::ParamSetV2_Request>();

        request->force_set = true;
        request->param_id = "NAV_DLL_ACT";
        request->value.type = 2;         
        request->value.integer_value = value; 

        while (!params_change->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << params_interface->get_topic() <<" not available, waiting again..."); 
        }
        auto result = params_change->async_send_request(request);
    }

    void
    RoboligoConnectorMavros::set_time_disarm_preflight(double time)
    {
        auto node = get_node();
        RCLCPP_INFO_STREAM(node->get_logger(), "Time DISARM in preflight will be change."); 
        auto request = std::make_shared<mavros_msgs::srv::ParamSetV2_Request>();

        request->force_set = true;
        request->param_id = "COM_DISARM_PRFLT";
        request->value.type = 3;         
        request->value.double_value = time; 

        while (!params_change->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << params_interface->get_topic() <<" not available, waiting again..."); 
        }
        auto result = params_change->async_send_request(request);
    }

    void
    RoboligoConnectorMavros::set_rcl_loss_exception(int value)
    {
        auto node = get_node();
        RCLCPP_INFO_STREAM(node->get_logger(), "RCL except will be change."); 
        auto request = std::make_shared<mavros_msgs::srv::ParamSetV2_Request>();

        request->force_set = true;
        request->param_id = "COM_RCL_EXCEPT";
        request->value.type = 2;         
        request->value.integer_value = value; 

        while (!params_change->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "service " << params_interface->get_topic() <<" not available, waiting again..."); 
        }
        auto result = params_change->async_send_request(request);   
    }


} // namespace roboligo

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(roboligo::RoboligoConnectorMavros, roboligo::ConnectorBase);           
           

