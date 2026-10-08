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

#include "roboligo_connector_px4/RoboligoConnectorPX4.hpp"

using namespace std::chrono_literals;

namespace roboligo
{
void
RoboligoConnectorPX4::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<bool>(plugin_name + ".verbose", verbose_);
  node->declare_parameter<double>(plugin_name + ".init_alt", init_alt_);

  node->get_parameter<bool>(plugin_name + ".verbose", verbose_);
  node->get_parameter<double>(plugin_name + ".init_alt", init_alt_);

  arming_interface = std::make_shared<roboligo::Service>("arming",
            "/fmu/in/vehicle_command");

  disarming_interface = std::make_shared<roboligo::Service>("disarming",
            "/fmu/in/vehicle_command");

  takingoff_interface = std::make_shared<roboligo::Service>("takingoff",
            "/fmu/in/vehicle_command");

  landing_interface = std::make_shared<roboligo::Service>("landing",
            "/fmu/in/vehicle_command");

  offboarding_interface = std::make_shared<roboligo::Service>("offboarding",
            "/fmu/in/offboard_control_mode");

  standingby_interface = std::make_shared<roboligo::Service>("standingby",
            "/fmu/in/vehicle_command");

  params_interface = std::make_shared<roboligo::Service>("params",
            "/fmu/in/vehicle_command");

  vehicle_command_pub = node->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

  offboard_control_mode_pub = node->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

  trajectory_setpoint_pub = node->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

  vehicle_global_position_sub = node->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
              current_alt_amsl_ = msg->alt;
            });

  vehicle_status_sub = node->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
              if(verbose_) {
                auto node_logger = get_node()->get_logger();
                RCLCPP_DEBUG(node_logger, "Vehicle Status: armed=%d, mode=%d",
                  msg->arming_state, msg->nav_state);
              }
            });

}

void
RoboligoConnectorPX4::on_set(RobotState & robot_state)
{
  auto node = get_node();
  RCLCPP_INFO(node->get_logger(), "PX4 Connector --> Starting On set");

  takingoff_value = robot_state.get_trigger("takeoff")->get_value();
  landing_value = robot_state.get_trigger("landing")->get_value();
  offboarding_value = robot_state.get_trigger("offboard")->get_value();
  standingby_value = robot_state.get_trigger("standby")->get_value();
  RCLCPP_INFO(node->get_logger(), "PX4 Connector takeoff set--> %s", takingoff_value.c_str());
  RCLCPP_INFO(node->get_logger(), "PX4 Connector landing set --> %s", landing_value.c_str());
  RCLCPP_INFO(node->get_logger(), "PX4 Connector offboard set --> %s",
      offboarding_value.c_str());
  RCLCPP_INFO(node->get_logger(), "PX4 Connector StandBy set --> %s", standingby_value.c_str());

  std::function<void()> arming_callback = std::bind(&RoboligoConnectorPX4::arming_callback,
      this);
  robot_state.get_trigger("arming")->register_callback(arming_callback);

  std::function<void()> disarming_callback = std::bind(&RoboligoConnectorPX4::disarming_callback,
      this);
  robot_state.get_trigger("disarming")->register_callback(disarming_callback);

  std::function<void()> takingoff_callback = std::bind(&RoboligoConnectorPX4::takingoff_callback,
      this);
  robot_state.get_trigger("takeoff")->register_callback(takingoff_callback);

  std::function<void()> landing_callback = std::bind(&RoboligoConnectorPX4::landing_callback,
      this);
  robot_state.get_trigger("landing")->register_callback(landing_callback);

  std::function<void()> offboarding_callback =
    std::bind(&RoboligoConnectorPX4::offboarding_callback, this);
  robot_state.get_trigger("offboard")->register_callback(offboarding_callback);

  std::function<void()> standingby_callback =
    std::bind(&RoboligoConnectorPX4::standingby_callback, this);
  robot_state.get_trigger("standby")->register_callback(standingby_callback);

  robot_state.imu->init("imu", "/fmu/out/sensor_imu");
  robot_state.gps->init("gps", "/fmu/out/vehicle_gps_position");
  robot_state.battery->init("battery", "/fmu/out/battery_status");
  robot_state.odom->init("odom", "/fmu/out/vehicle_odometry");
  robot_state.position_target->init("output_position_target", "/fmu/in/trajectory_setpoint");
  robot_state.input->init("input", "/cmd_vel");
  robot_state.input->set_stamp(robot_state.is_stamped());

  RCLCPP_INFO(node->get_logger(), "PX4 Connector --> Finishing On set");
}

void
RoboligoConnectorPX4::on_update(RobotState & robot_state)
{
  auto node = get_node();

  if(robot_state.is_available() && robot_state.input->is_available() &&
    robot_state.position_target->is_configured())
  {
    auto msg = px4_msgs::msg::TrajectorySetpoint();
    msg.timestamp = node->get_clock()->now().nanoseconds() / 1000;

    float vx = 0.0f, vy = 0.0f, vz = 0.0f, yaw_rate = 0.0f;

    if(robot_state.input->is_stamped()) {
      vx = robot_state.input->twist_stamped.data->twist.linear.x;
      vy = robot_state.input->twist_stamped.data->twist.linear.y;
      vz = robot_state.input->twist_stamped.data->twist.linear.z;
      yaw_rate = robot_state.input->twist_stamped.data->twist.angular.z;
    } else {
      vx = robot_state.input->twist.data->linear.x;
      vy = robot_state.input->twist.data->linear.y;
      vz = robot_state.input->twist.data->linear.z;
      yaw_rate = robot_state.input->twist.data->angular.z;
    }

    msg.velocity[0] = vx;
    msg.velocity[1] = vy;
    msg.velocity[2] = vz;
    msg.yawspeed = yaw_rate;

    trajectory_setpoint_pub->publish(msg);
  }
}

void
RoboligoConnectorPX4::arming_callback()
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "Arming was required");

  auto cmd = px4_msgs::msg::VehicleCommand();
  cmd.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  cmd.param1 = 1.0f;
  cmd.param2 = 0.0f;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;

  vehicle_command_pub->publish(cmd);
}

void
RoboligoConnectorPX4::disarming_callback()
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "Disarming was required");

  auto cmd = px4_msgs::msg::VehicleCommand();
  cmd.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  cmd.param1 = 0.0f;
  cmd.param2 = 0.0f;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;

  vehicle_command_pub->publish(cmd);
}

void
RoboligoConnectorPX4::takingoff_callback()
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "Taking off was required");

  node->get_parameter<double>(get_plugin_name() + ".init_alt", init_alt_);

  auto cmd = px4_msgs::msg::VehicleCommand();
  cmd.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  // param7 is AMSL; NaN makes PX4 fall back to MIS_TAKEOFF_ALT
  cmd.param4 = NAN;
  cmd.param5 = NAN;
  cmd.param6 = NAN;
  cmd.param7 = std::isfinite(current_alt_amsl_) ?
    static_cast<float>(current_alt_amsl_ + init_alt_) : NAN;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;

  vehicle_command_pub->publish(cmd);
}

void
RoboligoConnectorPX4::landing_callback()
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "Landing was required");

  auto cmd = px4_msgs::msg::VehicleCommand();
  cmd.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;

  vehicle_command_pub->publish(cmd);
}

void
RoboligoConnectorPX4::offboarding_callback()
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "Offboard mode was required");

  auto cmd = px4_msgs::msg::VehicleCommand();
  cmd.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  cmd.param1 = 1.0f;
  cmd.param2 = 6.0f;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;

  vehicle_command_pub->publish(cmd);

  auto ocm = px4_msgs::msg::OffboardControlMode();
  ocm.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  ocm.position = false;
  ocm.velocity = true;
  ocm.acceleration = false;
  ocm.attitude = false;
  ocm.body_rate = false;

  offboard_control_mode_pub->publish(ocm);
}

void
RoboligoConnectorPX4::standingby_callback()
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "StandBy mode was required");

  auto cmd = px4_msgs::msg::VehicleCommand();
  cmd.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  cmd.param1 = 1.0f;
  cmd.param2 = 2.0f;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;

  vehicle_command_pub->publish(cmd);
}

void
RoboligoConnectorPX4::set_data_loss_exception(int value)
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "Data loss exception will be changed.");
  RCLCPP_INFO_STREAM(node->get_logger(), "Note: Parameter configuration should be done via PX4 parameter interface");
}

void
RoboligoConnectorPX4::set_data_loss_offboard_time(double time)
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "Data loss offboard time will be changed to " << time << " seconds");
}

void
RoboligoConnectorPX4::set_data_loss_action(int value)
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "Data loss action will be changed to " << value);
}

void
RoboligoConnectorPX4::set_time_disarm_preflight(double time)
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "Preflight disarm time will be changed to " << time << " seconds");
}

void
RoboligoConnectorPX4::set_rcl_loss_exception(int value)
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "RC loss exception will be changed to " << value);
}

void
RoboligoConnectorPX4::set_init_altitude(double init_altitude)
{
  auto node = get_node();
  RCLCPP_INFO_STREAM(node->get_logger(), "Init altitude for takeoff will be changed to " << init_altitude << " meters.");
  node->set_parameter(rclcpp::Parameter(get_plugin_name() + ".init_alt", init_altitude));
  init_alt_ = init_altitude;
}

} // namespace roboligo

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(roboligo::RoboligoConnectorPX4, roboligo::ConnectorBase);
