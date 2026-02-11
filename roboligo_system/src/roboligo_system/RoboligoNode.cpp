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

#include "roboligo_system/RoboligoNode.hpp"

namespace roboligo
{

using namespace std::chrono_literals;

RoboligoNode::RoboligoNode(const rclcpp::NodeOptions & options)
: LifecycleNode("roboligo_node", options)
{
    RCLCPP_INFO(get_logger(),"Roboligo Node Create");
    get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      std::bind(&RoboligoNode::preShutdown, this));

// Configuracion de roboligo

}

RoboligoNode::~RoboligoNode()
{
  RCLCPP_INFO(get_logger(),"Roboligo state is %s", get_current_state().label().c_str());
  RCLCPP_INFO(get_logger(),"Roboligo Node Destroyed");
}

void RoboligoNode::preShutdown() 
{
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
RoboligoNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(),"Roboligo on_configure");

  declare_parameter("robot_name", robot_name_);
  declare_parameter("input_topic", input_topic_);
  declare_parameter("stamped", stamped_);
  get_parameter("robot_name", robot_name_);
  get_parameter("input_topic", input_topic_);
  get_parameter("stamped", stamped_);

  robot_state_->set_name(robot_name_);
  robot_state_->set_stamp(stamped_);

  for (auto & roboligo_system_node : get_roboligo_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Configuring [%s]", roboligo_system_node.first.c_str());
    roboligo_system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    if (roboligo_system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to configure [%s]", roboligo_system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  roboligo_init(); 

  if(stamped_ != robot_state_->input->is_stamped()){
    RCLCPP_ERROR_STREAM(get_logger(), "Mismatch between type input topics. Please Check it");
    return CallbackReturnT::FAILURE;
  }

  input_command = create_service<roboligo_interfaces::srv::RoboligoString>(
    "/roboligo/input", 
    std::bind(&RoboligoNode::input_callback, this, std::placeholders::_1, std::placeholders::_2));

  rclcpp::QoS qos_profile(10);
  qos_profile.best_effort();

  if(robot_state_->input->is_configured())
  {
    if(robot_state_->input->is_stamped())
    {
      robot_state_->input->twist_stamped.subscriber = create_subscription<geometry_msgs::msg::TwistStamped>(
            robot_state_->input->get_topic(), qos_profile,
            std::bind(&Input::callback_stamped, robot_state_->input, std::placeholders::_1));
    } else {
      robot_state_->input->twist.subscriber = create_subscription<geometry_msgs::msg::Twist>(
          robot_state_->input->get_topic(), qos_profile,
          std::bind(&Input::callback, robot_state_->input, std::placeholders::_1));
    }

  }

  if(robot_state_->output->is_configured())
  {
    if(robot_state_->output->is_stamped())
    {
      robot_state_->output->twist_stamped.publisher = create_publisher<geometry_msgs::msg::TwistStamped>(
            robot_state_->output->get_topic(), qos_profile);
    } else {
      robot_state_->output->twist.publisher = create_publisher<geometry_msgs::msg::Twist>(
          robot_state_->output->get_topic(), qos_profile);
    }
  }

  if(robot_state_->position_target->is_configured())
  {
    robot_state_->position_target->position_target.publisher = create_publisher<mavros_msgs::msg::PositionTarget>(
          robot_state_->position_target->get_topic(), qos_profile);
  }


  if(robot_state_->imu->is_configured())
  {
    robot_state_->imu->subscriber = create_subscription<sensor_msgs::msg::Imu>(
          robot_state_->imu->get_topic(), qos_profile,
          std::bind(&Imu::callback, robot_state_->imu, std::placeholders::_1));
  }


  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RoboligoNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  RCLCPP_INFO(get_logger(),"Roboligo on_activate");

  for (auto & roboligo_system_node : get_roboligo_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Activating [%s]", roboligo_system_node.first.c_str());
    roboligo_system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    if (roboligo_system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to activate [%s]", roboligo_system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RoboligoNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  RCLCPP_INFO(get_logger(),"Roboligo on_deactive");

  for (auto & roboligo_system_node : get_roboligo_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Deactivating [%s]", roboligo_system_node.first.c_str());
    roboligo_system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

    if (roboligo_system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to deactivate [%s]", roboligo_system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RoboligoNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(),"Roboligo state is %s", get_current_state().label().c_str());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RoboligoNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(),"Roboligo state is %s", get_current_state().label().c_str());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RoboligoNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

void 
RoboligoNode::roboligo_init()
{
  states_node_->init(robot_state_);
  connector_node_->init(robot_state_);
  robot_state_->show();
}

void
RoboligoNode::roboligo_cycle()
{
  states_node_->cycle(robot_state_);
  connector_node_->cycle(robot_state_);
}

void
RoboligoNode::input_callback(const std::shared_ptr<roboligo_interfaces::srv::RoboligoString::Request> request,
  std::shared_ptr<roboligo_interfaces::srv::RoboligoString::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), "Command Required: " << request->data);
  auto triggers = robot_state_->get_triggers();
  for (auto & trigger : triggers) {
    if (trigger.get_name() == request->data) {
        trigger.execute();
        response->success = true;
        return;
    }
  }
  RCLCPP_WARN(get_logger(), "Trigger '%s' has no callback registered", request->data.c_str());
  response->success = false;
  response->message = "Trigger unsupported";
}

std::map<std::string, RoboligoNodeInfo>
RoboligoNode::get_roboligo_system_nodes()
{
  std::map<std::string, RoboligoNodeInfo> ret;

  ret[states_node_->get_name()] = {states_node_, nullptr};
  ret[connector_node_->get_name()] = {connector_node_, nullptr};

  return ret;
}

}  // namespace roboligo