#include "pluginlib/class_loader.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "roboligo_connector/ConnectorNode.hpp"

namespace roboligo
{

using namespace std::chrono_literals;

ConnectorNode::ConnectorNode(
  const rclcpp::NodeOptions & options)
: LifecycleNode("connector_node", options)
{
  connector_loader_ = std::make_unique<pluginlib::ClassLoader<roboligo::ConnectorBase>>(
    "roboligo_common", "roboligo::ConnectorBase");
}

ConnectorNode::~ConnectorNode()
{
  trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DESTROY);
  std::cout << "Roboligo Connector Node state is :" << get_current_state().label().c_str() << std::endl; 
  std::vector<std::string> connector_types;
  std::string plugin;
  get_parameter("connector_types", connector_types);
  for (const auto & connector_type : connector_types) {
    get_parameter(connector_type + std::string(".plugin"), plugin);
    connector_loader_->unloadLibraryForClass(plugin);
  }
  connector_.reset();
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ConnectorNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::vector<std::string> connector_types;
  declare_parameter("connector_types", connector_types);
  
  get_parameter("connector_types", connector_types);


  for (const auto & connector_type : connector_types) {
    std::string plugin;
    declare_parameter(connector_type + std::string(".plugin"), plugin);
    get_parameter(connector_type + std::string(".plugin"), plugin);

    try {
      RCLCPP_INFO(get_logger(),
        "Loading ConnectorBase %s [%s]", connector_type.c_str(), plugin.c_str());

      connector_ = connector_loader_->createSharedInstance(plugin);

      try {
        connector_->initialize(shared_from_this(), connector_type);
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), e.what());
        return CallbackReturnT::FAILURE;
      }

      RCLCPP_INFO(get_logger(),
        "Loaded ConnectorBase %s [%s]", connector_type.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(),
        "Unable to load plugin roboligo::ConnectorBase. Error: %s", ex.what());
      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ConnectorNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ConnectorNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ConnectorNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ConnectorNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
    if(connector_->exit(*std::make_shared<roboligo::RobotState>(*robot_state_)))
    {
        return CallbackReturnT::SUCCESS;
    } else {
        return CallbackReturnT::FAILURE;
    }
    
}

CallbackReturnT
ConnectorNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

bool
ConnectorNode::init(std::shared_ptr<RobotState> robot_state)
{
  if (connector_ == nullptr) {return false;}
  return connector_->set(*robot_state);
}

bool
ConnectorNode::cycle(std::shared_ptr<RobotState> robot_state)
{
  if (connector_ == nullptr) {return false;}
  return connector_->update(*robot_state);
}

void 
ConnectorNode::reset_connector()
{
  connector_.reset();
}

}  // namespace roboligo