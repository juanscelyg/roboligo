#include "pluginlib/class_loader.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "roboligo_states/StatesNode.hpp"

namespace roboligo
{

using namespace std::chrono_literals;

StatesNode::StatesNode(
  const rclcpp::NodeOptions & options)
: LifecycleNode("states_node", options)
{
  classification_loader_ = std::make_unique<pluginlib::ClassLoader<roboligo::ClassificationBase>>(
    "roboligo_common", "roboligo::ClassificationBase");

}

StatesNode::~StatesNode()
{
  std::vector<std::string> robot_types;
  std::string plugin;
  get_parameter("robot_types", robot_types);
  for (const auto & robot_type : robot_types) {
    get_parameter(robot_type + std::string(".plugin"), plugin);
    classification_loader_->unloadLibraryForClass(plugin);
  }
  classification_.reset();
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
StatesNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::vector<std::string> robot_types;
  declare_parameter("robot_types", robot_types);
  get_parameter("robot_types", robot_types);

  if (robot_types.size() > 1) {
    RCLCPP_ERROR(get_logger(),
      "You must instance one robot.  [%lu] found", robot_types.size());
    return CallbackReturnT::FAILURE;
  }

  for (const auto & robot_type : robot_types) {
    std::string plugin;
    declare_parameter(robot_type + std::string(".plugin"), plugin);
    get_parameter(robot_type + std::string(".plugin"), plugin);

    try {
      RCLCPP_INFO(get_logger(),
        "Loading ClassificationBase %s [%s]", robot_type.c_str(), plugin.c_str());

      classification_ = classification_loader_->createSharedInstance(plugin);

      try {
        classification_->initialize(shared_from_this(), robot_type);
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), e.what());
        return CallbackReturnT::FAILURE;
      }

      RCLCPP_INFO(get_logger(),
        "Loaded ClassificationBase %s [%s]", robot_type.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(),
        "Unable to load plugin roboligo::ClassificationBase. Error: %s", ex.what());
      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
StatesNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
StatesNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
StatesNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
StatesNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::vector<std::string> robot_types;
  std::string plugin;
  get_parameter("robot_types", robot_types);
  for (const auto & robot_type : robot_types) {
    get_parameter(robot_type + std::string(".plugin"), plugin);
    classification_loader_->unloadLibraryForClass(plugin);
  }
  classification_.reset();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
StatesNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

bool
StatesNode::init(std::shared_ptr<RobotState> robot_state)
{
  if (classification_ == nullptr) {return false;}
  return classification_->set(*robot_state);
}

bool
StatesNode::cycle(std::shared_ptr<RobotState> robot_state)
{
  if (classification_ == nullptr) {return false;}
  return classification_->update(*robot_state);
}

void 
StatesNode::reset_classification()
{
  classification_.reset();
}

}  // namespace roboligo