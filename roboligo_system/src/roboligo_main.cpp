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

#include <atomic>
#include <chrono>
#include <cstdio>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "roboligo_system/RoboligoNode.hpp"

int main(int argc, char * argv[])
{
  std::atomic_bool working{true};
  double spin_time{0.01};

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  auto roboligo_node = roboligo::RoboligoNode::make_shared();

  exe.add_node(roboligo_node->get_node_base_interface());

  roboligo_node->declare_parameter("spin_time", spin_time);

  roboligo_node->get_parameter("spin_time", spin_time);

  const auto spin_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(spin_time));

  roboligo_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (roboligo_node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(roboligo_node->get_logger(), "Unable to configure Roboligo");
    rclcpp::shutdown();
    return 1;
  }
  roboligo_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (roboligo_node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(roboligo_node->get_logger(), "Unable to activate Roboligo");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::on_shutdown([&](){
      working.store(false, std::memory_order_relaxed);
      exe.cancel();
  });

  //roboligo_node->roboligo_init();

  rclcpp::WallRate rate(200);
  while (working.load(std::memory_order_relaxed)) {

    if (roboligo_node->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      roboligo_node->roboligo_cycle();
    }

    exe.spin_all(spin_duration);
    rate.sleep();
  }
  
  working.store(false, std::memory_order_relaxed);
  exe.cancel();

  rclcpp::shutdown();
  return 0;

}