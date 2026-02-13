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

#ifndef ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__INPUT_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__INPUT_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "roboligo_common/connector/Commander.hpp"

namespace roboligo
{
    /**
    * @struct RoboligoInputTwist
    * @brief Encapsulates a ROS 2 subscription and data storage for Twist messages.
    * 
    * @var subscriber Shared pointer to the ROS 2 Twist message subscriber.
    * @var data Shared pointer to the received Twist message data.
    */
    struct RoboligoInputTwist{
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
        geometry_msgs::msg::Twist::SharedPtr data;
    };

    /**
    * @struct RoboligoInputTwistStamped
    * @brief Encapsulates a ROS 2 subscription and data storage for TwistStamped messages.
    * 
    * @var subscriber Shared pointer to the ROS 2 TwistStamped message subscriber.
    * @var data Shared pointer to the received TwistStamped message data.
    */
    struct RoboligoInputTwistStamped{
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber;
        geometry_msgs::msg::TwistStamped::SharedPtr data;
    };

    /**
    * @class Input
    * @brief Commander that subscribes to and processes Twist messages.
    * 
    * Manages subscriptions to both Twist and TwistStamped message topics,
    * storing the received data and providing callback handlers for message processing.
    * 
    * @param new_name The name identifier for this input commander.
    * @param new_topic The ROS 2 topic to subscribe to.
    */
    class Input : public Commander
    {
    public:
        /**
        * @brief Constructor
        */
        Input(std::string new_name, std::string new_topic);
                
        /**
        * @brief Destructor
        */
        virtual ~Input() = default;

        RoboligoInputTwist twist; ///< RoboligoInputTwist element

        RoboligoInputTwistStamped twist_stamped; ///< RoboligoInputTwistStamped element
    
        /**
        * @brief Callback handler for Twist messages.
        * @param msg Shared pointer to the received Twist message.
        */
        void callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        
        /**
        * @brief Callback handler for TwistStamped messages.
        * @param msg Shared pointer to the received TwistStamped message.
        */
        void callback_stamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__INPUT_HPP_
