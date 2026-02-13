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

#ifndef ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__OUTPUT_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__OUTPUT_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>

#include "roboligo_common/connector/Commander.hpp"

namespace roboligo
{
    /**
    * @struct RoboligoOutputTwist
    * @brief Encapsulates a ROS 2 publisher and message data for Twist commands.
    * 
    * @var publisher Shared pointer to the ROS 2 Twist message publisher.
    * @var data Shared pointer to the Twist message data being published.
    */
    struct RoboligoOutputTwist{
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        geometry_msgs::msg::Twist::SharedPtr data;
    };

    /**
    * @struct RoboligoOutputTwistStamped
    * @brief Encapsulates a ROS 2 publisher and message data for stamped Twist commands.
    * 
    * @var publisher Shared pointer to the ROS 2 TwistStamped message publisher.
    * @var data Shared pointer to the TwistStamped message data being published.
    */
    struct RoboligoOutputTwistStamped{
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher;
        geometry_msgs::msg::TwistStamped::SharedPtr data;
    };

    /**
    * @struct RoboligoOutputPositionTarget
    * @brief Encapsulates a ROS 2 publisher and message data for position target commands with control parameters.
    * 
    * @var publisher Shared pointer to the ROS 2 PositionTarget message publisher.
    * @var data Shared pointer to the PositionTarget message data being published.
    * @var coordinate_frame MAVLink coordinate frame ID (default: 8 for BODY_NED).
    * @var type_mask Bitmask indicating which fields are used (default: 1991 = 0b011111000111).
    */
    struct RoboligoOutputPositionTarget{
        rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr publisher;
        mavros_msgs::msg::PositionTarget::SharedPtr data;
        int coordinate_frame = 8;
        int type_mask = 1991; //b011111000111
    };

    /**
     * @class Output
     * @brief Handles output commands for robot motion control.
     * 
     * Manages twist and position target messages for commanding robot movement.
     * Supports both stamped and unstamped geometric message types.
     */
    class Output : public Commander
    {
    public:
        /**
        * @brief Constructs an Output commander with a name and topic.
        * @param new_name The name identifier for this output commander.
        * @param new_topic The ROS topic name for publishing output messages.
        */
        Output(std::string new_name, std::string new_topic);

        /**
        * @brief Virtual destructor for proper cleanup of derived classes.
        */
        virtual ~Output() = default;

        RoboligoOutputTwist twist; ///<  Stores unstamped twist (velocity) output data

        RoboligoOutputTwistStamped twist_stamped; ///<  Stores timestamped twist (velocity) output data

        RoboligoOutputPositionTarget position_target; ///< Stores target position output data

        /**
        * @brief Sets output data from an unstamped Twist message.
        * @param msg Pointer to the geometry_msgs::msg::Twist message to store.
        */
        void set_data(const geometry_msgs::msg::Twist* msg);

        /**
        * @brief Sets output data from a timestamped TwistStamped message.
        * @param msg Pointer to the geometry_msgs::msg::TwistStamped message to store.
        */
        void set_data_stamped(const geometry_msgs::msg::TwistStamped* msg);
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__OUTPUT_HPP_
