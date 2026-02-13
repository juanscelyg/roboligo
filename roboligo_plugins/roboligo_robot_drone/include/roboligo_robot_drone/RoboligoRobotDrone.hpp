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

#ifndef ROBOLIGO_ROBOT_DRONE__ROBOLIGOROBOTDRONE_HPP_
#define ROBOLIGO_ROBOT_DRONE__ROBOLIGOROBOTDRONE_HPP_

#include "roboligo_connector_mavros/RoboligoConnectorMavros.hpp"

#include "roboligo_common/classification/ClassificationBase.hpp"
#include "roboligo_common/classification/Mode.hpp"
#include "roboligo_common/classification/Trigger.hpp"
#include "roboligo_common/types/RobotState.hpp"

namespace roboligo
{
    /**
     * @class RoboligoRobotDrone
     * @brief Drone robot controller that manages flight modes and transitions.
     * 
     * This class extends ClassificationBase to provide drone-specific functionality,
     * including mode management (standby, offboard) and trigger-based state transitions
     * (arming, takeoff, landing, disarming, etc.).
     */
    class RoboligoRobotDrone : public ClassificationBase
    {
    public:
        /**
        * @brief Default constructor.
        */
        RoboligoRobotDrone() {}
        
        /**
        * @brief Default destructor.
        */
        ~RoboligoRobotDrone() = default;

        /**
        * @brief Initializes the drone robot and its components.
        */
        void on_initialize() override;

        /**
        * @brief Sets the robot state.
        * @param robot_state Reference to the RobotState object to be set.
        */
        void on_set(RobotState & robot_state) override;

        /**
        * @brief Configures all available flight modes.
        */
        void set_modes(void);

        /**
        * @brief Configures all state transition triggers.
        */
        void set_triggers(void);

        std::shared_ptr<RoboligoConnectorMavros> connector_mavros_ = std::make_shared<RoboligoConnectorMavros>(); ///< Shared pointer to the MAVROS connector for communication

        Mode standby = Mode("standby", "AUTO.LOITER"); ///< Standby mode (AUTO.LOITER)

        Mode offboard = Mode("offboard", "OFFBOARD"); ///< Offboard control mode

        Trigger arming = Trigger("arming", "arming", 
            std::vector<Mode>{standby}, 
            std::vector<Mode>{offboard}); ///< Trigger for arming the drone

        Trigger takeoff = Trigger("takeoff", "AUTO.TAKEOFF", 
            std::vector<Mode>{standby, offboard}, 
            std::vector<Mode>{offboard}); ///< Trigger for takeoff

        Trigger landing = Trigger("landing", "AUTO.LAND", 
            std::vector<Mode>{standby, offboard}, 
            std::vector<Mode>{standby}); ///< Trigger for landing

        Trigger offboarding = Trigger("offboard", "OFFBOARD", 
            std::vector<Mode>{standby, offboard}, 
            std::vector<Mode>{standby, offboard}); ///< Trigger for offboard mode transition

        Trigger disarming = Trigger("disarming", "disarming", 
            std::vector<Mode>{standby, offboard}, 
            std::vector<Mode>{standby, offboard}); ///< Trigger for disarming the drone

        Trigger standingby = Trigger("standby", "AUTO:LOITER", 
            std::vector<Mode>{offboard}, 
            std::vector<Mode>{offboard}); ///< Trigger for returning to standby mode

    protected:
        bool verbose_{false}; ///< Flag to enable/disable verbose logging
        
        std::vector<Mode> modes_; ///< Vector containing all available flight modes.
        
        std::vector<Trigger> triggers_; ///< Vector containing all state transition triggers


    };
} // namespace roboligo
#endif // ROBOLIGO_ROBOT_DRONE__ROBOLIGOROBOTDRONE_HPP_