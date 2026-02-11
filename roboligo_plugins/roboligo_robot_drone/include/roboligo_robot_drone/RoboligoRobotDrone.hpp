#ifndef ROBOLIGO_ROBOT_DRONE__ROBOLIGOROBOTDRONE_HPP_
#define ROBOLIGO_ROBOT_DRONE__ROBOLIGOROBOTDRONE_HPP_

#include "roboligo_connector_mavros/RoboligoConnectorMavros.hpp"

#include "roboligo_common/classification/ClassificationBase.hpp"
#include "roboligo_common/classification/Mode.hpp"
#include "roboligo_common/classification/Trigger.hpp"
#include "roboligo_common/types/RobotState.hpp"

namespace roboligo
{
    class RoboligoRobotDrone : public ClassificationBase
    {
        public:
            RoboligoRobotDrone() {}

            ~RoboligoRobotDrone() = default;

            void on_initialize() override;

            void on_set(RobotState & robot_state) override;

            void set_modes(void);

            void set_triggers(void);

            std::shared_ptr<RoboligoConnectorMavros> connector_mavros_ = std::make_shared<RoboligoConnectorMavros>();

            bool verbose_{false}; 
            std::vector<Mode> modes_;
            Mode standby = Mode("standby", "AUTO.LOITER");
            Mode offboard = Mode("offboard", "OFFBOARD");

            std::vector<Trigger> triggers_;
            Trigger arming = Trigger("arming", "arming", 
                std::vector<Mode>{standby}, 
                std::vector<Mode>{offboard});
            Trigger takeoff = Trigger("takeoff", "AUTO.TAKEOFF", 
                std::vector<Mode>{standby, offboard}, 
                std::vector<Mode>{offboard});
            Trigger landing = Trigger("landing", "AUTO.LAND", 
                std::vector<Mode>{standby, offboard}, 
                std::vector<Mode>{standby});
            Trigger offboarding = Trigger("offboard", "OFFBOARD", 
                std::vector<Mode>{standby, offboard}, 
                std::vector<Mode>{standby, offboard});
            Trigger disarming = Trigger("disarming", "disarming", 
                std::vector<Mode>{standby, offboard}, 
                std::vector<Mode>{standby, offboard});
            Trigger standingby = Trigger("standby", "AUTO:LOITER", 
                std::vector<Mode>{offboard}, 
                std::vector<Mode>{offboard});

    };
} // namespace roboligo
#endif // ROBOLIGO_ROBOT_DRONE__ROBOLIGOROBOTDRONE_HPP_