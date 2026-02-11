#ifndef ROBOLIGO_COMMON_CONNECTOR__SENSOR_HPP_
#define ROBOLIGO_COMMON_CONNECTOR__SENSOR_HPP_

#include "roboligo_common/types/Interface.hpp"
#include "roboligo_common/types/Linker.hpp"

namespace roboligo
{
    class Sensor : public Linker
    {
        public:
            Sensor(std::string new_name, std::string new_topic);

            virtual ~Sensor() = default;

            void init(std::string new_name, std::string new_topic);

            
        protected:
            void initialize(std::string new_name, std::string new_topic);
            
            interfaces::modes mode_{interfaces::modes::SUBSCRIBER};
            
    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR__SENSOR_HPP_