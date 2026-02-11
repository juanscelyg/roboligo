#ifndef ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__SERVICE_HPP_
#define ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__SERVICE_HPP_

#include "roboligo_common/connector/Commander.hpp"

namespace roboligo
{
    class Service : public Commander
    {
    public:
        Service(std::string new_name, std::string new_value);

        virtual ~Service() = default;
    
        // (ToDO) Implementation
    };
} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR_COMMANDERS__SERVICE_HPP_
