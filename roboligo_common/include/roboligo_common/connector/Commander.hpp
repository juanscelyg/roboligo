#ifndef ROBOLIGO_COMMON_CONNECTOR__COMMANDER_HPP_
#define ROBOLIGO_COMMON_CONNECTOR__COMMANDER_HPP_

#include "roboligo_common/types/Linker.hpp"

namespace roboligo
{
    class Commander : public Linker
    {
    public:
        Commander(std::string new_name, std::string new_value);

        virtual ~Commander() = default;

        void set_stamp(bool stamped);

        bool is_stamped(void);

        void init(std::string new_name, std::string new_value);
        
    protected:
        void initialize(std::string new_name, std::string new_topic);
        
        bool use_stamp_{true};
    };

} // namespace roboligo
#endif // ROBOLIGO_COMMON_CONNECTOR__COMMANDER_HPP_