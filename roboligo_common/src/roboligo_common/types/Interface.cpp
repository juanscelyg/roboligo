#include "roboligo_common/types/Interface.hpp"

namespace roboligo
{
    namespace interfaces
    {
        inline std::ostream& operator<<(std::ostream& os, types t) {
            switch (t) {
                case types::MSG: os << "MSG"; break;
                case types::SRV: os << "SRV"; break;
            }
            return os;
        }

        inline std::ostream& operator<<(std::ostream& os, modes m) {
            switch (m) {
                case modes::PUBLISHER:  os << "PUBLISHER"; break;
                case modes::SUBSCRIBER: os << "SUBSCRIBER"; break;
                case modes::CALLER:     os << "CALLER"; break;
            }
            return os;
        }

        Interface::Interface(std::string name, modes mode, std::string value)
        {
            name_ = std::move(name);
            mode_ = mode; 
            if (mode == modes::CALLER) {
            type_ = type{ types::SRV, modes::CALLER, std::move(value) };
            } else if (mode == modes::PUBLISHER) {
            type_ = type{ types::MSG, modes::PUBLISHER, std::move(value) };
            } else { // modes::SUBSCRIBER
            type_ = type{ types::MSG, modes::SUBSCRIBER, std::move(value) };
            }
        }

        std::string
        Interface::get_name()
        {
            return name_;
        }

        types
        Interface::get_type()
        {
            return type_.type;
        }

        modes
        Interface::get_mode()
        {
            return type_.mode;
        }

        std::string
        Interface::get_value()
        {
            return type_.value;
        }

        void
        Interface::set_value(std::string value)
        {
            type_.value = value;
        }

        void 
        Interface::show(){
            std::cout << ":: Interface " << get_name() << " ::" << std::endl;
            std::cout << "-- Type:  " << get_type() << std::endl;
            std::cout << "-- Mode:  " << get_mode() << std::endl;
            std::cout << "-- Value: " << get_value() << std::endl;
        }

    }// namespace interfaces

} // namespace roboligo