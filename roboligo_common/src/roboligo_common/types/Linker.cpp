#include "roboligo_common/types/Linker.hpp"

namespace roboligo
{
    inline std::ostream& operator<<(std::ostream& os, Sense t) {
        switch (t) {
            case Sense::INPUT: os << "INPUT"; break;
            case Sense::OUTPUT: os << "OUTPUT"; break;
        }
        return os;
    }

    interfaces::modes
    Linker::get_mode()
    {
        return mode_;
    }

    void
    Linker::set_mode(interfaces::modes new_mode)
    {
        mode_ = new_mode;
    }

    std::string
    Linker::get_name()
    {
        return name;
    }

    void
    Linker::set_name(std::string new_name)
    {
        name = new_name;
    }

    std::string
    Linker::get_topic()
    {
        return topic_name;
    }

    void
    Linker::set_topic(std::string new_name)
    {
        topic_name = new_name;
    }

    void
    Linker::set_service(std::string new_name)
    {
        topic_name = new_name;
    }

    void
    Linker::set_interface(std::string name_, std::string topic_name_)
    {
        interface = std::make_shared<interfaces::Interface>(name_, 
            mode_, topic_name_);
    }

    bool
    Linker::is_available()
    {
        return available_;
    }

    void
    Linker::set_available(bool new_state)
    {
        available_ = new_state;
    }

    void
    Linker::set_configured(bool new_state)
    {
        configured_ = new_state;
    }

    bool
    Linker::is_configured()
    {
        return configured_;
    }


} // namespace roboligo