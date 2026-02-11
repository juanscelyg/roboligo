#ifndef ROBOLIGO_COMMON_TYPES__INTERFACE_HPP_
#define ROBOLIGO_COMMON_TYPES__INTERFACE_HPP_

#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

namespace roboligo 
{
    namespace interfaces 
    {
        enum class types 
        { 
            MSG, 
            SRV 
        };
        
        enum class modes 
        { 
            PUBLISHER, 
            SUBSCRIBER, 
            CALLER 
        };

        struct type {
            types type;             ///< Type MSG or SRV
            modes mode;             ///< Interface mode PUBLISHER, SUBSCRIBER, CALLER
            std::string value = ""; ///< Interface value as topic name (i. e. /prueba)
        };

        class Interface {
        public:
            Interface(std::string name, modes mode, std::string value);   

            virtual ~Interface() = default;

            std::string get_name(void);

            types get_type(void);

            modes get_mode(void);
           
            std::string get_value(void);

            void set_value(std::string value);

            void show();

        private:
            std::string name_{};    ///< Interface name
            type type_{};           ///< Interface type
            modes mode_{};          ///< Interface mode
        };

    } // namespace interfaces
} // namespace roboligo

#endif // ROBOLIGO_COMMON_TYPES__INTERFACE_HPP_
