#ifndef TWO_WHEEL_ID_CONTROL
#define TWO_WHEEL_ID_CONTROL

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <two_wheel_control_msgs/msg/command.hpp>


namespace two_wheel_id_controller{
    
    class TwoWheelIDController : public controller_interface::ControllerInterface{
        public:
            TwoWheelIDController();
            ~TwoWheelIDController();
        
            controller_interface::return_type init(const std::string &controller_name);
            
            controller_interface::InterfaceConfiguration command_interface_configuration();
            controller_interface::InterfaceConfiguration state_interface_configuration();

            controller_interface::return_type on_configure();
            controller_interface::return_type on_activate();
            controller_interface::return_type on_deactivate();

            controller_interface::return_type update();

        private:
            void subCallback(const two_wheel_control_msgs::msg::Command &msg);

            rclcpp::Subscription<two_wheel_control_msgs::msg::Command>::SharedPtr _sub;
    };

}



#endif // TWO_WHEEL_ID_CONTROL