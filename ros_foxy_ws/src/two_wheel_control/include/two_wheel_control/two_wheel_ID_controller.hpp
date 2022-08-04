#ifndef TWO_WHEEL_ID_CONTROL
#define TWO_WHEEL_ID_CONTROL

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <two_wheel_control_msgs/msg/command.hpp>

// Linear Algebra utils
#include<armadillo>


namespace two_wheel_controller{
    
    class TwoWheelIDController : public controller_interface::ControllerInterface{
        public:
            TwoWheelIDController();
            ~TwoWheelIDController();
        
            controller_interface::return_type init(const std::string &controller_name) override;
            
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update() override;

        private:
            void subCallback(const two_wheel_control_msgs::msg::Command::SharedPtr msg);

            rclcpp::Subscription<two_wheel_control_msgs::msg::Command>::SharedPtr _sub;
            
            bool pos_feed, vel_feed;
            double l;

            std::unique_ptr<arma::Col<double>> u, u_dot, u_m,  u_m_dot, q_w, q_w_dot, e, e_dot, G, tau;
            std::unique_ptr<arma::Mat<double>> Kp, Kd, B, T, T_out;
            
    };

}



#endif // TWO_WHEEL_ID_CONTROL