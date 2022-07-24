#ifndef TWO_WHEEL_ID_CONTROL
#define TWO_WHEEL_ID_CONTROL

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <two_wheel_control_msgs/msg/command.hpp>

// Linear Algebra utils
#include<armadillo>


namespace two_wheel_id_controller{
    
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
            arma::Col<double> u(); // measured state col vector [p, thetay, thetaz]
            arma::Col<double> u_dot();
            arma::Col<double> u_m(); //real measured col vector [wr, wl, thetay]
            arma::Col<double> u_m_dot();
            arma::Col<double> q_w(); // wanted state col vector [p, thetay, thetaz]
            amra::Col<double> q_w_dot();
            arma::Col<double> e();  // error
            arma::Col<double> e_dot();
            arma::Mat<double> Kp();
            arma::Mat<double> Kd();
            arma::Mat<double> B();
            arma::Col<double> G();
            arma::Mat<double> T(); // Conversion matrix between gamma input [f, My, Mz] and tau input [Mr, Ml]
            arma::Mat<double> T_out();  // Conversion matrix between generalized coord [v,thetay,thetaz] and measured output [wr, wl, thetay]
                                        //[T;[0 0 1]]
            arma::Col<double> tau(); // input vector of the system  
    };

}



#endif // TWO_WHEEL_ID_CONTROL