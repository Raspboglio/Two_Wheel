#ifndef TWO_WHEEL_ID_CONTROL
#define TWO_WHEEL_ID_CONTROL

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <two_wheel_control_msgs/msg/command.hpp>
#include <two_wheel_control_msgs/msg/state.hpp>
#include <two_wheel_control_msgs/msg/tuning_command.hpp>
#include <control_toolbox/pid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
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
            std::vector<control_toolbox::Pid> PID; 

            void subCallback(const two_wheel_control_msgs::msg::Command::SharedPtr msg);

            rclcpp::Subscription<two_wheel_control_msgs::msg::Command>::SharedPtr _sub;
            
            double l, v_ddot, x, y, z, w, max_angle;

            std::unique_ptr<arma::Col<double>> u, u_dot, u_m, u_m_dot, q_w, q_w_dot, e, e_dot, G, tau, gamma, last_u;
            std::unique_ptr<arma::Mat<double>> B, T, T_out;

            std::unique_ptr<rclcpp::Duration> period;

            // Tuning utilities
            bool tuning;
            rclcpp::Publisher<two_wheel_control_msgs::msg::State>::SharedPtr _state_pub;
            rclcpp::Subscription<two_wheel_control_msgs::msg::TuningCommand>::SharedPtr _tuning_command; 
            void tuningCallback(const two_wheel_control_msgs::msg::TuningCommand::SharedPtr msg);
            two_wheel_control_msgs::msg::State _pub_msg;

            // Odometry
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
            rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr _tf_pub;
            nav_msgs::msg::Odometry _odom_msg;
            geometry_msgs::msg::Pose _last_pose;
            tf2_msgs::msg::TFMessage _tf_msg;
            geometry_msgs::msg::TransformStamped _transform;
            void updateOdom();


            // Filtering
            std::vector<double> _avg;          
    };

}



#endif // TWO_WHEEL_ID_CONTROL