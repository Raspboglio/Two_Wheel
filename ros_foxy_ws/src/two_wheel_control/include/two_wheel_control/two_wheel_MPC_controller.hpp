#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <two_wheel_control_msgs/msg/state.hpp>
#include <two_wheel_control_msgs/msg/command.hpp>
#include <eigen3/Eigen/Dense>

namespace two_wheel_controller{

    class TwoWheelMPCController : public controller_interface::ControllerInterface{
    
    public:
    
        TwoWheelMPCController();
        ~TwoWheelMPCController();
        
        
        controller_interface::return_type init(const std::string &controller_name) override;
        
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::return_type update() override;


    private:
    
        Eigen::Matrix<double, 6, 6> A_gen,A;
        Eigen::Matrix<double, 3, 3> T;
        Eigen::Matrix<double, 3, 2> B;
        Eigen::Matrix<double, 3, 3> C; 
        

        double max_angle, l;

        std::unique_ptr<rclcpp::Duration> period;


        rclcpp::Publisher<two_wheel_control_msgs::msg::State>::SharedPtr _state_pub;
        two_wheel_control_msgs::msg::State _pub_msg;
        rclcpp::Subscription<two_wheel_control_msgs::msg::Command>::SharedPtr _sub;
        void subCallback(const two_wheel_control_msgs::msg::Command::SharedPtr msg);


        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
        geometry_msgs::msg::Pose _last_pose;
        nav_msgs::msg::Odometry _odom_msg;
        void updateOdom();

    };
    
}