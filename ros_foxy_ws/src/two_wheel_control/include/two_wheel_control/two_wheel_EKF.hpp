#ifndef TWO_WHEEL_EKF
#define TWO_WHEEL_EKF

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <controller_interface/controller_interface.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <two_wheel_control_msgs/msg/mixed_state.hpp>

namespace two_wheel_controller{

class TwoWheelEKF : public controller_interface::ControllerInterface{
public:

    TwoWheelEKF();
    ~TwoWheelEKF();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type init(const std::string &controller_name) override;

    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update() override;

private:

    // Param
    double wheel_r, update_freq;
    Eigen::Matrix<double, 3, 1> body_lenght;

    Eigen::Matrix<double, 7, 1> x;
    Eigen::Matrix<double, 2, 1> encoder, last_encoder;
    Eigen::Matrix<double, 3, 1> imu_rpy;


    two_wheel_control_msgs::msg::MixedState _state_msg;
    rclcpp::Publisher<two_wheel_control_msgs::msg::MixedState>::SharedPtr _state_pub;
    void publishState();

    /////////////TEMP
    double ang_speed_r, ang_speed_l, speed_r, speed_l;
    

};


}





#endif //TWO_WHEEL_EKF