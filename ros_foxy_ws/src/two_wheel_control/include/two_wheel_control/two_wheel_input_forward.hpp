#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <two_wheel_control_msgs/msg/input_command.hpp>

namespace two_wheel_controller{

class TwoWheelInputForward : public controller_interface::ControllerInterface{
public:
    TwoWheelInputForward();
    ~TwoWheelInputForward();

    controller_interface::return_type init(const std::string & controller_name) override;
    controller_interface::return_type update() override;
    
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state ) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:

    double Mr, Ml;
    rclcpp::Subscription<two_wheel_control_msgs::msg::InputCommand>::SharedPtr _command_sub;
    void commandCallback(const two_wheel_control_msgs::msg::InputCommand::SharedPtr msg);
};

}