#include <two_wheel_control/two_wheel_input_forward.hpp>

using namespace two_wheel_controller;

TwoWheelInputForward::TwoWheelInputForward(){

}

TwoWheelInputForward::~TwoWheelInputForward(){

}

controller_interface::return_type TwoWheelInputForward::init(const std::string & controller_name){
    controller_interface::return_type ret = controller_interface::return_type::OK;
    ret = controller_interface::ControllerInterface::init(controller_name);

    if(ret == controller_interface::return_type::ERROR){
        RCLCPP_ERROR(node_->get_logger(),"In TwoWheelInputForward failed to initialize controller interface");
        return ret;
    }

    Mr = 0;
    Ml = 0;
    _command_sub = node_->create_subscription<two_wheel_control_msgs::msg::InputCommand>("/input_command",10,std::bind(&TwoWheelInputForward::commandCallback,this, std::placeholders::_1));

}

controller_interface::InterfaceConfiguration TwoWheelInputForward::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    config.names.push_back("RW_joint/effort");
    config.names.push_back("LW_joint/effort");
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    return config;
}

controller_interface::InterfaceConfiguration TwoWheelInputForward::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    
    return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelInputForward::on_configure(const rclcpp_lifecycle::State & previous_state){
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelInputForward::on_activate(const rclcpp_lifecycle::State & previous_state){
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelInputForward::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    return ret;
}

controller_interface::return_type TwoWheelInputForward::update(){
    controller_interface::return_type ret = controller_interface::return_type::OK;

    command_interfaces_[0].set_value(Mr);
    command_interfaces_[1].set_value(Ml);
    
    return ret;
}

void TwoWheelInputForward::commandCallback(const two_wheel_control_msgs::msg::InputCommand::SharedPtr msg){
    Mr = msg->mr;
    Ml = msg->ml;
}

PLUGINLIB_EXPORT_CLASS(two_wheel_controller::TwoWheelInputForward, controller_interface::ControllerInterface)

