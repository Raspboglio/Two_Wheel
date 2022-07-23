#include<two_wheel_control/two_wheel_ID_controller.hpp>




using namespace two_wheel_id_controller;

TwoWheelIDController::TwoWheelIDController(){

}

TwoWheelIDController::~TwoWheelIDController(){
    
}

controller_interface::return_type TwoWheelIDController::init(const std::string &controller_name){
    _sub = this->node_->create_subscription<two_wheel_control_msgs::msg::Command>("two_wheel_command", 10, std::bind(this, &TwoWheelIDController::subCallback, std::placeholders::_1));
    // TODO: declare params

    return controller_interface::return_type::OK;
}

/*! 
*   Inverse Dynamic controller only need effort interfaces
*/
controller_interface::InterfaceConfiguration TwoWheelIDController::command_interface_configuration(){
    controller_interface::InterfaceConfiguration config;
    config.names.push_back("RW_joint/effort");
    config.names.push_back("LW_joint/effort");
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    return config;
}

/*!
*  
*/
controller_interface::InterfaceConfiguration TwoWheelIDController::state_interface_configuration(){
    controller_interface::InterfaceConfiguration config;


}

controller_interface::return_type TwoWheelIDController::on_configure(){

}

controller_interface::return_type TwoWheelIDController::on_activate(){

}

controller_interface::return_type TwoWheelIDController::on_deactivate(){

}

controller_interface::return_type TwoWheelIDController::update(){

}

void TwoWheelIDController::subCallback(const two_wheel_control_msgs::msg::Command &msg){

}

PLUGINLIB_EXPORT_CLASS(two_wheel_id_controller::TwoWheelIDController, controller_interface::ControllerInterface)
