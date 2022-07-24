#include<two_wheel_control/two_wheel_ID_controller.hpp>





using namespace two_wheel_id_controller;

TwoWheelIDController::TwoWheelIDController(){

}

TwoWheelIDController::~TwoWheelIDController(){
    
}

controller_interface::return_type TwoWheelIDController::init(const std::string &controller_name){
    controller_interface::return_type res = controller_interface::ControllerInterface::init(controller_name); 
    if(res == controller_interface::return_type::ERROR) return res;

    _sub = this->node_->create_subscription<two_wheel_control_msgs::msg::Command>("two_wheel_command", 10, std::bind(&TwoWheelIDController::subCallback, this,  std::placeholders::_1));
    // TODO: declare params: control strategy(p,v,pv),

    // Allocate memory for matices/vectors

    u.zeros(3);
    u_dot.zerso(3);
    q_w.zeros(3);
    q_w_dot.zeros(3);
    B.zeros(3,3);
    G.zeros(3);
    T.zeros(2,3);
    T_out.zeros(3,3);
    Kp.zeros(3,3);
    Kd.zeros(3,3);
    tau.zers(2);

    return controller_interface::return_type::OK;
}

/*! 
*   Inverse Dynamic controller only need effort interfaces
*/
controller_interface::InterfaceConfiguration TwoWheelIDController::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    config.names.push_back("RW_joint/effort");
    config.names.push_back("LW_joint/effort");
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    return config;
}

/*!
*  3 feedback configurations:
*   position
*   velocity
*   position,velocity
*/
controller_interface::InterfaceConfiguration TwoWheelIDController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    // TODO: add control strategy
    if(true){
        //PV control
        config.names.push_back("RW_joint/position");
        config.names.push_back("LW_joint/position");
        config.names.push_back("imu_sensor/orientation.y");
        config.names.push_back("RW_joint/velocity");
        config.names.push_back("LW_joint/velocity");
        config.names.push_back("imu_sensor/angular_velocity.y");
        
        

    }else{

    }
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_configure(const rclcpp_lifecycle::State & previous_state){
    // TODO: init B,T*,K* with passed parameters.
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_activate(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_deactivate(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TwoWheelIDController::update(){
    
    // TODO: insert strategy check
    if(true){
        // PV control
        // Measure current state
        for(int i = 0; i < 3; i++){
            u_m(i) = this->state_interfaces_[i].get_value();
            u_m_dot(i) = this->state_interfaces_[i+3].get_value();
        }

        u = pinv(T_out) * u_m;
        u_dot = pinv(T_out) * u_m_dot;

        e = q_w - u;
        e_dot = q_w_dot - u_dot;

        // TODO: Compute G

        tau = T * ((Kp * e + Kd * e_dot) * B + G); // TODO: check what we need between T, T' and T_inv

        for(int i = 0; i < 2; i++){
            this->command_interfaces_[i].set_value(tau(i));
        }
        
    }else{

    }
}

void TwoWheelIDController::subCallback(const two_wheel_control_msgs::msg::Command::SharedPtr msg){

}

PLUGINLIB_EXPORT_CLASS(two_wheel_id_controller::TwoWheelIDController, controller_interface::ControllerInterface)
