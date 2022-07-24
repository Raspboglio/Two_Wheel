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
    
    // Feedback params
    auto_declare<std::vector<std::string>>("feedback", std::vector<std::string>({"position", "velocity"}));
    
    // Physical parameters
    auto_declare<std::vector<double>>("body_I_diag", std::vector<double>({0,0,0}));
    auto_declare<std::vector<double>>("wheel_I_diag", std::vector<double>({0,0,0}));
    auto_declare<double>("body_mass", 0);
    auto_declare<double>("wheel_mass", 0);
    auto_declare<double>("body_height", 0);
    auto_declare<double>("wheel_redius", 0);

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
    std::vector<std::string> feedback = this->node_->get_parameter("feedback").as_string_array();
    
    if( std::find(feedback.begin(), feedback.end(), "position") !=  feedback.end()){
        //PV control
        config.names.push_back("RW_joint/position");
        config.names.push_back("LW_joint/position");
        config.names.push_back("imu_sensor/orientation.y");
    }

    if( std::find(feedback.begin(), feedback.end(), "velocity") ){
        config.names.push_back("RW_joint/velocity");
        config.names.push_back("LW_joint/velocity");
        config.names.push_back("imu_sensor/angular_velocity.y");
    }
    
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_configure(const rclcpp_lifecycle::State & previous_state){
    
    // Feedback strategy configuration
    std::vector<std::string> feedback = this->node_->get_parameter("feedback").as_string_array();
    if(feedback.size() == 0){
        CLCPP_ERROR(this->node_->get_logger(), "Feedback array must be non empty");
        return CallbackReturn::ERROR;
    }
    
    pos_feed = std::find( feedback.begin(), feedback.end(), "position") != feedback.end();
    vel_feed = std::find( feedback.begin(), feedback.end(), "velocity") != feedback.end();
    
    // Take physical parameters
    
    std::vector<double> I_body = this->node_->get_parameter("body_I_diag").as_double_array();
    std::vector<double> I_wheel = this->node_->get_parameter("wheel_I_diag").as_double_array();
    if(I_body.size() != 3 || I_wheel.size()!=3){
        CLCPP_ERROR(this->node_->get_logger(), "Inertia matrices must have 3 elements");
        return CallbackReturn::ERROR;
    }

    // Check non null vector
    null = true;
    for(int i = 0; i < 3 && null; i++){
        null = I_body[i] == 0 || I_wheel[i] == 0;
    }
    if(null){
        RCLCPP_ERROR(this->node_->get_logger(), "Inertia matrices can't be null, maybe you need to define that");
        return CallbackReturn::ERROR;
    }

    
    double m_b = this->node_->get_parameter("body_mass").as_double();
    double m_w = this->node_->get_parameter("wheel_mass").as_double();
    if(m_b == 0 || m_w == 0){
        RCLCPP_ERROR(this->node_->get_logger(), "masses can't be 0, probably you didn't define that");
        return CallbackReturn::ERROR;
    }
    
    double l = this->node_->get_parameter("body_height").as_double();
    double r = this->node_->get_parameter("wheel_radius").as_double();
    if(l == 0 || r == 0){
        RCLCPP_ERROR(this->node_->get_logger(), "length can't be 0, probably you didn't define that");
        return CallbackReturn::ERROR;
    }

    // TODO: init B,T*,K* with passed parameters.
    // Init B
    B(1,1) = 
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_activate(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_deactivate(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TwoWheelIDController::update(){
    
    
    // Measure current state
    for(int i = 0; i < 3; i++){
        if(pos_feed){
            u_m(i) = this->state_interfaces_[i].get_value();
        }
        if(vel_feed){
            u_m_dot(i) = this->state_interfaces_[i+3].get_value();
        }
    }

    e.zeros(3);
    e_dot.zeros(3);

    if(pos_feed){
        u = pinv(T_out) * u_m;
        e = q_w - u;
    }
    
    if(vel_feed){
        u_dot = pinv(T_out) * u_m_dot;
        e_dot = q_w_dot - u_dot;
    }
    
    // TODO: Compute G

    tau = T * ((Kp * e + Kd * e_dot) * B + G); // TODO: check what we need between T, T' and T_inv

    for(int i = 0; i < 2; i++){
        this->command_interfaces_[i].set_value(tau(i));
    }

    
}

void TwoWheelIDController::subCallback(const two_wheel_control_msgs::msg::Command::SharedPtr msg){

}

PLUGINLIB_EXPORT_CLASS(two_wheel_id_controller::TwoWheelIDController, controller_interface::ControllerInterface)
