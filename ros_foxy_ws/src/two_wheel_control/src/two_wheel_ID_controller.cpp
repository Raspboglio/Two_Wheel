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
    auto_declare<double>("wheel_distance", 0);
    auto_declare<std::vector<double>>("Kp", std::vector<double>({0}));
    auto_declare<std::vector<double>>("Kd", std::vector<double>({0}));

    // Allocate memory for matices/vectors
    u = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    u_dot= std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    q_w = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    q_w_dot = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    B = std::make_unique<arma::Col<double>>(3,3,arma::fill::zeros);
    G = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    T = std::make_unique<arma::Col<double>>(2, 3, arma::fill::zeros);
    T_out = std::make_unique<arma::Col<double>>(3, 3, arma::fill::zeros);
    Kp = std::make_unique<arma::Col<double>>(3, 3, arma::fill::zeros);
    Kd =std::make_unique<arma::Col<double>>(3, 3, arma::fill::zeros);
    tau = std::make_unique<arma::Col<double>>(2, arma::fill::zeros);

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

    if( std::find(feedback.begin(), feedback.end(), "velocity") != feedback.end() ){
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
        RCLCPP_ERROR(this->node_->get_logger(), "Feedback array must be non empty");
        return CallbackReturn::ERROR;
    }
    
    pos_feed = std::find( feedback.begin(), feedback.end(), "position") != feedback.end();
    vel_feed = std::find( feedback.begin(), feedback.end(), "velocity") != feedback.end();
    
    // Take physical parameters
    
    std::vector<double> I_body = this->node_->get_parameter("body_I_diag").as_double_array();
    std::vector<double> I_wheel = this->node_->get_parameter("wheel_I_diag").as_double_array();
    if(I_body.size() != 3 || I_wheel.size()!=3){
        RCLCPP_ERROR(this->node_->get_logger(), "Inertia matrices must have 3 elements");
        return CallbackReturn::ERROR;
    }

    // Check non null vector
    bool vect_null = true;
    for(int i = 0; i < 3 && vect_null; i++){
        vect_null = I_body[i] == 0 || I_wheel[i] == 0;
    }
    if(vect_null){
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
    double d = this->node_->get_parameter("wheel_distance").as_double();
    if(l == 0 || r == 0 || d == 0){
        RCLCPP_ERROR(this->node_->get_logger(), "length can't be 0, probably you didn't define that");
        return CallbackReturn::ERROR;
    }

    std::vector<double> Kp_coeff = this->node_->get_parameter("Kp").as_double_array();
    std::vector<double> Kd_coeff = this->node_->get_parameter("Kd").as_double_array();

    if((Kp_coeff.size() != 1 && Kp_coeff.size() != 3) || (Kd_coeff.size() != 1 && Kd_coeff.size() != 3)){
        RCLCPP_ERROR(this->node_->get_logger(), "Gains must be of 1 or 3 elements (one value for all or each value for each parameter)");
        return CallbackReturn::ERROR;
    }

    // Init B
    B->at(1,1) = I_wheel[2] + 3 * m_w;
    B->at(1,2) = l * m_w;
    B->at(2,1) = l * m_w;
    B->at(2,2) = m_w * l * l + I_body[2];
    B->at(3,3) = I_body[3] + 2 * I_wheel[3] + d * d * m_w / 2 + I_wheel[2] * d * d / (2 * r * r);

    // Init T*
    *T = {{1/r, -1, d/(2*r)}, {1/r, -1, d/(2*r)}};
    *T_out = {{1/r, -1, d/(2*r)}, {1/r, -1, d/(2*r)}, {0, 0, 1}};
    
    // Init K*
    if(Kp_coeff.size() == 1){
        for(int i = 0; i < 3; i++){
            Kp->at(i,i) = Kp_coeff[1];
        }
    }else{
        for(int i = 0; i < 3; i++){
            Kp->at(i,i) = Kp_coeff[i];
        }
    }

    if(Kd_coeff.size() == 1){
        for(int i = 0; i < 3; i++){
            Kd->at(i,i) = Kd_coeff[1];
        }
    }else{
        for(int i = 0; i < 3; i++){
            Kd->at(i,i) = Kd_coeff[i];
        }
    }
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_activate(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_deactivate(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TwoWheelIDController::update(){
    
    // TODO: check the order of the state interfaces (and command interfaces)
    // Measure current state
    for(int i = 0; i < 3; i++){
        if(pos_feed){
            u_m->at(i) = this->state_interfaces_[i].get_value();
        }
        if(vel_feed){
            u_m_dot->at(i) = this->state_interfaces_[i+3].get_value();
        }
    }

    e->zeros();
    e_dot->zeros();

    if(pos_feed){
        *u = arma::pinv(*T_out) * *u_m;
        *e = *q_w - *u;
    }

    if(vel_feed){
        *u_dot = arma::pinv(*T_out) * *u_m_dot;
        *e_dot = *q_w_dot - *u_dot;
    }
    
    // TODO: Compute G

    *tau = *T * ((*Kp * *e + *Kd * *e_dot) * *B + *G); // TODO: check what we need between T, T' and T_inv

    for(int i = 0; i < 2; i++){
        this->command_interfaces_[i].set_value(tau->at(i));
    }

    return controller_interface::return_type::OK;
}

void TwoWheelIDController::subCallback(const two_wheel_control_msgs::msg::Command::SharedPtr msg){
    
    if(pos_feed){
        q_w->at(1) = msg->p;
        q_w->at(2) = msg->thetay;
        q_w->at(3) = msg->thetaz;
    }

    if(vel_feed){
        q_w->at(1) = msg->p_dot;
        q_w->at(2) = msg->thetay_dot;
        q_w->at(3) = msg->thetaz_dot;
    }
}

PLUGINLIB_EXPORT_CLASS(two_wheel_id_controller::TwoWheelIDController, controller_interface::ControllerInterface)
