#include<two_wheel_control/two_wheel_ID_controller.hpp>


// TODO: reprogram the control of position or velocities


using namespace two_wheel_controller;

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
    auto_declare<double>("max_angle", 0);
    auto_declare<std::vector<double>>("Kp", std::vector<double>({0}));
    auto_declare<std::vector<double>>("Kd", std::vector<double>({0}));

    // Allocate memory for matices/vectors
    // RCLCPP_INFO(this->node_->get_logger(), "INIT MATRICES");
    u = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    u_dot= std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    u_m = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    u_m_dot = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    e = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    e_dot = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    q_w = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    q_w_dot = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    B = std::make_unique<arma::Mat<double>>(3,3,arma::fill::zeros);
    G = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    T = std::make_unique<arma::Mat<double>>(2, 3, arma::fill::zeros);
    T_out = std::make_unique<arma::Mat<double>>(3, 3, arma::fill::zeros);
    Kp = std::make_unique<arma::Mat<double>>(3, 3, arma::fill::zeros);
    Kd =std::make_unique<arma::Mat<double>>(3, 3, arma::fill::zeros);
    tau = std::make_unique<arma::Col<double>>(2, arma::fill::zeros);
    gamma = std::make_unique<arma::Col<double>>(3, arma::fill::zeros);

    // TODO: add initial values for inputs as parameters
    q_w = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    q_w_dot = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);


    return controller_interface::return_type::OK;
}

/*! 
*   Inverse Dynamic controller only need effort interfaces
*/
controller_interface::InterfaceConfiguration TwoWheelIDController::command_interface_configuration() const{
    // RCLCPP_INFO(this->node_->get_logger(), "INIT COMMANDS");
    controller_interface::InterfaceConfiguration config;
    config.names.push_back("RW_joint/effort");
    config.names.push_back("LW_joint/effort");

    // config.names.push_back("RW_joint/velocity");
    // config.names.push_back("LW_joint/velocity");
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
    // RCLCPP_INFO(this->node_->get_logger(), "INIT STATES");
    controller_interface::InterfaceConfiguration config;
    std::vector<std::string> feedback = this->node_->get_parameter("feedback").as_string_array();
    
    if( std::find(feedback.begin(), feedback.end(), "position") !=  feedback.end()){
        //PV control
        config.names.push_back("RW_joint/position");
        config.names.push_back("LW_joint/position");
        config.names.push_back("imu_sensor/orientation.x");
        config.names.push_back("imu_sensor/orientation.y");
        config.names.push_back("imu_sensor/orientation.z");
        config.names.push_back("imu_sensor/orientation.w");
    }

    if( std::find(feedback.begin(), feedback.end(), "velocity") != feedback.end() ){
        config.names.push_back("RW_joint/velocity");
        config.names.push_back("LW_joint/velocity");
        config.names.push_back("imu_sensor/angular_velocity.x");
    }
    
    

    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_configure(const rclcpp_lifecycle::State & previous_state){
    // RCLCPP_INFO(this->node_->get_logger(), "INIT CONFIGURE");
    // Feedback strategy configuration
    std::vector<std::string> feedback = this->node_->get_parameter("feedback").as_string_array();
    if(feedback.size() == 0){
        RCLCPP_ERROR(this->node_->get_logger(), "Feedback array must be non empty");
        return CallbackReturn::ERROR;
    }

    pos_feed = std::find( feedback.begin(), feedback.end(), "position") != feedback.end();
    vel_feed = std::find( feedback.begin(), feedback.end(), "velocity") != feedback.end();
    max_angle = this->node_->get_parameter("max_angle").as_double();
    if(max_angle <= 0){
        RCLCPP_ERROR(this->node_->get_logger(), "max_angle should be specified and positive");
        return CallbackReturn::ERROR;
    }
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
    
    l = this->node_->get_parameter("body_height").as_double();
    double r = this->node_->get_parameter("wheel_radius").as_double();
    double d = this->node_->get_parameter("wheel_distance").as_double();
    if(l == 0 || r == 0 || d == 0){
        RCLCPP_ERROR(this->node_->get_logger(), "length can't be 0, probably you didn't define that");
        return CallbackReturn::ERROR;
    }
    // RCLCPP_INFO(this->node_->get_logger(), "INIT PARAM MATRICES");
    std::vector<double> Kp_coeff = this->node_->get_parameter("Kp").as_double_array();
    std::vector<double> Kd_coeff = this->node_->get_parameter("Kd").as_double_array();

    if((Kp_coeff.size() != 1 && Kp_coeff.size() != 3) || (Kd_coeff.size() != 1 && Kd_coeff.size() != 3)){
        RCLCPP_ERROR(this->node_->get_logger(), "Gains must be of 1 or 3 elements (one value for all or each value for each parameter)");
        return CallbackReturn::ERROR;
    }

    // Init B
    
    *B = {{I_wheel[2]+3*m_w, l*m_w, 0}, 
        {l*m_w, m_w*l*l+I_body[2], 0}, 
        {0, 0, I_body[3]+2*I_wheel[3]+d*d*m_w/2+I_wheel[2]*d*d/(2*r*r)}};
    
    // Init T*
    *T = {{1/r, -1, d/(2*r)}, {1/r, -1, -d/(2*r)}};
    *T_out = {{1/r, -1, d/(2*r)}, {1/r, -1, -d/(2*r)}, {0, 1, 0}};
    
    // std::cout << "B: " << std::endl;
    // B->print();
    // std::cout << "T: " << std::endl;
    // T->print();
    // std::cout << "T out: " << std::endl;
    // T_out->print();

    // Init K*
    if(Kp_coeff.size() == 1){
        for(int i = 0; i < 3; i++){
            Kp->at(i,i) = Kp_coeff[1];
            
        }
    }else{
        for(int i = 0; i < 3; i++){
            Kp->at(i,i) = Kp_coeff[i];
            std::cout << Kp_coeff[i];
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
    // RCLCPP_INFO(this->node_->get_logger(), "UPDATE");
    // TODO: check the order of the state interfaces (and command interfaces)
    // Measure current state
    try{
        
        if(pos_feed){     
            u_m->at(0) = this->state_interfaces_[0].get_value();
            u_m->at(1) = this->state_interfaces_[1].get_value();

            //Handle quaternions
            x = this->state_interfaces_[2].get_value();
            y = this->state_interfaces_[3].get_value();
            z = this->state_interfaces_[4].get_value();
            w = this->state_interfaces_[5].get_value();
            u_m->at(2) = std::atan2(2 * (w * x + y * z), (1 - 2 * (x * x + y * y)));
        }
            
        if(vel_feed){
            u_m_dot->at(0) = this->state_interfaces_[6].get_value();
            u_m_dot->at(1) = this->state_interfaces_[7].get_value();
            u_m_dot->at(2) = this->state_interfaces_[8].get_value();
        }

        // std::cout << "measured position" << std::endl;
        // u_m->print();

        e->zeros();
        e_dot->zeros();

        // if(pos_feed){
        //     *u = arma::pinv(*T_out) * *u_m;
        //     *e = *q_w - *u;
        // }

        // if(vel_feed){
        //     *u_dot = arma::pinv(*T_out) * *u_m_dot;
        //     *e_dot = *q_w_dot - *u_dot;
        // }
        
        if(pos_feed){
            *u = arma::inv(*T_out) * *u_m;
            
            
            v_ddot = (Kp->at(0,0) * (q_w->at(0) - u->at(0)) + Kd->at(0,0) * (q_w_dot->at(0) - u_dot->at(0)) ) *  B->at(1,1) / (B->at(0,1) * 9.81 * l);
            if(std::abs(std::asin(v_ddot)) < max_angle){
                q_w->at(1) = std::asin( v_ddot );
            }else{
                RCLCPP_ERROR(this->node_->get_logger(),"Exceeded angle");
                q_w->at(1) = std::asin(std::copysign(max_angle,v_ddot)); //use copysign to crop the angle
            }

            // std::cout << "Pos error: " << (q_w->at(0) - u->at(0)) <<std::endl;
            // std::cout << "Vel error: " << (q_w_dot->at(0) - u_dot->at(0)) <<std::endl;
            // std::cout << "desired theta: " << q_w->at(1) << std::endl;
            // std::cout << "cur theta: " << u->at(1) << std::endl;
            *e = *q_w - *u;
        }

        if(vel_feed){
            *u_dot = arma::inv(*T_out) * *u_m_dot;
            q_w_dot->at(1) = 0;
            // std::cout << "cur vely: " << u_dot->at(1) << std::endl;
            *e_dot = *q_w_dot - *u_dot;
        }

        // Ignore p error since it's already taken into account in theta_y
        e->at(0) = 0;
        e_dot->at(0) = 0;

        // std::cout << "curr state: " << std::endl;
        // u->print();
        // std::cout << "curr vel: " << std::endl;
        // u_dot->print();

        // std::cout << "pos error: " << std::endl;
        // e->print();
        // std::cout << "vel error: " << std::endl;
        // e_dot->print();
        // std::cout << std::endl;

         
        

        //TODO: limit tau
        
        *tau = *T * (*B * (*Kp * *e + *Kd * *e_dot)); // TODO: check what we need between T, T' and T_inv
        
        // std::cout << "action/error: " << (tau->at(0) + tau->at(1)) / (Kp->at(1,1) * e->at(1) + Kd->at(1,1) * e_dot->at(1)) << std::endl;
        //Checking variable 
        // TODO: remove for real use since it allocates a lot of memory
        // arma::Col<double> Kp_e = *Kp * *e;
        // arma::Col<double> gamma = *B * (*Kp * *e + *Kd * *e_dot);
        // std::cout << "Kp = " << std::endl;
        // Kp->print();
        // std::cout << "Kp * e = " << std::endl;
        // Kp_e.print();
        // std::cout << "gamma = " << std::endl;
        // gamma.print();
        
        // std::cout << "actuation: " << std::endl;
        // tau->print();

    }catch(std::exception &err){
        RCLCPP_ERROR(this->node_->get_logger(), err.what());
    }

    for(int i = 0; i < 2; i++){
        this->command_interfaces_[i].set_value(tau->at(i));
    }

    return controller_interface::return_type::OK;
}

void TwoWheelIDController::subCallback(const two_wheel_control_msgs::msg::Command::SharedPtr msg){
    
    if(pos_feed){
        q_w->at(0) = msg->p;
        
        q_w->at(2) = msg->thetaz;
    }

    if(vel_feed){
        q_w_dot->at(0) = msg->p_dot;
       
        q_w_dot->at(2) = msg->thetaz_dot;
    }
    // RCLCPP_INFO(this->node_->get_logger(), "Received new command p:\n%.4f\n%.4f\n v: \n%.4f\n%.4f", q_w->at(0), q_w->at(2), q_w_dot->at(0), q_w_dot->at(2));
}

PLUGINLIB_EXPORT_CLASS(two_wheel_controller::TwoWheelIDController, controller_interface::ControllerInterface)
