#include<two_wheel_control/two_wheel_ID_controller.hpp>

// TODO: Remove the velocity hardware interfaces and only take positions/orientations. Then compute velocities (more realistic)
// TOOD: change everything to realtime pub/sub
using namespace two_wheel_controller;

TwoWheelIDController::TwoWheelIDController(){

}

TwoWheelIDController::~TwoWheelIDController(){
    
}

controller_interface::return_type TwoWheelIDController::init(const std::string &controller_name){
    controller_interface::return_type res = controller_interface::ControllerInterface::init(controller_name); 
    if(res == controller_interface::return_type::ERROR) return res;
    // Physical parameters
    auto_declare<std::vector<double>>("body_I_diag", std::vector<double>({0,0,0}));
    auto_declare<std::vector<double>>("wheel_I_diag", std::vector<double>({0,0,0}));
    auto_declare<double>("body_mass", 0);
    auto_declare<double>("wheel_mass", 0);
    auto_declare<double>("body_height", 0);
    auto_declare<double>("wheel_redius", 0);
    auto_declare<double>("wheel_distance", 0);
    auto_declare<double>("max_angle", 0);
    auto_declare<std::vector<double>>("P", std::vector<double>({0}));
    auto_declare<std::vector<double>>("I", std::vector<double>({0}));
    auto_declare<std::vector<double>>("D", std::vector<double>({0}));
    auto_declare<std::vector<double>>("max_I", std::vector<double>({0}));
    // Tuning
    auto_declare<bool>("tuning", false);
    auto_declare<double>("update_rate", 0);
    // Filtering
    auto_declare<int>("avg_size",1);

    // Allocate memory for matices/vectors
    // RCLCPP_INFO(this->node_->get_logger(), "INIT MATRICES");
    u = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    last_u = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
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
    tau = std::make_unique<arma::Col<double>>(2, arma::fill::zeros);
    gamma = std::make_unique<arma::Col<double>>(3, arma::fill::zeros);

    // TODO: add initial values for inputs as parameters
    q_w = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);
    q_w_dot = std::make_unique<arma::Col<double>>(3,arma::fill::zeros);

    // Allocate PID memory
    for(int i = 0; i < 6; i++){
        control_toolbox::Pid tmp;
        PID.push_back(tmp);
    }

    // Init position
    _last_pose.position.x = 0;
    _last_pose.position.y = 0;
    _last_pose.position.z = 0;
    _last_pose.orientation.w = 0;
    _last_pose.orientation.x = 0;
    _last_pose.orientation.y = 0;
    _last_pose.orientation.z = 0;
    

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
    
    config.names.push_back("RW_joint/position");
    config.names.push_back("LW_joint/position");
    config.names.push_back("imu_sensor/orientation.x");
    config.names.push_back("imu_sensor/orientation.y");
    config.names.push_back("imu_sensor/orientation.z");
    config.names.push_back("imu_sensor/orientation.w");
    

    config.names.push_back("RW_joint/velocity");
    config.names.push_back("LW_joint/velocity");
    config.names.push_back("imu_sensor/angular_velocity.y");
    
    
    

    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_configure(const rclcpp_lifecycle::State & previous_state){
    // RCLCPP_INFO(this->node_->get_logger(), "INIT CONFIGURE");
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
    std::vector<double> P_coeff = this->node_->get_parameter("P").as_double_array();
    std::vector<double> D_coeff = this->node_->get_parameter("D").as_double_array();
    std::vector<double> I_coeff = this->node_->get_parameter("I").as_double_array();
    std::vector<double> max_I = this->node_->get_parameter("max_I").as_double_array();

    if(P_coeff.size() != 6 || I_coeff.size() != 6 || D_coeff.size() != 6 || max_I.size() != 6){
        RCLCPP_ERROR(node_->get_logger(), "PID gains must be vector of dimension 6 (1 for each state variable)");
        return CallbackReturn::ERROR;
    }

    for(int i = 0; i < 6; i++){
        PID[i].initPid(P_coeff[i], I_coeff[i], D_coeff[i], max_I[i], -max_I[i]);
    }

    // Tuning

    double update_rate = node_->get_parameter("update_rate").as_double();
    if(update_rate <= 0){
        RCLCPP_ERROR(node_->get_logger(), "update_rate must be >=0, probably you didn't define that");
        return CallbackReturn::ERROR;
    }

    period = std::make_unique<rclcpp::Duration>(rclcpp::Duration::from_seconds(1/update_rate));

    tuning = this->node_->get_parameter("tuning").as_bool();
    _state_pub = this->node_->create_publisher<two_wheel_control_msgs::msg::State>("state", 10);
    _odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("/odom",10);
    // _tf_pub = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf",50);
    if(tuning){
        _tuning_command = this->node_->create_subscription<two_wheel_control_msgs::msg::TuningCommand>("tuning", 10, std::bind(&TwoWheelIDController::tuningCallback, this, std::placeholders::_1));
    }else{
        _sub = this->node_->create_subscription<two_wheel_control_msgs::msg::Command>("two_wheel_command", 10, std::bind(&TwoWheelIDController::subCallback, this,  std::placeholders::_1));
        if(_sub == nullptr){
            RCLCPP_ERROR(node_->get_logger(), "Subscription creation failed");
            return CallbackReturn::ERROR;
        }
    }

    // Filtering

    int avg_size = node_->get_parameter("avg_size").as_int();
    _avg.resize(avg_size,0);

    // Init B
    *B = {{I_wheel[2]+3*m_w, l*m_w, 0}, 
        {l*m_w, m_w*l*l+I_body[2], 0}, 
        {0, 0, I_body[3]+2*I_wheel[3]+d*d*m_w/2+I_wheel[2]*d*d/(2*r*r)}};
    
    // Init T*
    *T = {{1/r, -1, -d/(2*r)}, {1/r, -1, d/(2*r)}}; // TODO: not sure about the z contrib
    *T_out = {{1/r, -1, -d/(2*r)}, {1/r, -1, d/(2*r)}, {0, 1, 0}};
    
    // std::cout << "B: " << std::endl;
    // B->print();
    // std::cout << "T: " << std::endl;
    // T->print();
    // std::cout << "T out: " << std::endl;
    // T_out->print();

    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_activate(const rclcpp_lifecycle::State & previous_state){
    std::vector<control_toolbox::Pid>::iterator pid_it = PID.begin();
    // for(; pid_it != PID.end(); pid_it++){
    //     pid_it->reset();
    // }
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelIDController::on_deactivate(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TwoWheelIDController::update(){
    // Measure current state
    try{          
        u_m->at(0) = this->state_interfaces_[0].get_value();
        u_m->at(1) = this->state_interfaces_[1].get_value();

        //Handle quaternions
        x = this->state_interfaces_[2].get_value();
        y = this->state_interfaces_[3].get_value();
        z = this->state_interfaces_[4].get_value();
        w = this->state_interfaces_[5].get_value();
        if(std::abs(2*(w*y - z*x)) < 1){
            u_m->at(2) = std::asin(2*(w*y - z*x));
        }else{
            u_m->at(2) = std::asin(2*(w*y - z*x) + std::copysign(1,(w*y - z*x)));
        }
            
        
        u_m_dot->at(0) = this->state_interfaces_[6].get_value();
        u_m_dot->at(1) = this->state_interfaces_[7].get_value();
        u_m_dot->at(2) = this->state_interfaces_[8].get_value();
        
        *u = arma::inv(*T_out) * *u_m;
        *u_dot = arma::inv(*T_out) * *u_m_dot;
        // std::cout << "measured position" << std::endl;
        // u_m->print();


        updateOdom();

        e->zeros();
        e_dot->zeros();
        if(tuning){
            e->at(1) = q_w->at(1) - u->at(1);
            e_dot->at(1) = q_w_dot->at(1) - u_dot->at(1);
        }else{
            e->at(0) = q_w->at(0) - u->at(0);
            e_dot->at(0) = q_w_dot->at(0) - u_dot->at(0);
            // std::cout << q_w_dot->at(0) << std::endl << u_dot->at(0) << std::endl << e_dot->at(0) << std::endl << std::endl;


            v_ddot = (PID[0].computeCommand(e->at(0), period->nanoseconds()) + PID[3].computeCommand(e_dot->at(0), period->nanoseconds())) *  B->at(1,1) / (B->at(0,1) * 9.81 * l);
            _avg.erase(_avg.begin());
            if(std::abs(std::asin(v_ddot)) < max_angle){
                _avg.push_back(std::asin(v_ddot));
            }else{
                RCLCPP_ERROR(this->node_->get_logger(),"Exceeded angle");
                _avg.push_back(std::asin(std::copysign(max_angle,v_ddot)));
            }
            // Filtering with moving avarange
            q_w->at(1) = 0;
            std::vector<double>::iterator avg_it = _avg.begin();
            for(; avg_it != _avg.end(); avg_it++){
                q_w->at(1) += *avg_it;    
            }
            q_w->at(1) = q_w->at(1) / _avg.size();

            // if(std::abs(std::asin(v_ddot)) < max_angle){
            //     q_w->at(1) = std::asin(v_ddot);
            // }else{
            //     RCLCPP_ERROR(this->node_->get_logger(),"Exceeded angle");
            //     q_w->at(1) = std::asin(std::copysign(max_angle,v_ddot));
            // }

            // std::cout << "Pos error: " << (q_w->at(0) - u->at(0)) <<std::endl;
            // std::cout << "Vel error: " << (q_w_dot->at(0) - u_dot->at(0)) <<std::endl;
            // std::cout << "desired theta: " << q_w->at(1) << std::endl;
            // std::cout << "cur theta: " << u->at(1) << std::endl;
            *e = *q_w - *u;
            
            q_w_dot->at(1) = 0;
            // std::cout << "cur vely: " << u_dot->at(1) << std::endl;
            *e_dot = *q_w_dot - *u_dot;
            
            // Ignore p error since it's already taken into account in theta_y
            e->at(0) = 0;
            e_dot->at(0) = 0;
        }
        // std::cout << "pos error: " << std::endl;
        // e->print();
        // std::cout << "vel error: " << std::endl;
        // e_dot->print();
        // std::cout << std::endl;        

        // std::cout << "curr state: " << std::endl;
        // u->print();
        // std::cout << "curr vel: " << std::endl;
        // u_dot->print();
        
        _pub_msg.p = u->at(0);
        _pub_msg.thetay = u->at(1) * 10;
        _pub_msg.thetaz = u->at(2);
        _pub_msg.v = u_dot->at(0) ;
        _pub_msg.omegay = u_dot->at(1);
        _pub_msg.omegaz = u_dot->at(2);
        _pub_msg.thetay_tgt = q_w->at(1) * 10;
        _state_pub->publish(_pub_msg);
        

        //TODO: limit tau
        *G = {0,-9.81 * l * std::sin(u->at(1)), 0};
        gamma->at(0) = 0;
        gamma->at(1) = PID[1].computeCommand(e->at(1), period->nanoseconds()) + PID[4].computeCommand(e_dot->at(1), period->nanoseconds());
        gamma->at(2) = PID[2].computeCommand(e->at(2), period->nanoseconds()) + PID[5].computeCommand(e_dot->at(2), period->nanoseconds());

        *tau = *T * (*B * *gamma + *G); // TODO: check what we need between T, T' and T_inv
        *last_u = *u;

    }catch(std::exception &err){
        RCLCPP_ERROR(this->node_->get_logger(), err.what());
    }

    for(int i = 0; i < 2; i++){
        this->command_interfaces_[i].set_value(tau->at(i));
    }

    return controller_interface::return_type::OK;
}

void TwoWheelIDController::subCallback(const two_wheel_control_msgs::msg::Command::SharedPtr msg){
    
    
    q_w->at(0) = msg->p;
    q_w->at(2) = msg->thetaz;

    q_w_dot->at(0) = msg->p_dot;
    q_w_dot->at(2) = msg->thetaz_dot;
    
    // RCLCPP_INFO(this->node_->get_logger(), "Received new command p:\n%.4f\n%.4f\n v: \n%.4f\n%.4f", q_w->at(0), q_w->at(2), q_w_dot->at(0), q_w_dot->at(2));
}

void TwoWheelIDController::tuningCallback(const two_wheel_control_msgs::msg::TuningCommand::SharedPtr msg){
    q_w->zeros();
    q_w_dot->zeros();
        
    q_w->at(1) = msg->thetay;
    q_w_dot->at(1) = msg->thetay_dot;
    
    // RCLCPP_INFO(this->node_->get_logger(), "Received new command \n%.4f\n%.4f\n", q_w->at(1), q_w_dot->at(1));
}

void TwoWheelIDController::updateOdom(){
    // TODO: use u_dot where possible once modified hardware interfaces
    _odom_msg.child_frame_id = "base_link";
    _odom_msg.header.frame_id = "odom";
    _odom_msg.header.stamp = node_->get_clock()->now();
    
    // Compute velocities
    _odom_msg.twist.twist.linear.x = (u_dot->at(0)) * std::cos(u->at(2));
    _odom_msg.twist.twist.linear.y = (u_dot->at(0)) * std::cos(u->at(2));
    _odom_msg.twist.twist.linear.z = 0;

    _odom_msg.twist.twist.angular.x = 0;
    _odom_msg.twist.twist.angular.y = 0;
    _odom_msg.twist.twist.angular.z = u_dot->at(2);
    
    // Compute positions
    // Consider speed constant in the step we obtain circular trajectories (actually accelerations are constant)
    if((std::abs(u->at(2) - last_u->at(2))) < 1e-4){
        // Only linear movement
        _odom_msg.pose.pose.position.x = _last_pose.position.x + (u->at(0) - last_u->at(0)) * std::cos(u->at(2));
        _odom_msg.pose.pose.position.y = _last_pose.position.y + (u->at(0) - last_u->at(0)) * std::sin(u->at(2));
        _odom_msg.pose.pose.position.z = 0;    
    }else{

        // TODO: method taken from diff drive controller, check validity
        _odom_msg.pose.pose.position.x = _last_pose.position.x + u_dot->at(0) / u_dot->at(2) * (std::sin(u->at(2)) - std::sin(last_u->at(2)));
        _odom_msg.pose.pose.position.y = _last_pose.position.y + u_dot->at(0) / u_dot->at(2) * (std::cos(u->at(2)) - std::cos(last_u->at(2)));
        _odom_msg.pose.pose.position.z = 0;

    }
    
    // We only know thetaz from the encoder, the other will be taken from the IMU
    _odom_msg.pose.pose.orientation.w = std::cos(u->at(2) * 0.5);
    _odom_msg.pose.pose.orientation.x = 0;
    _odom_msg.pose.pose.orientation.y = 0;
    _odom_msg.pose.pose.orientation.z = std::sin(u->at(2) * 0.5);

    // _transform.header.stamp = node_->get_clock()->now();
    // _transform.child_frame_id = "base_link";
    // _transform.header.frame_id = "odom";
    // _transform.transform.translation.x = _odom_msg.pose.pose.position.x;
    // _transform.transform.translation.y = _odom_msg.pose.pose.position.y;
    // _transform.transform.translation.z = _odom_msg.pose.pose.position.z;

    // _transform.transform.rotation.w = _odom_msg.pose.pose.orientation.w;
    // _transform.transform.rotation.x = _odom_msg.pose.pose.orientation.x;
    // _transform.transform.rotation.y = _odom_msg.pose.pose.orientation.y;
    // _transform.transform.rotation.z = _odom_msg.pose.pose.orientation.z;


    // _tf_msg.transforms.push_back(_transform);
    _last_pose = _odom_msg.pose.pose;

    _odom_pub->publish(_odom_msg);
    // _tf_pub->publish(_tf_msg);

}

PLUGINLIB_EXPORT_CLASS(two_wheel_controller::TwoWheelIDController, controller_interface::ControllerInterface)
