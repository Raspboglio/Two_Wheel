#include <two_wheel_control/two_wheel_MPC_controller.hpp>

using namespace two_wheel_controller;

TwoWheelMPCController::TwoWheelMPCController(){

}

TwoWheelMPCController::~TwoWheelMPCController(){

}

controller_interface::return_type TwoWheelMPCController::init(const std::string &controller_name){
    auto ret = controller_interface::ControllerInterface::init(controller_name);
    if(ret == controller_interface::return_type::ERROR){
        RCLCPP_ERROR(node_->get_logger(), "Error during controller initialization");
        return ret;
    }

    auto_declare<std::vector<double>>("body_I_diag", std::vector<double>({0,0,0}));
    auto_declare<std::vector<double>>("wheel_I_diag", std::vector<double>({0,0,0}));
    auto_declare<double>("body_mass", 0);
    auto_declare<double>("wheel_mass", 0);
    auto_declare<double>("body_height", 0);
    auto_declare<double>("wheel_redius", 0);
    auto_declare<double>("wheel_distance", 0);
    auto_declare<double>("max_angle", 0);
    auto_declare<double>("update_rate", 0);
    auto_declare<std::vector<double>>("Q_diag", std::vector<double>(4, 0));
    auto_declare<std::vector<double>>("R_diag", std::vector<double>(2, 0));
    auto_declare<double>("prediction_horizon", 10);
    _last_pose.position.x = 0;
    _last_pose.position.y = 0;
    _last_pose.position.z = 0;
    _last_pose.orientation.w = 0;
    _last_pose.orientation.x = 0;
    _last_pose.orientation.y = 0;
    _last_pose.orientation.z = 0;  

    

}

controller_interface::InterfaceConfiguration TwoWheelMPCController::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    
    config.names.push_back("RW_joint/effort");
    config.names.push_back("LW_joint/effort");
    
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    return config;
}

controller_interface::InterfaceConfiguration TwoWheelMPCController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;

    config.names.push_back("RW_joint/position");
    config.names.push_back("LW_joint/position");
    config.names.push_back("imu_sensor/orientation.x");
    config.names.push_back("imu_sensor/orientation.y");
    config.names.push_back("imu_sensor/orientation.z");
    config.names.push_back("imu_sensor/orientation.w");
    
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelMPCController::on_configure(const rclcpp_lifecycle::State & previous_state){    

    max_angle = node_->get_parameter("max_angle").as_double();
    if(max_angle <= 0){
        RCLCPP_ERROR(node_->get_logger(), "max_angle should be specified and positive");
        return CallbackReturn::ERROR;
    }

    update_rate = node_->get_parameter("update_rate").as_double();
    if(update_rate <= 0){
        RCLCPP_ERROR(node_->get_logger(), "Error, update rate must be bigger than 0");
        return CallbackReturn::ERROR;
    }

    // Take physical parameters
    std::vector<double> I_body = node_->get_parameter("body_I_diag").as_double_array();
    std::vector<double> I_wheel = node_->get_parameter("wheel_I_diag").as_double_array();
    if(I_body.size() != 3 || I_wheel.size()!=3){
        RCLCPP_ERROR(node_->get_logger(), "Inertia matrices must have 3 elements");
        return CallbackReturn::ERROR;
    }
    bool vect_null = true;
    for(int i = 0; i < 3 && vect_null; i++){
        vect_null = I_body[i] == 0 || I_wheel[i] == 0;
    }
    if(vect_null){
        RCLCPP_ERROR( node_->get_logger(), "Inertia matrices can't be null, maybe you need to define that");
        return CallbackReturn::ERROR;
    }

    
    double m_b =  node_->get_parameter("body_mass").as_double();
    double m_w =  node_->get_parameter("wheel_mass").as_double();
    if(m_b == 0 || m_w == 0){
        RCLCPP_ERROR( node_->get_logger(), "masses can't be 0, probably you didn't define that");
        return CallbackReturn::ERROR;
    }
    
    l =  node_->get_parameter("body_height").as_double();
    double r =  node_->get_parameter("wheel_radius").as_double();
    double d =  node_->get_parameter("wheel_distance").as_double();
    if(l == 0 || r == 0 || d == 0){
        RCLCPP_ERROR(node_->get_logger(), "length can't be 0, probably you didn't define that");
        return CallbackReturn::ERROR;
    }
 
    // Tuning

    double update_rate = node_->get_parameter("update_rate").as_double();
    if(update_rate <= 0){
        RCLCPP_ERROR(node_->get_logger(), "update_rate must be >=0, probably you didn't define that");
        return CallbackReturn::ERROR;
    }

    period = std::make_unique<rclcpp::Duration>(rclcpp::Duration::from_seconds(1/update_rate));

    _state_pub = node_->create_publisher<two_wheel_control_msgs::msg::State>("state", 10);
    _odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("/odom",10);
    // _tf_pub = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf",50);
    
    _sub = node_->create_subscription<geometry_msgs::msg::Twist>("two_wheel_command", 10, std::bind(&TwoWheelMPCController::subCallback, this,  std::placeholders::_1));
    if(_sub == nullptr){
        RCLCPP_ERROR(node_->get_logger(), "Subscription creation failed");
        return CallbackReturn::ERROR;
    }

    std::vector<double> Q_diag = node_->get_parameter("Q_diag").as_double_array();
    if(Q_diag.size() != 4){
        RCLCPP_ERROR(node_->get_logger(), "Q_diag must be a vector of length 4");
        return CallbackReturn::ERROR;
    }
    std::vector<double> R_diag = node_->get_parameter("R_diag").as_double_array();
    if(R_diag.size() != 2){
        RCLCPP_ERROR(node_->get_logger(), "R_diag must be a vector of length 2");
        return CallbackReturn::ERROR;
    }

    double Hp = node_->get_parameter("prediction_horizon").as_double();


    pos << 0, 0, 0;
    last_pos = pos;
    state << 0, 0, 0, 0;
    error = state;
    goal = state;

    A_gen <<
        0, 0, 1, 0,
        0, I_wheel[2]+3*m_w, l*m_w, 0,
        0, l*m_w, m_w*l*l+I_body[2], 0,
        0, 0, 0, I_body[3]+2*I_wheel[3]+d*d*m_w/2+I_wheel[2]*d*d/(2*r*r);

    T << 
        1, 0, 0, 0, 
        0, 1/r, -1, d/(2*r),
        0, 1/r, -1, -d/(2*r), 
        0, 0, 1, 0;
        

    // x = T_inv * x_hat where x is the real controllable state variables (RW, LW, theta_y) and x_hat si the theretical state variable (p, theta_y, theta_z)
    A = T * A_gen * T.inverse(); 

    // TODO: B coefficient should take into account inertias since u is not an acceleration but an effort
    B <<
        0, 0,
        -1, 0,
        0, -1,
        0, 0;

    C << 
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0, 
        0, 0, 0, 1;
 
    D.setZero();

    Q << 
        Q_diag[0], 0, 0, 0,
        0, Q_diag[1], 0, 0,
        0, 0, Q_diag[2], 0,
        0, 0, 0, Q_diag[3]; 

    
    R <<
        R_diag[0], 0,
        0, R_diag[1];

    const int s_dim = TwoWheelDynamic::STATE_DIM, c_dim = TwoWheelDynamic::CON_DIM;

    std::shared_ptr<ct::core::ControlledSystem<s_dim, c_dim>> system_dynamics(new TwoWheelDynamic(I_body, I_wheel, m_b, m_w, l, l / 2 + r, r, d));
    
    std::shared_ptr<ct::core::SystemLinearizer<s_dim, c_dim>> linearizer(new ct::core::SystemLinearizer<s_dim, c_dim>(system_dynamics));

    std::shared_ptr<ct::optcon::CostFunctionQuadratic<s_dim, c_dim>> cost_fun( new ct::optcon::CostFunctionAnalytical<s_dim, c_dim>() );

    
    std::shared_ptr<ct::optcon::TermQuadratic<s_dim, c_dim>> final_cost( new ct::optcon::TermQuadratic<s_dim, c_dim>(Q,R) );
    std::shared_ptr<ct::optcon::TermQuadratic<s_dim, c_dim>> inter_cost( new ct::optcon::TermQuadratic<s_dim, c_dim>(Q,R) );
    // TODO: add config file dir
    cost_fun->addFinalTerm(final_cost);
    cost_fun->addIntermediateTerm(inter_cost);

    ct::core::StateVector<s_dim> x0;
    x0.setZero();


    //TODO: 10 is the number of iteration
    ct::core::Time time_horizon = Hp/update_rate;

    ct::optcon::ContinuousOptConProblem<s_dim, c_dim> optConProblem(
        time_horizon, x0, system_dynamics, cost_fun, linearizer);

    ct::optcon::NLOptConSettings ilqr_settings;
    ilqr_settings.dt = 1/update_rate;  // the control discretization in [sec]
    ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings.discretization = ct::optcon::NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings.max_iterations = 10;
    ilqr_settings.nlocp_algorithm = ct::optcon::NLOptConSettings::NLOCP_ALGORITHM::ILQR;
    ilqr_settings.lqocp_solver = ct::optcon::NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
    ilqr_settings.printSummary = true;
    size_t K = ilqr_settings.computeK(time_horizon);

    ct::core::FeedbackArray<s_dim, c_dim> u0_fb(K, ct::core::FeedbackMatrix<s_dim, c_dim>::Zero());
    ct::core::ControlVectorArray<c_dim> u0_ff(K, ct::core::ControlVector<c_dim>::Zero());
    ct::core::StateVectorArray<s_dim> x_ref_init(K + 1, x0);
    ct::optcon::NLOptConSolver<s_dim, c_dim>::Policy_t init_controller(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

    ct::optcon::NLOptConSolver<s_dim, c_dim> nl_solver(optConProblem, ilqr_settings);

    nl_solver.setInitialGuess(init_controller);

    nl_solver.solve();
    ct::core::StateFeedbackController<s_dim, c_dim> nl_controller = nl_solver.getSolution(); 

    // TODO: need's initial guess?

    // Init MPC 
    // Reuse ilqr settings
    ilqr_settings.max_iterations = 2;
    ilqr_settings.printSummary = false;

    ct::optcon::mpc_settings mpc_settings;

    mpc_settings.stateForwardIntegration_ = true;
    mpc_settings.postTruncation_ = true;
    mpc_settings.measureDelay_ = true;
    mpc_settings.delayMeasurementMultiplier_ = 1.0;
    mpc_settings.mpc_mode = ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
    mpc_settings.coldStart_ = false;

    _mpc = std::make_unique<ct::optcon::MPC<ct::optcon::NLOptConSolver<s_dim, c_dim>>>(optConProblem, ilqr_settings, mpc_settings);

    _mpc->setInitialGuess(nl_controller);

    // Init B
    // *B = {{I_wheel[2]+3*m_w, l*m_w, 0}, 
    //     {l*m_w, m_w*l*l+I_body[2], 0}, 
    //     {0, 0, I_body[3]+2*I_wheel[3]+d*d*m_w/2+I_wheel[2]*d*d/(2*r*r)}};
    
    // // Init T*
    // *T = {{1/r, -1, -d/(2*r)}, {1/r, -1, d/(2*r)}}; // TODO: not sure about the z contrib
    // *T_out = {{1/r, -1, -d/(2*r)}, {1/r, -1, d/(2*r)}, {0, 1, 0}};
    
    // std::cout << "B: " << std::endl;
    // B->print();
    // std::cout << "T: " << std::endl;
    // T->print();
    // std::cout << "T out: " << std::endl;
    // T_out->print();
    
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelMPCController::on_activate(const rclcpp_lifecycle::State & previous_state){
    start_time = (double)(1e-9 * node_->get_clock()->now().nanoseconds());
    return CallbackReturn::SUCCESS;
}
       
       
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelMPCController::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TwoWheelMPCController::update(){
    last_pos = pos;
    
    x = this->state_interfaces_[2].get_value();
    y = this->state_interfaces_[3].get_value();
    z = this->state_interfaces_[4].get_value();
    w = this->state_interfaces_[5].get_value();
    pos(0) = state_interfaces_.at(0).get_value();
    pos(1) = state_interfaces_.at(1).get_value();

    if(std::abs(2*(w*y - z*x)) < 1){
        pos(2) = std::asin(2*(w*y - z*x));
    }else{
        pos(2) = std::asin(2*(w*y - z*x) + std::copysign(1,(w*y - z*x)));
    }

    // Estimate speed from positions, maybe some more advanced method?
    state << pos(2), (pos(0) - last_pos(0)) * update_rate, (pos(1) - last_pos(1)) * update_rate, (pos(2) - last_pos(2)) * update_rate;
    error = -(goal - state);
    cur_time = (double)(1e-9 * node_->get_clock()->now().nanoseconds());

    
    _mpc->prepareIteration(cur_time);

    success = _mpc->finishIteration(error, cur_time - start_time, new_policy, ts_new_policy);

    if(!success){
        RCLCPP_ERROR(node_->get_logger(), "MPC couldn't find a solution");
        return controller_interface::return_type::ERROR;
    }


    auto ctrl = _mpc->getSolver().getControlTrajectory();
    auto traj = _mpc->getSolver().getStateTrajectory();
    // new_policy.computeControl(error, cur_time - start_time, control);
    // ct::core::DiscreteArray<ct::core::FeedbackMatrix<4,2>> feedback = new_policy.K();

    control = ctrl[0];

    // std::cout << "\nerror: \n" << error << "\ncommand: \n" << control << std::endl;
    
    // ctrl.print();
    traj.print();


    for(long int i = 0; i < 2; i++){
        command_interfaces_.at(i).set_value(control(i));
    }

    _pub_msg.p = pos(0);
    _pub_msg.thetay = pos(1);
    _pub_msg.thetaz = pos(2);
    _pub_msg.v = state(1);
    _pub_msg.omegay = state(2);
    _pub_msg.omegaz = state(3);

    _state_pub->publish(_pub_msg);

}

// TODO: check T
void TwoWheelMPCController::subCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    
    goal(0) = 0;
    goal(1) = msg->linear.x;
    goal(2) = 0;
    goal(3) = msg->angular.z;

    goal = T * goal;
    
    RCLCPP_INFO(this->node_->get_logger(), "Received new goal wr: %.2f wl: %.2f ",goal(1), goal(2));
}

void TwoWheelMPCController::updateOdom(){
    // TODO: use u_dot where possible once modified hardware interfaces
    _odom_msg.child_frame_id = "base_link";
    _odom_msg.header.frame_id = "odom";
    _odom_msg.header.stamp = node_->get_clock()->now();
    
    // Compute velocities
    // _odom_msg.twist.twist.linear.x = (u_dot->at(0)) * std::cos(u->at(2));
    // _odom_msg.twist.twist.linear.y = (u_dot->at(0)) * std::cos(u->at(2));
    // _odom_msg.twist.twist.linear.z = 0;

    // _odom_msg.twist.twist.angular.x = 0;
    // _odom_msg.twist.twist.angular.y = 0;
    // _odom_msg.twist.twist.angular.z = u_dot->at(2);
    
    // // Compute positions
    // // Consider speed constant in the step we obtain circular trajectories (actually accelerations are constant)
    // if((std::abs(u->at(2) - last_u->at(2))) < 1e-4){
    //     // Only linear movement
    //     _odom_msg.pose.pose.position.x = _last_pose.position.x + (u->at(0) - last_u->at(0)) * std::cos(u->at(2));
    //     _odom_msg.pose.pose.position.y = _last_pose.position.y + (u->at(0) - last_u->at(0)) * std::sin(u->at(2));
    //     _odom_msg.pose.pose.position.z = 0;    
    // }else{

    //     // TODO: method taken from diff drive controller, check validity
    //     _odom_msg.pose.pose.position.x = _last_pose.position.x + u_dot->at(0) / u_dot->at(2) * (std::sin(u->at(2)) - std::sin(last_u->at(2)));
    //     _odom_msg.pose.pose.position.y = _last_pose.position.y + u_dot->at(0) / u_dot->at(2) * (std::cos(u->at(2)) - std::cos(last_u->at(2)));
    //     _odom_msg.pose.pose.position.z = 0;

    // }
    
    // // We only know thetaz from the encoder, the other will be taken from the IMU
    // _odom_msg.pose.pose.orientation.w = std::cos(u->at(2) * 0.5);
    // _odom_msg.pose.pose.orientation.x = 0;
    // _odom_msg.pose.pose.orientation.y = 0;
    // _odom_msg.pose.pose.orientation.z = std::sin(u->at(2) * 0.5);

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

PLUGINLIB_EXPORT_CLASS(two_wheel_controller::TwoWheelMPCController, controller_interface::ControllerInterface)