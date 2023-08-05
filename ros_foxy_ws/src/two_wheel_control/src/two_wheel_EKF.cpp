#include <two_wheel_control/two_wheel_EKF.hpp>

using namespace two_wheel_controller;

TwoWheelEKF::TwoWheelEKF(){

}

TwoWheelEKF::~TwoWheelEKF(){

}

controller_interface::return_type TwoWheelEKF::init(const std::string &controller_name){
    auto ret = controller_interface::ControllerInterface::init(controller_name);
    if(ret == controller_interface::return_type::ERROR){
        return ret;
    }

    auto_declare<std::vector<double>>("body_lenght", std::vector<double>());
    auto_declare<double>("wheel_radius",0);
    auto_declare<double>("update_frequency",0);

    _state_pub = this->node_->create_publisher<two_wheel_control_msgs::msg::MixedState>("/kalman_state", 10);

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration TwoWheelEKF::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
}

controller_interface::InterfaceConfiguration TwoWheelEKF::state_interface_configuration() const{

    controller_interface::InterfaceConfiguration config;
    config.names.push_back("RW_joint/position");
    config.names.push_back("LW_joint/position");
    config.names.push_back("imu_sensor/orientation.w");
    config.names.push_back("imu_sensor/orientation.x");
    config.names.push_back("imu_sensor/orientation.y");
    config.names.push_back("imu_sensor/orientation.z");
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelEKF::on_configure(const rclcpp_lifecycle::State &previous_state){

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    // Acquire configuarion variables
    node_->get_parameter("wheel_radius", wheel_r);
    if(wheel_r <= 0){
        RCLCPP_ERROR(node_->get_logger(),"In TwoWheelEKF wheel radius must be provided and grater than 0");
        ret = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    node_->get_parameter("update_frequency", update_freq);
    if(update_freq <= 0){
        RCLCPP_ERROR(node_->get_logger(), "In TwoWheelEKF update frequency must be provided and greater than0");
        ret = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    std::vector<double> lenghts;
    node_->get_parameter("body_lenght", lenghts);
    if(lenghts.size() != 3){
        RCLCPP_ERROR(node_->get_logger(),"In TwoWheekEKF body lenghts must be provided as a vector of 3 elements");
        ret = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    for(int i = 0; i < lenghts.size(); i++){
        if(lenghts[i] <= 0){
            RCLCPP_ERROR(node_->get_logger(),"In TwoWheekEKF body lenghts must be all greater than 0");
            ret = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
    }

    body_lenght = Eigen::Matrix<double,3,1>(lenghts.data());

    return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelEKF::on_activate(const rclcpp_lifecycle::State &previous_state) {

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    // Initialize variables 

    x << 0,0,0,0,0,0,0;
    last_encoder << 0,0;
    return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TwoWheelEKF::on_deactivate(const rclcpp_lifecycle::State &previous_state){
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    return ret;
}

controller_interface::return_type TwoWheelEKF::update(){
    last_encoder = encoder;
    encoder(0) = state_interfaces_[0].get_value();
    encoder(1) = state_interfaces_[1].get_value();
    
    Eigen::Quaterniond imu_quat(state_interfaces_[2].get_value(), // w
                                state_interfaces_[3].get_value(), // x
                                state_interfaces_[4].get_value(), // y 
                                state_interfaces_[5].get_value());  // z

    // TODO: Implement the correct kalman filter
    ang_speed_r = (encoder(0) - last_encoder(0)) * update_freq;
    ang_speed_l = (encoder(1) - last_encoder(1)) * update_freq;
    speed_r = ang_speed_r * wheel_r;
    speed_l = ang_speed_l * wheel_r;

    imu_rpy = imu_quat.toRotationMatrix().eulerAngles(2,1,0);
    // double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    // double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    // angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;
    double sinp = std::sqrt(1 + 2 * (imu_quat.w() * imu_quat.y() - imu_quat.x() * imu_quat.z()));
    double cosp = std::sqrt(1 - 2 * (imu_quat.w() * imu_quat.y() - imu_quat.x() * imu_quat.z()));
    double pitch = 2 * std::atan2(sinp,cosp) - 3.14/2;
    
    
    x(0) = x(0) + x(4) * std::cos(x(3)) / update_freq;
    x(1) = x(1) + x(4) * std::sin(x(3)) / update_freq;

    x(5) = (pitch - x(2)) * update_freq;
    x(2) = pitch;
    // x(5) = (imu_rpy(1) - x(2)) * update_freq;
    // //TODO: Normalize between pi/2 -pi/2 instead of 0 -pi/2
    // x(2) = imu_rpy(1);
    
    x(3) = x(3) + x(6) / update_freq;
    x(4) = (speed_r + speed_l) / 2;
    x(6) = (speed_r - speed_l) / body_lenght(1);  

    publishState();

    return controller_interface::return_type::OK;
}

void TwoWheelEKF::publishState(){
    _state_msg.x = x(0);
    _state_msg.y = x(1);
    _state_msg.thetay = x(2);
    _state_msg.thetaz = x(3);
    _state_msg.v = x(4);
    _state_msg.omegay = x(5);
    _state_msg.omegaz = x(6);

    _state_pub->publish(_state_msg);
}

PLUGINLIB_EXPORT_CLASS(two_wheel_controller::TwoWheelEKF, controller_interface::ControllerInterface)