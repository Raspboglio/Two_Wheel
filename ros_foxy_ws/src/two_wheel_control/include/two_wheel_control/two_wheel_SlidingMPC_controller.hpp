namespace two_wheel_controller{

    class TwoWheelMPCController : public controller_interface::ControllerInterface{
    
    public:
    
        TwoWheelMPCController();
        ~TwoWheelMPCController();
        
        
        controller_interface::return_type init(const std::string &controller_name) override;
        
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::return_type update() override;


    private:
    
        Eigen::Matrix<double, 4, 4> A_gen,A;
        Eigen::Matrix<double, 4, 4> T;
        Eigen::Matrix<double, 4, 2> B;
        Eigen::Matrix<double, 4, 4> C; 
        Eigen::Matrix<double, 4, 2> D;
        Eigen::Matrix<double, 4, 4> Q;
        Eigen::Matrix<double, 2, 2> R;
        Eigen::Matrix<double, 3, 1> pos, last_pos;
        Eigen::Matrix<double, 4, 1> state, goal, error;
        ct::core::ControlVector<2,double> control;

        std::unique_ptr<ct::optcon::MPC<ct::optcon::NLOptConSolver<4, 2>>> _mpc;
        ct::core::StateFeedbackController<4, 2> new_policy;
        ct::core::Time ts_new_policy;
        bool success;
        ct::core::Time start_time, cur_time;
        // TODO: decide what to limit as inequalities
        
        
        double max_angle, l, update_rate, w, x, y, z;

        std::unique_ptr<rclcpp::Duration> period;


        rclcpp::Publisher<two_wheel_control_msgs::msg::State>::SharedPtr _state_pub;
        two_wheel_control_msgs::msg::State _pub_msg;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _sub;
        void subCallback(const geometry_msgs::msg::Twist::SharedPtr msg);


        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
        geometry_msgs::msg::Pose _last_pose;
        nav_msgs::msg::Odometry _odom_msg;
        void updateOdom();

    };
}