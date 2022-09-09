#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <two_wheel_control_msgs/msg/state.hpp>
#include <two_wheel_control_msgs/msg/command.hpp>
#include <eigen3/Eigen/Dense>
#include <ct/core/core.h>
#include <ct/optcon/optcon.h>

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
    
        Eigen::Matrix<double, 3, 3> A_gen,A;
        Eigen::Matrix<double, 3, 3> T;
        Eigen::Matrix<double, 3, 2> B;
        Eigen::Matrix<double, 3, 3> C; 
        Eigen::Matrix<double, 3, 2> D;
        Eigen::Matrix<double, 3, 3> Q;
        Eigen::Matrix<double, 2, 2> R;


        // TODO: decide what to limit as inequalities
        
        
        double max_angle, l;

        std::unique_ptr<rclcpp::Duration> period;


        rclcpp::Publisher<two_wheel_control_msgs::msg::State>::SharedPtr _state_pub;
        two_wheel_control_msgs::msg::State _pub_msg;
        rclcpp::Subscription<two_wheel_control_msgs::msg::Command>::SharedPtr _sub;
        void subCallback(const two_wheel_control_msgs::msg::Command::SharedPtr msg);


        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
        geometry_msgs::msg::Pose _last_pose;
        nav_msgs::msg::Odometry _odom_msg;
        void updateOdom();

    };

    // Linearized system, for the non linear one we also need the position feedback while we only use velocity here
    class TwoWheelDynamic : public ct::core::ControlledSystem<3, 2>{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            static const size_t STATE_DIM = 3;
            static const size_t CON_DIM = 2;

            typedef ControlledSystem<3, 2> Base;
            typedef typename Base::time_t time_t;   

            TwoWheelDynamic() = delete;

            TwoWheelDynamic(Eigen::Matrix<double, STATE_DIM, STATE_DIM> A, Eigen::Matrix<double, STATE_DIM, CON_DIM> B, std::shared_ptr<ct::core::Controller<3, 2>> controller = nullptr) : ControlledSystem<3, 2>(controller){
                _A = A;
                _B = B;
            }

            TwoWheelDynamic(const TwoWheelDynamic &other){
                this->_A = other._A;
                this->_B = other._B;
            }

            ~TwoWheelDynamic() = default;

            TwoWheelDynamic* clone() const override{
                return new TwoWheelDynamic(*this);
            }

            void computeControlledDynamics(const ct::core::StateVector<STATE_DIM> &x, const ct::core::Time &t, const ct::core::ControlVector<CON_DIM> &control , ct::core::StateVector<STATE_DIM>& derivative) override{
                derivative = _A * x + _B * control;
            }


        private:

            Eigen::Matrix<double, STATE_DIM, STATE_DIM> _A;
            Eigen::Matrix<double, STATE_DIM, CON_DIM> _B;
    };
    
}