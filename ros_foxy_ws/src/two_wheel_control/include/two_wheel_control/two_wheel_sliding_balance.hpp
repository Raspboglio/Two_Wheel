#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

#include <two_wheel_control_msgs/msg/input_command.hpp>
#include <two_wheel_control_msgs/msg/mixed_state.hpp>
#include <two_wheel_control_msgs/msg/acceleration_reference.hpp>

#define DEFAULT_UPDATE_FREQUENCY 100.0
#define DEFAULT_LENGHTS std::vector<double>{0.1, 0.5, 1}
#define DEFAULT_INERTIA std::vector<double>{0.5, 0.5, 0.5}
#define DEFAULT_MB 1
#define DEFAULT_MW 0.1
#define DEFAULT_RW 0.1
#define DEFAULT_K std::vector<double>{1, 1}
#define DEFAULT_NU 1
#define CONST_g 9.81


namespace two_wheel_controller{
    
typedef struct{
    std::vector<double> lenght;
    std::vector<double> inertia;
    double mb;
    double mw;
    double rw;
} sliding_parameter;

typedef struct{
    std::vector<double> k;
    double nu;
} sliding_gain;

class TwoWheelSlidingBalance : public rclcpp::Node{
public:
    
    TwoWheelSlidingBalance() : Node("two_wheel_sliding_balance"){
        
        //Get Parameters
        update_frequency = this->declare_parameter<double>("update_frequency", 0);
        if(update_frequency <= 0){
            RCLCPP_ERROR(get_logger(), "In TwoWheelSlidingBalance update_frequency is not set, setting default value");
            update_frequency = DEFAULT_UPDATE_FREQUENCY;
        }
        
        params.lenght = this->declare_parameter<std::vector<double>>("body_lenght", std::vector<double>());
        if(params.lenght.size() != 3){
            RCLCPP_ERROR(get_logger(), "In TwoWheelSlidingBalance body lenght must have 3 values, setting default value");
            params.lenght = DEFAULT_LENGHTS;
        }

        params.inertia = this->declare_parameter<std::vector<double>>("robot_inertia", std::vector<double>());
        if(params.inertia.size() != 3){
            RCLCPP_ERROR(get_logger(), "In TwoWheelSlidingBalance robot_inertia must have 3 values, setting default value");
            params.lenght = DEFAULT_INERTIA;
        }

        params.mb = this->declare_parameter<double>("body_mass", 0);
        if(params.mb <= 0){
            RCLCPP_ERROR(get_logger(), "In TwoWheelSlidingBalance body_mass is not set, setting default value");
            params.mb = DEFAULT_MB;
        }

        params.mw = this->declare_parameter<double>("wheel_mass", 0);
        if(params.mw <= 0){
            RCLCPP_ERROR(get_logger(), "In TwoWheelSlidingBalance wheel_mass is not set, setting default value");
            params.mw = DEFAULT_MW;
        }

        params.rw = this->declare_parameter<double>("wheel_radius",0);
        if(params.rw <= 0){
            RCLCPP_ERROR(get_logger(), "In TwoWheelSlidingBalance wheel_radius is not set, setting default value");
            params.rw = DEFAULT_RW;
        }

        gains.k = this->declare_parameter<std::vector<double>>("k_gains", std::vector<double>());
        if(gains.k.size() != 2){
            RCLCPP_ERROR(get_logger(),"In TwoWheelSlidingBalance k gains must have 2 values, setting default value");
            gains.k = DEFAULT_K;
        }
        
        gains.nu = this->declare_parameter<double>("nu_gain",0);
        if(gains.nu <= 0){
            RCLCPP_ERROR(get_logger(), "In TwoWheelSlidingBalance nu_gain is not set, setting default value");
            gains.nu = DEFAULT_NU;
        }

        //Initialize parameters
        x = Eigen::Matrix<double, 7, 1>::Zero();
        aref.push_back(0);
        aprev = 0;
        aprev_dot = 0;

        //Create ROS2 structure
        _command_pub = create_publisher<two_wheel_control_msgs::msg::InputCommand>("/input_command", 10);
        _state_sub = create_subscription<two_wheel_control_msgs::msg::MixedState>("/kalman_state",10,std::bind(&TwoWheelSlidingBalance::stateCallback,this,std::placeholders::_1));
        _reference_sub = create_subscription<two_wheel_control_msgs::msg::AccelerationReference>("/acceleration_referece", 10, std::bind(&TwoWheelSlidingBalance::referenceCallback, this, std::placeholders::_1));

        _update_timer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration((double)1e9/update_frequency), std::bind(&TwoWheelSlidingBalance::updateCallback, this));
    }

    ~TwoWheelSlidingBalance();

private:

    Eigen::Matrix<double, 7, 1> x;
    double update_frequency;
    
    two_wheel_control_msgs::msg::InputCommand command_msg;
    rclcpp::Publisher<two_wheel_control_msgs::msg::InputCommand>::SharedPtr _command_pub;
    rclcpp::TimerBase::SharedPtr _update_timer;
    void updateCallback();

    double s, a, b, y, y_dot;
    double Fx;
    sliding_parameter params;
    sliding_gain gains;
    void slidingSurface();

    std::vector<double> aref, aref_dot, aref_ddot;
    rclcpp::Subscription<two_wheel_control_msgs::msg::AccelerationReference>::SharedPtr _reference_sub;
    void referenceCallback(const two_wheel_control_msgs::msg::AccelerationReference::SharedPtr msg);

    double aprev, aprev_dot, acur, acur_dot, acur_ddot;
    Eigen::Matrix<double, 3, 1> ref;
    void generateRef();


    rclcpp::Subscription<two_wheel_control_msgs::msg::MixedState>::SharedPtr _state_sub;
    void stateCallback(const two_wheel_control_msgs::msg::MixedState::SharedPtr msg);

    double accToAngle(double acc);

    // TODO: can add an extra check where it will set the default value in that case aswell, it can be passed as a bool function pointer
    template <typename T>
    T getParam(std::string name, T default_value){
        T ret = default_value;
        try{
            if(get_parameter(name).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET){
                std::string error_msg = "In TwoWheelSlidingBalance " + name + " not set, setting default value";
                RCLCPP_ERROR(get_logger(), error_msg);
                ret = default_value;
            }
            ret = get_parameter(name).get_value<T>();
        }catch(std::exception &e){
            std::string error_msg = "In TwoWheelSlidingBalance " + name + " not set, setting default value";
            RCLCPP_ERROR(get_logger(), error_msg);
            ret = default_value;
        }
        return ret;
    }

};

}