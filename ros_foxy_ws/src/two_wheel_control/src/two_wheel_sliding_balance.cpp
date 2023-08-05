#include <two_wheel_control/two_wheel_sliding_balance.hpp>

using namespace two_wheel_controller;

TwoWheelSlidingBalance::~TwoWheelSlidingBalance(){

}

void TwoWheelSlidingBalance::updateCallback(){

    generateRef();
    slidingSurface();
    command_msg.mr = params.rw / 2 * (Fx);
    command_msg.ml = params.rw / 2 * (Fx);
    _command_pub->publish(command_msg);
}

void TwoWheelSlidingBalance::slidingSurface(){

    y = x(2);
    y_dot = x(5);

    a = params.lenght[2] * CONST_g * params.mb / (2 * params.inertia[1]) * std::sin(y);
    b = -(params.rw + params.lenght[2] * std::cos(y) / 2) / params.inertia[1];
    
    s = ref(1) - y_dot + gains.k[1] * (ref(0) - y);

    Fx = (ref(2) - a + gains.k[1] * (ref(1) - y_dot) + gains.k[0] * std::tanh(gains.nu*s)) / b;
}

void TwoWheelSlidingBalance::generateRef(){

    if(aref_ddot.size() == 0 && aref_dot.size() == 0){
        
        //Regulation
        acur = aref[0];
        acur_dot = (acur - aprev) * update_frequency;
        acur_ddot = (acur_dot - aprev_dot) * update_frequency;

        if(aref.size() > 1){
            //Partial Trajectory 
            aref.erase(aref.begin());
        }
    }else{
        //Full Trajectory
        acur = aref[0];
        acur_dot = aref_dot[0];
        acur_ddot = aref_ddot[0];

        if(aref.size() > 1){
            aref.erase(aref.begin());
        }
        aref_dot.erase(aref_dot.begin());
        aref_ddot.erase(aref_ddot.begin());
    }

    ref << accToAngle(acur),
        0,
        0;
        // accToAngle(acur_dot),
        // accToAngle(acur_ddot);
    
    aprev = acur;
    aprev_dot = acur_dot;
}


void TwoWheelSlidingBalance::stateCallback(const two_wheel_control_msgs::msg::MixedState::SharedPtr msg){

    x << msg->x, 
        msg->y, 
        msg->thetay, 
        msg->thetaz, 
        msg->v, 
        msg->omegay,
        msg->omegaz;
}

void TwoWheelSlidingBalance::referenceCallback(const two_wheel_control_msgs::msg::AccelerationReference::SharedPtr msg){
    if(msg->acc.size() == 0){
        RCLCPP_ERROR(get_logger(), "In TwoWheelSlidingBalance reference acceleration vector can't be empty, keeping last");
        return;
    }
    // If at least one derivative reference is != 0 the dimension must agree
    if(msg->acc_dot.size() != 0 || msg->acc_ddot.size() != 0){
        if(msg->acc.size() != msg->acc_dot.size() || msg->acc.size() != msg->acc_ddot.size()){
            RCLCPP_ERROR(get_logger(), "In TwoWheelSlidingBalance dimensions of references must agree, keeping last");
            return;
        }
    }
    aref = msg->acc;
    aref_dot = msg->acc_dot;
    aref_ddot = msg->acc_ddot;
}

double TwoWheelSlidingBalance::accToAngle(double acc){
    return ((params.mb + 2*params.mw) * (params.lenght[2] + 2 * params.rw)) / (CONST_g * params.lenght[2]*params.mb) * acc;
}

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwoWheelSlidingBalance>());
    rclcpp::shutdown();


    return 0;
}

