#include <rclcpp/rclcpp.hpp>

#include <two_wheel_control_msgs/msg/acceleration_reference.hpp>

namespace two_wheel_controller{

class DiffDriveMPCNode : public rclcpp::Node{
public:
    DiffDriveMPCNode() : Node("diff_drive_controller"){

    }




private:

    void updateCallback();

    two_wheel_control_msgs::msg::AccelerationReference control_msg;
    std::shared_ptr<rclcpp::Publisher<two_wheel_control_msgs::msg::AccelerationReference>> _command_pub;

};

}