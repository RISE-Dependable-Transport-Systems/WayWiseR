#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono;

class TeleopGateway : public rclcpp::Node
{
public:
    TeleopGateway()
        : Node("teleop_gateway")
    {
        wheelbase_ = this->declare_parameter("wheelbase", 0.33);
        min_allowed_linear_velocity_ = this->declare_parameter("min_allowed_linear_velocity", 0.01);
        min_allowed_angular_velocity_ = this->declare_parameter("min_allowed_angular_velocity", 0.001);

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_in", 10, std::bind(&TeleopGateway::topic_callback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_out", 10);
    }

private:
    float correct_angular_velocity_for_reverse(float linVel, float angVel) const
    {
        if (std::abs(angVel) < min_allowed_angular_velocity_)
            return 0;
        if (linVel < 0)
            angVel = -angVel;
        return angVel;
    }

    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr twi_msg_in) const
    {
        auto twi_msg_out = geometry_msgs::msg::Twist();
        float linVel = twi_msg_in->linear.x;
        float angVel = 0.0;
        if (std::abs(linVel) < min_allowed_linear_velocity_)
            linVel = 0;
        else
            angVel = correct_angular_velocity_for_reverse(linVel, twi_msg_in->angular.z);
        twi_msg_out.linear.x = linVel;
        twi_msg_out.angular.z = angVel;
        publisher_->publish(twi_msg_out);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    float wheelbase_, min_allowed_linear_velocity_, min_allowed_angular_velocity_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopGateway>());
    rclcpp::shutdown();
    return 0;
}
