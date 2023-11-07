#include <memory>
#include <cmath>
#include <chrono>
#include <vector>

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
        reverse_steer_correction_topics_ = this->declare_parameter("reverse_steer_correction_topics", std::vector<std::string>({"joy_vel"}));

        twist_subscribers_.resize(reverse_steer_correction_topics_.size());
        for (size_t i = 0; i < reverse_steer_correction_topics_.size(); ++i)
        {
            auto topic_name = reverse_steer_correction_topics_[i];
            twist_subscribers_[i] = this->create_subscription<geometry_msgs::msg::Twist>(
                topic_name, 10,
                [this, topic_name_ = topic_name](const geometry_msgs::msg::Twist::SharedPtr twist_msg_in)
                {
                    twist_subscriber_callback(twist_msg_in, topic_name_);
                });
            twist_publishers_[topic_name] = this->create_publisher<geometry_msgs::msg::Twist>(topic_name + "_corrected", 10);
        }
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

    void twist_subscriber_callback(const geometry_msgs::msg::Twist::SharedPtr twi_msg_in, const std::string &topic_name)
    {
        auto twi_msg_out = geometry_msgs::msg::Twist();
        float linVel = twi_msg_in->linear.x;
        float angVel = 0.0;
        if (std::abs(linVel) < min_allowed_linear_velocity_)
            linVel = 0.0;
        else
            angVel = correct_angular_velocity_for_reverse(linVel, twi_msg_in->angular.z);
        twi_msg_out.linear.x = linVel;
        twi_msg_out.angular.z = angVel;
        twist_publishers_[topic_name]->publish(twi_msg_out);
    }
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> twist_subscribers_;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> twist_publishers_;
    std::vector<std::string> reverse_steer_correction_topics_;
    float wheelbase_, min_allowed_linear_velocity_, min_allowed_angular_velocity_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopGateway>());
    rclcpp::shutdown();
    return 0;
}
