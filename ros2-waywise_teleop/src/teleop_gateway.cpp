#include <memory>
#include <cmath>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

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
        emergency_stop_set_joy_button_index_ = this->declare_parameter("emergency_stop_set_joy_button_index", 5);
        emergency_stop_clear_joy_button_index_ = this->declare_parameter("emergency_stop_clear_joy_button_index", 7);
        joy_emergency_stop_timeout_ = this->declare_parameter("joy_emergency_stop_timeout", 1.0);
        emergency_stop_value_ = false;
        RCLCPP_INFO(get_logger(), "emergency_stop can be ACTIVATED using button %d and CLEARED using button %d of joystick. It can also be managed by publishing boolean data on /emergency_stop topic.", emergency_stop_set_joy_button_index_, emergency_stop_clear_joy_button_index_);
        RCLCPP_INFO(get_logger(), "To activate emergency_stop from command line: ros2 topic pub --once /emergency_stop std_msgs/msg/Bool \"data: true\"");
        RCLCPP_INFO(get_logger(), "To clear emergency_stop from command line: ros2 topic pub --once /emergency_stop std_msgs/msg/Bool \"data: false\"");

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
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&TeleopGateway::joy_callback, this, std::placeholders::_1));
        emergency_stop_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop", 10, std::bind(&TeleopGateway::emergency_stop_callback, this, std::placeholders::_1));
        mux_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_mux", 10, std::bind(&TeleopGateway::mux_callback, this, std::placeholders::_1));
        gateway_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_out", 10);

        joy_watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)std::round(1000.0 * joy_emergency_stop_timeout_)), std::bind(&TeleopGateway::joy_watchdog_callback, this));
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

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        last_joy_msg_received_time_ = this->now();
        if (joy_msg->buttons[emergency_stop_set_joy_button_index_] == 1)
        {
            if (!emergency_stop_value_)
            {
                emergency_stop_value_ = true;
                publish_zero_cmd_vel();
                RCLCPP_WARN(get_logger(), "emergency_stop ACTIVATED from /joy topic.");
            }
        }
        else if (joy_msg->buttons[emergency_stop_clear_joy_button_index_] == 1)
        {
            if (emergency_stop_value_)
            {
                emergency_stop_value_ = false;
                RCLCPP_WARN(get_logger(), "emergency_stop CLEARED from /joy topic.");
            }
        }
    }

    void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr emergency_stop_msg)
    {
        if (emergency_stop_msg->data)
        {
            if (!emergency_stop_value_)
            {
                emergency_stop_value_ = true;
                publish_zero_cmd_vel();
                RCLCPP_WARN(get_logger(), "emergency_stop ACTIVATED from /emergency_stop topic.");
            }
        }
        else
        {
            if (emergency_stop_value_)
            {
                emergency_stop_value_ = false;
                RCLCPP_WARN(get_logger(), "emergency_stop CLEARED from /emergency_stop topic.");
            }
        }
    }

    void mux_callback(const geometry_msgs::msg::Twist::SharedPtr twi_msg_in)
    {
        if (emergency_stop_value_)
        {
            publish_zero_cmd_vel();
        }
        else
        {
            gateway_publisher_->publish(*twi_msg_in);
        }
    }

    void publish_zero_cmd_vel() const
    {
        auto twi_msg_out = geometry_msgs::msg::Twist();
        twi_msg_out.linear.x = 0.0;
        twi_msg_out.angular.z = 0.0;
        gateway_publisher_->publish(twi_msg_out);
    }

    void joy_watchdog_callback()
    {
        // Calculate the time elapsed since the last message was received
        auto time_since_last_msg = (this->now() - last_joy_msg_received_time_).seconds();
        if (time_since_last_msg > joy_emergency_stop_timeout_)
        {
            if (is_joy_alive_)
            {
                emergency_stop_value_ = true;
                publish_zero_cmd_vel();
                RCLCPP_WARN(get_logger(), "/joy topic has stopped publishing for %.2f seconds. emergency_stop ACTIVATED.", time_since_last_msg);
                is_joy_alive_ = false;
            }
        }
        else
        {
            if (!is_joy_alive_)
            {
                is_joy_alive_ = true;
                if (emergency_stop_value_)
                    RCLCPP_WARN(get_logger(), "Receiving messages from /joy topic now, but emergency_stop is ACTIVATED.");
                else
                    RCLCPP_WARN(get_logger(), "Receiving messages from /joy topic now. Monitoring emergency_stop button.");
            }
        }
    }

    std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> twist_subscribers_;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> twist_publishers_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mux_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gateway_publisher_;
    rclcpp::TimerBase::SharedPtr joy_watchdog_timer_;

    std::vector<std::string> reverse_steer_correction_topics_;
    float wheelbase_, min_allowed_linear_velocity_, min_allowed_angular_velocity_;
    int emergency_stop_set_joy_button_index_, emergency_stop_clear_joy_button_index_;
    float joy_emergency_stop_timeout_;
    bool emergency_stop_value_;
    bool is_joy_alive_ = false;
    rclcpp::Time last_joy_msg_received_time_ = this->now();
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopGateway>());
    rclcpp::shutdown();
    return 0;
}
