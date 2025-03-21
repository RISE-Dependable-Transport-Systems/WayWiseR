#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class TwistAngularCorrection : public rclcpp::Node
{
public:
  TwistAngularCorrection()
  : Node("twist_angular_correction")
  {
    topic_names_ = this->declare_parameter("topic_names", std::vector<std::string>({"joy_vel"}));

    twist_subscribers_.resize(topic_names_.size());
    for (size_t i = 0; i < topic_names_.size(); ++i) {
      auto topic_name = topic_names_[i];
      twist_subscribers_[i] = this->create_subscription<geometry_msgs::msg::Twist>(
        topic_name, 10,
        [this, topic_name_ = topic_name](const geometry_msgs::msg::Twist::SharedPtr twist_msg) {
          subscriber_callback(twist_msg, topic_name_);
        });
      twist_publishers_map_[topic_name] =
        this->create_publisher<geometry_msgs::msg::Twist>(topic_name + "_corrected", 10);
    }
  }

private:
  void subscriber_callback(
    const geometry_msgs::msg::Twist::SharedPtr twist_msg,
    const std::string & topic_name)
  {
    // Correct angular velocity direction for reverse driving
    if (twist_msg->linear.x < 0) {
      twist_msg->angular.z = -twist_msg->angular.z;
    }
    twist_publishers_map_[topic_name]->publish(*twist_msg);
  }

  // Publishers and subscribers
  std::map<std::string,
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> twist_publishers_map_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> twist_subscribers_;

  // Parameters
  std::vector<std::string> topic_names_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistAngularCorrection>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
