#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class WayWiseRover : public rclcpp::Node
{
public:
  WayWiseRover() : Node("waywise_rover"), count_(0) {
    string_pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));

    timer_ = this->create_wall_timer(500ms, std::bind(&WayWiseRover::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, WayWise! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    auto odom = nav_msgs::msg::Odometry();
    auto tf = geometry_msgs::msg::TransformStamped();

    string_pub_->publish(message);
    odom_pub_->publish(odom);
    tf_pub_->sendTransform(tf);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WayWiseRover>());
  rclcpp::shutdown();
  return 0;
}
