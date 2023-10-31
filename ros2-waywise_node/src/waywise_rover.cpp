#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <QCoreApplication>
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

class WayWiseRover : public rclcpp::Node
{
public:
    WayWiseRover() : Node("waywise_rover") {
        // -- ROS --
        // get ROS parameters
        odom_frame_ = declare_parameter("odom_frame", odom_frame_);
        base_frame_ = declare_parameter("base_frame", base_frame_);

        // publishers
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
        // currently publishing periodically
        timer_ = this->create_wall_timer(mUpdateVehicleStatePeriod, std::bind(&WayWiseRover::timer_callback, this));

        // subscribers
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&WayWiseRover::twist_callback, this, _1));

        // -- WayWise --
        mCarState.reset(new CarState);

        mCarMovementController.reset(new CarMovementController(mCarState));
    }

private:
    void timer_callback() {
        auto thisTimeCalled = std::chrono::high_resolution_clock::now();
        static auto previousTimeCalled = thisTimeCalled - mUpdateVehicleStatePeriod;

        mCarState->simulationStep(std::chrono::duration_cast<std::chrono::milliseconds>(thisTimeCalled - previousTimeCalled).count());

        double x_ = mCarState->getPosition().getX();
        double y_ = mCarState->getPosition().getY();
        double yawRad_ = mCarState->getPosition().getYaw() * M_PI / 180.0;
        static double previousYawRad_ = yawRad_;

        // -- Prepare Odom
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = now();
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        // Position in the coordinate frame given by header.frame_id
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = sin(yawRad_ / 2.0);
        odom.pose.pose.orientation.w = cos(yawRad_ / 2.0);

        // TODO: position uncertainty?

        // Velocity in the coordinate frame given by child_frame_id
        odom.twist.twist.linear.x = mCarState->getSpeed();
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = (yawRad_ - previousYawRad_) / (std::chrono::duration_cast<std::chrono::milliseconds>(thisTimeCalled - previousTimeCalled).count() / 1000.0);

        // TODO: velocity uncertainty?

        // -- Prepare Transform
        auto tf = geometry_msgs::msg::TransformStamped();
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = base_frame_;
        tf.header.stamp = now();
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom.pose.pose.orientation;

        // -- Publish
        odom_pub_->publish(odom);
        tf_pub_->sendTransform(tf);

        previousTimeCalled = thisTimeCalled;
        previousYawRad_ = yawRad_;
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg) {
        // RCLCPP_INFO(this->get_logger(), "got Twist: linear %f, angular %f", twist_msg->linear.x, twist_msg->angular.z);
        mCarMovementController->setDesiredSpeed(twist_msg->linear.x);
        // NOTE / TODO: WayWise has a sign error here (curvature in wrong direction)
        mCarMovementController->setDesiredSteeringCurvature(-(twist_msg->angular.z / std::abs(twist_msg->linear.x))); // ω = |v|/r => 1/r =  ω/|v|
    } 

    // parameters
    std::string odom_frame_;
    std::string base_frame_;

    // publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    // WayWise
    std::chrono::milliseconds mUpdateVehicleStatePeriod = 100ms;
    QSharedPointer<CarState> mCarState;
    QSharedPointer<CarMovementController> mCarMovementController;
};

int main(int argc, char * argv[]) {
    QCoreApplication a(argc, argv);

    rclcpp::init(argc, argv);
    a.processEvents();

    auto waywiseNode = std::make_shared<WayWiseRover>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(waywiseNode);

    while (rclcpp::ok()) {
            exec.spin_some();
            a.processEvents();
    }

    exec.remove_node(waywiseNode);
    rclcpp::shutdown();

    return 0;
}
