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

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class WayWiseRover : public rclcpp::Node
{
public:
    WayWiseRover() : Node("waywise_rover") {
        // get ROS parameters
        odom_frame_ = declare_parameter("odom_frame", odom_frame_);
        base_frame_ = declare_parameter("base_frame", base_frame_);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));

        timer_ = this->create_wall_timer(mUpdateVehicleStatePeriod, std::bind(&WayWiseRover::timer_callback, this));

        // init WayWise
        mCarState.reset(new CarState);

        // --- Lower-level control setup ---
        mCarMovementController.reset(new CarMovementController(mCarState));

        mCarMovementController->setDesiredSpeed(3);
        mCarMovementController->setDesiredSteering(0.5);

    }

private:
    void timer_callback() {
//        RCLCPP_INFO(this->get_logger(), "Alive!");
        mCarState->simulationStep(mUpdateVehicleStatePeriod.count());

        double x_ = mCarState->getPosition().getX();
        double y_ = mCarState->getPosition().getY();
        double yawRad_ = mCarState->getPosition().getYaw() * M_PI / 180.0;
        //qDebug() << x_ << y_ << yaw_;

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
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

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
    }

    // Parameters
    std::string odom_frame_;
    std::string base_frame_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

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
