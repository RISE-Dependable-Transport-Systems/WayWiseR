#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <QCoreApplication>
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/vehicles/controller/vescmotorcontroller.h"
#include "WayWise/sensors/imu/imuorientationupdater.h"
#include "WayWise/logger/logger.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class WayWiseRover : public rclcpp::Node
{
public:
    WayWiseRover() : Node("waywise_rover")
    {
        // -- ROS --
        // get ROS parameters
        odom_frame_ = declare_parameter("odom_frame", "odom");
        base_frame_ = declare_parameter("base_frame", "base_link");
        erpm_min_ = this->declare_parameter("erpm_min", 0.0);
        erpm_max_ = this->declare_parameter("erpm_max", 0.0);
        speed_to_erpm_gain_ = this->declare_parameter("speed_to_erpm_gain", 0.0);
        servo_offset_ = this->declare_parameter("servo_offset", 0.5);
        servo_min_ = this->declare_parameter("servo_min", 0.0);
        servo_max_ = this->declare_parameter("servo_max", 1.0);
        steering_angle_to_servo_gain_ = this->declare_parameter("steering_angle_to_servo_gain", 0.0);
        invert_servo_output_ = this->declare_parameter("invert_servo_output", false);
        wheelbase_ = this->declare_parameter("wheelbase", 0.33);
        min_turning_radius_ = this->declare_parameter("min_turning_radius", 0.67);
        odom_publish_rate_ = this->declare_parameter("odom_publish_rate", 30);
        publish_odom_to_baselink_tf_ = this->declare_parameter("publish_odom_to_baselink_tf", true);

        // publishers
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        if (publish_odom_to_baselink_tf_)
            tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
        // currently publishing periodically
        mUpdateVehicleStatePeriod = std::chrono::milliseconds((int)std::round(1000.0 / odom_publish_rate_));
        timer_ = this->create_wall_timer(mUpdateVehicleStatePeriod, std::bind(&WayWiseRover::timer_callback, this));

        // subscribers
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&WayWiseRover::twist_callback, this, _1));

        // -- WayWise --
        mCarState.reset(new CarState);
        
        // --- Lower-level control setup ---
        mCarMovementController.reset(new CarMovementController(mCarState));
        mCarMovementController->setSpeedToRPMFactor(speed_to_erpm_gain_);
        mCarState->setAxisDistance(wheelbase_);
        mCarState->setMaxSteeringAngle(atan(mCarState->getAxisDistance() / min_turning_radius_));

        // setup and connect VESC, simulate movements if unable to connect
        mVESCMotorController.reset(new VESCMotorController());
        foreach (const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts())
        {
            if (portInfo.description().toLower().replace("-", "").contains("chibios"))
            { // assumption: Serial device with ChibiOS in description is VESC
                mVESCMotorController->connectSerial(portInfo);
                RCLCPP_INFO(get_logger(), "VESCMotorController connected to: %s", portInfo.systemLocation().toLocal8Bit().data());
            }
        }
        if (mVESCMotorController->isSerialConnected())
        {
            mCarMovementController->setMotorController(mVESCMotorController);

            // VESC is a special case that can also control the servo
            const auto servoController = mVESCMotorController->getServoController();
            servoController->setInvertOutput(invert_servo_output_);
            servoController->setServoRange(servo_max_ - servo_min_);
            servoController->setServoCenter(servo_offset_);
            mCarMovementController->setServoController(servoController);
            is_in_simulation_mode_ = false;
        }
        else
        {
            is_in_simulation_mode_ = true;
            RCLCPP_INFO(get_logger(), "VESCMotorController is not connected. Waywise_rover is in simulation mode!");
        }
    }

private:
    void timer_callback()
    {
        auto thisTimeCalled = std::chrono::high_resolution_clock::now();
        static auto previousTimeCalled = thisTimeCalled - mUpdateVehicleStatePeriod;

        if (is_in_simulation_mode_)
        {
            mCarState->simulationStep(std::chrono::duration_cast<std::chrono::milliseconds>(thisTimeCalled - previousTimeCalled).count(), posType_);
        }

        double x_ = mCarState->getPosition(posType_).getX();
        double y_ = mCarState->getPosition(posType_).getY();
        double yawRad_ = mCarState->getPosition(posType_).getYaw() * M_PI / 180.0;
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

        if (publish_odom_to_baselink_tf_)
        {
            // -- Prepare Transform
            auto tf = geometry_msgs::msg::TransformStamped();
            tf.header.frame_id = odom_frame_;
            tf.child_frame_id = base_frame_;
            tf.header.stamp = now();
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation = odom.pose.pose.orientation;

            // -- Publish Transform
            tf_pub_->sendTransform(tf);
        }

        // -- Publish Odom
        odom_pub_->publish(odom);

        previousTimeCalled = thisTimeCalled;
        previousYawRad_ = yawRad_;
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
    {
        // RCLCPP_INFO(this->get_logger(), "got Twist: linear %f, angular %f", twist_msg->linear.x, twist_msg->angular.z);
        float clipped_linear_velocity = (speed_to_erpm_gain_ > 0) ? clip_min_max(speed_to_erpm_gain_ * twist_msg->linear.x, erpm_min_, erpm_max_) / speed_to_erpm_gain_ : 0.0;
        mCarMovementController->setDesiredSpeed(clipped_linear_velocity);

        // NOTE / TODO: WayWise has a sign error here (curvature in wrong direction)
        float desired_steering_curvature = (clipped_linear_velocity > 0) ? -(twist_msg->angular.z / twist_msg->linear.x) : std::numeric_limits<double>::infinity(); // ω = |v|/r => 1/r =  ω/|v|
        mCarMovementController->setDesiredSteeringCurvature(desired_steering_curvature);
    }

    float clip_min_max(float value, float min_value, float max_value) const
    {
        return std::min(
            std::max(
                value,
                (min_value + std::numeric_limits<float>::epsilon())),
            (max_value - std::numeric_limits<float>::epsilon()));
    }

    // parameters
    std::string odom_frame_;
    std::string base_frame_;
    float servo_min_, servo_max_, servo_offset_, steering_angle_to_servo_gain_;
    float erpm_min_, erpm_max_, speed_to_erpm_gain_;
    float wheelbase_, min_turning_radius_;
    bool invert_servo_output_, publish_odom_to_baselink_tf_;
    int odom_publish_rate_;
    PosType posType_ = PosType::fused;

    // internal variables
    bool is_in_simulation_mode_ = true;

    // publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    // WayWise
    std::chrono::milliseconds mUpdateVehicleStatePeriod;
    QSharedPointer<CarState> mCarState;
    QSharedPointer<CarMovementController> mCarMovementController;
    QSharedPointer<VESCMotorController> mVESCMotorController;
};

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    rclcpp::init(argc, argv);
    a.processEvents();

    auto waywiseNode = std::make_shared<WayWiseRover>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(waywiseNode);

    while (rclcpp::ok())
    {
        exec.spin_some();
        a.processEvents();
    }

    exec.remove_node(waywiseNode);
    rclcpp::shutdown();

    return 0;
}
