#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "WayWise/autopilot/purepursuitwaypointfollower.h"
#include "WayWise/autopilot/waypointfollower.h"
#include "WayWise/communication/mavsdkvehicleserver.h"
#include "WayWise/communication/parameterserver.h"
#include "WayWise/logger/logger.h"
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include <QCoreApplication>
#include <QObject>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

class WayWiseAutoPilot : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  WayWiseAutoPilot()
  : QObject(), Node("waywise_autopilot")
  {
    // -- ROS --
    // get ROS parameters
    speed_to_erpm_factor_ = this->declare_parameter("speed_to_erpm_factor", 0.0);
    wheelbase_ = this->declare_parameter("wheelbase", 0.33);
    min_turning_radius_ = this->declare_parameter("min_turning_radius", 0.67);
    autopilot_cmd_publish_rate_ = this->declare_parameter("autopilot_cmd_publish_rate", 30);

    waywise_control_tower_address_ = this->declare_parameter(
      "waywise_control_tower_address",
      "127.0.0.1");

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&WayWiseAutoPilot::odom_callback, this, _1));
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/waywise_vel", 10);
    autopilot_timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds((int)std::round(1000.0 / autopilot_cmd_publish_rate_)),
      std::bind(&WayWiseAutoPilot::autopilot_timer_callback, this));

    // -- WayWise --
    mCarState.reset(new CarState);

    // --- Movement control setup ---
    mCarMovementController.reset(new CarMovementController(mCarState));
    mCarMovementController->setSpeedToRPMFactor(speed_to_erpm_factor_);
    mCarState->setAxisDistance(wheelbase_);
    mCarState->setMaxSteeringAngle(atan(mCarState->getAxisDistance() / min_turning_radius_));

    // Setup MAVLINK communication towards ControlTower
    mMavsdkVehicleServer.reset(
      new MavsdkVehicleServer(
        mCarState,
        QHostAddress(QString::fromStdString(waywise_control_tower_address_))));
    mMavsdkVehicleServer->setMovementController(mCarMovementController);

    // --- Autopilot ---
    mWaypointFollower.reset(new PurepursuitWaypointFollower(mCarMovementController));
    mWaypointFollower->setPurePursuitRadius(1.0);
    mWaypointFollower->setRepeatRoute(false);
    mWaypointFollower->setAdaptivePurePursuitRadiusActive(true);
    mMavsdkVehicleServer->setWaypointFollower(mWaypointFollower);
  }

private:
  void autopilot_timer_callback()
  {
    double mDesiredSpeed = mCarMovementController->getDesiredSpeed();        // [m/s]
    double mDesiredSteering = mCarMovementController->getDesiredSteering();  // [-1.0:1.0]
    double steeringAngle_rad = mDesiredSteering * mCarState->getMaxSteeringAngle();
    if (abs(steeringAngle_rad) > mCarState->getMaxSteeringAngle()) {
      steeringAngle_rad = mCarState->getMaxSteeringAngle() * ((steeringAngle_rad > 0) ? 1.0 : -1.0);
    }
    double mDesiredSteeringCurvature = tan(steeringAngle_rad) / mCarState->getAxisDistance();
    double mDesiredAngularVelocity = -mDesiredSpeed * mDesiredSteeringCurvature;  // ω = v/r

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = mDesiredSpeed;
    twist_msg.angular.z = mDesiredAngularVelocity;

    twist_pub_->publish(twist_msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    PosPoint currentPosition = mCarState->getPosition(waywise_posType_used_);
    double newYaw_deg_ = tf2::getYaw(odom_msg->pose.pose.orientation) * (180.0 / M_PI);
    currentPosition.setX(odom_msg->pose.pose.position.x);
    currentPosition.setY(odom_msg->pose.pose.position.y);
    currentPosition.setYaw(newYaw_deg_);
    currentPosition.setTime(
      QTime::currentTime().addSecs(
        -QDateTime::currentDateTime().offsetFromUtc()));
    mCarState->setPosition(currentPosition);
    currentPosition.setType(PosType::fused);  // the 'fused' position type is communicated to topics
                                              // & potentially MAVLINK
    mCarState->setPosition(currentPosition);
  }

  // ROS parameters
  float speed_to_erpm_factor_;

  float wheelbase_, min_turning_radius_;

  int autopilot_cmd_publish_rate_;

  std::string waywise_control_tower_address_;

  // internal variables
  PosType waywise_posType_used_ = PosType::simulated;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  rclcpp::TimerBase::SharedPtr autopilot_timer_;

  // subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // WayWise
  QSharedPointer<CarState> mCarState;
  QSharedPointer<CarMovementController> mCarMovementController;
  QSharedPointer<PurepursuitWaypointFollower> mWaypointFollower;
  QSharedPointer<MavsdkVehicleServer> mMavsdkVehicleServer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QCoreApplication a(argc, argv);

  a.processEvents();

  auto waywiseNode = std::make_shared<WayWiseAutoPilot>();
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

#include "waywise_autopilot.moc"
