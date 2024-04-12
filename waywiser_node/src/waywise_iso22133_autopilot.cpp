#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <QDebug>

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
#include "iso_msgs/msg/cartesian_trajectory.hpp"
#include "iso_msgs/msg/abort.hpp"
#include "iso_msgs/msg/start.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

class WayWiseISO22133AutoPilot : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  WayWiseISO22133AutoPilot()
  : QObject(), Node("waywise_iso22133_autopilot")
  {
    // -- ROS --
    // get ROS parameters
    speed_to_rpm_factor_ = this->declare_parameter("speed_to_rpm_factor", 0.0);
    wheelbase_ = this->declare_parameter("wheelbase", 0.33);
    min_turning_radius_ = this->declare_parameter("min_turning_radius", 0.67);
    autopilot_cmd_publish_rate_ = this->declare_parameter("autopilot_cmd_publish_rate", 30);

    waywise_control_tower_address_ = this->declare_parameter(
      "waywise_control_tower_address",
      "127.0.0.1");

    auto abort_topic = iso_msgs::msg::Abort::TOPIC_NAME;
    abort_sub_ = this->create_subscription<iso_msgs::msg::Abort>(
      abort_topic, 10, std::bind(&WayWiseISO22133AutoPilot::abort_callback, this, _1));
    auto start_topic = iso_msgs::msg::Start::TOPIC_NAME;
    start_sub_ = this->create_subscription<iso_msgs::msg::Start>(
      start_topic, 10, std::bind(&WayWiseISO22133AutoPilot::start_callback, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&WayWiseISO22133AutoPilot::odom_callback, this, _1));
    traj_sub_ = this->create_subscription<iso_msgs::msg::CartesianTrajectory>(
      "/trajectory", 10, std::bind(&WayWiseISO22133AutoPilot::traj_callback, this, _1));
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/waywise_vel", 10);
    autopilot_timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds((int)std::round(1000.0 / autopilot_cmd_publish_rate_)),
      std::bind(&WayWiseISO22133AutoPilot::autopilot_timer_callback, this));

    // -- WayWise --
    mCarState.reset(new CarState);

    // --- Movement control setup ---
    mCarMovementController.reset(new CarMovementController(mCarState));
    mCarMovementController->setSpeedToRPMFactor(speed_to_rpm_factor_);
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

  void traj_callback(const iso_msgs::msg::CartesianTrajectory::SharedPtr traj_msg)
  {
    //qDebug() << "Got traj message";
    QList<PosPoint> route;
    PosPoint newTrajPoint;

    if (!mWaypointFollower.isNull()) {
      mWaypointFollower->clearRoute();
      for (const auto &point : traj_msg->points)
      {
        // TODO: Populate the route correctly, what do we need to populate?
        newTrajPoint.setX(point.pose.position.x);
        newTrajPoint.setY(point.pose.position.y);
        newTrajPoint.setHeight(point.pose.position.z);
        auto speed = sqrt(pow(point.twist.linear.x, 2) + pow(point.twist.linear.y, 2));
        newTrajPoint.setSpeed(speed);

        route.append(newTrajPoint);
      }
      mWaypointFollower->addRoute(route);
    } else {
      qDebug() << "WaywiseISO22133autopilot: got new mission but no "
                        "WaypointFollower is set to receive it.";
    }
  }

  void abort_callback(const iso_msgs::msg::Abort::SharedPtr abort_msg)
  {
    qDebug() << "WaywiseISO22133autopilot: Abort received!";
    if (mWaypointFollower) {
        mWaypointFollower->stop();
    }

    if (mCarMovementController) {
        mCarMovementController->setDesiredSteering(0.0);
        mCarMovementController->setDesiredSpeed(0.0);
    }
    mWaypointFollower->resetState();
  }

  void start_callback(const iso_msgs::msg::Start::SharedPtr start_msg)
  {
    qDebug() << "Object Starting";
    mCarState->setFlightMode(VehicleState::FlightMode::Mission);
    mWaypointFollower->startFollowingRoute(false);
  }

  // ROS parameters
  float speed_to_rpm_factor_;

  float wheelbase_, min_turning_radius_;

  int autopilot_cmd_publish_rate_;

  std::string waywise_control_tower_address_;

  // internal variables
  PosType waywise_posType_used_ = PosType::simulated;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  rclcpp::TimerBase::SharedPtr autopilot_timer_;

  // subscribers
  rclcpp::Subscription<iso_msgs::msg::Abort>::SharedPtr abort_sub_;
  rclcpp::Subscription<iso_msgs::msg::Start>::SharedPtr start_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<iso_msgs::msg::CartesianTrajectory>::SharedPtr traj_sub_;

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

  auto waywiseNode = std::make_shared<WayWiseISO22133AutoPilot>();
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

#include "waywise_iso22133_autopilot.moc"
