#include "waywise_car_autopilot.hpp"
#include "moc_waywise_car_autopilot.cpp"

using namespace std::placeholders;

void WaywiseCarAutopilot::initialize_node()
{
  setup_parameters();
  setup_publishers();
  setup_subscribers();
  setup_timers();
  setup_autopilot();
  provide_parameters_to_parameter_server();

  RCLCPP_INFO(get_logger(), "%s is initialized!", this->get_name());
}

void WaywiseCarAutopilot::setup_parameters()
{
  // ROS parameters
  odom_topic_ = this->declare_parameter("odom_topic", "/odom");
  speed_to_erpm_factor_ = this->declare_parameter("speed_to_erpm_factor", 0.0);
  length_ = this->declare_parameter("length", 0.33);
  width_ = this->declare_parameter("width", 0.33);
  wheelbase_ = this->declare_parameter("wheelbase", 0.33);
  min_turning_radius_ = this->declare_parameter("min_turning_radius", 0.67);
  autopilot_cmd_publish_rate_ = this->declare_parameter("autopilot_cmd_publish_rate", 30);
  waywise_control_tower_address_ = this->declare_parameter(
    "waywise_control_tower_address",
    "127.0.0.1");
  purepursuit_radius_ = this->declare_parameter("purepursuit_radius", 1.0);

  max_angular_velocity_ = this->declare_parameter("max_angular_velocity", 0.5);
  standstill_velocity_threshold_ = this->declare_parameter("standstill_velocity_threshold", 0.05);

  odom_frame_ = declare_parameter("odom_frame", "odom");
  base_frame_ = declare_parameter("base_frame", "base_link");

  enu_refernce_topic_ = this->declare_parameter("enu_refernce_topic", "/enu_refernce");
}

void WaywiseCarAutopilot::setup_publishers()
{
  // Publishers
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/waywise_vel", 10);
}

void WaywiseCarAutopilot::setup_subscribers()
{
  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&WaywiseCarAutopilot::odom_callback, this, _1));
  enu_refernce_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    enu_refernce_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&WaywiseCarAutopilot::enu_refernce_callback, this, _1)
  );
}

void WaywiseCarAutopilot::setup_timers()
{
  // Timers
  autopilot_timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds((int)std::round(1000.0 / autopilot_cmd_publish_rate_)),
    std::bind(&WaywiseCarAutopilot::autopilot_timer_callback, this));
}

void WaywiseCarAutopilot::setup_autopilot()
{
  mCarState.reset(new CarState);
  setup_autopilot(mCarState);
}

void WaywiseCarAutopilot::setup_autopilot(QSharedPointer<CarState> carState)
{
  // -- WayWise --
  mCarState = carState;
  mCarState->setLength(length_);
  mCarState->setWidth(width_);
  mCarState->setAxisDistance(wheelbase_);
  mCarState->setMaxSteeringAngle(atan(wheelbase_ / min_turning_radius_));

  // --- Movement control setup ---
  mCarMovementController.reset(new CarMovementController(mCarState));
  mCarMovementController->setSpeedToRPMFactor(speed_to_erpm_factor_);
  mFollowPoint.reset(new FollowPoint(mCarMovementController));

  // --- Positioning setup ---
  mGNSSReceiver.reset(new GNSSReceiver(mCarState));

  // Setup MAVLINK communication towards ControlTower
  mMavsdkVehicleServer.reset(
    new MavsdkVehicleServer(
      mCarState,
      QHostAddress(QString::fromStdString(waywise_control_tower_address_))));
  mMavsdkVehicleServer->setMovementController(mCarMovementController);
  mMavsdkVehicleServer->setGNSSReceiver(mGNSSReceiver);

  // --- Autopilot ---
  mWaypointFollower.reset(new PurepursuitWaypointFollower(mCarMovementController));
  mWaypointFollower->setPurePursuitRadius(purepursuit_radius_);
  mWaypointFollower->setRepeatRoute(false);
  mWaypointFollower->setAdaptivePurePursuitRadiusActive(true);
  mMavsdkVehicleServer->setWaypointFollower(mWaypointFollower);
}

void WaywiseCarAutopilot::autopilot_timer_callback()
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

void WaywiseCarAutopilot::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  auto current_pose = odom_msg->pose.pose;
  PosPoint currentPosition = mCarState->getPosition(waywise_posType_used_);
  double newYaw_deg_ = tf2::getYaw(current_pose.orientation) * (180.0 / M_PI);
  currentPosition.setX(current_pose.position.x);
  currentPosition.setY(current_pose.position.y);
  currentPosition.setHeight(current_pose.position.z);
  currentPosition.setYaw(newYaw_deg_);
  currentPosition.setTime(
    QTime::currentTime().addSecs(
      -QDateTime::currentDateTime().offsetFromUtc()));
  mCarState->setPosition(currentPosition);

  if (waywise_posType_used_ != PosType::fused) {
    currentPosition.setType(PosType::fused);   // the 'fused' position type is communicated to topics
                                               // & potentially MAVLINK
    mCarState->setPosition(currentPosition);
  }
}

void WaywiseCarAutopilot::enu_refernce_callback(
  const geometry_msgs::msg::Vector3::SharedPtr enuRef_msg)
{
  llh_t mEnuReference{enuRef_msg->x, enuRef_msg->y, enuRef_msg->z};
  mGNSSReceiver->setEnuRef(mEnuReference);

  RCLCPP_INFO(
    this->get_logger(),
    "Updated enu reference to: latitude=%.2f, longitude=%.2f, height=%.2f",
    mEnuReference.latitude, mEnuReference.longitude, mEnuReference.height);
}

void WaywiseCarAutopilot::provide_parameters_to_parameter_server()
{
  mCarState->provideParametersToParameterServer();
  mWaypointFollower->provideParametersToParameterServer();
  mFollowPoint->provideParametersToParameterServer();
  mMavsdkVehicleServer->provideParametersToParameterServer();
}
