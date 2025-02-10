#include "waywise_car_autopilot.hpp"
#include "moc_waywise_car_autopilot.cpp"

using namespace std::placeholders;

void WaywiseCarAutopilot::initialize_node()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
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
  update_world_position_with_odom_ = this->declare_parameter(
    "update_world_position_with_odom",
    false);
  update_world_position_with_tf_ = this->declare_parameter(
    "update_world_position_with_tf",
    false);

  max_angular_velocity_ = this->declare_parameter("max_angular_velocity", 0.5);
  standstill_velocity_threshold_ = this->declare_parameter("standstill_velocity_threshold", 0.05);
  publish_joint_states_ = this->declare_parameter("publish_joint_states", false);

  odom_frame_ = declare_parameter("odom_frame", "odom");
  base_frame_ = declare_parameter("base_frame", "base_link");
  world_frame_ = declare_parameter("world_frame", "map");
  rear_axle_frame_ = this->declare_parameter("rear_axle_frame", base_frame_);
  center_frame_ = this->declare_parameter("center_frame", "");
  rear_end_frame_ = this->declare_parameter("rear_end_frame", "");

  enu_refernce_topic_ = this->declare_parameter("enu_refernce_topic", "/enu_refernce");
  vehicle_pose_topic_ = declare_parameter("vehicle_pose_topic", "/car_pose");

  urdf_file_ = this->declare_parameter("urdf_file", "");
  front_steering_joint_names_ = declare_parameter<std::vector<std::string>>(
    "front_steering_joint_names",
    std::vector<std::string>{"left_front_wheel_steering_joint", "right_front_wheel_steering_joint"}
  );
  front_wheel_joint_names_ = declare_parameter<std::vector<std::string>>(
    "front_wheel_joint_names",
    std::vector<std::string>{"left_front_wheel_joint", "right_front_wheel_joint"}
  );

  rear_wheel_joint_names_ = declare_parameter<std::vector<std::string>>(
    "rear_wheel_joint_names",
    std::vector<std::string>{"left_rear_wheel_joint", "right_rear_wheel_joint"}
  );
}

void WaywiseCarAutopilot::setup_publishers()
{
  // Publishers
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/waywise_vel", 10);
  vehicle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(vehicle_pose_topic_, 10);

  if (publish_joint_states_) {
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("waywise_joint_states", 10);
  }
}

void WaywiseCarAutopilot::setup_subscribers()
{
  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&WaywiseCarAutopilot::odom_callback, this, _1));
  enu_refernce_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    enu_refernce_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&WaywiseCarAutopilot::enu_reference_callback, this, _1)
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

  if (loadURDFFile()) {
    Eigen::Vector3d offset = getLinkPosition(urdfModel, center_frame_) -
      getLinkPosition(urdfModel, rear_axle_frame_);
    mCarState->setRearAxleToCenterOffset(xyz_t{offset.x(), offset.y(), offset.z()});

    offset = getLinkPosition(urdfModel, rear_end_frame_) -
      getLinkPosition(urdfModel, rear_axle_frame_);
    mCarState->setRearAxleToRearEndOffset(xyz_t{offset.x(), offset.y(), offset.z()});
  }

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
  if (update_world_position_with_tf_) {
    try {
      geometry_msgs::msg::TransformStamped map_to_base_link_msg_tfs = tf_buffer_->lookupTransform(
        world_frame_, rear_axle_frame_, tf2::TimePointZero);

      geometry_msgs::msg::Pose world_pose;
      world_pose.position.x = map_to_base_link_msg_tfs.transform.translation.x;
      world_pose.position.y = map_to_base_link_msg_tfs.transform.translation.y;
      world_pose.position.z = map_to_base_link_msg_tfs.transform.translation.z;
      world_pose.orientation = map_to_base_link_msg_tfs.transform.rotation;
      update_world_positon(world_pose);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Failed to update world position: %s", ex.what());
    }
  }

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
  PosPoint currentPosition = mCarState->getPosition(PosType::odom);
  currentPosition.setX(current_pose.position.x);
  currentPosition.setY(current_pose.position.y);
  currentPosition.setHeight(current_pose.position.z);
  currentPosition.updateWithOffsetAndYawRotation(
    -(mCarState->getRearAxleToCenterOffset()), tf2::getYaw(current_pose.orientation));
  currentPosition.setTime(
    QTime::currentTime().addSecs(
      -QDateTime::currentDateTime().offsetFromUtc()));
  mCarState->setPosition(currentPosition);

  if (update_world_position_with_odom_) {
    currentPosition.setType(PosType::fused);   // the 'fused' position type is communicated to topics
                                               // & potentially MAVLINK
    mCarState->setPosition(currentPosition);
  }

  geometry_msgs::msg::Twist current_twist = odom_msg->twist.twist;
  mCarState->setVelocity(
    xyz_t{current_twist.linear.x, current_twist.linear.y,
      current_twist.linear.z});

  if (fabs(mCarState->getSpeed()) >= standstill_velocity_threshold_) {
    // NOTE / TODO: WayWise has a sign error here (curvature in wrong direction)
    float steering_curvature = -(current_twist.angular.z / mCarState->getSpeed());    // ω = v/r => 1/r = ω/v
    mCarState->setSteering(
      atan(mCarState->getAxisDistance() * steering_curvature) / mCarState->getMaxSteeringAngle()
    );
  } else {
    mCarState->setSteering(current_twist.angular.z / max_angular_velocity_);
  }

  // -- Publish Joint states
  if (publish_joint_states_) {
    static auto previousTimeCalled = this->get_clock()->now();
    auto thisTimeCalled = this->get_clock()->now();
    double timePassed_ms = (thisTimeCalled.nanoseconds() - previousTimeCalled.nanoseconds()) / 1e6;
    previousTimeCalled = thisTimeCalled;
    if (timePassed_ms > 0) {
      sensor_msgs::msg::JointState joint_state_msg;
      update_joint_states_msg(joint_state_msg, timePassed_ms);
      joint_state_pub_->publish(joint_state_msg);
    }
  }
}

void WaywiseCarAutopilot::enu_reference_callback(
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

bool WaywiseCarAutopilot::loadURDFFile()
{
  bool isLoaded = false;
  if (!urdf_file_.empty()) {
    if (urdf_file_.find("xml version=") != std::string::npos) {
      isLoaded = urdfModel.initString(urdf_file_);
    } else {
      isLoaded = urdfModel.initFile(urdf_file_);
    }
  }
  if (!isLoaded) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF file!");
  }
  return isLoaded;
}

void WaywiseCarAutopilot::update_world_positon(geometry_msgs::msg::Pose world_pose)
{
  PosPoint currentPosition = mCarState->getPosition(PosType::fused);
  currentPosition.setX(world_pose.position.x);
  currentPosition.setY(world_pose.position.y);
  currentPosition.setHeight(world_pose.position.z);
  currentPosition.setYaw(tf2::getYaw(world_pose.orientation) * (180.0 / M_PI));
  currentPosition.setTime(
    QTime::currentTime().addSecs(
      -QDateTime::currentDateTime().offsetFromUtc()));
  mCarState->setPosition(currentPosition);

  geometry_msgs::msg::PoseStamped world_pose_stamped;
  world_pose_stamped.pose = world_pose;
  world_pose_stamped.header.frame_id = world_frame_;
  world_pose_stamped.header.stamp = this->get_clock()->now();
  vehicle_pose_pub_->publish(world_pose_stamped);
}

Eigen::Vector3d WaywiseCarAutopilot::getLinkPosition(
  const urdf::Model & urdfModel,
  const std::string & link_name) const
{
  Eigen::Vector3d position(0, 0, 0);
  if (!link_name.empty() && link_name != base_frame_) {
    const urdf::LinkConstSharedPtr link = urdfModel.getLink(link_name);
    if (!link) {
      RCLCPP_ERROR(this->get_logger(), "Link %s not found", link_name.c_str());
      return Eigen::Vector3d::Zero();
    }

    const urdf::Pose & pose = link->parent_joint->parent_to_joint_origin_transform;
    position.x() = pose.position.x;
    position.y() = pose.position.y;
    position.z() = pose.position.z;
  }
  return position;
}

double WaywiseCarAutopilot::update_joint_states_msg(
  sensor_msgs::msg::JointState & joint_state_msg,
  double timePassed_ms)
{
  static double wheel_position = 0.0;

  joint_state_msg.header.stamp = this->now();

  // Calculate wheel speed and steering angle
  double wheel_rad_per_sec = (30.0 / M_PI) * mCarState->getSpeed() *
    mCarMovementController->getSpeedToRPMFactor();
  double steeringAngle_rad = -mCarState->getSteering() * mCarState->getMaxSteeringAngle();
  if (abs(steeringAngle_rad) > mCarState->getMaxSteeringAngle()) {
    steeringAngle_rad = mCarState->getMaxSteeringAngle() * ((steeringAngle_rad > 0) ? 1.0 : -1.0);
  }
  wheel_position += wheel_rad_per_sec * timePassed_ms * 1000.0;
  wheel_position = fmod(wheel_position, 2.0 * M_PI);
  if (wheel_position < 0) {
    wheel_position += 2.0 * M_PI;
  }

  // Clear previous data (if any)
  joint_state_msg.name.clear();
  joint_state_msg.position.clear();
  joint_state_msg.velocity.clear();
  joint_state_msg.effort.clear();

  // Add wheel joint names and states
  for (const auto & steering_joint_name : front_steering_joint_names_) {
    joint_state_msg.name.push_back(steering_joint_name);
    joint_state_msg.position.push_back(steeringAngle_rad); // Steering angle in radians
    joint_state_msg.velocity.push_back(0.0); // Velocity in rad/s
    joint_state_msg.effort.push_back(0.0); // Effort (not used here)
  }

  for (const auto & wheel_joint_name : front_wheel_joint_names_) {
    joint_state_msg.name.push_back(wheel_joint_name);
    joint_state_msg.position.push_back(wheel_position); // Wheel position in radians
    joint_state_msg.velocity.push_back(wheel_rad_per_sec); // Wheel velocity in rad/s
    joint_state_msg.effort.push_back(0.0); // Effort (not used here)
  }

  for (const auto & wheel_joint_name : rear_wheel_joint_names_) {
    joint_state_msg.name.push_back(wheel_joint_name);
    joint_state_msg.position.push_back(wheel_position); // Wheel position in radians
    joint_state_msg.velocity.push_back(wheel_rad_per_sec); // Wheel velocity in rad/s
    joint_state_msg.effort.push_back(0.0); // Effort (not used here)
  }

  return wheel_position;
}
