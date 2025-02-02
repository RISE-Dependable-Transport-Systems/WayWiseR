#include "waywise_car.hpp"
#include "moc_waywise_car.cpp"

using namespace std::placeholders;

void WayWiseCar::initialize_node()
{
  setup_parameters();
  setup_publishers();
  setup_subscribers();
  setup_timers();
  setup_hardware();

  RCLCPP_INFO(get_logger(), "%s is initialized!", this->get_name());
}

void WayWiseCar::setup_parameters()
{
  // ROS parameters
  odom_topic_ = this->declare_parameter("odom_topic", "/odom");

  erpm_min_ = this->declare_parameter("erpm_min", 0.0);
  erpm_max_ = this->declare_parameter("erpm_max", 0.0);
  speed_to_erpm_factor_ = this->declare_parameter("speed_to_erpm_factor", 0.0);

  invert_servo_output_ = this->declare_parameter("invert_servo_output", false);
  servo_offset_ = this->declare_parameter("servo_offset", 0.5);
  servo_min_ = this->declare_parameter("servo_min", 0.0);
  servo_max_ = this->declare_parameter("servo_max", 1.0);

  length_ = this->declare_parameter("length", 0.33);
  width_ = this->declare_parameter("width", 0.33);
  wheelbase_ = this->declare_parameter("wheelbase", 0.33);
  min_turning_radius_ = this->declare_parameter("min_turning_radius", 0.67);
  odom_and_tf_publish_rate_ = this->declare_parameter("odom_and_tf_publish_rate", 30);
  publish_odom_to_baselink_tf_ = this->declare_parameter("publish_odom_to_baselink_tf", true);
  publish_world_to_odom_tf_ = this->declare_parameter("publish_world_to_odom_tf", false);
  update_world_position_with_odom_ = this->declare_parameter(
    "update_world_position_with_odom",
    false);
  enable_imu_for_odom_ = this->declare_parameter("enable_imu_for_odom", true);
  imu_for_position_fusion_ = declare_parameter("imu_for_position_fusion", "");

  max_angular_velocity_ = this->declare_parameter("max_angular_velocity", 0.5);
  standstill_velocity_threshold_ = this->declare_parameter("standstill_velocity_threshold", 0.05);

  odom_frame_ = declare_parameter("odom_frame", "odom");
  base_frame_ = declare_parameter("base_frame", "base_link");
  world_frame_ = declare_parameter("world_frame", "map");

  nav_sat_fix_topic_ = this->declare_parameter("nav_sat_fix_topic", "/gnss_fix");
  enu_refernce_topic_ = this->declare_parameter("enu_refernce_topic", "/enu_refernce");
}

void WayWiseCar::setup_publishers()
{
  // Publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  if (publish_odom_to_baselink_tf_ || publish_world_to_odom_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  nav_sat_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(nav_sat_fix_topic_, 10);
  enu_refernce_pub_ =
    create_publisher<geometry_msgs::msg::Vector3>(
    enu_refernce_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
}

void WayWiseCar::setup_subscribers()
{
  // Subscribers
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&WayWiseCar::twist_callback, this, _1));
}

void WayWiseCar::setup_timers()
{
  // Timers
  mUpdateVehicleStatePeriod =
    std::chrono::milliseconds((int)std::round(1000.0 / odom_and_tf_publish_rate_));

  simulation_timer_ =
    this->create_wall_timer(
    mUpdateVehicleStatePeriod,
    std::bind(&WayWiseCar::simulation_timer_callback, this));
}

void WayWiseCar::setup_hardware()
{
  mCarState.reset(new CarState);
  setup_hardware(mCarState);
}

void WayWiseCar::setup_hardware(QSharedPointer<CarState> carState)
{
  // -- WayWise --
  mCarState = carState;
  mCarState->setLength(length_);
  mCarState->setWidth(width_);
  mCarState->setAxisDistance(wheelbase_);
  mCarState->setMaxSteeringAngle(atan(wheelbase_ / min_turning_radius_));

  // --- Lower-level control setup ---
  mCarMovementController.reset(new CarMovementController(mCarState));
  mCarMovementController->setSpeedToRPMFactor(speed_to_erpm_factor_);

  // setup and connect VESC, simulate movements if unable to connect
  mVESCMotorController.reset(new VESCMotorController());
  foreach(const QSerialPortInfo & portInfo, QSerialPortInfo::availablePorts())
  {
    if (portInfo.description().toLower().replace("-", "").contains("chibios")) {   // assumption: Serial device with ChibiOS in
                                                                                   // description is VESC
      mVESCMotorController->connectSerial(portInfo);
      RCLCPP_INFO(
        get_logger(), "VESCMotorController connected to: %s",
        portInfo.systemLocation().toLocal8Bit().data());
    }
  }

  if (mVESCMotorController->isSerialConnected()) {
    mCarMovementController->setMotorController(mVESCMotorController);
    mVESCMotorController->setPollValuesPeriod(mUpdateVehicleStatePeriod.count());

    // VESC is a special case that can also control the servo
    const auto servoController = mVESCMotorController->getServoController();
    servoController->setInvertOutput(invert_servo_output_);
    servoController->setServoRange(servo_max_ - servo_min_);
    servoController->setServoCenter(servo_offset_);
    mCarMovementController->setServoController(servoController);

    // publish on motorcontroller callback when connected
    QObject::connect(
      mCarMovementController.get(), &CarMovementController::updatedOdomPositionAndYaw, this,
      &WayWiseCar::updated_waywise_odomPos_callback);

    simulation_timer_->cancel();
  } else {
    // publish periodically with timer when no motorcontroller connected
    // (simulation)
    RCLCPP_INFO(
      get_logger(),
      "VESCMotorController is not connected. "
      "waywise_car is in simulation mode!");
  }

  // --- Positioning setup ---
  // Position Fuser
  positionFuser = new SDVPVehiclePositionFuser(this);
  // GNSS (with fused IMU when using u-blox F9R)
  mUbloxRover.reset(new UbloxRover(mCarState));
  foreach(const QSerialPortInfo & portInfo, QSerialPortInfo::availablePorts()) {
    // qDebug()<<portInfo.manufacturer();
    if (portInfo.manufacturer().toLower().replace("-", "").contains("ublox")) {
      if (mUbloxRover->connectSerial(portInfo)) {
        qDebug() << "UbloxRover connected to:" << portInfo.systemLocation();

        //mUbloxRover->setIMUOrientationOffset(0.0, 0.0, 0.0);
      }
    }
  }
  QObject::connect(
    mUbloxRover.get(), &UbloxRover::txNavPvt, this, &WayWiseCar::publish_ublox_nav_sat_fix);
  QObject::connect(
    mUbloxRover.get(), &UbloxRover::updatedEnuReference, this,
    &WayWiseCar::publish_enu_refernce);

  rtcmClient = new RtcmClient(this);
  QObject::connect(
    mUbloxRover.get(), &UbloxRover::updatedGNSSPositionAndYaw, positionFuser,
    &SDVPVehiclePositionFuser::correctPositionAndYawGNSS);

  // -- NTRIP/TCP client setup for feeding RTCM data into GNSS receiver
  QObject::connect(
    mUbloxRover.get(), &UbloxRover::gotNmeaGga, rtcmClient, &RtcmClient::forwardNmeaGgaToServer);
  QObject::connect(
    rtcmClient, &RtcmClient::rtcmData,
    mUbloxRover.get(), &UbloxRover::writeRtcmToUblox);
  QObject::connect(
    rtcmClient, &RtcmClient::baseStationPosition,
    mUbloxRover.get(), &UbloxRover::setEnuRef);
  if (rtcmClient->connectWithInfoFromFile("./rtcmServerInfo.txt")) {
    qDebug() << "RtcmClient: connected to" << QString(
      rtcmClient->getCurrentHost() + ":" + QString::number(rtcmClient->getCurrentPort()));
  } else {
    qDebug() << "RtcmClient: not connected";
  }


  // IMU
  if (!imu_for_position_fusion_.empty()) {
    if (imu_for_position_fusion_ == "vesc") {
      if (mVESCMotorController->isSerialConnected()) {
        mIMUOrientationUpdater = mVESCMotorController->getIMUOrientationUpdater(mCarState);
        QObject::connect(
          mIMUOrientationUpdater.get(), &IMUOrientationUpdater::updatedIMUOrientation, positionFuser,
          &SDVPVehiclePositionFuser::correctPositionAndYawIMU);
        RCLCPP_INFO(this->get_logger(), "Using vesc IMU for position fusion.");
      } else {
        RCLCPP_INFO(
          get_logger(),
          "vesc IMU is configured for position fusion but VESCMotorController is not connected.");
      }
    } else if (imu_for_position_fusion_ == "bno055") {
      mIMUOrientationUpdater.reset(new BNO055OrientationUpdater(mCarState, "/dev/i2c-1"));
      QObject::connect(
        mIMUOrientationUpdater.get(), &IMUOrientationUpdater::updatedIMUOrientation, positionFuser,
        &SDVPVehiclePositionFuser::correctPositionAndYawIMU);
      RCLCPP_INFO(this->get_logger(), "Using bno055 IMU for position fusion.");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Unknown IMU variant is requested for position fusion!");
    }
  }

  // Odometry
  QObject::connect(
    mCarMovementController.get(), &CarMovementController::updatedOdomPositionAndYaw, positionFuser,
    &SDVPVehiclePositionFuser::correctPositionAndYawOdom);
  QObject::connect(
    mCarMovementController.get(), &CarMovementController::updatedOdomPositionAndYaw, this,
    &WayWiseCar::updated_waywise_odomPos_callback);

  // Watchdog that warns when EventLoop is slowed down
  watchdog = new SimpleWatchdog(this);
}

void WayWiseCar::simulation_timer_callback()
{
  auto thisTimeCalled = std::chrono::high_resolution_clock::now();
  static auto previousTimeCalled = thisTimeCalled - mUpdateVehicleStatePeriod;
  double timePassed_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(
    thisTimeCalled -
    previousTimeCalled).count();

  mCarState->simulationStep(timePassed_ms, PosType::simulated);
  PosPoint currentPosition = mCarState->getPosition(PosType::simulated);
  currentPosition.setType(PosType::odom)
  mCarState->setPosition(currentPosition)
  if (publish_world_to_odom_tf_) {
    currentPosition.setType(PosType::fused)
    mCarState->setPosition(currentPosition)
  }

  publish_odom_and_tfs(timePassed_ms);

  previousTimeCalled = thisTimeCalled;
}

void WayWiseCar::updated_waywise_odomPos_callback(
  QSharedPointer<VehicleState> vehicleState,
  double distanceDriven)
{
  // suppress 'unused' warnings
  (void)vehicleState;
  (void)distanceDriven;

  auto thisTimeCalled = std::chrono::high_resolution_clock::now();
  static auto previousTimeCalled = thisTimeCalled - mUpdateVehicleStatePeriod;
  double timePassed_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(
    thisTimeCalled -
    previousTimeCalled).count();

  publish_odom_and_tfs(timePassed_ms);

  previousTimeCalled = thisTimeCalled;
}

void WayWiseCar::publish_odom_and_tfs(double timePassed_ms)
{
  PosPoint currentPosition = mCarState->getPosition(PosType::odom);
  if (update_world_position_with_odom_) {
    currentPosition.setType(PosType::fused);   // the 'fused' position type is communicated to topics
                                               // & potentially MAVLINK
    mCarState->setPosition(currentPosition);
  }

  double x_ = currentPosition.getX();
  double y_ = currentPosition.getY();
  double yawRad_ = currentPosition.getYaw() * M_PI / 180.0;
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
  odom.twist.twist.angular.z = (yawRad_ - previousYawRad_) / (timePassed_ms / 1000.0);

  // TODO: velocity uncertainty?

  if (publish_odom_to_baselink_tf_ || publish_world_to_odom_tf_) {
    // -- Prepare Transform
    auto odom_to_base_link_msg_tf = geometry_msgs::msg::Transform();
    odom_to_base_link_msg_tf.translation.x = x_;
    odom_to_base_link_msg_tf.translation.y = y_;
    odom_to_base_link_msg_tf.translation.z = 0.0;
    odom_to_base_link_msg_tf.rotation = odom.pose.pose.orientation;

    if (publish_odom_to_baselink_tf_) {
      auto odom_to_base_link_msg_tfs = geometry_msgs::msg::TransformStamped();
      odom_to_base_link_msg_tfs.header.frame_id = odom_frame_;
      odom_to_base_link_msg_tfs.child_frame_id = base_frame_;
      odom_to_base_link_msg_tfs.header.stamp = now();
      odom_to_base_link_msg_tfs.transform = odom_to_base_link_msg_tf;

      // -- Publish Transform
      tf_pub_->sendTransform(odom_to_base_link_msg_tfs);
    }

    if (publish_world_to_odom_tf_) {
      // -- Prepare Transform
      auto map_to_odom_msg_tfs = geometry_msgs::msg::TransformStamped();
      map_to_odom_msg_tfs.header.frame_id = world_frame_;
      map_to_odom_msg_tfs.child_frame_id = odom_frame_;
      map_to_odom_msg_tfs.header.stamp = now();

      if (!update_world_position_with_odom_) {
        tf2::Transform odom_to_base_link_tf2_tf, map_to_base_link_tf2_tf;

        tf2::fromMsg(odom_to_base_link_msg_tf, odom_to_base_link_tf2_tf);

        PosPoint worldPosition = mCarState->getPosition(PosType::fused);
        auto map_to_base_link_msg_tf = geometry_msgs::msg::Transform();
        map_to_base_link_msg_tf.translation.x = worldPosition.getX();
        map_to_base_link_msg_tf.translation.y = worldPosition.getY();
        map_to_base_link_msg_tf.translation.z = worldPosition.getHeight();
        double worldYawRad_ = worldPosition.getYaw() * M_PI / 180.0;
        map_to_base_link_msg_tf.rotation.x = 0.0;
        map_to_base_link_msg_tf.rotation.y = 0.0;
        map_to_base_link_msg_tf.rotation.z = sin(worldYawRad_ / 2.0);
        map_to_base_link_msg_tf.rotation.w = cos(worldYawRad_ / 2.0);
        tf2::fromMsg(map_to_base_link_msg_tf, map_to_base_link_tf2_tf);

        tf2::toMsg(
          map_to_base_link_tf2_tf * odom_to_base_link_tf2_tf.inverse(),
          map_to_odom_msg_tfs.transform);
      }

      // -- Publish Transform
      tf_pub_->sendTransform(map_to_odom_msg_tfs);
    }
  }

  // -- Publish Odom
  odom_pub_->publish(odom);

  previousYawRad_ = yawRad_;
}

void WayWiseCar::twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  // RCLCPP_INFO(this->get_logger(), "got Twist: linear %f, angular %f",
  // twist_msg->linear.x, twist_msg->angular.z);
  float clipped_linear_velocity =
    (speed_to_erpm_factor_ > 0.0) ?
    clip_min_max(
    speed_to_erpm_factor_ * twist_msg->linear.x, erpm_min_,
    erpm_max_) / speed_to_erpm_factor_ :
    0.0;
  clipped_linear_velocity =
    (fabs(clipped_linear_velocity) >=
    standstill_velocity_threshold_) ? clipped_linear_velocity : 0.0;
  mCarMovementController->setDesiredSpeed(clipped_linear_velocity);

  float clipped_angular_velocity = clip_min_max(
    twist_msg->angular.z, -max_angular_velocity_,
    max_angular_velocity_);
  if (fabs(clipped_linear_velocity) >= standstill_velocity_threshold_) {
    // NOTE / TODO: WayWise has a sign error here (curvature in wrong direction)
    float desired_steering_curvature = -(clipped_angular_velocity / twist_msg->linear.x);    // ω = v/r => 1/r = ω/v
    mCarMovementController->setDesiredSteeringCurvature(desired_steering_curvature);
    // RCLCPP_INFO(this->get_logger(), "clipped_linear_velocity %f,
    // desired_steering_curvature %f", clipped_linear_velocity,
    // desired_steering_curvature);
  } else {
    // NOTE / TODO: WayWise has a sign error here (steering in wrong direction)
    float desired_steering =
      (fabs(clipped_angular_velocity) >=
      0.01) ? -(clipped_angular_velocity / max_angular_velocity_) : 0.0;
    mCarMovementController->setDesiredSteering(desired_steering);
    // RCLCPP_INFO(
    //   this->get_logger(), "clipped_angular_velocity %f, desired_steering %f", twist_msg->angular.z, clipped_angular_velocity,
    //   desired_steering);
  }
}

void WayWiseCar::publish_ublox_nav_sat_fix(const ubx_nav_pvt & ubxPvt)
{
  sensor_msgs::msg::NavSatFix nav_sat_fix_msg;

  // Set the header timestamp
  nav_sat_fix_msg.header.stamp = this->now();
  nav_sat_fix_msg.header.frame_id = "gnss"; // Set your frame ID

  // Set the status of the fix
  nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  if (ubxPvt.fix_type == 3) { // 3D fix
    nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  } else if (ubxPvt.fix_type == 2) { // 2D fix
    nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  } else { // No fix
    nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }

  // Set the service type (GPS, GLONASS, etc.)
  nav_sat_fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  // Set the latitude, longitude, and altitude
  nav_sat_fix_msg.latitude = ubxPvt.lat; // Latitude in degrees
  nav_sat_fix_msg.longitude = ubxPvt.lon; // Longitude in degrees
  nav_sat_fix_msg.altitude = ubxPvt.height; // Altitude in meters

  // Set the position covariance (assuming diagonal covariance matrix)
  nav_sat_fix_msg.position_covariance[0] = ubxPvt.h_acc * ubxPvt.h_acc; // Latitude variance
  nav_sat_fix_msg.position_covariance[4] = ubxPvt.h_acc * ubxPvt.h_acc; // Longitude variance
  nav_sat_fix_msg.position_covariance[8] = ubxPvt.v_acc * ubxPvt.v_acc; // Altitude variance
  nav_sat_fix_msg.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  // Publish the message
  nav_sat_fix_pub_->publish(nav_sat_fix_msg);
}

void WayWiseCar::publish_enu_refernce(const llh_t enuRef)
{
  auto enuRef_msg = geometry_msgs::msg::Vector3();
  enuRef_msg.x = enuRef.latitude;
  enuRef_msg.y = enuRef.longitude;
  enuRef_msg.z = enuRef.height;
  enu_refernce_pub_->publish(enuRef_msg);
}

float WayWiseCar::clip_min_max(float value, float min_value, float max_value) const
{
  return std::min(
    std::max(value, (min_value + std::numeric_limits<float>::epsilon())),
    (max_value - std::numeric_limits<float>::epsilon()));
}
