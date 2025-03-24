#include "waywise_car.hpp"
#include "moc_waywise_car.cpp"

void WayWiseCar::initialize_node()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  setup_parameters();
  setup_publishers();
  setup_subscribers();
  setup_timers();
  setup_autopilot();
  #ifdef WAYWISE_HW_INTERFACE_
  setup_hardware();
  #endif
  provide_parameters_to_parameter_server();

  RCLCPP_INFO(get_logger(), "%s is initialized!", this->get_name());
}

void WayWiseCar::setup_parameters()
{
  // ROS parameters
  odom_topic_ = this->declare_parameter("odom_topic", "/odom");
  speed_to_erpm_factor_ = this->declare_parameter("speed_to_erpm_factor", 0.0);
  length_ = this->declare_parameter("length", 0.33);
  width_ = this->declare_parameter("width", 0.33);
  wheelbase_ = this->declare_parameter("wheelbase", 0.33);
  min_turning_radius_ = this->declare_parameter("min_turning_radius", 0.67);
  autopilot_twist_publish_rate_ = this->declare_parameter("autopilot_twist_publish_rate", 30);
  waywise_control_tower_address_ = this->declare_parameter(
    "waywise_control_tower_address",
    "127.0.0.1");
  waywise_control_tower_port_ = this->declare_parameter("waywise_control_tower_port", 14540);
  purepursuit_radius_ = this->declare_parameter("purepursuit_radius", 1.0);
  update_world_position_with_odom_ = this->declare_parameter(
    "update_world_position_with_odom",
    false);
  update_world_position_with_tf_ = this->declare_parameter(
    "update_world_position_with_tf",
    false);

  max_angular_velocity_ = this->declare_parameter("max_angular_velocity", 0.5);
  standstill_velocity_threshold_ = this->declare_parameter("standstill_velocity_threshold", 0.05);
  joint_states_publish_rate_ = this->declare_parameter("joint_states_publish_rate", 0);

  odom_frame_ = declare_parameter("odom_frame", "odom");
  base_frame_ = declare_parameter("base_frame", "base_link");
  world_frame_ = declare_parameter("world_frame", "map");
  rear_axle_frame_ = this->declare_parameter("rear_axle_frame", base_frame_);
  center_frame_ = this->declare_parameter("center_frame", base_frame_);
  rear_end_frame_ = this->declare_parameter("rear_end_frame", "");

  rear_axle_frame_to_base_frame_offset_ =
    this->declare_parameter("rear_axle_frame_to_base_frame_offset", std::vector<double>({0.0}));
  rear_axle_frame_to_center_frame_offset_ =
    this->declare_parameter("rear_axle_frame_to_center_frame_offset", std::vector<double>({0.0}));
  rear_axle_frame_to_rear_end_frame_offset_ =
    this->declare_parameter("rear_axle_frame_to_rear_end_frame_offset", std::vector<double>({0.0}));

  nav_sat_fix_topic_ = this->declare_parameter("nav_sat_fix_topic", "/gnss_fix");
  enu_refernce_topic_ = this->declare_parameter("enu_refernce_topic", "/enu_refernce");
  vehicle_pose_topic_ = declare_parameter("vehicle_pose_topic", "/car_pose");

  preplanned_route_filepath_ = this->declare_parameter("preplanned_route_filepath", "");
  autopilot_state_control_topic_ =
    this->declare_parameter("autopilot_state_control_topic", "/autopilot_state_control");
  start_with_autopilot_ = this->declare_parameter("start_with_autopilot", true);
  desired_linear_velocity_ = this->declare_parameter("desired_linear_velocity", 0.2);
  mission_status_topic_ = this->declare_parameter("mission_status_topic", "/mission_status");
  end_goal_alignment_type_ = this->declare_parameter("end_goal_alignment_type", 0);
  vehicle_alignment_reference_point_topic_ = declare_parameter(
    "vehicle_alignment_reference_point_topic",
    "/vehicle_alignment_reference_point");
  autopilot_center_pose_topic_ = declare_parameter(
    "autopilot_center_pose_topic",
    "/autopilot_center_pose");

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

  #ifdef WAYWISE_HW_INTERFACE_
  erpm_min_ = this->declare_parameter("erpm_min", 0.0);
  erpm_max_ = this->declare_parameter("erpm_max", 0.0);
  invert_servo_output_ = this->declare_parameter("invert_servo_output", false);
  servo_offset_ = this->declare_parameter("servo_offset", 0.5);
  servo_min_ = this->declare_parameter("servo_min", 0.0);
  servo_max_ = this->declare_parameter("servo_max", 1.0);
  odom_and_tf_publish_rate_ = this->declare_parameter("odom_and_tf_publish_rate", 30);
  publish_odom_to_baselink_tf_ = this->declare_parameter("publish_odom_to_baselink_tf", true);
  publish_world_to_odom_tf_ = this->declare_parameter("publish_world_to_odom_tf", false);
  imu_for_position_fusion_ = declare_parameter("imu_for_position_fusion", "");
  min_battery_voltage_ = this->declare_parameter("min_battery_voltage", 0.0);
  battery_voltage_topic_ = this->declare_parameter("battery_voltage_topic", "/battery_voltage");
  #endif
}

void WayWiseCar::setup_publishers()
{
  // Publishers
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/waywise_vel", 10);
  vehicle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(vehicle_pose_topic_, 10);

  route_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "waypoints", 10);
  mission_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(mission_status_topic_, 10);
  vehicle_alignment_reference_point_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    vehicle_alignment_reference_point_topic_, 10);
  autopilot_center_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    autopilot_center_pose_topic_, 10);

  if (joint_states_publish_rate_ > 0) {
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("waywise_joint_states", 10);
  }

  #ifdef WAYWISE_HW_INTERFACE_
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  if (publish_odom_to_baselink_tf_ || publish_world_to_odom_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  nav_sat_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(nav_sat_fix_topic_, 10);
  enu_refernce_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
    enu_refernce_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  battery_voltage_pub_ = create_publisher<std_msgs::msg::Float32>(battery_voltage_topic_, 10);
  #endif
}

void WayWiseCar::setup_subscribers()
{
  // Subscribers
  autopilot_state_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    autopilot_state_control_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&WayWiseCar::autopilot_state_control_callback, this, _1)
  );
  enu_refernce_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    enu_refernce_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&WayWiseCar::enu_reference_callback, this, _1)
  );

  #ifdef WAYWISE_HW_INTERFACE_
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&WayWiseCar::twist_callback, this, _1));
  #else
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&WayWiseCar::odom_callback, this, _1));
  #endif
}

void WayWiseCar::setup_timers()
{
  // Timers
  autopilot_timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds((int)std::round(1000.0 / autopilot_twist_publish_rate_)),
    std::bind(&WayWiseCar::autopilot_timer_callback, this));

  #ifdef WAYWISE_HW_INTERFACE_
  odom_publish_period_ms_ = 1000.0 / odom_and_tf_publish_rate_;
  odom_publish_timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(odom_publish_period_ms_),
    std::bind(&WayWiseCar::odom_publish_timer_callback, this));
  #endif
}

void WayWiseCar::setup_autopilot()
{
  mCarState.reset(new CarState);
  setup_autopilot(mCarState);
}

void WayWiseCar::setup_autopilot(QSharedPointer<CarState> carState)
{
  // -- WayWise --
  mCarState = carState;
  mCarState->setLength(length_);
  mCarState->setWidth(width_);
  mCarState->setAxisDistance(wheelbase_);
  mCarState->setMaxSteeringAngle(atan(wheelbase_ / min_turning_radius_));
  mCarState->setEndGoalAlignmentType(
    static_cast<AutopilotEndGoalAlignmentType>(end_goal_alignment_type_));

  // --- Set rear axle offsets ---
  bool offset_params_initiazed = true;
  if (rear_axle_frame_to_base_frame_offset_.size() != 3 ||
    rear_axle_frame_to_center_frame_offset_.size() != 3 ||
    rear_axle_frame_to_rear_end_frame_offset_.size() != 3)
  {
    offset_params_initiazed = false;
  }

  if (rear_axle_frame_to_center_frame_offset_.size() == 3) {
    mCarState->setRearAxleToCenterOffset(
      xyz_t{
      rear_axle_frame_to_center_frame_offset_[0],
      rear_axle_frame_to_center_frame_offset_[1],
      rear_axle_frame_to_center_frame_offset_[2]
    });
  }

  if (rear_axle_frame_to_rear_end_frame_offset_.size() == 3) {
    mCarState->setRearAxleToRearEndOffset(
      xyz_t{
      rear_axle_frame_to_rear_end_frame_offset_[0],
      rear_axle_frame_to_rear_end_frame_offset_[1],
      rear_axle_frame_to_rear_end_frame_offset_[2]
    });
  }

  if (!offset_params_initiazed && loadURDFFile()) {
    Eigen::Vector3d offset;

    if (rear_axle_frame_to_base_frame_offset_.size() != 3) {
      offset = getLinkPosition(urdfModel, base_frame_) -
        getLinkPosition(urdfModel, rear_axle_frame_);
      rear_axle_frame_to_base_frame_offset_ = {offset.x(), offset.y(), offset.z()};
    }

    if (rear_axle_frame_to_center_frame_offset_.size() != 3) {
      offset = getLinkPosition(urdfModel, center_frame_) -
        getLinkPosition(urdfModel, rear_axle_frame_);
      rear_axle_frame_to_center_frame_offset_ = {offset.x(), offset.y(), offset.z()};
      mCarState->setRearAxleToCenterOffset(xyz_t{offset.x(), offset.y(), offset.z()});
    }

    if (rear_axle_frame_to_rear_end_frame_offset_.size() != 3) {
      offset = getLinkPosition(urdfModel, rear_end_frame_) -
        getLinkPosition(urdfModel, rear_axle_frame_);
      rear_axle_frame_to_rear_end_frame_offset_ = {offset.x(), offset.y(), offset.z()};
      mCarState->setRearAxleToRearEndOffset(xyz_t{offset.x(), offset.y(), offset.z()});
    }
  } else {
    if (rear_axle_frame_to_base_frame_offset_.size() != 3) {
      rear_axle_frame_to_base_frame_offset_ = {0.0, 0.0, 0.0};
    }

    if (rear_axle_frame_to_center_frame_offset_.size() != 3) {
      rear_axle_frame_to_center_frame_offset_ = {0.0, 0.0, 0.0};
    }

    if (rear_axle_frame_to_rear_end_frame_offset_.size() != 3) {
      rear_axle_frame_to_rear_end_frame_offset_ = {0.0, 0.0, 0.0};
    }
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
      QHostAddress(QString::fromStdString(waywise_control_tower_address_)),
      waywise_control_tower_port_));
  mMavsdkVehicleServer->setMovementController(mCarMovementController);
  mMavsdkVehicleServer->setGNSSReceiver(mGNSSReceiver);

  // --- Autopilot ---
  mWaypointFollower.reset(new PurepursuitWaypointFollower(mCarMovementController));
  mWaypointFollower->setPurePursuitRadius(purepursuit_radius_);
  mWaypointFollower->setRepeatRoute(false);
  mWaypointFollower->setAdaptivePurePursuitRadiusActive(true);
  mMavsdkVehicleServer->setWaypointFollower(mWaypointFollower);

  if (start_with_autopilot_) {
    currentMissionState = MissionState::WaitingForRoute;
  }

  // --- Load preplanned route ---
  if (!preplanned_route_filepath_.empty()) {
    if (preplanned_route_filepath_[0] == '~') {
      preplanned_route_filepath_ = std::string(std::getenv("HOME")) +
        preplanned_route_filepath_.substr(1);
    }
    mWaypointList = read_route_from_XMLFile(preplanned_route_filepath_);
  }

  // Watchdog that warns when EventLoop is slowed down
  mSimpleWatchdog.reset(new SimpleWatchdog(this));
}

#ifdef WAYWISE_HW_INTERFACE_
void WayWiseCar::setup_hardware()
{
  // -- WayWise --
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
    mVESCMotorController->setPollValuesPeriod(odom_publish_period_ms_);

    // VESC is a special case that can also control the servo
    const auto servoController = mVESCMotorController->getServoController();
    servoController->setInvertOutput(invert_servo_output_);
    servoController->setServoRange(servo_max_ - servo_min_);
    servoController->setServoCenter(servo_offset_);
    mCarMovementController->setServoController(servoController);

    if (min_battery_voltage_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Param 'min_battery_voltage' is not set. Please set a value to get low battery warnings!");
    } else {
      RCLCPP_INFO(
        get_logger(), "Low battery warning is set to %f V", min_battery_voltage_);
    }

    QObject::connect(
      mVESCMotorController.get(), &VESCMotorController::gotStatusValues,
      [&](double rpm, int tachometer, int tachometer_abs, double voltageInput, double temperature,
      int errorID) {
        Q_UNUSED(rpm)
        Q_UNUSED(tachometer)
        Q_UNUSED(tachometer_abs)
        Q_UNUSED(temperature)
        Q_UNUSED(errorID)

        static int count = 0;
        if (count++ % odom_and_tf_publish_rate_) { // reduce output rate to 1 Hz
          return;
        }

        if (min_battery_voltage_ > 0.0 && voltageInput < min_battery_voltage_) {
          RCLCPP_WARN(
            get_logger(), "Battery voltage is low: %f V. Please recharge the battery!",
            voltageInput);
        }

        publish_battery_voltage(voltageInput);
      });
  } else {
    // publish periodically with timer when no motorcontroller connected
    // (simulation)
    is_in_ww_simulation_mode_ = true;
    RCLCPP_INFO(
      get_logger(),
      "VESCMotorController is not connected. "
      "waywise_car is in simulation mode!");
  }

  // --- Positioning setup ---
  // Position Fuser
  mSDVPVehiclePositionFuser.reset(new SDVPVehiclePositionFuser(this));
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

  mRtcmClient.reset(new RtcmClient(this));
  QObject::connect(
    mUbloxRover.get(), &UbloxRover::updatedGNSSPositionAndYaw, mSDVPVehiclePositionFuser.get(),
    &SDVPVehiclePositionFuser::correctPositionAndYawGNSS);

  // -- NTRIP/TCP client setup for feeding RTCM data into GNSS receiver
  QObject::connect(
    mUbloxRover.get(), &UbloxRover::gotNmeaGga,
    mRtcmClient.get(), &RtcmClient::forwardNmeaGgaToServer);
  QObject::connect(
    mRtcmClient.get(), &RtcmClient::rtcmData,
    mUbloxRover.get(), &UbloxRover::writeRtcmToUblox);
  QObject::connect(
    mRtcmClient.get(), &RtcmClient::baseStationPosition,
    mUbloxRover.get(), &UbloxRover::setEnuRef);
  if (mRtcmClient->connectWithInfoFromFile("./rtcmServerInfo.txt")) {
    qDebug() << "RtcmClient: connected to" << QString(
      mRtcmClient->getCurrentHost() + ":" + QString::number(mRtcmClient->getCurrentPort()));
  } else {
    qDebug() << "RtcmClient: not connected";
  }


  // IMU
  if (!imu_for_position_fusion_.empty()) {
    if (imu_for_position_fusion_ == "vesc") {
      if (mVESCMotorController->isSerialConnected()) {
        mIMUOrientationUpdater = mVESCMotorController->getIMUOrientationUpdater(mCarState);
        QObject::connect(
          mIMUOrientationUpdater.get(), &IMUOrientationUpdater::updatedIMUOrientation,
          mSDVPVehiclePositionFuser.get(),
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
        mIMUOrientationUpdater.get(), &IMUOrientationUpdater::updatedIMUOrientation,
        mSDVPVehiclePositionFuser.get(),
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
    mCarMovementController.get(), &CarMovementController::updatedOdomPositionAndYaw,
    mSDVPVehiclePositionFuser.get(),
    &SDVPVehiclePositionFuser::correctPositionAndYawOdom);
}
#endif

void WayWiseCar::provide_parameters_to_parameter_server()
{
  mCarState->provideParametersToParameterServer();
  mWaypointFollower->provideParametersToParameterServer();
  mFollowPoint->provideParametersToParameterServer();
  mMavsdkVehicleServer->provideParametersToParameterServer();
}

// ----------------- Callback methods -----------------
void WayWiseCar::autopilot_timer_callback()
{
  auto now = this->get_clock()->now();
  static auto previousTimeCalled = now;
  // Detect clock reset
  if (now < previousTimeCalled) {
    RCLCPP_WARN(this->get_logger(), "Clock reset detected! Resetting autopilot state.");
    mWaypointFollower->clearRoute();
    mWaypointFollower->resetState();
    currentMissionState = MissionState::Idle;
    mWaypointList.clear();
    previousTimeCalled = now;
    return;
  }
  previousTimeCalled = now;

  if (update_world_position_with_tf_) {
    static bool transform_warning_logged_ = false;
    try {
      geometry_msgs::msg::TransformStamped map_to_base_link_msg_tfs = tf_buffer_->lookupTransform(
        world_frame_, rear_axle_frame_, tf2::TimePointZero);
      geometry_msgs::msg::Pose world_pose;
      world_pose.position.x = map_to_base_link_msg_tfs.transform.translation.x;
      world_pose.position.y = map_to_base_link_msg_tfs.transform.translation.y;
      world_pose.position.z = map_to_base_link_msg_tfs.transform.translation.z;
      world_pose.orientation = map_to_base_link_msg_tfs.transform.rotation;
      update_world_positon(world_pose);
      if (transform_warning_logged_) {
        RCLCPP_INFO(
          get_logger(), "Transform from %s to %s is available now.",
          world_frame_.c_str(), rear_axle_frame_.c_str());
        transform_warning_logged_ = false;
      }
    } catch (tf2::TransformException & ex) {
      if (!transform_warning_logged_) {
        RCLCPP_WARN(
          get_logger(), "Transform from %s to %s not available yet!",
          world_frame_.c_str(), rear_axle_frame_.c_str());
        transform_warning_logged_ = true;
      }
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

  switch (currentMissionState) {
    case MissionState::Idle: {
        // Check if mWaypointFollower is started via MAVLINK
        if (mWaypointFollower->isActive()) {
          if (mWaypointFollower->getCurrentRoute().size() > 0) {
            currentMissionState = MissionState::ActiveMission;
          } else {
            currentMissionState = MissionState::WaitingForRoute;
          }
        }
      } break;
    case MissionState::WaitingForRoute: {
        // Check if mWaypointFollower is stopped via MAVLINK
        if (!mWaypointFollower->isActive()) {
          currentMissionState = MissionState::Idle;
          stop_waypoint_follower();
        } else {
          if (mWaypointList.isEmpty() && !preplanned_route_filepath_.empty()) {
            mWaypointList = read_route_from_XMLFile(preplanned_route_filepath_);
          }
          if (!mWaypointList.isEmpty()) {
            currentMissionState = MissionState::ActiveMission;
            start_waypoint_follower(mWaypointList);
          }
        }
      } break;
    case MissionState::ActiveMission: {
        // Check if mWaypointFollower is stopped via MAVLINK
        if (!mWaypointFollower->isActive()) {
          currentMissionState = MissionState::Idle;
          stop_waypoint_follower();
        } else {
          geometry_msgs::msg::PoseStamped world_pose_stamped;
          world_pose_stamped.header.frame_id = world_frame_;
          world_pose_stamped.header.stamp = this->get_clock()->now();
          QSharedPointer<VehicleState> referenceVehicleState = mCarState;
          if (mCarState->hasTrailingVehicle() && mCarState->getSpeed() < 0) { // position defined by trailer when backing (if exists)
            referenceVehicleState = mCarState->getTrailingVehicle();
          }
          PosPoint currentVehiclePosition = referenceVehicleState->getPosition(PosType::fused);
          world_pose_stamped.pose.position.x = currentVehiclePosition.getX();
          world_pose_stamped.pose.position.y = currentVehiclePosition.getY();
          world_pose_stamped.pose.position.z = currentVehiclePosition.getHeight();
          tf2::Quaternion orientation;
          orientation.setRPY(0.0, 0.0, currentVehiclePosition.getYaw() * M_PI / 180.0);
          world_pose_stamped.pose.orientation = tf2::toMsg(orientation);
          autopilot_center_pose_pub_->publish(world_pose_stamped);

          QPointF vehicleAlignmentReferencePointXY =
            mWaypointFollower->getVehicleAlignmentReferencePoint();
          world_pose_stamped.pose.position.x = vehicleAlignmentReferencePointXY.x();
          world_pose_stamped.pose.position.y = vehicleAlignmentReferencePointXY.y();
          vehicle_alignment_reference_point_pub_->publish(world_pose_stamped);
        }
      } break;
    default:
      break;
  }
}

void WayWiseCar::autopilot_state_control_callback(
  const std_msgs::msg::Bool::SharedPtr bool_msg)
{
  if (bool_msg->data) {
    if (currentMissionState == MissionState::Idle) {
      if (mWaypointList.isEmpty()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for a route to follow...");
        currentMissionState = MissionState::WaitingForRoute;
        return;
      }
      currentMissionState = MissionState::ActiveMission;
      start_waypoint_follower(mWaypointList);
    }
  } else {
    if (currentMissionState != MissionState::Idle) {
      currentMissionState = MissionState::Idle;
      stop_waypoint_follower();
    }
  }
}

void WayWiseCar::enu_reference_callback(
  const geometry_msgs::msg::Vector3::SharedPtr enuRef_msg)
{
  llh_t mEnuReference{enuRef_msg->x, enuRef_msg->y, enuRef_msg->z};
  mGNSSReceiver->setEnuRef(mEnuReference);

  RCLCPP_INFO(
    this->get_logger(),
    "Updated enu reference to: latitude=%.2f, longitude=%.2f, height=%.2f",
    mEnuReference.latitude, mEnuReference.longitude, mEnuReference.height);
}

#ifdef WAYWISE_HW_INTERFACE_
void WayWiseCar::odom_publish_timer_callback()
{
  if (is_in_ww_simulation_mode_) { // simulation mode
    mCarState->simulationStep(odom_publish_period_ms_, PosType::odom);
    if (publish_world_to_odom_tf_) {
      PosPoint currentPosition = mCarState->getPosition(PosType::odom);
      currentPosition.setType(PosType::fused);
      mCarState->setPosition(currentPosition);
    }
  }

  // -- Publish Odometry and Tfs
  publish_odom_and_tfs();

  // -- Publish Joint states
  if (joint_states_publish_rate_ > 0) {
    publish_joint_states();
  }
}

void WayWiseCar::twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  // RCLCPP_INFO(this->get_logger(), "got Twist: linear %f, angular %f",
  // twist_msg->linear.x, twist_msg->angular.z);
  float clipped_linear_velocity =
    (speed_to_erpm_factor_ > 0.0) ?
    std::clamp(
    speed_to_erpm_factor_ * twist_msg->linear.x, erpm_min_,
    erpm_max_) / speed_to_erpm_factor_ :
    0.0;
  clipped_linear_velocity =
    (fabs(clipped_linear_velocity) >=
    standstill_velocity_threshold_) ? clipped_linear_velocity : 0.0;
  mCarMovementController->setDesiredSpeed(clipped_linear_velocity);

  float clipped_angular_velocity = std::clamp(
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
#else
void WayWiseCar::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  static xyz_t rear_axle_frame_to_odom_child_frame_offset;
  if (!received_first_odom_msg_) {
    if (odom_msg->child_frame_id != rear_axle_frame_) { // rear_axle_frame_ is the vehicle reference point for waywise
      if (odom_msg->child_frame_id == center_frame_) {
        rear_axle_frame_to_odom_child_frame_offset = mCarState->getRearAxleToCenterOffset();
      } else if (odom_msg->child_frame_id == rear_end_frame_) {
        rear_axle_frame_to_odom_child_frame_offset = mCarState->getRearAxleToRearEndOffset();
      } else if (loadURDFFile()) {
        Eigen::Vector3d offset = getLinkPosition(urdfModel, odom_msg->child_frame_id) -
          getLinkPosition(urdfModel, rear_axle_frame_);
        rear_axle_frame_to_odom_child_frame_offset = {offset.x(), offset.y(), offset.z()};
      } else {
        static bool transform_warning_logged_ = false;
        try {
          geometry_msgs::msg::TransformStamped rear_axle_frame_to_odom_child_frame_msg_tfs =
            tf_buffer_->lookupTransform(
            rear_axle_frame_, odom_msg->child_frame_id, tf2::TimePointZero);
          rear_axle_frame_to_odom_child_frame_offset = {
            rear_axle_frame_to_odom_child_frame_msg_tfs.transform.translation.x,
            rear_axle_frame_to_odom_child_frame_msg_tfs.transform.translation.y,
            rear_axle_frame_to_odom_child_frame_msg_tfs.transform.translation.z
          };
          if (transform_warning_logged_) {
            RCLCPP_INFO(
              get_logger(), "Transform from %s to %s is available now.",
              rear_axle_frame_.c_str(), odom_msg->child_frame_id.c_str());
            transform_warning_logged_ = false;
          }
        } catch (tf2::TransformException & ex) {
          if (!transform_warning_logged_) {
            RCLCPP_WARN(
              get_logger(), "Transform from %s to %s not available yet!",
              rear_axle_frame_.c_str(), odom_msg->child_frame_id.c_str());
            transform_warning_logged_ = true;
          }
          return;
        }
      }
      received_first_odom_msg_ = true;
      RCLCPP_INFO(this->get_logger(), "Received first odom message.");
    }
  }

  geometry_msgs::msg::Pose odom_pose = odom_msg->pose.pose;
  PosPoint currentPosition = mCarState->getPosition(PosType::odom);
  currentPosition.setX(odom_pose.position.x);
  currentPosition.setY(odom_pose.position.y);
  currentPosition.setHeight(odom_pose.position.z);
  currentPosition.updateWithOffsetAndYawRotation(
    -rear_axle_frame_to_odom_child_frame_offset, tf2::getYaw(odom_pose.orientation));
  currentPosition.setTime(
    QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
  mCarState->setPosition(currentPosition);

  if (update_world_position_with_odom_) {
    geometry_msgs::msg::Pose world_pose = odom_pose;
    if (odom_msg->child_frame_id != rear_axle_frame_) {
      world_pose.position.x = currentPosition.getX();
      world_pose.position.y = currentPosition.getY();
      world_pose.position.z = currentPosition.getHeight();
    }
    update_world_positon(world_pose);
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
  if (joint_states_publish_rate_ > 0) {
    publish_joint_states();
  }
}
#endif

// ----------------- Utility methods -----------------
void WayWiseCar::update_world_positon(geometry_msgs::msg::Pose world_pose)
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

QList<PosPoint> WayWiseCar::read_route_from_XMLFile(const std::string xml_filepath_)
{
  QFile file(QString::fromUtf8(xml_filepath_.c_str()));

  QXmlStreamReader stream(&file);
  QList<PosPoint> importedRoute;

  if (!file.open(QIODevice::ReadOnly)) {
    RCLCPP_INFO(this->get_logger(), "could not open file");
  }

  if (stream.readNextStartElement()) {
    if (stream.name() == "routes") {
      PosPoint vehiclePosition;
      bool use_curent_vehicle_position_as_enuref = true;
      llh_t importedEnuRef{0.0, 0.0, 0.0};

      while (stream.readNextStartElement()) {
        if (stream.name() == "enuref") {
          while (stream.readNextStartElement()) {
            if (stream.name() == "Latitude") {
              importedEnuRef.latitude = stream.readElementText().toDouble();
              use_curent_vehicle_position_as_enuref = false;
            }
            if (stream.name() == "Longitude") {
              importedEnuRef.longitude = stream.readElementText().toDouble();
            }
            if (stream.name() == "Height") {
              importedEnuRef.height = stream.readElementText().toDouble();
            }
          }
        }

        if (use_curent_vehicle_position_as_enuref && !received_first_odom_msg_) {
          return importedRoute;
        }

        if (stream.name() == "route") {
          while (stream.readNextStartElement()) {
            if (stream.name() == "point") {
              PosPoint importedPoint;

              while (stream.readNextStartElement()) {
                if (stream.name() == "x") {
                  importedPoint.setX(stream.readElementText().toDouble());
                }
                if (stream.name() == "y") {
                  importedPoint.setY(stream.readElementText().toDouble());
                }
                if (stream.name() == "z") {
                  importedPoint.setHeight(stream.readElementText().toDouble());
                }
                if (stream.name() == "speed") {
                  importedPoint.setSpeed(stream.readElementText().toDouble());
                }
                if (stream.name() == "attributes") {
                  importedPoint.setAttributes(stream.readElementText().toUInt());
                }
              }

              if (use_curent_vehicle_position_as_enuref) {
                vehiclePosition = mCarState->getPosition(PosType::fused);
                if (mCarState->hasTrailingVehicle() && importedPoint.getSpeed() < 0) {
                  vehiclePosition = mCarState->getTrailingVehicle()->getPosition(PosType::fused);
                }

                importedPoint.setX(importedPoint.getX() + vehiclePosition.getX());
                importedPoint.setY(importedPoint.getY() + vehiclePosition.getY());
              } else {
                llh_t importedAbsPoint = coordinateTransforms::enuToLlh(
                  importedEnuRef, {importedPoint.getX(),
                    importedPoint.getY(), importedPoint.getHeight()});

                xyz_t importedEnuPoint = coordinateTransforms::llhToEnu(
                  mGNSSReceiver->getEnuRef(), importedAbsPoint);

                importedPoint.setX(importedEnuPoint.x);
                importedPoint.setY(importedEnuPoint.y);
                importedPoint.setHeight(importedEnuPoint.z);
              }

              importedRoute.append(importedPoint);
            }
          }
        }
      }
    }
  }

  return importedRoute;
}

void WayWiseCar::start_waypoint_follower(QList<PosPoint> & waypointList)
{
  mWaypointFollower->clearRoute();
  mWaypointFollower->addRoute(waypointList);
  mWaypointFollower->startFollowingRoute(false);
  RCLCPP_INFO(
    this->get_logger(), "Started waypoint follower with a route of %d waypoints",
    waypointList.size());
  publish_route_markers();
  mission_status_pub_->publish(std_msgs::msg::Bool().set__data(true));
}

void WayWiseCar::stop_waypoint_follower()
{
  mWaypointFollower->stop();
  RCLCPP_INFO(this->get_logger(), "Waypoint follower is stopped.");
  mission_status_pub_->publish(std_msgs::msg::Bool().set__data(false));
}

void WayWiseCar::update_waypoint_follower_route(QList<PosPoint> & waypointList)
{
  mWaypointFollower->addRoute(waypointList);
  mWaypointList = mWaypointFollower->getCurrentRoute();
  publish_route_markers();
}

void WayWiseCar::publish_route_markers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;

  for (const auto & waypoint : mWaypointList) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = waypoint.getX();
    marker.pose.position.y = waypoint.getY();
    marker.pose.position.z = 0.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;   // Full opacity

    marker_array.markers.push_back(marker);
  }

  route_marker_pub_->publish(marker_array);
}

#ifdef WAYWISE_HW_INTERFACE_
void WayWiseCar::publish_odom_and_tfs()
{
  static xyz_t rear_axle_frame_to_base_frame_offset_xyz = {
    rear_axle_frame_to_base_frame_offset_[0], rear_axle_frame_to_base_frame_offset_[1],
    rear_axle_frame_to_base_frame_offset_[2]};

  PosPoint odom_to_base_link_position = mCarState->posInVehicleFrameToPosPointENU(
    rear_axle_frame_to_base_frame_offset_xyz, PosType::odom);

  double x_ = odom_to_base_link_position.getX();
  double y_ = odom_to_base_link_position.getY();
  double yawRad_ = odom_to_base_link_position.getYaw() * M_PI / 180.0;
  static double previousYawRad_ = yawRad_;

// -- Prepare odom msg
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = now();
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

// Position in the coordinate frame given by header.frame_id
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(yawRad_ / 2.0);
  odom_msg.pose.pose.orientation.w = cos(yawRad_ / 2.0);

// TODO: position uncertainty?

// Velocity in the coordinate frame given by child_frame_id
  odom_msg.twist.twist.linear.x = mCarState->getSpeed();
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = (yawRad_ - previousYawRad_) / (odom_publish_period_ms_ / 1000.0);
  previousYawRad_ = yawRad_;

// TODO: velocity uncertainty?

// -- Update world position with odom if enabled
  if (update_world_position_with_odom_) {
    geometry_msgs::msg::Pose world_pose = odom_msg.pose.pose;
    if (odom_msg.child_frame_id != rear_axle_frame_) {
      PosPoint currentPosition = mCarState->getPosition(PosType::odom);
      world_pose.position.x = currentPosition.getX();
      world_pose.position.y = currentPosition.getY();
      world_pose.position.z = currentPosition.getHeight();
    }
    update_world_positon(world_pose);
  }

  if (publish_odom_to_baselink_tf_ || publish_world_to_odom_tf_) {
    // -- Prepare Transform
    auto odom_to_base_link_msg_tf = geometry_msgs::msg::Transform();
    odom_to_base_link_msg_tf.translation.x = x_;
    odom_to_base_link_msg_tf.translation.y = y_;
    odom_to_base_link_msg_tf.translation.z = 0.0;
    odom_to_base_link_msg_tf.rotation = odom_msg.pose.pose.orientation;

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

        PosPoint world_to_base_link_position = mCarState->posInVehicleFrameToPosPointENU(
          mCarState->getRearAxleToCenterOffset(), PosType::fused);
        auto map_to_base_link_msg_tf = geometry_msgs::msg::Transform();
        map_to_base_link_msg_tf.translation.x = world_to_base_link_position.getX();
        map_to_base_link_msg_tf.translation.y = world_to_base_link_position.getY();
        map_to_base_link_msg_tf.translation.z = world_to_base_link_position.getHeight();
        double worldYawRad_ = world_to_base_link_position.getYaw() * M_PI / 180.0;
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
  odom_pub_->publish(odom_msg);
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

void WayWiseCar::publish_battery_voltage(double voltage)
{
  auto voltage_msg = std_msgs::msg::Float32();
  voltage_msg.data = voltage;
  battery_voltage_pub_->publish(voltage_msg);
}
#endif

bool WayWiseCar::loadURDFFile()
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

Eigen::Vector3d WayWiseCar::getLinkPosition(
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

void WayWiseCar::publish_joint_states()
{
  static auto previousTimeCalled = this->get_clock()->now();
  auto thisTimeCalled = this->get_clock()->now();
  double timePassed_ms = (thisTimeCalled.nanoseconds() - previousTimeCalled.nanoseconds()) / 1e6;
  if (timePassed_ms < 1000.0 / joint_states_publish_rate_) {
    // Not enough time has passed; skip the execution.
    return;
  }
  previousTimeCalled = thisTimeCalled;
  sensor_msgs::msg::JointState joint_state_msg;
  update_joint_states_msg(joint_state_msg, timePassed_ms);
  joint_state_pub_->publish(joint_state_msg);
}

double WayWiseCar::update_joint_states_msg(
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
  wheel_position += wheel_rad_per_sec * timePassed_ms / 1000.0;
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
