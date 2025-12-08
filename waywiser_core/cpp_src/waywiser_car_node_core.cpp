#include "waywiser_car_node_core.hpp"
#include "moc_waywiser_car_node_core.cpp"

void WaywiserCar::initialize_node()
{
  mCarState.reset(new CarState());
  mCarInterfaceComponent.reset(new CarInterfaceComponent(this, mCarState, false));
  enable_autopilot_component_ = declare_parameter("enable_autopilot_component", true);
  if (enable_autopilot_component_) {
    mCarAutopilotComponent.reset(new CarAutopilotComponent(this, mCarState));
  }

  initialize_node(mCarState, mCarInterfaceComponent, mCarAutopilotComponent);
}

void WaywiserCar::initialize_node(
  QSharedPointer<CarState> carState,
  QSharedPointer<CarInterfaceComponent> carInterfaceComponent,
  QSharedPointer<CarAutopilotComponent> carAutopilotComponent)
{
  auto use_sim_time = this->get_parameter("use_sim_time").as_bool();
  if (use_sim_time) {
    if (rclcpp::ok() && this->get_clock()->now().nanoseconds() == 0) {
      RCLCPP_WARN(this->get_logger(), "Waiting for /clock to be published...");
    }

    while (rclcpp::ok() && this->get_clock()->now().nanoseconds() == 0) {
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    RCLCPP_INFO(this->get_logger(), "Receiving /clock msgs now.");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  mCarState = carState;
  mCarInterfaceComponent = carInterfaceComponent;
  mCarAutopilotComponent = carAutopilotComponent;

  setup_parameters();

  mCarInterfaceComponent->setup_vehicle_interface();

  mEmergencyStopState = mCarInterfaceComponent->getEmergencyStopState();

  if (enable_autopilot_component_) {
    mCarAutopilotComponent->setupAutopilot(mEmergencyStopState);
    if (mCarAutopilotComponent->getEnableMavlinkInterface()) {
      mCarAutopilotComponent->provideParametersToParameterServer();
    }
  }

  setup_publishers();
  setup_subscribers();
  setup_timers();

  mCarAutopilotComponent->reset();
  mCarInterfaceComponent->reset();

  RCLCPP_INFO(get_logger(), "%s is initialized!", this->get_name());
  RCLCPP_INFO(
    get_logger(), "Minimum target speed: %f, Maximum target speed: %f",
    min_target_speed, max_target_speed);
}

void WaywiserCar::setup_parameters()
{
  // ROS parameters
  urdf_file_ = declare_parameter("urdf_file", "");
  mUrdfModel = getURDFModel(urdf_file_);

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

  odom_frame_ = declare_parameter("odom_frame", "odom");
  base_frame_ = declare_parameter("base_frame", "base_link");
  world_frame_ = declare_parameter("world_frame", "map");
  rear_axle_frame_ = declare_parameter("rear_axle_frame", base_frame_);
  chassis_frame_ = declare_parameter("chassis_frame", base_frame_);
  front_end_frame_ = declare_parameter("front_end_frame", base_frame_);
  rear_end_frame_ = declare_parameter("rear_end_frame", base_frame_);
  left_end_frame_ = declare_parameter("left_end_frame", base_frame_);
  right_end_frame_ = declare_parameter("right_end_frame", base_frame_);

  battery_state_topic_ = declare_parameter("battery_state_topic", "/battery_state");
  odom_topic_ = declare_parameter("odom_topic", "/odom");
  fused_nav_sat_fix_extended_topic_ = declare_parameter(
    "fused_nav_sat_fix_extended_topic", "/nav_sat_fix_extended");
  vehicle_pose_topic_ = declare_parameter("vehicle_pose_topic", "/car_pose");
  emergency_stop_update_topic_ =
    declare_parameter("emergency_stop_update_topic", "/emergency_stop/target_state");
  car_control_command_topic_ = declare_parameter(
    "car_control_command_topic",
    "/waywiser_control_cmd");

  mission_status_topic_ = declare_parameter("mission_status_topic", "/mission_status");
  vehicle_alignment_reference_point_topic_ = declare_parameter(
    "vehicle_alignment_reference_point_topic", "/vehicle_alignment_reference_point");
  autopilot_center_pose_topic_ = declare_parameter(
    "autopilot_center_pose_topic", "/autopilot_center_pose");

  emergency_stop_status_topic_ =
    declare_parameter("emergency_stop_status_topic", "/emergency_stop/current_state");
  autopilot_state_control_topic_ =
    declare_parameter("autopilot_state_control_topic", "/autopilot_state_control");

  tof_sensor_names_ = this->declare_parameter<std::vector<std::string>>(
    "tof_sensors", {}, rcl_interfaces::msg::ParameterDescriptor{});
  if (tof_sensor_names_.size() > 1) {
    RCLCPP_WARN(
      this->get_logger(),
      "More than one ToF sensor is not currently supported. "
      "Only the first sensor will be used: '%s'. Ignoring others.",
      tof_sensor_names_[0].c_str());

    tof_sensor_names_.resize(1);   // TODO: enable setting multiple tof sensors
  }

  enable_visualization_msgs_ = declare_parameter("enable_visualization_msgs", false);
  publish_odom_to_baselink_tf_ = declare_parameter("publish_odom_to_baselink_tf", true);
  publish_world_to_odom_tf_ = declare_parameter("publish_world_to_odom_tf", false);
  invert_steering_feedback_from_odom_ = declare_parameter(
    "invert_steering_feedback_from_odom", false);

  joint_states_publish_rate_ = declare_parameter("joint_states_publish_rate", 0);
  wheel_diameter_ = declare_parameter("wheel_diameter", 1.0);
  invert_steering_joint_state_ = declare_parameter("invert_steering_joint_state", false);

  // Setup component parameters
  mCarInterfaceComponent->setLength(declare_parameter("length", 0.8));
  mCarInterfaceComponent->setWidth(declare_parameter("width", 0.335));
  mCarInterfaceComponent->setWheelbase(declare_parameter("wheelbase", 0.48));
  mCarInterfaceComponent->setMinTurningRadius(declare_parameter("min_turning_radius", 0.67));
  mCarInterfaceComponent->setErpmMin(declare_parameter("erpm_min", 2000.0));
  mCarInterfaceComponent->setErpmMax(declare_parameter("erpm_max", 4000.0));
  mCarInterfaceComponent->setSpeedToRPMFactor(declare_parameter("speed_to_erpm_factor", 4123.3));
  mCarInterfaceComponent->setInvertServoOutput(declare_parameter("invert_servo_output", false));
  mCarInterfaceComponent->setServoOffset(declare_parameter("servo_offset", 0.5));
  mCarInterfaceComponent->setServoRange(declare_parameter("servo_range", 1.0));
  mCarInterfaceComponent->setMinBatteryVoltage(declare_parameter("min_battery_voltage", 0.0));

  min_target_speed = mCarInterfaceComponent->getErpmMin() /
    mCarInterfaceComponent->getSpeedToRPMFactor();
  max_target_speed = mCarInterfaceComponent->getErpmMax() /
    mCarInterfaceComponent->getSpeedToRPMFactor();

  mCarInterfaceComponent->setImuVariant(
    get_imu_variant_param(this, "imu_variant"));
  mCarInterfaceComponent->setVehicleInterfaceType(
    get_vehicle_interface_type_param(this, "vehicle_interface_type"));
  mCarInterfaceComponent->setSpeedControlType(
    get_speed_control_type_param(this, "speed_control_type"));

  auto pid_speed_controller_gains_opt = get_vector3_param(this, "pid_speed_controller_gains");
  if (pid_speed_controller_gains_opt) {
    mCarInterfaceComponent->setPIDSpeedControllerGains(
      pid_speed_controller_gains_opt->x, pid_speed_controller_gains_opt->y,
      pid_speed_controller_gains_opt->z);
  }

  mCarInterfaceComponent->setVehicleStatePollRate(
    declare_parameter("vehicle_state_poll_rate", 10));

  std::map<std::string, std::tuple<int, int>> tof_sensors_info;
  for (const auto & tof_sensor_name : tof_sensor_names_) {
    int i2c_bus = this->declare_parameter<int>(tof_sensor_name + ".i2c_bus", 1);
    int i2c_address =
      this->declare_parameter<int>(tof_sensor_name + ".i2c_address", 0);
    tof_sensors_info[tof_sensor_name] = std::make_tuple(i2c_bus, i2c_address);

    std::string topic_name = this->declare_parameter<std::string>(
      tof_sensor_name + ".topic", "");
    tof_sensor_topics_[tof_sensor_name] = topic_name;
  }
  mCarInterfaceComponent->setToFSensorsInfo(tof_sensors_info);

  std::ostringstream log_stream;
  log_stream << "CarInterfaceComponent offset parameters:\n";
  // Rear axle to base
  auto vector3_param = get_vector3_param(this, "rear_axle_frame_to_base_frame_offset");
  if (!vector3_param && mUrdfModel) {
    vector3_param = getFramePositionOffset(mUrdfModel, base_frame_, rear_axle_frame_);
  }
  log_stream << "rear_axle_frame_to_base_frame_offset: " << vector3_param->c_str() << "\n";
  mCarInterfaceComponent->setRearAxleToBaseOffset(vector3_param->to_type<xyz_t>());

  // Rear axle to center
  vector3_param = get_vector3_param(this, "rear_axle_frame_to_center_frame_offset");
  if (!vector3_param && mUrdfModel) {
    vector3_param = getFramePositionOffset(mUrdfModel, chassis_frame_, rear_axle_frame_);
  }
  log_stream << "rear_axle_frame_to_center_frame_offset: " << vector3_param->c_str() << "\n";
  mCarInterfaceComponent->setRearAxleToCenterOffset(vector3_param->to_type<xyz_t>());

  // Rear axle to rear end
  vector3_param = get_vector3_param(this, "rear_axle_frame_to_rear_end_frame_offset");
  if (!vector3_param && mUrdfModel) {
    vector3_param = getFramePositionOffset(mUrdfModel, rear_end_frame_, rear_axle_frame_);
  }
  log_stream << "rear_axle_frame_to_rear_end_frame_offset: " << vector3_param->c_str() << "\n";
  mCarInterfaceComponent->setRearAxleToRearEndOffset(vector3_param->to_type<xyz_t>());

  // Output log_stream
  RCLCPP_INFO_STREAM(get_logger(), log_stream.str());

  vector3_param = get_vector3_param(this, "enuref");
  if (vector3_param) {
    enuref_ = vector3_param->to_type<llh_t>();
  }
  mCarState->setEnuRef(enuref_);

  if (enable_autopilot_component_) {
    waypoint_follower_bypass_mux_ = declare_parameter("waypoint_follower_bypass_mux", false);
    mCarAutopilotComponent->setAutopilotTimerRate(declare_parameter("autopilot_timer_rate", 10));
    mCarAutopilotComponent->setEnableMavlinkInterface(
      declare_parameter("enable_mavlink_interface", true));
    mCarAutopilotComponent->setWaywiseControlTowerAddress(
      declare_parameter("waywise_control_tower_address", "127.0.0.1"));
    mCarAutopilotComponent->setWaywiseControlTowerPort(
      declare_parameter("waywise_control_tower_port", 14540));
    mCarAutopilotComponent->setPurePursuitRadius(declare_parameter("purepursuit_radius", 1.0));
    mCarAutopilotComponent->setEndGoalAlignmentType(
      static_cast<AutopilotEndGoalAlignmentType>(declare_parameter("end_goal_alignment_type", 0)));
    mCarAutopilotComponent->setEndGoalAlignmentThreshold(
      declare_parameter("end_goal_alignment_threshold", 0.1));
    mCarAutopilotComponent->setPositionAccuracyThresholdForMission(
      declare_parameter("position_accuracy_threshold_for_mission", 0.05));
    mCarAutopilotComponent->setYawAccuracyThresholdForMission(
      declare_parameter("yaw_accuracy_threshold_for_mission", 1.0));
    mCarAutopilotComponent->setAdaptiveApproachSpeedEnabled(
      declare_parameter("adaptive_approach_speed_enabled", true));
    auto min_approach_speed = declare_parameter("min_approach_speed", -1.0);
    min_approach_speed = std::max(min_approach_speed, min_target_speed);
    mCarAutopilotComponent->setMinApproachSpeed(min_approach_speed);
  }
}

void WaywiserCar::setup_publishers()
{
  // Publishers
  if (publish_odom_to_baselink_tf_ || publish_world_to_odom_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  switch (mCarInterfaceComponent->getVehicleInterfaceType()) {
    case VehicleInterfaceType::VESC:
      {
        battery_state_pub_ =
          create_publisher<waywiser_core::msg::BatteryState>(battery_state_topic_, 10);
        QObject::connect(
          mCarInterfaceComponent.get(), &CarInterfaceComponent::battery_voltage_received,
          [&](const double voltage) {

            int8_t battery_state = waywiser_core::msg::BatteryState::NORMAL;
            if (mCarInterfaceComponent->getMinBatteryVoltage() > 0.0 &&
            voltage < mCarInterfaceComponent->getMinBatteryVoltage())
            {
              RCLCPP_WARN(
                get_logger(),
                "Battery voltage is low: %f V. Please recharge the battery!", voltage);
              battery_state = waywiser_core::msg::BatteryState::LOW_VOLTAGE;

              if (!mEmergencyStopState->is_active()) {
                auto emergency_stop_msg = waywiser_twist_safety::msg::EmergencyStopState();
                emergency_stop_msg.state = waywiser_twist_safety::msg::EmergencyStopState::ACTIVE;
                emergency_stop_msg.sender_id = this->get_name();
                emergency_stop_update_pub_->publish(emergency_stop_msg);
              }
            }
            waywiser_core::msg::BatteryState battery_state_msg;
            battery_state_msg.voltage = voltage;
            battery_state_msg.state = battery_state;
            battery_state_pub_->publish(battery_state_msg);
          });
        [[fallthrough]];
      }
    case VehicleInterfaceType::WAYWISE_SIMULATED:
      {
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

        QObject::connect(
          mCarInterfaceComponent->getMovementController().get(),
          &MovementController::updatedOdomPositionAndYaw,
          [&](QSharedPointer<VehicleState> vehicleState, double distanceMoved) {
            Q_UNUSED(vehicleState)
            Q_UNUSED(distanceMoved)

            // -- Publish Odom
            publish_odom();
          });
      } break;
    default:
      break;
  }

  vehicle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    vehicle_pose_topic_, 10);
  emergency_stop_update_pub_ = create_publisher<waywiser_twist_safety::msg::EmergencyStopState>(
    emergency_stop_update_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);
  car_control_command_pub_ = create_publisher<waywiser_core::msg::CarControlCommand>(
    car_control_command_topic_, 10);
  for (const auto & tof_sensor_name : tof_sensor_names_) {
    tof_pubs_[tof_sensor_name] =
      create_publisher<std_msgs::msg::Float32>(tof_sensor_topics_[tof_sensor_name], 10);
  }
  QObject::connect(
    mCarInterfaceComponent.get(), &CarInterfaceComponent::tof_distance_received,
    [&](const std::string sensor_name, double distance) {
      std_msgs::msg::Float32 distance_msg;
      distance_msg.data = distance;
      tof_pubs_[sensor_name]->publish(distance_msg);
    });

  if (enable_autopilot_component_) {
    if (!waypoint_follower_bypass_mux_) {
      autopilot_twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "waywiser_autopilot_vel",
        10);
    }
    mission_status_pub_ =
      create_publisher<waywiser_core::msg::MissionState>(
      mission_status_topic_,
      QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);
    vehicle_alignment_reference_point_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      vehicle_alignment_reference_point_topic_, 10);
    autopilot_center_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      autopilot_center_pose_topic_, 10);
    route_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "waypoint_markers", QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);
    if (enable_visualization_msgs_) {
      autopilot_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "autopilot_markers", QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);
    }

    QObject::connect(
      mCarAutopilotComponent.get(), &CarAutopilotComponent::updatedMissionState,
      [&](MissionState state) {
        switch (state) {
          case MissionState::FollowRouteInit:
            {
              publish_route_markers();
            } break;
          case MissionState::Idle:
          case MissionState::FollowRouteFinished:
            {
              if (enable_visualization_msgs_) {
                publish_autopilot_markers();
              }
            } break;
          default:
            break;
        }
      });

    QObject::connect(
      mCarAutopilotComponent.get(), &CarAutopilotComponent::gnssFixAccuracyAssertionFailed,
      [&](GnssFixStatus gnssFixStatus) {
        if (!mEmergencyStopState->is_active()) {
          std::stringstream emergency_stop_reason;
          emergency_stop_reason << "GNSS accuracy dropped below thresholds: " <<
            std::fixed << std::setprecision(2) << gnssFixStatus.horizontalAccuracy <<
            " m and " << std::fixed << std::setprecision(2) <<
            gnssFixStatus.headingAccuracy << " deg.";

          auto emergency_stop_msg = waywiser_twist_safety::msg::EmergencyStopState();
          emergency_stop_msg.state = waywiser_twist_safety::msg::EmergencyStopState::ACTIVE;
          emergency_stop_msg.sender_id = this->get_name();
          emergency_stop_msg.stamp = this->get_clock()->now();
          emergency_stop_msg.reason = emergency_stop_reason.str();
          emergency_stop_update_pub_->publish(emergency_stop_msg);
        }
      });
  }

  if (enable_visualization_msgs_ && joint_states_publish_rate_ > 0) {
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("waywiser_joint_states", 10);
  }
}

void WaywiserCar::setup_subscribers()
{
  // Subscribers
  switch (mCarInterfaceComponent->getVehicleInterfaceType()) {
    case VehicleInterfaceType::EXT_SIMULATED:
      {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          odom_topic_, 10, std::bind(&WaywiserCar::odom_callback, this, _1));
      } break;
    default:
      break;
  }

  fused_nav_sat_fix_extended_sub_ =
    this->create_subscription<waywiser_core::msg::NavSatFixExtended>(
    fused_nav_sat_fix_extended_topic_, 10, std::bind(
      &WaywiserCar::fused_nav_sat_fix_extended_callback, this, _1));

  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&WaywiserCar::twist_callback, this, _1));

  emergency_stop_status_sub_ =
    this->create_subscription<waywiser_twist_safety::msg::EmergencyStopState>(
    emergency_stop_status_topic_,
    10, std::bind(&WaywiserCar::emergency_stop_status_callback, this, _1)
    );

  if (enable_autopilot_component_) {
    autopilot_state_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      autopilot_state_control_topic_,
      QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS,
      std::bind(&WaywiserCar::autopilot_state_control_callback, this, _1)
    );
    path_with_twists_sub_ = this->create_subscription<waywiser_core::msg::PathWithTwists>(
      "waywiser_path", QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS,
      std::bind(&WaywiserCar::path_with_twists_callback, this, _1));
  }
}

void WaywiserCar::setup_timers()
{
  // Timers
  auto timer_rate_ = std::max(
    mCarInterfaceComponent->getVehicleStatePollRate(),
    mCarAutopilotComponent->getAutopilotTimerRate());
  if (timer_rate_ > 0) {
    node_management_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(), // uses sim time if enabled
      std::chrono::milliseconds(1000 / timer_rate_),
      std::bind(&WaywiserCar::node_management_timer_callback, this)
    );
  }

  if (enable_autopilot_component_) {
    autopilot_state_machine_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(), // uses sim time if enabled
      std::chrono::milliseconds(1000 / mCarAutopilotComponent->getAutopilotTimerRate()),
      std::bind(&CarAutopilotComponent::processMissionStateMachine, mCarAutopilotComponent)
    );
  }
}

// ----------------- Callback methods -----------------
void WaywiserCar::node_management_timer_callback()
{
  static auto previousTimeCalled = this->get_clock()->now();
  auto thisTimeCalled = this->get_clock()->now();
  double timePassedSinceLastCall_ms =
    (thisTimeCalled.nanoseconds() - previousTimeCalled.nanoseconds()) / 1e6;
  // Detect clock reset
  if (timePassedSinceLastCall_ms < 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "Clock reset detected! Resetting interface and autopilot componenets.");
    mCarAutopilotComponent->reset();
    mCarInterfaceComponent->reset();
    previousTimeCalled = thisTimeCalled;
    return;
  }

  if (mCarInterfaceComponent->getVehicleInterfaceType() == VehicleInterfaceType::EXT_SIMULATED &&
    !received_first_odom_msg_)
  {
    return;
  }

  // publish world pose
  publish_world_pose();

  // publish tfs
  if (publish_odom_to_baselink_tf_ || publish_world_to_odom_tf_) {
    publish_tfs();
  }

  if (enable_autopilot_component_) {
    // Publish ref poses (e.g., for logging) if the mission is ongoing
    if (mCarAutopilotComponent->isActive()) {
      auto vehicleAlignmentReferencePosPoint =
        mCarAutopilotComponent->getVehicleAlignmentReferencePosPoint();
      if (vehicleAlignmentReferencePosPoint) {
        // Publish autopilot twist
        auto autopilot_twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
        if (mEmergencyStopState->is_clear()) {
          auto autopilotMovementController =
            mCarAutopilotComponent->getAutopilotMovementController();
          double mDesiredSpeed = autopilotMovementController->getDesiredSpeed();                       // [m/s]
          double mDesiredSteering = autopilotMovementController->getDesiredSteering();                     // [-1.0:1.0]
          double steeringAngle_rad = mDesiredSteering * mCarState->getMaxSteeringAngle();
          if (abs(steeringAngle_rad) > mCarState->getMaxSteeringAngle()) {
            steeringAngle_rad = mCarState->getMaxSteeringAngle() *
              ((steeringAngle_rad > 0) ? 1.0 : -1.0);
          }
          double mDesiredSteeringCurvature = tan(steeringAngle_rad) /
            mCarState->getAxisDistance();
          double mDesiredAngularVelocity = -mDesiredSpeed * mDesiredSteeringCurvature;       // ω = v/r

          autopilot_twist_msg->linear.x = mDesiredSpeed;
          autopilot_twist_msg->angular.z = mDesiredAngularVelocity;

          if (waypoint_follower_bypass_mux_) {
            process_twist_msg(autopilot_twist_msg);
          } else {
            autopilot_twist_pub_->publish(*autopilot_twist_msg);
          }
        }

        geometry_msgs::msg::PoseStamped world_pose_stamped;
        world_pose_stamped.header.frame_id = world_frame_;
        world_pose_stamped.header.stamp = this->get_clock()->now();
        QSharedPointer<VehicleState> referenceVehicleState =
          mCarAutopilotComponent->getReferenceVehicleState();
        PosPoint currentVehiclePosition = referenceVehicleState->getPosition(PosType::fused);
        world_pose_stamped.pose.position.x = currentVehiclePosition.getX();
        world_pose_stamped.pose.position.y = currentVehiclePosition.getY();
        world_pose_stamped.pose.position.z = currentVehiclePosition.getHeight();
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, currentVehiclePosition.getYaw() * M_PI / 180.0);
        world_pose_stamped.pose.orientation = tf2::toMsg(orientation);
        autopilot_center_pose_pub_->publish(world_pose_stamped);

        world_pose_stamped.pose.position.x = vehicleAlignmentReferencePosPoint->getX();
        world_pose_stamped.pose.position.y = vehicleAlignmentReferencePosPoint->getY();
        vehicle_alignment_reference_point_pub_->publish(world_pose_stamped);

        if (enable_visualization_msgs_) {
          publish_autopilot_markers();
        }
      }
    }

    // -- Publish mission status --
    waywiser_core::msg::MissionState missionStateMsg;
    missionStateMsg.state = static_cast<uint8_t>(mCarAutopilotComponent->getCurrentMissionState());
    mission_status_pub_->publish(missionStateMsg);
  }

  // -- Publish Joint states
  if (joint_states_publish_rate_ > 0 && enable_visualization_msgs_) {
    publish_joint_states(timePassedSinceLastCall_ms);
  }
  previousTimeCalled = thisTimeCalled;
}

void WaywiserCar::autopilot_state_control_callback(
  const std_msgs::msg::Bool::SharedPtr bool_msg)
{
  mCarAutopilotComponent->switchAutopilot(bool_msg->data);
}

void WaywiserCar::emergency_stop_status_callback(
  const waywiser_twist_safety::msg::EmergencyStopState::SharedPtr emergency_stop_msg)
{
  if (emergency_stop_msg->state == waywiser_twist_safety::msg::EmergencyStopState::ACTIVE) {
    mCarInterfaceComponent->activate_emergency_stop(emergency_stop_msg->sender_id);
  } else if (emergency_stop_msg->state == waywiser_twist_safety::msg::EmergencyStopState::CLEAR) {
    mCarInterfaceComponent->clear_emergency_stop(emergency_stop_msg->sender_id);
  }
}

void WaywiserCar::twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  if (waypoint_follower_bypass_mux_ && mCarAutopilotComponent->isActive()) {
    mCarAutopilotComponent->stopWaypointFollower();  // autopilot is overriden by twist message
  }

  process_twist_msg(twist_msg);
}

void WaywiserCar::process_twist_msg(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  static auto previousTimeCalled = this->get_clock()->now();
  auto thisTimeCalled = this->get_clock()->now();
  double dt = (thisTimeCalled.nanoseconds() - previousTimeCalled.nanoseconds()) / 1e9;
  previousTimeCalled = thisTimeCalled;

  if (dt > 0.0) {
    mCarInterfaceComponent->updateControlCommand(*twist_msg, dt);
    mCarInterfaceComponent->executeControlCommand();
    car_control_command_pub_->publish(mCarInterfaceComponent->getCarControlCommand().to_msg());
  }
}

void WaywiserCar::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  static xyz_t rear_axle_frame_to_odom_child_frame_offset;
  if (!received_first_odom_msg_) {
    if (odom_msg->child_frame_id != rear_axle_frame_) { // rear_axle_frame_ is the vehicle reference point for waywise
      if (odom_msg->child_frame_id == base_frame_) {
        rear_axle_frame_to_odom_child_frame_offset =
          mCarInterfaceComponent->getRearAxleToBaseOffset();
      } else if (odom_msg->child_frame_id == chassis_frame_) {
        rear_axle_frame_to_odom_child_frame_offset = mCarState->getRearAxleToCenterOffset();
      } else if (odom_msg->child_frame_id == rear_end_frame_) {
        rear_axle_frame_to_odom_child_frame_offset = mCarState->getRearAxleToRearEndOffset();
      } else if (mUrdfModel) {
        rear_axle_frame_to_odom_child_frame_offset = getFramePositionOffset(
          mUrdfModel, odom_msg->child_frame_id, rear_axle_frame_).to_type<xyz_t>();
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

  update_pospoint_from_pose(
    mCarState, rear_axle_frame_to_odom_child_frame_offset, odom_msg->pose.pose, PosType::odom);

  geometry_msgs::msg::Twist current_twist = odom_msg->twist.twist;
  mCarState->setVelocity(
    xyz_t{current_twist.linear.x, current_twist.linear.y, current_twist.linear.z});

  static float min_linear_speed = mCarInterfaceComponent->getErpmMin() /
    mCarInterfaceComponent->getSpeedToRPMFactor();
  double steering = 0.0;
  if (fabs(mCarState->getSpeed()) >= min_linear_speed) {
    // NOTE / TODO: WayWise has a sign error here (curvature in wrong direction)
    float steering_curvature = -(current_twist.angular.z / mCarState->getSpeed());    // ω = v/r => 1/r = ω/v
    steering = atan(mCarState->getAxisDistance() * steering_curvature) /
      mCarState->getMaxSteeringAngle();
  } else {
    static float max_linear_speed = mCarInterfaceComponent->getErpmMax() /
      mCarInterfaceComponent->getSpeedToRPMFactor();
    static float max_angular_speed = fabs(
      tan(mCarState->getMaxSteeringAngle()) * max_linear_speed / mCarState->getAxisDistance());   // rad/s
    steering = (current_twist.angular.z / max_angular_speed);
  }
  if (invert_steering_feedback_from_odom_) {
    steering = -steering;
  }
  mCarState->setSteering(steering);
}

void WaywiserCar::path_with_twists_callback(const waywiser_core::msg::PathWithTwists::SharedPtr msg)
{
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_ = msg->path.poses;
  std::vector<geometry_msgs::msg::Twist> twists_ = msg->twists;

  QList<PosPoint> waypointList;
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    PosPoint currentPoint;
    currentPoint.setX(waypoints_[i].pose.position.x);
    currentPoint.setY(waypoints_[i].pose.position.y);
    currentPoint.setSpeed(twists_[i].linear.x);

    waypointList.append(currentPoint);
  }
  mCarAutopilotComponent->updateWaypointFollowerRoute(waypointList);
}

void WaywiserCar::fused_nav_sat_fix_extended_callback(
  const waywiser_core::msg::NavSatFixExtended::SharedPtr msg)
{
  static xyz_t nav_sat_frame_to_rear_axle_frame_offset;
  static bool nav_sat_frame_to_rear_axle_frame_offset_available;
  if (!nav_sat_frame_to_rear_axle_frame_offset_available) {
    if (msg->header.frame_id != rear_axle_frame_) { // rear_axle_frame_ is the vehicle reference point for waywise
      if (msg->header.frame_id == base_frame_) {
        nav_sat_frame_to_rear_axle_frame_offset =
          mCarInterfaceComponent->getRearAxleToBaseOffset();
      } else if (msg->header.frame_id == chassis_frame_) {
        nav_sat_frame_to_rear_axle_frame_offset = mCarState->getRearAxleToCenterOffset();
      } else if (msg->header.frame_id == rear_end_frame_) {
        nav_sat_frame_to_rear_axle_frame_offset = mCarState->getRearAxleToRearEndOffset();
      } else if (mUrdfModel) {
        nav_sat_frame_to_rear_axle_frame_offset = getFramePositionOffset(
          mUrdfModel, msg->header.frame_id, rear_axle_frame_).to_type<xyz_t>();
      } else {
        static bool transform_warning_logged_ = false;
        try {
          geometry_msgs::msg::TransformStamped rear_axle_frame_to_nav_sat_frame_msg_tfs =
            tf_buffer_->lookupTransform(
            rear_axle_frame_, msg->header.frame_id, tf2::TimePointZero);
          nav_sat_frame_to_rear_axle_frame_offset = {
            rear_axle_frame_to_nav_sat_frame_msg_tfs.transform.translation.x,
            rear_axle_frame_to_nav_sat_frame_msg_tfs.transform.translation.y,
            rear_axle_frame_to_nav_sat_frame_msg_tfs.transform.translation.z
          };
          if (transform_warning_logged_) {
            RCLCPP_INFO(
              get_logger(), "Transform from %s to %s is available now.",
              msg->header.frame_id.c_str(), rear_axle_frame_.c_str());
            transform_warning_logged_ = false;
          }
        } catch (tf2::TransformException & ex) {
          if (!transform_warning_logged_) {
            RCLCPP_WARN(
              get_logger(), "Transform from %s to %s not available yet!",
              msg->header.frame_id.c_str(), rear_axle_frame_.c_str());
            transform_warning_logged_ = true;
          }
          return;
        }
      }
      nav_sat_frame_to_rear_axle_frame_offset_available = true;
    }
  }

  xyz_t xyz = coordinateTransforms::llhToEnu(
    mCarState->getEnuRef(),
    {msg->latitude, msg->longitude, msg->altitude});
  PosPoint posPoint = mCarState->getPosition(PosType::fused);
  posPoint.setXYZ(xyz);
  posPoint.updateWithOffsetAndYawRotation(
    -nav_sat_frame_to_rear_axle_frame_offset,
    coordinateTransforms::yawNEDtoENU(msg->heading) * M_PI / 180.0);
  posPoint.setTime(
    QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
  mCarState->setPosition(posPoint);

  GnssFixStatus gnssFixStatus;
  gnssFixStatus.isFusedOnChip = msg->is_fused_on_chip;
  gnssFixStatus.fixType = static_cast<GNSS_FIX_TYPE>(msg->fix_type);
  gnssFixStatus.horizontalAccuracy = msg->horizontal_accuracy;
  gnssFixStatus.verticalAccuracy = msg->vertical_accuracy;
  gnssFixStatus.headingAccuracy = msg->heading_accuracy;
  gnssFixStatus.lastRtcmCorrectionAge = msg->last_rtcm_correction_age;
  gnssFixStatus.numSatellites = msg->num_satellites;
  mCarAutopilotComponent->setGnssFixStatus(gnssFixStatus);
}

// ----------------- Publish helper methods -----------------
void WaywiserCar::publish_odom()
{
  PosPoint odom_position = mCarState->getPosition(PosType::odom);

  double x_ = odom_position.getX();
  double y_ = odom_position.getY();
  double yawRad_ = odom_position.getYaw() * M_PI / 180.0;
  static double previousYawRad_ = yawRad_;

  // -- Prepare odom msg
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = now();
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = rear_axle_frame_;

  // Position in the coordinate frame given by header.frame_id
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(yawRad_ / 2.0);
  odom_msg.pose.pose.orientation.w = cos(yawRad_ / 2.0);

  // Velocity in the coordinate frame given by child_frame_id
  odom_msg.twist.twist.linear.x = mCarState->getSpeed();
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = (yawRad_ - previousYawRad_) *
    mCarInterfaceComponent->getVehicleStatePollRate();
  previousYawRad_ = yawRad_;

  // publish odom
  odom_pub_->publish(odom_msg);
}

void WaywiserCar::publish_tfs()
{
  PosPoint odom_position = mCarState->getPosition(PosType::odom);

  double x_ = odom_position.getX();
  double y_ = odom_position.getY();
  double yawRad_ = odom_position.getYaw() * M_PI / 180.0;

  if (rear_axle_frame_ != base_frame_) {
    PosPoint odom_to_base_link_position = mCarState->posInVehicleFrameToPosPointENU(
      mCarInterfaceComponent->getRearAxleToBaseOffset(), PosType::odom);

    x_ = odom_to_base_link_position.getX();
    y_ = odom_to_base_link_position.getY();
  }

  // -- Prepare Transform
  auto odom_to_base_link_msg_tf = geometry_msgs::msg::Transform();
  odom_to_base_link_msg_tf.translation.x = x_;
  odom_to_base_link_msg_tf.translation.y = y_;
  odom_to_base_link_msg_tf.translation.z = 0.0;
  odom_to_base_link_msg_tf.rotation.x = 0.0;
  odom_to_base_link_msg_tf.rotation.y = 0.0;
  odom_to_base_link_msg_tf.rotation.z = sin(yawRad_ / 2.0);
  odom_to_base_link_msg_tf.rotation.w = cos(yawRad_ / 2.0);

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

    tf2::Transform odom_to_base_link_tf2_tf, map_to_base_link_tf2_tf;

    tf2::fromMsg(odom_to_base_link_msg_tf, odom_to_base_link_tf2_tf);

    PosPoint world_to_base_link_position = mCarState->posInVehicleFrameToPosPointENU(
      mCarInterfaceComponent->getRearAxleToBaseOffset(), PosType::fused);
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

    // -- Publish Transform
    tf_pub_->sendTransform(map_to_odom_msg_tfs);
  }
}

void WaywiserCar::publish_world_pose()
{
  PosPoint currentPosition = mCarState->getPosition(PosType::fused);
  geometry_msgs::msg::PoseStamped world_pose_stamped;
  world_pose_stamped.pose.position.x = currentPosition.getX();
  world_pose_stamped.pose.position.y = currentPosition.getY();
  world_pose_stamped.pose.position.z = currentPosition.getHeight();
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, currentPosition.getYaw() * M_PI / 180.0);
  world_pose_stamped.pose.orientation = tf2::toMsg(orientation);
  world_pose_stamped.header.frame_id = world_frame_;
  world_pose_stamped.header.stamp = this->get_clock()->now();
  vehicle_pose_pub_->publish(world_pose_stamped);
}

void WaywiserCar::publish_route_markers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  std::string marker_ns = std::string(this->get_name()) + "/route_markers";

  // Delete previous markers
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = this->get_clock()->now();
  marker.ns = marker_ns;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  route_marker_pub_->publish(marker_array);

  marker_array.markers.clear();
  // Create new markers
  int marker_id = 0;
  auto mWaypointList = mCarAutopilotComponent->getWaypointList();
  for (int i = 0; i < mWaypointList.size(); ++i) {
    const PosPoint & waypoint = mWaypointList.at(i);
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = marker_ns;
    marker.id = marker_id++;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = waypoint.getX();
    marker.pose.position.y = waypoint.getY();
    marker.pose.position.z = 0.0;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, waypoint.getYaw() * M_PI / 180.0);
    marker.pose.orientation = tf2::toMsg(orientation);

    auto marker_radius = mCarState->getWidth() / 5.0;
    marker.scale.x = marker_radius;
    marker.scale.y = marker_radius;
    marker.scale.z = marker_radius;
    if (i == 0) {
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;

      marker.scale.x = marker.scale.x * 4.0;
      marker.scale.y = marker.scale.y * 2.0;
    } else if (i == mWaypointList.size() - 1) {
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;

      marker.scale.x = marker.scale.x * 2.0;
      marker.scale.y = marker.scale.y * 2.0;
    } else {
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    }
    marker.color.a = 1.0f;   // Full opacity
    marker_array.markers.push_back(marker);
  }

  route_marker_pub_->publish(marker_array);
}

void WaywiserCar::publish_autopilot_markers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  std::string marker_ns = std::string(this->get_name()) + "/autopilot_markers";

  // Delete previous markers
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = this->get_clock()->now();
  marker.ns = marker_ns;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  autopilot_marker_pub_->publish(marker_array);

  marker_array.markers.clear();
  double radius = mCarState->getAutopilotRadius();
  if (mCarAutopilotComponent->getCurrentMissionState() != MissionState::Idle && radius > 0.0) {
    int marker_id = 0;
    // Create new autopilot radius marker
    visualization_msgs::msg::Marker marker_autopilot_radius;
    marker_autopilot_radius.header.frame_id = world_frame_;
    marker_autopilot_radius.header.stamp = this->get_clock()->now();
    marker_autopilot_radius.ns = marker_ns;
    marker_autopilot_radius.id = marker_id++;
    marker_autopilot_radius.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_autopilot_radius.action = visualization_msgs::msg::Marker::ADD;
    marker_autopilot_radius.pose.orientation.w = 1.0;

    marker_autopilot_radius.scale.x = mCarState->getWidth() / 10.0;

    marker_autopilot_radius.color.r = 0.0;
    marker_autopilot_radius.color.g = 0.0;
    marker_autopilot_radius.color.b = 1.0;
    marker_autopilot_radius.color.a = 1.0;   // Fully opaque

    int num_points = 36;      // number of points used to approximate the circle

    QSharedPointer<VehicleState> referenceVehicleState =
      mCarAutopilotComponent->getReferenceVehicleState();
    PosPoint currentVehiclePosition = referenceVehicleState->getPosition(PosType::fused);

    // Generate points along the circumference
    for (int i = 0; i <= num_points; ++i) {
      double angle = 2 * M_PI * i / num_points;
      geometry_msgs::msg::Point p;
      p.x = currentVehiclePosition.getX() + radius * cos(angle);
      p.y = currentVehiclePosition.getY() + radius * sin(angle);
      p.z = currentVehiclePosition.getHeight();
      marker_autopilot_radius.points.push_back(p);
    }
    marker_array.markers.push_back(marker_autopilot_radius);

    // Create target pose marker
    visualization_msgs::msg::Marker marker_target_pose;
    marker_target_pose.header.frame_id = world_frame_;
    marker_target_pose.header.stamp = this->get_clock()->now();
    marker_target_pose.ns = marker_ns;
    marker_target_pose.id = marker_id++;
    marker_target_pose.type = visualization_msgs::msg::Marker::SPHERE;
    marker_target_pose.action = visualization_msgs::msg::Marker::ADD;
    QPointF targetPoseXY = mCarState->getAutopilotTargetPoint();
    marker_target_pose.pose.position.x = targetPoseXY.x();
    marker_target_pose.pose.position.y = targetPoseXY.y();
    marker_target_pose.pose.position.z = currentVehiclePosition.getHeight();
    marker_target_pose.pose.orientation.w = 1.0;
    marker_target_pose.scale.x = mCarState->getWidth() / 2.0;
    marker_target_pose.scale.y = mCarState->getWidth() / 2.0;
    marker_target_pose.scale.z = mCarState->getWidth() / 2.0;
    marker_target_pose.color.r = 1.0f;
    marker_target_pose.color.g = 0.0f;
    marker_target_pose.color.b = 0.0f;
    marker_target_pose.color.a = 1.0f;   // Full opacity
    marker_array.markers.push_back(marker_target_pose);

    autopilot_marker_pub_->publish(marker_array);
  }
}

void WaywiserCar::publish_joint_states(double timePassedSinceLastCall_ms)
{
  static double timePassedSinceLastUpdate_ms = 0.0;
  timePassedSinceLastUpdate_ms += timePassedSinceLastCall_ms;
  if (timePassedSinceLastUpdate_ms >= 1000.0 / joint_states_publish_rate_) {
    sensor_msgs::msg::JointState joint_state_msg;
    update_joint_states_msg(joint_state_msg, timePassedSinceLastUpdate_ms);
    joint_state_pub_->publish(joint_state_msg);
    timePassedSinceLastUpdate_ms = 0.0;
  }
}

// ----------------- Utility methods -----------------

double WaywiserCar::update_joint_states_msg(
  sensor_msgs::msg::JointState & joint_state_msg,
  double timePassedSinceLastUpdate_ms)
{
  static double wheel_position = 0.0;

  joint_state_msg.header.stamp = this->now();

  // Calculate wheel speed and steering angle
  double wheel_rad_per_sec = (60.0 / M_PI) * mCarState->getSpeed() / wheel_diameter_;
  double steeringAngle_rad = mCarState->getSteering() * mCarState->getMaxSteeringAngle();
  if (abs(steeringAngle_rad) > mCarState->getMaxSteeringAngle()) {
    steeringAngle_rad = mCarState->getMaxSteeringAngle() * ((steeringAngle_rad > 0) ? 1.0 : -1.0);
  }

  if (invert_steering_joint_state_) {
    steeringAngle_rad = -steeringAngle_rad;
  }

  wheel_position += wheel_rad_per_sec * timePassedSinceLastUpdate_ms / 1000.0;
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
