#include "waywiser_truck_node_core.hpp"
#include "moc_waywiser_truck_node_core.cpp"

void WaywiserTruck::initialize_node()
{
  mTruckState.reset(new TruckState());
  has_trailer_ = this->declare_parameter("has_trailer", false);
  mTruckInterfaceComponent.reset(
    new TruckInterfaceComponent(this, mTruckState, has_trailer_, false)
  );
  enable_autopilot_component_ = declare_parameter("enable_autopilot_component", true);
  if (enable_autopilot_component_) {
    mTruckAutopilotComponent.reset(new TruckAutopilotComponent(this, mTruckState));
  }

  if (has_trailer_) {
    mTruckInterfaceComponent->setTrailerMavlinkComponentID(MAV_COMP_ID_USER1);
  }

  WaywiserCar::initialize_node(mTruckState, mTruckInterfaceComponent, mTruckAutopilotComponent);
}

void WaywiserTruck::setup_parameters()
{
  // ROS parameters
  WaywiserCar::setup_parameters();

  if (has_trailer_) {
    trailer_wheel_joint_names_ = declare_parameter<std::vector<std::string>>(
      "trailer_wheel_joint_names",
      std::vector<std::string>{"semitrailer_rlw_link_joint", "semitrailer_rrw_link_joint"}
    );
    truck_trailer_link_joint_name_ = declare_parameter<std::string>(
      "truck_trailer_link_joint_name", "truck_trailer_link_joint"
    );

    hitch_frame_ = declare_parameter("hitch_frame", base_frame_);
    trailer_base_frame_ = declare_parameter("trailer_base_frame", "trailer");
    trailer_rear_axle_frame_ = this->declare_parameter(
      "trailer_rear_axle_frame", trailer_base_frame_);
    trailer_center_frame_ = this->declare_parameter("trailer_center_frame", trailer_base_frame_);
    trailer_rear_end_frame_ = this->declare_parameter("trailer_rear_end_frame", "");
    trailer_hitch_frame_ = this->declare_parameter("trailer_hitch_frame", "");

    angle_sensor_topic_ = this->declare_parameter("angle_sensor_topic", "/sensors/angle");
    trailer_pose_topic_ = declare_parameter("trailer_pose_topic", "/trailer_pose");

    invert_trailer_joint_state_ = this->declare_parameter("invert_trailer_joint_state", false);

    // Setup component parameters
    mTruckInterfaceComponent->setTrailerLength(declare_parameter("trailer_length", 0.96));
    mTruckInterfaceComponent->setTrailerWidth(declare_parameter("trailer_width", 0.21));
    mTruckInterfaceComponent->setTrailerWheelbase(declare_parameter("trailer_wheelbase", 0.64));
    mTruckInterfaceComponent->setAngleSensorOffset(declare_parameter("angle_sensor_offset", 0.0));

    mTruckAutopilotComponent->setPurePursuitForwardGain(
      declare_parameter(
        "purepursuit_forward_gain",
        1.0));
    mTruckAutopilotComponent->setPurePursuitReverseGain(
      declare_parameter(
        "purepursuit_reverse_gain",
        -1.0));

    std::ostringstream log_stream;
    log_stream << "TruckInterfaceComponent offset parameters:\n";
    // Rear axle to hitch
    auto vector3_param = get_vector3_param(this, "rear_axle_frame_to_hitch_frame_offset");
    if (!vector3_param && mUrdfModel) {
      vector3_param = getFramePositionOffset(mUrdfModel, hitch_frame_, rear_axle_frame_);
    }
    log_stream << "rear_axle_frame_to_hitch_frame_offset: " << vector3_param->c_str() << "\n";
    mTruckInterfaceComponent->setRearAxleToHitchOffset(vector3_param->to_type<xyz_t>());

    // Trailer rear axle to trailer base
    vector3_param = get_vector3_param(this, "trailer_rear_axle_frame_to_trailer_base_frame_offset");
    if (!vector3_param && mUrdfModel) {
      vector3_param =
        getFramePositionOffset(mUrdfModel, trailer_base_frame_, trailer_rear_axle_frame_);
    }
    log_stream << "trailer_rear_axle_frame_to_trailer_base_frame_offset: " <<
      vector3_param->c_str() << "\n";
    mTruckInterfaceComponent->setTrailerRearAxleToTrailerBaseOffset(vector3_param->to_type<xyz_t>());

    // Trailer rear axle to trailer rear end
    vector3_param =
      get_vector3_param(this, "trailer_rear_axle_frame_to_trailer_rear_end_frame_offset");
    if (!vector3_param && mUrdfModel) {
      vector3_param = getFramePositionOffset(
        mUrdfModel, trailer_rear_end_frame_,
        trailer_rear_axle_frame_);
    }
    log_stream << "trailer_rear_axle_frame_to_trailer_rear_end_frame_offset: " <<
      vector3_param->c_str() << "\n";
    mTruckInterfaceComponent->setTrailerRearAxleToTrailerRearEndOffset(
      vector3_param->to_type<xyz_t>());

    // Trailer rear axle to trailer center
    vector3_param =
      get_vector3_param(this, "trailer_rear_axle_frame_to_trailer_center_frame_offset");
    if (!vector3_param && mUrdfModel) {
      vector3_param = getFramePositionOffset(
        mUrdfModel, trailer_center_frame_,
        trailer_rear_axle_frame_);
    }
    log_stream << "trailer_rear_axle_frame_to_trailer_center_frame_offset: " <<
      vector3_param->c_str() << "\n";
    mTruckInterfaceComponent->setTrailerRearAxleToTrailerCenterOffset(
      vector3_param->to_type<xyz_t>());

    // Trailer rear axle to trailer hitch
    vector3_param =
      get_vector3_param(this, "trailer_rear_axle_frame_to_trailer_hitch_frame_offset");
    if (!vector3_param && mUrdfModel) {
      vector3_param =
        getFramePositionOffset(mUrdfModel, trailer_hitch_frame_, trailer_rear_axle_frame_);
    }
    log_stream << "trailer_rear_axle_frame_to_trailer_hitch_frame_offset: " <<
      vector3_param->c_str() << "\n";
    mTruckInterfaceComponent->setTrailerRearAxleToTrailerHitchOffset(vector3_param->to_type<xyz_t>());

    // Output log_stream
    RCLCPP_INFO_STREAM(get_logger(), log_stream.str());
  }
}

void WaywiserTruck::setup_publishers()
{
  // Publishers
  WaywiserCar::setup_publishers();

  if (has_trailer_) {
    trailer_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(trailer_pose_topic_, 10);

    switch (mTruckInterfaceComponent->getVehicleInterfaceType()) {
      case VehicleInterfaceType::VESC:
      case VehicleInterfaceType::WAYWISE_SIMULATED:
        {
          angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(angle_sensor_topic_, 10);
        } break;
      default:
        break;
    }
  }
}

void WaywiserTruck::setup_subscribers()
{
  // Subscribers
  WaywiserCar::setup_subscribers();

  if (has_trailer_) {
    switch (mTruckInterfaceComponent->getVehicleInterfaceType()) {
      case VehicleInterfaceType::EXT_SIMULATED:
        {
          angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            angle_sensor_topic_, 10,
            [this](const std_msgs::msg::Float32::SharedPtr angle_msg) {
              mTruckState->setTrailerAngle(angle_msg->data);
            });
        } break;
      default:
        break;
    }
  }
}

void WaywiserTruck::setup_timers()
{
  // Timers
  WaywiserCar::setup_timers();
}

// ----------------- Callback methods -----------------
void WaywiserTruck::node_management_timer_callback()
{
  WaywiserCar::node_management_timer_callback();

  if (has_trailer_) {
    switch (mTruckInterfaceComponent->getVehicleInterfaceType()) {
      case VehicleInterfaceType::VESC:
      case VehicleInterfaceType::WAYWISE_SIMULATED:
        {
          publish_trailer_angle();
        } break;
      default:
        break;
    }
  }
}

// ----------------- Publish helper methods -----------------
void WaywiserTruck::publish_tfs()
{

  WaywiserCar::publish_tfs();

  if (has_trailer_) {
    if (publish_odom_to_baselink_tf_) {
      PosPoint odom_to_trailer_base_link_position =
        mTruckState->getTrailingVehicle()->posInVehicleFrameToPosPointENU(
        mTruckInterfaceComponent->getTrailerRearAxleToTrailerBaseOffset(), PosType::odom);

      double trailer_x = odom_to_trailer_base_link_position.getX();
      double trailer_y = odom_to_trailer_base_link_position.getY();
      double trailer_yaw_rad = odom_to_trailer_base_link_position.getYaw() * M_PI / 180.0;

      auto odom_to_trailer_msg_tf = geometry_msgs::msg::Transform();
      odom_to_trailer_msg_tf.translation.x = trailer_x;
      odom_to_trailer_msg_tf.translation.y = trailer_y;
      odom_to_trailer_msg_tf.rotation.z = sin(trailer_yaw_rad / 2.0);
      odom_to_trailer_msg_tf.rotation.w = cos(trailer_yaw_rad / 2.0);

      auto odom_to_trailer_msg_tfs = geometry_msgs::msg::TransformStamped();
      odom_to_trailer_msg_tfs.header.frame_id = odom_frame_;
      odom_to_trailer_msg_tfs.child_frame_id = trailer_base_frame_;
      odom_to_trailer_msg_tfs.header.stamp = now();
      odom_to_trailer_msg_tfs.transform = odom_to_trailer_msg_tf;

      // -- Publish Transform
      tf_pub_->sendTransform(odom_to_trailer_msg_tfs);
    }
  }
}

void WaywiserTruck::publish_world_pose()
{
  WaywiserCar::publish_world_pose();

  if (has_trailer_) {
    geometry_msgs::msg::PoseStamped world_pose_stamped;
    world_pose_stamped.header.frame_id = world_frame_;
    world_pose_stamped.header.stamp = this->get_clock()->now();

    PosPoint currentTrailerPosition =
      mTruckState->getTrailingVehicle()->getPosition(PosType::fused);
    world_pose_stamped.pose.position.x = currentTrailerPosition.getX();
    world_pose_stamped.pose.position.y = currentTrailerPosition.getY();
    world_pose_stamped.pose.position.z = currentTrailerPosition.getHeight();
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, currentTrailerPosition.getYaw() * M_PI / 180.0);
    world_pose_stamped.pose.orientation = tf2::toMsg(orientation);
    trailer_pose_pub_->publish(world_pose_stamped);
  }
}

void WaywiserTruck::publish_trailer_angle()
{
  std_msgs::msg::Float32 angle_msg;
  angle_msg.data = mTruckState->getTrailerAngleDegrees();
  angle_pub_->publish(angle_msg);
}

// ----------------- Utility methods -----------------
double WaywiserTruck::update_joint_states_msg(
  sensor_msgs::msg::JointState & joint_state_msg, double timePassedSinceLastCall_ms)
{
  double wheel_position = WaywiserCar::update_joint_states_msg(
    joint_state_msg, timePassedSinceLastCall_ms);

  if (has_trailer_) {
    double wheel_rad_per_sec = (30.0 / M_PI) * mTruckState->getSpeed() *
      mTruckInterfaceComponent->getSpeedToRPMFactor();

    for (const auto & wheel_joint_name : trailer_wheel_joint_names_) {
      joint_state_msg.name.push_back(wheel_joint_name);
      joint_state_msg.position.push_back(wheel_position); // Wheel position in radians
      joint_state_msg.velocity.push_back(wheel_rad_per_sec); // Wheel velocity in rad/s
      joint_state_msg.effort.push_back(0.0); // Effort (not used here)
    }

    joint_state_msg.name.push_back(truck_trailer_link_joint_name_);
    joint_state_msg.position.push_back(
      invert_trailer_joint_state_ ? -mTruckState->getTrailerAngleRadians() :
      mTruckState->getTrailerAngleRadians()
    );
    joint_state_msg.velocity.push_back(0.0);   // Velocity in rad/s
    joint_state_msg.effort.push_back(0.0);   // Effort (not used here)
  }

  return wheel_position;
}
