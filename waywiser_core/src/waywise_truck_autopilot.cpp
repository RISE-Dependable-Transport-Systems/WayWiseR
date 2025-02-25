#include "waywise_truck_autopilot.hpp"
#include "moc_waywise_truck_autopilot.cpp"

using namespace std::placeholders;

void WaywiseTruckAutopilot::setup_parameters()
{
  // Call base class setup_parameters
  WaywiseCarAutopilot::setup_parameters();

  // Additional parameters for truck
  hitch_frame_ = declare_parameter("hitch_frame", base_frame_);

  has_trailer_ = this->declare_parameter("has_trailer", false);
  if (has_trailer_) {
    trailer_length_ = this->declare_parameter("trailer_length", 10.0);
    trailer_width_ = this->declare_parameter("trailer_width", 6.0);
    trailer_wheelbase_ = this->declare_parameter("trailer_wheelbase", 8.0);
    purepursuit_forward_gain_ = this->declare_parameter("purepursuit_forward_gain", 1.0);
    purepursuit_reverse_gain_ = this->declare_parameter("purepursuit_reverse_gain", -1.0);
    angle_sensor_topic_ = this->declare_parameter("angle_sensor_topic", "/sensors/angle");

    trailer_base_frame_ = declare_parameter("trailer_base_frame", "trailer");
    trailer_rear_axle_frame_ = this->declare_parameter(
      "trailer_rear_axle_frame",
      trailer_base_frame_);
    trailer_center_frame_ = this->declare_parameter("trailer_center_frame", trailer_base_frame_);
    trailer_rear_end_frame_ = this->declare_parameter("trailer_rear_end_frame", "");
    trailer_hitch_frame_ = this->declare_parameter("trailer_hitch_frame", "");

    trailer_wheel_joint_names_ = declare_parameter<std::vector<std::string>>(
      "trailer_wheel_joint_names",
      std::vector<std::string>{"semitrailer_rlw_link_joint", "semitrailer_rrw_link_joint"}
    );
    truck_trailer_link_joint_name_ = declare_parameter<std::string>(
      "truck_trailer_link_joint_name", "truck_trailer_link_joint"
    );
    invert_trailer_joint_state_ = this->declare_parameter("invert_trailer_joint_state", false);

    trailer_pose_topic_ = declare_parameter("trailer_pose_topic", "/trailer_pose");
  }
}

void WaywiseTruckAutopilot::setup_publishers()
{
  // Call base class setup_publishers
  WaywiseCarAutopilot::setup_publishers();

  if (has_trailer_) {
    trailer_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(trailer_pose_topic_, 10);
  }
}

void WaywiseTruckAutopilot::setup_subscribers()
{
  // Call base class setup_subscribers
  WaywiseCarAutopilot::setup_subscribers();

  if (has_trailer_) {
    angle_sensor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      angle_sensor_topic_, 10,
      std::bind(&WaywiseTruckAutopilot::angle_sensor_callback, this, _1));
  }
}

void WaywiseTruckAutopilot::setup_timers()
{
  // Call base class setup_timers
  WaywiseCarAutopilot::setup_timers();
}

void WaywiseTruckAutopilot::setup_autopilot()
{
  mTruckState.reset(new TruckState);

  // Additional setup for truck
  mTruckState->setPurePursuitForwardGain(purepursuit_forward_gain_);
  mTruckState->setPurePursuitReverseGain(purepursuit_reverse_gain_);
  if (has_trailer_) {
    mTrailerMavlinkComponentID = (int) MAV_COMP_ID_USER1;

    mTrailerState.reset(new TrailerState(mTrailerMavlinkComponentID, Qt::white));
    mTrailerState->setLength(trailer_length_);
    mTrailerState->setWidth(trailer_width_);
    mTrailerState->setWheelBase(trailer_wheelbase_);

    if (loadURDFFile()) {
      Eigen::Vector3d offset = getLinkPosition(urdfModel, trailer_center_frame_) -
        getLinkPosition(urdfModel, trailer_rear_axle_frame_);
      mTrailerState->setRearAxleToCenterOffset(xyz_t{offset.x(), offset.y(), offset.z()});

      offset = getLinkPosition(urdfModel, trailer_rear_end_frame_) -
        getLinkPosition(urdfModel, trailer_rear_axle_frame_);
      mTrailerState->setRearAxleToRearEndOffset(xyz_t{offset.x(), offset.y(), offset.z()});

      offset = getLinkPosition(urdfModel, trailer_hitch_frame_) -
        getLinkPosition(urdfModel, trailer_rear_axle_frame_);
      mTrailerState->setRearAxleToHitchOffset(xyz_t{offset.x(), offset.y(), offset.z()});
    }

    mTruckState->setTrailingVehicle(mTrailerState);
  }

  // Call base class setup_hardware with mTruckState
  WaywiseCarAutopilot::setup_autopilot(mTruckState);

  // Additional setup for truck
  if (loadURDFFile()) {
    Eigen::Vector3d offset = getLinkPosition(urdfModel, hitch_frame_) -
      getLinkPosition(urdfModel, rear_axle_frame_);
    mTruckState->setRearAxleToHitchOffset(xyz_t{offset.x(), offset.y(), offset.z()});
  }
}

void WaywiseTruckAutopilot::angle_sensor_callback(const std_msgs::msg::Float32::SharedPtr angle_msg)
{
  mTruckState->setTrailerAngle(angle_msg->data);
}

void WaywiseTruckAutopilot::update_world_positon(geometry_msgs::msg::Pose world_pose)
{
  WaywiseCarAutopilot::update_world_positon(world_pose);

  if (has_trailer_) {
    geometry_msgs::msg::PoseStamped world_pose_stamped;
    world_pose_stamped.header.frame_id = world_frame_;
    world_pose_stamped.header.stamp = this->get_clock()->now();

    PosPoint currentTrailerPosition = mTrailerState->getPosition(PosType::fused);
    world_pose_stamped.pose.position.x = currentTrailerPosition.getX();
    world_pose_stamped.pose.position.y = currentTrailerPosition.getY();
    world_pose_stamped.pose.position.z = currentTrailerPosition.getHeight();
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, currentTrailerPosition.getYaw() * M_PI / 180.0);
    world_pose_stamped.pose.orientation = tf2::toMsg(orientation);
    trailer_pose_pub_->publish(world_pose_stamped);
  }
}

double WaywiseTruckAutopilot::update_joint_states_msg(
  sensor_msgs::msg::JointState & joint_state_msg,
  double timePassed_ms)
{
  double wheel_position = WaywiseCarAutopilot::update_joint_states_msg(
    joint_state_msg, timePassed_ms);

  if (has_trailer_) {
    double wheel_rad_per_sec = (30.0 / M_PI) * mTruckState->getSpeed() *
      mCarMovementController->getSpeedToRPMFactor();

    for (const auto & wheel_joint_name : trailer_wheel_joint_names_) {
      joint_state_msg.name.push_back(wheel_joint_name);
      joint_state_msg.position.push_back(wheel_position); // Wheel position in radians
      joint_state_msg.velocity.push_back(wheel_rad_per_sec); // Wheel velocity in rad/s
      joint_state_msg.effort.push_back(0.0); // Effort (not used here)
    }
  }

  joint_state_msg.name.push_back(truck_trailer_link_joint_name_);
  joint_state_msg.position.push_back(
    invert_trailer_joint_state_ ? -mTruckState->getTrailerAngleRadians() : mTruckState->getTrailerAngleRadians()
  );
  joint_state_msg.velocity.push_back(0.0);   // Velocity in rad/s
  joint_state_msg.effort.push_back(0.0);   // Effort (not used here)

  return wheel_position;
}
