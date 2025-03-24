#include "waywise_truck.hpp"
#include "moc_waywise_truck.cpp"

#include "mavsdk/mavsdk.h" // used for setting trailer componet id: MAV_COMP_ID_USER1

void WayWiseTruck::setup_parameters()
{
  // Call base class setup_parameters
  WayWiseCar::setup_parameters();

  // Additional parameters for truck
  hitch_frame_ = declare_parameter("hitch_frame", base_frame_);
  rear_axle_frame_to_hitch_frame_offset_ = declare_parameter(
    "rear_axle_frame_to_hitch_frame_offset", std::vector<double>({0.0}));

  #ifdef WAYWISE_HW_INTERFACE_
  // ToF Sensors
  tof_sensor_names = this->declare_parameter<std::vector<std::string>>(
    "tof_sensors", {}, rcl_interfaces::msg::ParameterDescriptor{});
  if (tof_sensor_names.size() > 1) {
    RCLCPP_WARN(
      this->get_logger(),
      "More than one ToF sensor is not currently supported. "
      "Only the first sensor will be used: '%s'. Ignoring others.",
      tof_sensor_names[0].c_str());

    tof_sensor_names.resize(1);   // TODO: enable setting multiple tof sensors
  }

  for (const auto & tof_sensor_name : tof_sensor_names) {
    int i2c_addr =
      this->declare_parameter<int>(tof_sensor_name + ".i2c_addr", 0);
    std::string topic_name = this->declare_parameter<std::string>(
      tof_sensor_name + ".topic", "");

    ToFSensorInfo tof_sensor_info;
    tof_sensor_info.i2c_addr = i2c_addr;
    tof_sensor_info.topic_name = topic_name;
    tof_sensors_[tof_sensor_name] = tof_sensor_info;
  }
  #endif


  has_trailer_ = this->declare_parameter("has_trailer", false);
  if (has_trailer_) {
    trailer_base_frame_ = declare_parameter("trailer_base_frame", "trailer");

    trailer_length_ = this->declare_parameter("trailer_length", 10.0);
    trailer_width_ = this->declare_parameter("trailer_width", 6.0);
    trailer_wheelbase_ = this->declare_parameter("trailer_wheelbase", 8.0);

    purepursuit_forward_gain_ = this->declare_parameter("purepursuit_forward_gain", 1.0);
    purepursuit_reverse_gain_ = this->declare_parameter("purepursuit_reverse_gain", -1.0);
    angle_sensor_topic_ = this->declare_parameter("angle_sensor_topic", "/sensors/angle");

    trailer_rear_axle_frame_ = this->declare_parameter(
      "trailer_rear_axle_frame",
      trailer_base_frame_);
    trailer_center_frame_ = this->declare_parameter("trailer_center_frame", trailer_base_frame_);
    trailer_rear_end_frame_ = this->declare_parameter("trailer_rear_end_frame", "");
    trailer_hitch_frame_ = this->declare_parameter("trailer_hitch_frame", "");

    trailer_rear_axle_frame_to_trailer_base_frame_offset_ = declare_parameter(
      "trailer_rear_axle_frame_to_trailer_base_frame_offset", std::vector<double>({0.0}));
    trailer_rear_axle_frame_to_trailer_center_frame_offset_ = declare_parameter(
      "trailer_rear_axle_frame_to_trailer_center_frame_offset", std::vector<double>({0.0}));
    trailer_rear_axle_frame_to_trailer_rear_end_frame_offset_ = declare_parameter(
      "trailer_rear_axle_frame_to_trailer_rear_end_frame_offset", std::vector<double>({0.0}));
    trailer_rear_axle_frame_to_trailer_hitch_frame_offset_ = declare_parameter(
      "trailer_rear_axle_frame_to_trailer_hitch_frame_offset", std::vector<double>({0.0}));

    trailer_wheel_joint_names_ = declare_parameter<std::vector<std::string>>(
      "trailer_wheel_joint_names",
      std::vector<std::string>{"semitrailer_rlw_link_joint", "semitrailer_rrw_link_joint"}
    );
    truck_trailer_link_joint_name_ = declare_parameter<std::string>(
      "truck_trailer_link_joint_name", "truck_trailer_link_joint"
    );
    invert_trailer_joint_state_ = this->declare_parameter("invert_trailer_joint_state", false);
    trailer_pose_topic_ = declare_parameter("trailer_pose_topic", "/trailer_pose");

    #ifdef WAYWISE_HW_INTERFACE_
    angle_sensor_offset_ = this->declare_parameter("angle_sensor_offset", 0.0);
    #endif
  }
}

void WayWiseTruck::setup_publishers()
{
  // Call base class setup_publishers
  WayWiseCar::setup_publishers();

  // Additional publishers for truck
  if (has_trailer_) {
    trailer_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(trailer_pose_topic_, 10);
  }

  #ifdef WAYWISE_HW_INTERFACE_
  for (const auto & tof_sensor_name : tof_sensor_names) {
    tof_sensors_[tof_sensor_name].publisher =
      this->create_publisher<std_msgs::msg::Float32>(tof_sensors_[tof_sensor_name].topic_name, 10);
  }

  if (has_trailer_) {
    angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(angle_sensor_topic_, 10);
  }
  #endif
}

void WayWiseTruck::setup_subscribers()
{
  // Call base class setup_subscribers
  WayWiseCar::setup_subscribers();

  // Additional subscribers for truck
  #ifndef WAYWISE_HW_INTERFACE_
  if (has_trailer_) {
    angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      angle_sensor_topic_, 10,
      std::bind(&WayWiseTruck::angle_sensor_callback, this, _1));
  }
  #endif
}

void WayWiseTruck::setup_timers()
{
  // Call base class setup_timers
  WayWiseCar::setup_timers();
}

void WayWiseTruck::setup_autopilot()
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

    // --- Set trailer rear axle offsets ---
    bool offset_params_initiazed = true;
    if (trailer_rear_axle_frame_to_trailer_base_frame_offset_.size() != 3 ||
      trailer_rear_axle_frame_to_trailer_center_frame_offset_.size() != 3 ||
      trailer_rear_axle_frame_to_trailer_rear_end_frame_offset_.size() != 3 ||
      trailer_rear_axle_frame_to_trailer_hitch_frame_offset_.size() != 3)
    {
      offset_params_initiazed = false;
    }

    if (trailer_rear_axle_frame_to_trailer_center_frame_offset_.size() == 3) {
      mTrailerState->setRearAxleToCenterOffset(
        xyz_t{
        trailer_rear_axle_frame_to_trailer_center_frame_offset_[0],
        trailer_rear_axle_frame_to_trailer_center_frame_offset_[1],
        trailer_rear_axle_frame_to_trailer_center_frame_offset_[2]
      });
    }

    if (trailer_rear_axle_frame_to_trailer_rear_end_frame_offset_.size() == 3) {
      mTrailerState->setRearAxleToRearEndOffset(
        xyz_t{
        trailer_rear_axle_frame_to_trailer_rear_end_frame_offset_[0],
        trailer_rear_axle_frame_to_trailer_rear_end_frame_offset_[1],
        trailer_rear_axle_frame_to_trailer_rear_end_frame_offset_[2]
      });
    }

    if (trailer_rear_axle_frame_to_trailer_hitch_frame_offset_.size() == 3) {
      mTrailerState->setRearAxleToHitchOffset(
        xyz_t{
        trailer_rear_axle_frame_to_trailer_hitch_frame_offset_[0],
        trailer_rear_axle_frame_to_trailer_hitch_frame_offset_[1],
        trailer_rear_axle_frame_to_trailer_hitch_frame_offset_[2]
      });
    }

    if (!offset_params_initiazed && loadURDFFile()) {
      Eigen::Vector3d offset;

      if (trailer_rear_axle_frame_to_trailer_base_frame_offset_.size() != 3) {
        offset = getLinkPosition(urdfModel, trailer_base_frame_) -
          getLinkPosition(urdfModel, trailer_rear_axle_frame_);
        trailer_rear_axle_frame_to_trailer_base_frame_offset_ = {offset.x(), offset.y(),
          offset.z()};
      }

      if (trailer_rear_axle_frame_to_trailer_center_frame_offset_.size() != 3) {
        offset = getLinkPosition(urdfModel, trailer_center_frame_) -
          getLinkPosition(urdfModel, trailer_rear_axle_frame_);
        trailer_rear_axle_frame_to_trailer_center_frame_offset_ = {offset.x(), offset.y(),
          offset.z()};
        mTrailerState->setRearAxleToCenterOffset(xyz_t{offset.x(), offset.y(), offset.z()});
      }

      if (trailer_rear_axle_frame_to_trailer_rear_end_frame_offset_.size() != 3) {
        offset = getLinkPosition(urdfModel, trailer_rear_end_frame_) -
          getLinkPosition(urdfModel, trailer_rear_axle_frame_);
        trailer_rear_axle_frame_to_trailer_rear_end_frame_offset_ = {offset.x(), offset.y(),
          offset.z()};
        mTrailerState->setRearAxleToRearEndOffset(xyz_t{offset.x(), offset.y(), offset.z()});
      }

      if (trailer_rear_axle_frame_to_trailer_hitch_frame_offset_.size() != 3) {
        offset = getLinkPosition(urdfModel, trailer_hitch_frame_) -
          getLinkPosition(urdfModel, trailer_rear_axle_frame_);
        trailer_rear_axle_frame_to_trailer_hitch_frame_offset_ = {offset.x(), offset.y(),
          offset.z()};
        mTrailerState->setRearAxleToHitchOffset(xyz_t{offset.x(), offset.y(), offset.z()});
      }
    } else {
      if (trailer_rear_axle_frame_to_trailer_base_frame_offset_.size() != 3) {
        trailer_rear_axle_frame_to_trailer_base_frame_offset_ = {0.0, 0.0, 0.0};
      }

      if (trailer_rear_axle_frame_to_trailer_center_frame_offset_.size() != 3) {
        trailer_rear_axle_frame_to_trailer_center_frame_offset_ = {0.0, 0.0, 0.0};
      }

      if (trailer_rear_axle_frame_to_trailer_rear_end_frame_offset_.size() != 3) {
        trailer_rear_axle_frame_to_trailer_rear_end_frame_offset_ = {0.0, 0.0, 0.0};
      }

      if (trailer_rear_axle_frame_to_trailer_hitch_frame_offset_.size() != 3) {
        trailer_rear_axle_frame_to_trailer_hitch_frame_offset_ = {0.0, 0.0, 0.0};
      }
    }

    mTruckState->setTrailingVehicle(mTrailerState);
  }

  // Call base class setup_hardware with mTruckState
  WayWiseCar::setup_autopilot(mTruckState);

  // Additional setup for truck
  if (rear_axle_frame_to_hitch_frame_offset_.size() == 3) {
    mTruckState->setRearAxleToHitchOffset(
      xyz_t{
      rear_axle_frame_to_hitch_frame_offset_[0],
      rear_axle_frame_to_hitch_frame_offset_[1],
      rear_axle_frame_to_hitch_frame_offset_[2]
    });
  } else if (loadURDFFile()) {
    Eigen::Vector3d offset = getLinkPosition(urdfModel, hitch_frame_) -
      getLinkPosition(urdfModel, rear_axle_frame_);
    rear_axle_frame_to_hitch_frame_offset_ = {offset.x(), offset.y(), offset.z()};
    mTruckState->setRearAxleToHitchOffset(xyz_t{offset.x(), offset.y(), offset.z()});
  } else {
    rear_axle_frame_to_hitch_frame_offset_ = {0.0, 0.0, 0.0};
  }
}

#ifdef WAYWISE_HW_INTERFACE_
void WayWiseTruck::setup_hardware()
{
  // Additional setup for truck
  if (has_trailer_) {
    // ToF Sensors
    for (const auto & tof_sensor_name : tof_sensor_names) {
      ToFSensorInfo tof_sensor_info = tof_sensors_[tof_sensor_name];
      tof_sensor_info.sensor.reset(new VL53L0XToFSensor());
      QObject::connect(
        tof_sensor_info.sensor.get(), &ToFSensor::updatedDistance, this,
        [this, tof_sensor_name](double distance) {
          updated_tof_distance_callback(tof_sensor_name, distance);
        });
      tof_sensors_[tof_sensor_name] = tof_sensor_info;
    }

    // Angle Sensor
    mAngleSensorUpdater.reset(new AS5600Updater(mTruckState, angle_sensor_offset_));
    if (!mAngleSensorUpdater->isConnected()) {
      mTruckState->setSimulateTrailer(true);
      RCLCPP_WARN(
        this->get_logger(),
        "Angle sensor not connected. Trailer angle will be simulated.");
    }
  }

  // Call base class setup_hardware with mTruckState
  WayWiseCar::setup_hardware();
}
#endif

// ----------------- Callback methods -----------------
#ifdef WAYWISE_HW_INTERFACE_
void WayWiseTruck::updated_tof_distance_callback(
  const std::string & tof_sensor_name,
  double distance_m)
{
  auto sensor_info = tof_sensors_[tof_sensor_name];

  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(distance_m);
  sensor_info.publisher->publish(msg);
  RCLCPP_INFO(
    this->get_logger(), "Published ToF distance %.2f from %s", distance_m,
    tof_sensor_name.c_str());
}
#else
void WayWiseTruck::angle_sensor_callback(const std_msgs::msg::Float32::SharedPtr angle_msg)
{
  mTruckState->setTrailerAngle(angle_msg->data);
}
#endif

// ----------------- Utility methods -----------------
void WayWiseTruck::update_world_positon(geometry_msgs::msg::Pose world_pose)
{
  WayWiseCar::update_world_positon(world_pose);

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

double WayWiseTruck::update_joint_states_msg(
  sensor_msgs::msg::JointState & joint_state_msg,
  double timePassed_ms)
{
  double wheel_position = WayWiseCar::update_joint_states_msg(joint_state_msg, timePassed_ms);

  if (has_trailer_) {
    double wheel_rad_per_sec = (30.0 / M_PI) * mTruckState->getSpeed() *
      mCarMovementController->getSpeedToRPMFactor();

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


#ifdef WAYWISE_HW_INTERFACE_
void WayWiseTruck::publish_odom_and_tfs()
{
  WayWiseCar::publish_odom_and_tfs();

  if (has_trailer_) {
    if (publish_odom_to_baselink_tf_) {
      static xyz_t trailer_rear_axle_frame_to_trailer_base_frame_offset_xyz = {
        trailer_rear_axle_frame_to_trailer_base_frame_offset_[0],
        trailer_rear_axle_frame_to_trailer_base_frame_offset_[1],
        trailer_rear_axle_frame_to_trailer_base_frame_offset_[2]
      };
      PosPoint odom_to_trailer_base_link_position = mTrailerState->posInVehicleFrameToPosPointENU(
        trailer_rear_axle_frame_to_trailer_base_frame_offset_xyz, PosType::odom);

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

    publish_trailer_angle();
  }
}

void WayWiseTruck::publish_trailer_angle()
{
  std_msgs::msg::Float32 angle_msg;
  angle_msg.data = mTruckState->getTrailerAngleDegrees();
  angle_pub_->publish(angle_msg);
}
#endif
