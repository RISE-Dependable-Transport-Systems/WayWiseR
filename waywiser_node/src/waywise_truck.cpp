#include "waywise_truck.hpp"
#include "moc_waywise_truck.cpp"

#include "mavsdk/mavsdk.h" // used for setting trailer componet id: MAV_COMP_ID_USER1

void WayWiseTruck::setup_parameters()
{
  // Call base class setup_parameters
  WayWiseCar::setup_parameters();

  // Additional parameters for truck

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


  has_trailer_ = this->declare_parameter("has_trailer", false);
  if (has_trailer_) {
    trailer_base_frame_ = declare_parameter("trailer_base_frame_", "trailer");

    trailer_length_ = this->declare_parameter("trailer_length", 10.0);
    trailer_width_ = this->declare_parameter("trailer_width", 6.0);
    trailer_wheelbase_ = this->declare_parameter("trailer_wheelbase", 8.0);

    angle_sensor_offset_ = this->declare_parameter("angle_sensor_offset", 0.0);
    angle_sensor_topic_ = this->declare_parameter("angle_sensor_topic", "/sensors/angle");
  }
}

void WayWiseTruck::setup_publishers()
{
  // Call base class setup_publishers
  WayWiseCar::setup_publishers();

  // Additional publishers for truck
  for (const auto & tof_sensor_name : tof_sensor_names) {
    tof_sensors_[tof_sensor_name].publisher =
      this->create_publisher<std_msgs::msg::Float32>(tof_sensors_[tof_sensor_name].topic_name, 10);
  }

  if (has_trailer_) {
    angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(angle_sensor_topic_, 10);
  }
}

void WayWiseTruck::setup_subscribers()
{
  // Call base class setup_subscribers
  WayWiseCar::setup_subscribers();
}

void WayWiseTruck::setup_timers()
{
  // Call base class setup_timers
  WayWiseCar::setup_timers();
}

void WayWiseTruck::setup_hardware()
{
  mTruckState.reset(new TruckState);

  // Additional setup for truck
  geometry_msgs::msg::TransformStamped transformStamped;
  if (has_trailer_) {
    mTrailerState.reset(new TrailerState((int) MAV_COMP_ID_USER1, Qt::white));
    mTrailerState->setLength(trailer_length_);
    mTrailerState->setWidth(trailer_width_);
    mTrailerState->setWheelBase(trailer_wheelbase_);

    mTruckState->setTrailingVehicle(mTrailerState);

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
    mTruckState->setSimulateTrailer(!mAngleSensorUpdater->isConnected());
  }

  // Call base class setup_hardware with mTruckState
  WayWiseCar::setup_hardware(mTruckState);
}

void WayWiseTruck::publish_odom_and_tf(double timePassed_ms)
{
  WayWiseCar::publish_odom_and_tf(timePassed_ms);

  PosPoint currentPosition = mTruckState->getPosition(waywise_posType_used_);

  double truck_x = currentPosition.getX();
  double truck_y = currentPosition.getY();
  double truck_yaw_rad = currentPosition.getYaw() * M_PI / 180.0;

  if (publish_odom_to_baselink_tf_ && has_trailer_) {
    // -- Calculate and publish Trailer transform and angle sensor data
    double trailer_angle_rad = mTruckState->getTrailerAngleRadians();
    double trailer_x = truck_x - cos(truck_yaw_rad + trailer_angle_rad) * trailer_wheelbase_;
    double trailer_y = truck_y - sin(truck_yaw_rad + trailer_angle_rad) * trailer_wheelbase_;
    double trailer_yaw_rad = truck_yaw_rad + trailer_angle_rad;

    auto trailer_tf = geometry_msgs::msg::TransformStamped();
    trailer_tf.header.frame_id = odom_frame_;
    trailer_tf.child_frame_id = trailer_base_frame_;
    trailer_tf.header.stamp = now();
    trailer_tf.transform.translation.x = trailer_x;
    trailer_tf.transform.translation.y = trailer_y;
    trailer_tf.transform.translation.z = 0.0;
    trailer_tf.transform.rotation.z = sin(trailer_yaw_rad / 2.0);
    trailer_tf.transform.rotation.w = cos(trailer_yaw_rad / 2.0);

    tf_pub_->sendTransform(trailer_tf);

    publish_trailer_angle();
  }
}

void WayWiseTruck::publish_trailer_angle()
{
  std_msgs::msg::Float32 angle_msg;
  angle_msg.data = mTruckState->getTrailerAngleDegrees();
  angle_pub_->publish(angle_msg);
}

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
