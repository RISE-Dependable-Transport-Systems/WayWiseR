#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>

#include "WayWise/core/simplewatchdog.h"
#include "WayWise/logger/logger.h"
#include "WayWise/sensors/angle/as5600updater.h"
#include "WayWise/sensors/fusion/sdvpvehiclepositionfuser.h"
#include "WayWise/sensors/gnss/rtcmclient.h"
#include "WayWise/sensors/gnss/ubloxrover.h"
#include "WayWise/sensors/imu/bno055orientationupdater.h"
#include "WayWise/sensors/imu/imuorientationupdater.h"
#include "WayWise/sensors/tof/vl53l0xtofsensor.h"
#include "WayWise/vehicles/truckstate.h"
#include "WayWise/vehicles/trailerstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/vehicles/controller/vescmotorcontroller.h"

#include <QCoreApplication>
#include <QObject>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "mavsdk/mavsdk.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

class WayWiseTruck : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  WayWiseTruck()
  : QObject(), Node("waywise_truck")
  {
    // -- ROS --
    // get ROS parameters
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    trailer_frame_ = declare_parameter("trailer_frame", "trailer");

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
    odom_publish_rate_ = this->declare_parameter("odom_publish_rate", 30);
    publish_odom_to_baselink_tf_ = this->declare_parameter("publish_odom_to_baselink_tf", true);
    imu_for_position_fusion_ = declare_parameter("imu_for_position_fusion", "");
    odom_topic_ = this->declare_parameter("odom_topic", "/odom");

    max_angular_velocity_ = this->declare_parameter("max_angular_velocity", 0.5);
    standstill_velocity_threshold_ = this->declare_parameter("standstill_velocity_threshold", 0.05);

    has_trailer_ = this->declare_parameter("has_trailer", false);

    if (has_trailer_) {
      trailer_length_ = this->declare_parameter("trailer_length", 10.0);
      trailer_width_ = this->declare_parameter("trailer_width", 6.0);
      trailer_wheelbase_ = this->declare_parameter("trailer_wheelbase", 8.0);

      angle_sensor_offset_ = this->declare_parameter("angle_sensor_offset", 0.0);
      angle_sensor_topic_ = this->declare_parameter("angle_sensor_topic", "/sensors/angle");
    }

    // publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    if (publish_odom_to_baselink_tf_) {
      tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
    }

    if (has_trailer_) {
      angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(angle_sensor_topic_, 10);
    }

    mUpdateVehicleStatePeriod =
      std::chrono::milliseconds((int)std::round(1000.0 / odom_publish_rate_));

    // subscribers
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&WayWiseTruck::twist_callback, this, _1));

    // -- WayWise --
    mTruckState.reset(new TruckState);
    mTruckState->setLength(length_);
    mTruckState->setWidth(width_);
    mTruckState->setAxisDistance(wheelbase_);
    mTruckState->setMaxSteeringAngle(atan(wheelbase_ / min_turning_radius_));
    if (has_trailer_) {
      mTrailerState.reset(new TrailerState((int) MAV_COMP_ID_USER1, Qt::white));
      mTrailerState->setLength(trailer_length_);
      mTrailerState->setWidth(trailer_width_);
      mTrailerState->setWheelBase(trailer_wheelbase_);

      mTruckState->setTrailingVehicle(mTrailerState);
    }

    // --- Lower-level control setup ---
    mCarMovementController.reset(new CarMovementController(mTruckState));
    mCarMovementController->setSpeedToRPMFactor(speed_to_erpm_factor_);

    // setup and connect VESC, simulate movements if unable to connect
    mVESCMotorController.reset(new VESCMotorController());
    foreach(const QSerialPortInfo & portInfo, QSerialPortInfo::availablePorts())
    {
      if (portInfo.description().toLower().replace("-", "").contains("chibios")) { // assumption: Serial device with ChibiOS in
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

      is_in_simulation_mode_ = false;
      waywise_posType_used_ = PosType::fused;
    } else {
      // publish periodically with timer when no motorcontroller connected
      // (simulation)
      is_in_simulation_mode_ = true;
      waywise_posType_used_ = PosType::simulated;
      simulation_timer_ =
        this->create_wall_timer(
        mUpdateVehicleStatePeriod,
        std::bind(&WayWiseTruck::simulation_timer_callback, this));

      RCLCPP_INFO(
        get_logger(),
        "VESCMotorController is not connected. "
        "waywise_truck is in simulation mode!");
    }

    // --- Positioning setup ---
    // Position Fuser
    positionFuser = new SDVPVehiclePositionFuser(this);
    // GNSS (with fused IMU when using u-blox F9R)
    mUbloxRover.reset(new UbloxRover(mTruckState));
    foreach(const QSerialPortInfo & portInfo, QSerialPortInfo::availablePorts()) {
      // qDebug()<<portInfo.manufacturer();
      if (portInfo.manufacturer().toLower().replace("-", "").contains("ublox")) {
        if (mUbloxRover->connectSerial(portInfo)) {
          qDebug() << "UbloxRover connected to:" << portInfo.systemLocation();

          //mUbloxRover->setIMUOrientationOffset(0.0, 0.0, 0.0);
        }
      }
    }

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
          mIMUOrientationUpdater = mVESCMotorController->getIMUOrientationUpdater(mTruckState);
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
        mIMUOrientationUpdater.reset(new BNO055OrientationUpdater(mTruckState, "/dev/i2c-1"));
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


    if (has_trailer_) {
      // ToF Sensors
      std::vector<std::string> tof_sensor_names = this->declare_parameter<std::vector<std::string>>(
        "tof_sensors", {}, rcl_interfaces::msg::ParameterDescriptor{});

      RCLCPP_WARN(
        this->get_logger(),
        "ToF sensors: %ld",
        tof_sensor_names.size());


      if (tof_sensor_names.size() > 1) {
        RCLCPP_WARN(
          this->get_logger(),
          "More than one ToF sensor is not currently supported. "
          "Only the first sensor will be used: '%s'. Ignoring others.",
          tof_sensor_names[0].c_str());

        tof_sensor_names.resize(1); // TODO: enable setting multiple tof sensors
      }
      for (const auto & tof_sensor_name : tof_sensor_names) {
        int i2c_addr =
          this->declare_parameter<int>(tof_sensor_name + ".i2c_addr", 0);
        std::string topic_name = this->declare_parameter<std::string>(
          tof_sensor_name + ".topic", "");

        ToFSensorInfo tof_sensor_info;
        tof_sensor_info.i2c_addr = i2c_addr;
        tof_sensor_info.topic_name = topic_name;

        // tof_sensor_info.sensor.reset(new VL53L0XToFSensor(i2c_addr)); // TODO: enable setting i2c address
        tof_sensor_info.sensor.reset(new VL53L0XToFSensor());
        QObject::connect(
          tof_sensor_info.sensor.get(), &ToFSensor::updatedDistance, this,
          [this, tof_sensor_name](double distance) {
            updated_tof_distance_callback(tof_sensor_name, distance);
          });

        tof_sensor_info.publisher = this->create_publisher<std_msgs::msg::Float32>(topic_name, 10);

        tof_sensors_[tof_sensor_name] = tof_sensor_info;
      }

      // Angle Sensor
      mAngleSensorUpdater.reset(new AS5600Updater(mTruckState, angle_sensor_offset_));
    }

    // Odometry
    QObject::connect(
      mCarMovementController.get(), &CarMovementController::updatedOdomPositionAndYaw, positionFuser,
      &SDVPVehiclePositionFuser::correctPositionAndYawOdom);
    QObject::connect(
      mCarMovementController.get(), &CarMovementController::updatedOdomPositionAndYaw, this,
      &WayWiseTruck::updated_waywise_odomPos_callback);

    // Watchdog that warns when EventLoop is slowed down
    watchdog = new SimpleWatchdog(this);
    RCLCPP_INFO(
      get_logger(),
      "Waywise truck node initialized!");
  }

private:
  void simulation_timer_callback()
  {
    auto thisTimeCalled = std::chrono::high_resolution_clock::now();
    static auto previousTimeCalled = thisTimeCalled - mUpdateVehicleStatePeriod;
    double timePassed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      thisTimeCalled -
      previousTimeCalled).count();

    mTruckState->simulationStep(timePassed_ms, waywise_posType_used_);

    publish_odom_and_tf(timePassed_ms);

    previousTimeCalled = thisTimeCalled;
  }

  void updated_waywise_odomPos_callback(
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

    publish_odom_and_tf(timePassed_ms);
    publish_trailer_angle(); // TODO: connect this with updatedAngleSensor signal

    previousTimeCalled = thisTimeCalled;
  }

  void publish_odom_and_tf(double timePassed_ms)
  {
    PosPoint currentPosition = mTruckState->getPosition(waywise_posType_used_);
    if (waywise_posType_used_ != PosType::fused) {
      currentPosition.setType(PosType::fused); // the 'fused' position type is communicated to topics
                                               // & potentially MAVLINK
      mTruckState->setPosition(currentPosition);
    }

    double truck_x = currentPosition.getX();
    double truck_y = currentPosition.getY();
    double truck_yaw_rad = currentPosition.getYaw() * M_PI / 180.0;
    static double previousYawRad_ = truck_yaw_rad;

    // -- Prepare Odom
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    // Position in the coordinate frame given by header.frame_id
    odom.pose.pose.position.x = truck_x;
    odom.pose.pose.position.y = truck_y;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = sin(truck_yaw_rad / 2.0);
    odom.pose.pose.orientation.w = cos(truck_yaw_rad / 2.0);

    // TODO: position uncertainty?

    // Velocity in the coordinate frame given by child_frame_id
    odom.twist.twist.linear.x = mTruckState->getSpeed();
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = (truck_yaw_rad - previousYawRad_) / (timePassed_ms / 1000.0);

    // TODO: velocity uncertainty?

    if (publish_odom_to_baselink_tf_) {
      // -- Prepare Transform
      auto tf = geometry_msgs::msg::TransformStamped();
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.header.stamp = now();
      tf.transform.translation.x = truck_x;
      tf.transform.translation.y = truck_y;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom.pose.pose.orientation;

      // -- Publish Transform
      tf_pub_->sendTransform(tf);

      // -- Calculate and Publish Trailer Transform
      if (has_trailer_) {
        double trailer_angle_rad = mTruckState->getTrailerAngleRadians();
        double trailer_x = truck_x - cos(truck_yaw_rad + trailer_angle_rad) * trailer_wheelbase_;
        double trailer_y = truck_y - sin(truck_yaw_rad + trailer_angle_rad) * trailer_wheelbase_;
        double trailer_yaw_rad = truck_yaw_rad + trailer_angle_rad;

        auto trailer_tf = geometry_msgs::msg::TransformStamped();
        trailer_tf.header.frame_id = odom_frame_;
        trailer_tf.child_frame_id = trailer_frame_;
        trailer_tf.header.stamp = now();
        trailer_tf.transform.translation.x = trailer_x;
        trailer_tf.transform.translation.y = trailer_y;
        trailer_tf.transform.translation.z = 0.0;
        trailer_tf.transform.rotation.z = sin(trailer_yaw_rad / 2.0);
        trailer_tf.transform.rotation.w = cos(trailer_yaw_rad / 2.0);

        tf_pub_->sendTransform(trailer_tf);
      }
    }

    // -- Publish Odom
    odom_pub_->publish(odom);

    previousYawRad_ = truck_yaw_rad;
  }

  void publish_trailer_angle()
  {
    std_msgs::msg::Float32 angle_msg;
    angle_msg.data = mTruckState->getTrailerAngleDegrees();
    angle_pub_->publish(angle_msg);
  }

  void updated_tof_distance_callback(const std::string & tof_sensor_name, double distance_m)
  {
    auto sensor_info = tof_sensors_[tof_sensor_name];

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(distance_m);
    sensor_info.publisher->publish(msg);
    RCLCPP_INFO(
      this->get_logger(), "Published ToF distance %.2f from %s", distance_m,
      tof_sensor_name.c_str());
  }

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
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
      float desired_steering_curvature = -(clipped_angular_velocity / twist_msg->linear.x);  // ω = v/r => 1/r = ω/v
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

  float clip_min_max(float value, float min_value, float max_value) const
  {
    return std::min(
      std::max(value, (min_value + std::numeric_limits<float>::epsilon())),
      (max_value - std::numeric_limits<float>::epsilon()));
  }

  // ROS parameters
  std::string odom_frame_, odom_topic_;
  std::string base_frame_;
  std::string trailer_frame_;

  float erpm_min_, erpm_max_, speed_to_erpm_factor_;

  bool invert_servo_output_;
  float servo_min_, servo_max_, servo_offset_;

  float length_, width_, wheelbase_, min_turning_radius_;
  float trailer_length_, trailer_width_, trailer_wheelbase_;

  bool publish_odom_to_baselink_tf_;
  int odom_publish_rate_;
  std::string imu_for_position_fusion_;

  float angle_sensor_offset_;


  float standstill_velocity_threshold_;
  float max_angular_velocity_;

  std::string angle_sensor_topic_;
  bool has_trailer_;

  // internal variables
  bool is_in_simulation_mode_ = true;
  PosType waywise_posType_used_ = PosType::simulated;

  // publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;

  rclcpp::TimerBase::SharedPtr simulation_timer_;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sensor_sub_;

  // WayWise
  std::chrono::milliseconds mUpdateVehicleStatePeriod;
  QSharedPointer<TruckState> mTruckState;
  QSharedPointer<TrailerState> mTrailerState;
  QSharedPointer<CarMovementController> mCarMovementController;
  QSharedPointer<VESCMotorController> mVESCMotorController;
  QSharedPointer<IMUOrientationUpdater> mIMUOrientationUpdater;
  QSharedPointer<UbloxRover> mUbloxRover;
  SDVPVehiclePositionFuser * positionFuser;
  RtcmClient * rtcmClient;
  SimpleWatchdog * watchdog;
  QSharedPointer<AngleSensorUpdater> mAngleSensorUpdater;
  QSharedPointer<ToFSensor> mToFSensor;
  struct ToFSensorInfo
  {
    int i2c_addr;
    std::string topic_name;
    QSharedPointer<ToFSensor> sensor;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher;
  };
  std::map<std::string, ToFSensorInfo> tof_sensors_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QCoreApplication a(argc, argv);

  a.processEvents();

  auto waywiseNode = std::make_shared<WayWiseTruck>();
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

#include "waywise_truck.moc"
