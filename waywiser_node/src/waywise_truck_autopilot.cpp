#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "WayWise/autopilot/purepursuitwaypointfollower.h"
#include "WayWise/autopilot/waypointfollower.h"
#include "WayWise/communication/mavsdkvehicleserver.h"
#include "WayWise/communication/parameterserver.h"
#include "WayWise/logger/logger.h"
#include "WayWise/vehicles/truckstate.h"
#include "WayWise/vehicles/trailerstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/autopilot/followpoint.h"
#include <QCoreApplication>
#include <QObject>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

class WaywiseTruckAutopilot : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  WaywiseTruckAutopilot()
  : QObject(), Node("waywise_truck_autopilot")
  {
    // -- ROS --
    // get ROS parameters
    speed_to_erpm_factor_ = this->declare_parameter("speed_to_erpm_factor", 0.0);
    length_ = this->declare_parameter("length", 0.33);
    width_ = this->declare_parameter("width", 0.33);
    wheelbase_ = this->declare_parameter("wheelbase", 0.33);
    min_turning_radius_ = this->declare_parameter("min_turning_radius", 0.67);
    autopilot_cmd_publish_rate_ = this->declare_parameter("autopilot_cmd_publish_rate", 30);
    waywise_control_tower_address_ = this->declare_parameter(
      "waywise_control_tower_address",
      "127.0.0.1");
    odom_topic_ = this->declare_parameter("odom_topic", "/odom");
    purepursuit_radius_ = this->declare_parameter("purepursuit_radius", 1.0);

    has_trailer_ = this->declare_parameter("has_trailer", false);
    if (has_trailer_) {
      trailer_length_ = this->declare_parameter("trailer_length", 10.0);
      trailer_width_ = this->declare_parameter("trailer_width", 6.0);
      trailer_wheelbase_ = this->declare_parameter("trailer_wheelbase", 8.0);
      purepursuit_forward_gain_ = this->declare_parameter("purepursuit_forward_gain", 1.0);
      purepursuit_reverse_gain_ = this->declare_parameter("purepursuit_reverse_gain", -1.0);
      angle_sensor_topic_ = this->declare_parameter("angle_sensor_topic", "/sensors/angle");

      angle_sensor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        angle_sensor_topic_, 10,
        std::bind(&WaywiseTruckAutopilot::angle_sensor_callback, this, _1));
    }

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&WaywiseTruckAutopilot::odom_callback, this, _1));

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/waywise_vel", 10);
    autopilot_timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds((int)std::round(1000.0 / autopilot_cmd_publish_rate_)),
      std::bind(&WaywiseTruckAutopilot::autopilot_timer_callback, this));

    // -- WayWise --
    mTruckState.reset(new TruckState);
    mTruckState->setLength(length_);
    mTruckState->setWidth(width_);
    mTruckState->setAxisDistance(wheelbase_);
    mTruckState->setMaxSteeringAngle(
      atan(
        mTruckState->getAxisDistance() /
        min_turning_radius_));
    mTruckState->setPurePursuitForwardGain(purepursuit_forward_gain_);
    mTruckState->setPurePursuitReverseGain(purepursuit_reverse_gain_);

    // --- Movement control setup ---
    mCarMovementController.reset(new CarMovementController(mTruckState));
    mCarMovementController->setSpeedToRPMFactor(speed_to_erpm_factor_);
    mTruckState->setAxisDistance(wheelbase_);
    mTruckState->setMaxSteeringAngle(atan(mTruckState->getAxisDistance() / min_turning_radius_));
    mFollowPoint.reset(new FollowPoint(mCarMovementController));

    // Setup MAVLINK communication towards ControlTower
    mMavsdkVehicleServer.reset(
      new MavsdkVehicleServer(
        mTruckState,
        QHostAddress(QString::fromStdString(waywise_control_tower_address_))));
    mMavsdkVehicleServer->setMovementController(mCarMovementController);

    if (has_trailer_) {
      mTrailerMavlinkComponentID = (int) MAV_COMP_ID_USER1;

      mTrailerState.reset(new TrailerState(mTrailerMavlinkComponentID, Qt::white));
      mTrailerState->setLength(trailer_length_);
      mTrailerState->setWidth(trailer_width_);
      mTrailerState->setWheelBase(trailer_wheelbase_);

      mTruckState->setTrailerState(mTrailerState);
    }

    ParameterServer::getInstance()->provideFloatParameter(
      "VEH_LENGTH",
      std::function<void(float)>(
        [this](float value) {
          this->length_ = value;
          mTruckState->setLength(this->length_);
        }),
      std::function<float(void)>(
        [this]() {
          return this->length_;
        })
    );

    ParameterServer::getInstance()->provideFloatParameter(
      "VEH_WIDTH",
      std::function<void(float)>(
        [this](float value) {
          this->width_ = value;
          mTruckState->setWidth(this->width_);
        }),
      std::function<float(void)>(
        [this]() {
          return this->width_;
        })
    );

    ParameterServer::getInstance()->provideFloatParameter(
      "VEH_WHLBASE",
      std::function<void(float)>(
        [this](float value) {
          this->wheelbase_ = value;
          mTruckState->setAxisDistance(this->wheelbase_);
        }),
      std::function<float(void)>(
        [this]() {
          return this->wheelbase_;
        })
    );

    ParameterServer::getInstance()->provideIntParameter(
      "TRLR_COMP_ID",
      std::function<void(int)>(
        [this](int value) {
          this->mTrailerMavlinkComponentID = value;
          mTrailerState->setId(this->mTrailerMavlinkComponentID);
        }),
      std::function<int(void)>(
        [this]() {
          return this->mTrailerMavlinkComponentID;
        })
    );

    if (has_trailer_) {
      ParameterServer::getInstance()->provideFloatParameter(
        "TRLR_LENGTH",
        std::function<void(float)>(
          [this](float value) {
            this->trailer_length_ = value;
            mTrailerState->setLength(this->trailer_length_);
          }),
        std::function<float(void)>(
          [this]() {
            return this->trailer_length_;
          })
      );

      ParameterServer::getInstance()->provideFloatParameter(
        "TRLR_WIDTH",
        std::function<void(float)>(
          [this](float value) {
            this->trailer_width_ = value;
            mTrailerState->setWidth(this->trailer_width_);
          }),
        std::function<float(void)>(
          [this]() {
            return this->trailer_width_;
          })
      );

      ParameterServer::getInstance()->provideFloatParameter(
        "TRLR_WHLBASE",
        std::function<void(float)>(
          [this](float value) {
            this->trailer_wheelbase_ = value;
            mTrailerState->setWheelBase(this->trailer_wheelbase_);
          }),
        std::function<float(void)>(
          [this]() {
            return this->trailer_wheelbase_;
          })
      );

      mavsdk::Mavsdk::Configuration config =
        mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::Custom};
      config.set_system_id(mTruckState->getId());
      config.set_always_send_heartbeats(true);
      config.set_component_id(mTrailerState->getId());
      mTrailerMavsdk.reset(new mavsdk::Mavsdk{config});

      mTrailerMavsdk->subscribe_on_new_system(
        [this]() {
          for (const auto & system : mTrailerMavsdk->systems()) {
            auto mavlinkPassthrough = new mavsdk::MavlinkPassthrough(system);
            mavlinkPassthrough->subscribe_message(
              MAVLINK_MSG_ID_HEARTBEAT,
              [this, system, mavlinkPassthrough](const mavlink_message_t & message) {
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&message, &heartbeat);
                // unsubscribe from further heartbeats by deleting passthrough
                delete mavlinkPassthrough;

                if ((MAV_TYPE) heartbeat.type == MAV_TYPE_GCS) {
                  mTrailerMavlinkPassthrough.reset(new mavsdk::MavlinkPassthrough(system));
                  is_connected_to_controltower = true;
                }
              });
          }
        });

      mTrailerMavsdk->intercept_outgoing_messages_async(
        [this](mavlink_message_t & message) {
          switch (message.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT: // Fix some info in heartbeat s.th. MAVSDK / ControlTower detects vehicle correctly
              mavlink_heartbeat_t heartbeat;
              mavlink_msg_heartbeat_decode(&message, &heartbeat);
              if (message.compid == mTrailerState->getId()) {
                heartbeat.type = MAV_TYPE_ONBOARD_CONTROLLER;
                heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
              }
              mavlink_msg_heartbeat_encode(message.sysid, message.compid, &message, &heartbeat);
              break;
            default:;
              //            qDebug() << "out:" << message.msgid;
          }
          return true;
        });

      mTrailerMavsdk->setup_udp_remote(waywise_control_tower_address_, 14540);
    }

    // --- Autopilot ---
    mWaypointFollower.reset(new PurepursuitWaypointFollower(mCarMovementController));
    mWaypointFollower->setPurePursuitRadius(purepursuit_radius_);
    mWaypointFollower->setRepeatRoute(false);
    mWaypointFollower->setAdaptivePurePursuitRadiusActive(true);
    mMavsdkVehicleServer->setWaypointFollower(mWaypointFollower);
  }

private:
  void autopilot_timer_callback()
  {
    double mDesiredSpeed = mCarMovementController->getDesiredSpeed();        // [m/s]
    double mDesiredSteering = mCarMovementController->getDesiredSteering();  // [-1.0:1.0]
    double steeringAngle_rad = mDesiredSteering * mTruckState->getMaxSteeringAngle();
    if (abs(steeringAngle_rad) > mTruckState->getMaxSteeringAngle()) {
      steeringAngle_rad = mTruckState->getMaxSteeringAngle() *
        ((steeringAngle_rad > 0) ? 1.0 : -1.0);
    }
    double mDesiredSteeringCurvature = tan(steeringAngle_rad) / mTruckState->getAxisDistance();
    double mDesiredAngularVelocity = -mDesiredSpeed * mDesiredSteeringCurvature;  // ω = v/r

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = mDesiredSpeed;
    twist_msg.angular.z = mDesiredAngularVelocity;

    twist_pub_->publish(twist_msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    PosPoint currentPosition = mTruckState->getPosition(waywise_posType_used_);
    double newYaw_deg_ = tf2::getYaw(odom_msg->pose.pose.orientation) * (180.0 / M_PI);
    currentPosition.setX(odom_msg->pose.pose.position.x);
    currentPosition.setY(odom_msg->pose.pose.position.y);
    currentPosition.setYaw(newYaw_deg_);
    currentPosition.setTime(
      QTime::currentTime().addSecs(
        -QDateTime::currentDateTime().offsetFromUtc()));
    mTruckState->setPosition(currentPosition);
    currentPosition.setType(PosType::fused);  // the 'fused' position type is communicated to topics
                                              // & potentially MAVLINK
    mTruckState->setPosition(currentPosition);
  }

  void angle_sensor_callback(const std_msgs::msg::Float32::SharedPtr angle_msg)
  {
    double agnle_in_degrees = angle_msg->data;
    double angle_in_radians = agnle_in_degrees * (M_PI / 180.0);
    mTruckState->setTrailerAngle(0, angle_in_radians, agnle_in_degrees); // raw_angle is not currently used

    if (is_connected_to_controltower) {
      double trailerYawRad = (mTruckState->getPosition(PosType::fused).getYaw() * (M_PI / 180.0)) -
        angle_in_radians;

      mTrailerMavlinkPassthrough->queue_message(
        [this, trailerYawRad](MavlinkAddress mavlink_address, uint8_t channel)->mavlink_message_t {
          mavlink_address.system_id = mTruckState->getId();
          mavlink_address.component_id = mTrailerState->getId();
          mavlink_msg_attitude_pack_chan(
            mavlink_address.system_id,
            mavlink_address.component_id,
            channel,
            &mTrailerYawMsg,
            0.0,            // time_boot_ms (not used)
            0.0,            // roll (not used)
            0.0,            // pitch (not used)
            trailerYawRad,   // yaw (your desired yaw value)
            0.0,            // rollspeed (not used)
            0.0,            // pitchspeed (not used)
            0.0             // yawspeed (not used)
          );
          return mTrailerYawMsg;
        });
    }
  }

  // ROS parameters
  float speed_to_erpm_factor_;
  float length_, width_, wheelbase_, min_turning_radius_;
  int autopilot_cmd_publish_rate_;
  std::string waywise_control_tower_address_, odom_topic_;
  float purepursuit_radius_;

  float trailer_length_, trailer_width_, trailer_wheelbase_;
  float purepursuit_forward_gain_, purepursuit_reverse_gain_;
  std::string angle_sensor_topic_;
  bool has_trailer_;

  // internal variables
  PosType waywise_posType_used_ = PosType::simulated;
  bool is_connected_to_controltower = false;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  rclcpp::TimerBase::SharedPtr autopilot_timer_;

  // subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sensor_sub_;


  // WayWise
  QSharedPointer<TruckState> mTruckState;
  QSharedPointer<TrailerState> mTrailerState;
  QSharedPointer<CarMovementController> mCarMovementController;
  QSharedPointer<PurepursuitWaypointFollower> mWaypointFollower;
  QSharedPointer<MavsdkVehicleServer> mMavsdkVehicleServer;
  std::shared_ptr<mavsdk::Mavsdk> mTrailerMavsdk;
  std::shared_ptr<mavsdk::MavlinkPassthrough> mTrailerMavlinkPassthrough;
  mavlink_message_t mTrailerYawMsg;
  mavlink_attitude_t mTrailerAttitudeData = {};
  QSharedPointer<FollowPoint> mFollowPoint;
  int mTrailerMavlinkComponentID = -1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QCoreApplication a(argc, argv);

  a.processEvents();

  auto waywiseNode = std::make_shared<WaywiseTruckAutopilot>();
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

#include "waywise_truck_autopilot.moc"
