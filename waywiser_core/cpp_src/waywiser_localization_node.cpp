#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <QObject>
#include <QString>
#include <QCoreApplication>

#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "qobject_node.hpp"
#include "localization_component.hpp"
#include "waywiser_core_utils.hpp"
#include "waywiser/waywiser_utils.hpp"
#include "waywiser_description/waywiser_description_utils.hpp"

#include "waywiser_core/msg/nav_sat_fix_extended.hpp"

using namespace std::placeholders;

class WaywiserLocalization : public QObjectNode
{
  Q_OBJECT

public:
  WaywiserLocalization(
    const std::string & node_name = "waywiser_localization_node",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : QObjectNode(node_name, options)
  {
    node_logger_ = this->get_logger();
    qInstallMessageHandler(qtMessageHandler);

    // Check if the ROS clock is available
    auto use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (use_sim_time) {
      if (rclcpp::ok() && this->get_clock()->now().nanoseconds() == 0) {
        RCLCPP_WARN(this->get_logger(), "Waiting for /clock to be published...");
      }

      while (rclcpp::ok() && this->get_clock()->now().nanoseconds() == 0) {
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
      }
      RCLCPP_WARN(this->get_logger(), "Receiving /clock msgs now.");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ROS parameters
    urdf_file_ = declare_parameter("urdf_file", "");
    mUrdfModel = getURDFModel(urdf_file_);
    world_frame_ = this->declare_parameter("world_frame", "map");
    gnss_reference_frame_ = this->declare_parameter("gnss_reference_frame", "gnss_base_link");
    gnss_chip_frame_ = declare_parameter("gnss_chip_frame", gnss_reference_frame_);
    gnss_antenna_frame_ = declare_parameter("gnss_antenna_frame", gnss_reference_frame_);
    nav_sat_fix_extended_topic_ = this->declare_parameter(
      "nav_sat_fix_extended_topic", "/nav_sat_fix_extended");
    fused_nav_sat_fix_extended_topic_ = this->declare_parameter(
      "fused_nav_sat_fix_extended_topic", "/fused_nav_sat_fix_extended");
    rtcm_frequency_topic_ = this->declare_parameter(
      "rtcm_frequency_topic", "/rtcm_frequency");
    imu_topic_ = this->declare_parameter("imu_topic", "");
    odom_topic_ = this->declare_parameter("odom_topic", "");
    publish_world_to_fused_tf_ = this->declare_parameter("publish_world_to_fused_tf", false);
    gnss_variant_ = get_receiver_variant_param(this, "gnss_variant");
    gnss_print_verbose_ = this->declare_parameter("gnss_print_verbose", false);
    gnss_sensor_fusion_on_chip_ = this->declare_parameter("gnss_sensor_fusion_on_chip", true);
    gnss_sensor_fusion_imu_autoalign_ = this->declare_parameter(
      "gnss_sensor_fusion_imu_autoalign", false);
    gnss_sensor_fusion_force_recalibrate_ = this->declare_parameter(
      "gnss_sensor_fusion_force_recalibrate", false);
    gnss_message_rate_ = this->declare_parameter("gnss_message_rate", 10);
    gnss_dynamic_model_ = static_cast<DynamicModel>(this->declare_parameter(
        "gnss_dynamic_model", 12));
    use_sdvp_position_fusion_ = this->declare_parameter("use_sdvp_position_fusion", false);
    position_fusion_input_timer_rate_ = this->declare_parameter(
      "position_fusion_input_timer_rate", 10);
    ext_startup_timeout_ = this->declare_parameter(
      "ext_startup_timeout", 5.0);

    std::ostringstream log_stream;
    log_stream << "\nLocalizationComponent offset parameters:\n";
    std::optional<vector3_t> vector3_param;


    // GNSS antenna to GNSS chip
    vector3_param = get_vector3_param(this, "gnss_antenna_to_gnss_chip_offset");
    if (!vector3_param && mUrdfModel) {
      vector3_param = getFramePositionOffset(mUrdfModel, gnss_chip_frame_, gnss_antenna_frame_);
    }
    log_stream << " gnss_antenna_to_gnss_chip_offset: " << vector3_param->c_str() << "\n";
    gnss_antenna_to_gnss_chip_offset_ = vector3_param->to_type<xyz_t>();

    // GNSS chip orientation
    vector3_param = get_vector3_param(this, "gnss_chip_orientation_offset");
    if (!vector3_param && mUrdfModel) {
      vector3_param = getFrameRotationOffset(mUrdfModel, gnss_reference_frame_, gnss_chip_frame_);
    }
    log_stream << " gnss_chip_orientation_offset: " << vector3_param->c_str() << "\n";
    gnss_chip_orientation_offset_ = vector3_param->to_type<xyz_t>();

    // GNSS chip to rear axle
    vector3_param = get_vector3_param(this, "gnss_chip_to_reference_point_offset");
    if (!vector3_param && mUrdfModel) {
      vector3_param = getFramePositionOffset(mUrdfModel, gnss_reference_frame_, gnss_chip_frame_);
    }
    log_stream << " gnss_chip_to_reference_point_offset: " << vector3_param->c_str() << "\n";
    gnss_chip_to_reference_point_offset_ = vector3_param->to_type<xyz_t>();

    // Output log_stream
    RCLCPP_INFO_STREAM(get_logger(), log_stream.str());

    vector3_param = get_vector3_param(this, "enuref");
    if (vector3_param) {
      enuref_ = vector3_param->to_type<llh_t>();
    }

    // WayWise & WayWiseR components
    mObjectState.reset(new ObjectState());
    mObjectState->setEnuRef(enuref_);

    mLocalizationComponent.reset(new LocalizationComponent(this, mObjectState));
    mLocalizationComponent->setUseSdvpPositionFusion(use_sdvp_position_fusion_);
    mLocalizationComponent->setGnssVariant(gnss_variant_);
    mLocalizationComponent->setPositionFusionInputTimerRate(position_fusion_input_timer_rate_);
    mLocalizationComponent->setGnssPrintVerbose(gnss_print_verbose_);
    mLocalizationComponent->setGnssFusionOnChip(gnss_sensor_fusion_on_chip_);
    mLocalizationComponent->setGnssSensorFusionImuAutoalign(gnss_sensor_fusion_imu_autoalign_);
    mLocalizationComponent->setGnssSensorFusionForceRecalibrate(
      gnss_sensor_fusion_force_recalibrate_);
    mLocalizationComponent->setGnssMessageRate(gnss_message_rate_);
    mLocalizationComponent->setGnssDynamicModel(gnss_dynamic_model_);
    mLocalizationComponent->setGnssAntennaToGnssChipOffset(gnss_antenna_to_gnss_chip_offset_);
    mLocalizationComponent->setChipToBaseOffset(gnss_chip_to_reference_point_offset_);
    mLocalizationComponent->setGnssChipOrientationOffset(gnss_chip_orientation_offset_);

    mLocalizationComponent->reset(); // TODO: detect and call this also with sim clock reset
    mLocalizationComponent->setup_localization();

    if (gnss_variant_ == RECEIVER_VARIANT::EXTERNAL) {
      ext_startup_watchdog_timer_ = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        this->get_clock(), // uses sim time if enabled
        std::chrono::milliseconds((int) ext_startup_timeout_ * 1000),
        std::bind(&WaywiserLocalization::ext_startup_watchdog_timer_callback, this)
      );
    }

    // Subscribers
    if (imu_topic_ != "") {
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10, std::bind(&WaywiserLocalization::imu_callback, this, _1));
    }

    if (odom_topic_ != "") {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&WaywiserLocalization::odom_callback, this, _1));
    }

    if (mLocalizationComponent->getGnssVariant() == RECEIVER_VARIANT::EXTERNAL) {
      nav_sat_fix_extended_sub_ = this->create_subscription<waywiser_core::msg::NavSatFixExtended>(
        nav_sat_fix_extended_topic_, 10, std::bind(
          &WaywiserLocalization::external_nav_sat_fix_extended_callback, this, _1));
    }

    // Publishers
    if (publish_world_to_fused_tf_) {
      tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
      QObject::connect(
        mObjectState.get(), &ObjectState::positionUpdated,
        [&](PosType type) {
          if (type == PosType::fused) {
            publish_world_to_fused_tf();
          }
        }
      );
    }

    fused_nav_sat_fix_extended_pub_ = this->create_publisher<waywiser_core::msg::NavSatFixExtended>(
      fused_nav_sat_fix_extended_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);

    QObject::connect(
      mLocalizationComponent->getGnssReceiver().get(), &GNSSReceiver::updatedGNSSPositionAndYaw,
      [&](QSharedPointer<ObjectState> objectState, double distanceMoved,
      GnssFixStatus gnssFixStatus) {
        Q_UNUSED(objectState)
        Q_UNUSED(distanceMoved)

        publish_fused_nav_sat_fix_extended_data(gnssFixStatus);
      });

    rtcm_frequency_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      rtcm_frequency_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);


    switch (mLocalizationComponent->getGnssVariant()) {
      case RECEIVER_VARIANT::UBLX_ZED_F9P:
      case RECEIVER_VARIANT::UBLX_ZED_F9R:
        {
          QObject::connect(
            mLocalizationComponent->getRtcmClient().get(), &RtcmClient::rtcmData,
            [&](const QByteArray & data) {
              Q_UNUSED(data)
              publish_rtcm_frequency();
            });
        } break;
      default:
        break;
    }

    startup_time_ = this->now();

    RCLCPP_INFO(get_logger(), "%s is initialized!", this->get_name());
  }

private:
// Callback methods
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    static tf2::Quaternion imu_frame_to_base_frame_rotation;
    static bool imu_frame_to_base_frame_tf_available = false;

    if (!imu_frame_to_base_frame_tf_available) {
      static bool transform_warning_logged = false;
      try {
        geometry_msgs::msg::TransformStamped imu_frame_to_base_frame_tfs =
          tf_buffer_->lookupTransform(
          gnss_reference_frame_, imu_msg->header.frame_id, tf2::TimePointZero);

        imu_frame_to_base_frame_rotation = tf2::Quaternion(
          imu_frame_to_base_frame_tfs.transform.rotation.x,
          imu_frame_to_base_frame_tfs.transform.rotation.y,
          imu_frame_to_base_frame_tfs.transform.rotation.z,
          imu_frame_to_base_frame_tfs.transform.rotation.w
        );

        if (transform_warning_logged) {
          RCLCPP_WARN(
            get_logger(), "Transform from %s to %s is available now.",
            imu_msg->header.frame_id.c_str(), gnss_reference_frame_.c_str());
          transform_warning_logged = false;
        }
      } catch (tf2::TransformException & ex) {
        if (!transform_warning_logged) {
          RCLCPP_WARN(
            get_logger(), "Transform from %s to %s not available yet!",
            imu_msg->header.frame_id.c_str(), gnss_reference_frame_.c_str());
          transform_warning_logged = true;
        }
        return;
      }
      imu_frame_to_base_frame_tf_available = true;
    }

    tf2::Quaternion q_imu(
      imu_msg->orientation.x,
      imu_msg->orientation.y,
      imu_msg->orientation.z,
      imu_msg->orientation.w
    );

    // Apply transform
    tf2::Quaternion q_base = imu_frame_to_base_frame_rotation * q_imu;

    double rollRad, pitchRad, yawRad;
    tf2::Matrix3x3(q_base).getRPY(rollRad, pitchRad, yawRad);

    PosPoint imuPosition = mObjectState->getPosition(PosType::IMU);
    imuPosition.setRoll(rollRad * 180.0 / M_PI);
    imuPosition.setPitch(pitchRad * 180.0 / M_PI);
    imuPosition.setYaw(yawRad * 180.0 / M_PI);
    imuPosition.setTime(
      QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
    mObjectState->setPosition(imuPosition);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    static xyz_t odom_child_frame_to_gnss_reference_frame_offset;
    static bool odom_child_frame_to_gnss_reference_frame_tf_available = false;

    if (!odom_child_frame_to_gnss_reference_frame_tf_available) {
      static bool transform_warning_logged_ = false;
      try {
        geometry_msgs::msg::TransformStamped odom_child_frame_to_gnss_reference_frame_msg_tfs =
          tf_buffer_->lookupTransform(
          gnss_reference_frame_, odom_msg->child_frame_id, tf2::TimePointZero);
        odom_child_frame_to_gnss_reference_frame_offset = {
          odom_child_frame_to_gnss_reference_frame_msg_tfs.transform.translation.x,
          odom_child_frame_to_gnss_reference_frame_msg_tfs.transform.translation.y,
          odom_child_frame_to_gnss_reference_frame_msg_tfs.transform.translation.z
        };
        if (transform_warning_logged_) {
          RCLCPP_WARN(
            get_logger(), "Transform from %s to %s is available now.",
            odom_msg->child_frame_id.c_str(), gnss_reference_frame_.c_str());
          transform_warning_logged_ = false;
        }
      } catch (tf2::TransformException & ex) {
        if (!transform_warning_logged_) {
          RCLCPP_WARN(
            get_logger(), "Transform from %s to %s not available yet!",
            odom_msg->child_frame_id.c_str(), gnss_reference_frame_.c_str());
          transform_warning_logged_ = true;
        }
        return;
      }
      odom_child_frame_to_gnss_reference_frame_tf_available = true;
    }

    update_pospoint_from_pose(
      mObjectState, odom_child_frame_to_gnss_reference_frame_offset, odom_msg->pose.pose,
      PosType::odom);
  }

  void external_nav_sat_fix_extended_callback(
    const waywiser_core::msg::NavSatFixExtended::SharedPtr msg)
  {
    auto gnssReceiver = mLocalizationComponent->getGnssReceiver();
    if (gnssReceiver->getReceiverVariant() == RECEIVER_VARIANT::WAYWISE_SIMULATED) {
      mLocalizationComponent->setGnssVariant(RECEIVER_VARIANT::EXTERNAL);
      mLocalizationComponent->reset();
      mLocalizationComponent->setup_localization();
      gnssReceiver = mLocalizationComponent->getGnssReceiver();
      gnssReceiver->setReceiverState(RECEIVER_STATE::READY);
      RCLCPP_WARN(
        get_logger(), "Started receiving GNSS data from topic '%s'.",
        nav_sat_fix_extended_topic_.c_str());
    } else if (gnssReceiver->getReceiverState() != RECEIVER_STATE::READY) {
      gnssReceiver->setReceiverState(RECEIVER_STATE::READY);
      RCLCPP_INFO(
        get_logger(), "Started receiving GNSS data from topic '%s'.",
        nav_sat_fix_extended_topic_.c_str());
    }
    if (ext_startup_watchdog_timer_ && !ext_startup_watchdog_timer_->is_canceled()) {
      ext_startup_watchdog_timer_->cancel();
    }

    gnssReceiver->simulationStep(
      [&](QTime time, QSharedPointer<ObjectState> objectState) {
        Q_UNUSED(time)
        Q_UNUSED(objectState)

        gnssReceiver->updateGNSSPositionAndYaw(
          {msg->latitude, msg->longitude, msg->altitude},
          msg->heading, msg->is_fused_on_chip);

        // GNSS fix status
        GnssFixStatus gnssFixStatus;
        gnssFixStatus.isFusedOnChip = msg->is_fused_on_chip;
        gnssFixStatus.fixType = static_cast<GNSS_FIX_TYPE>(msg->fix_type);
        gnssFixStatus.horizontalAccuracy = msg->horizontal_accuracy;
        gnssFixStatus.verticalAccuracy = msg->vertical_accuracy;
        gnssFixStatus.headingAccuracy = msg->heading_accuracy;
        gnssFixStatus.lastRtcmCorrectionAge = msg->last_rtcm_correction_age;
        gnssFixStatus.numSatellites = msg->num_satellites;

        return gnssFixStatus;
      }
    );
  }

  void ext_startup_watchdog_timer_callback()
  {
    RCLCPP_WARN(
      get_logger(),
      "Timedout waiting for external nav_sat_fix_extended topic '%s'. Falling back to WayWise simulation.",
      nav_sat_fix_extended_topic_.c_str());
    ext_startup_watchdog_timer_->cancel();

    mLocalizationComponent->setGnssVariant(RECEIVER_VARIANT::WAYWISE_SIMULATED);
    mLocalizationComponent->reset();
    mLocalizationComponent->setup_localization();
  }

// Utility methods
  void publish_fused_nav_sat_fix_extended_data(const GnssFixStatus & gnssFixStatus)
  {
    PosPoint gnssPos = mObjectState->getPosition(PosType::GNSS);

    // Publish navSatFixExt
    waywiser_core::msg::NavSatFixExtended nav_sat_fix_extended_msg;
    nav_sat_fix_extended_msg.header.stamp = this->now();
    nav_sat_fix_extended_msg.header.frame_id = gnss_reference_frame_;

    llh_t llh = coordinateTransforms::enuToLlh(mObjectState->getEnuRef(), gnssPos.getXYZ());
    nav_sat_fix_extended_msg.latitude = llh.latitude;   // Latitude in degrees
    nav_sat_fix_extended_msg.longitude = llh.longitude;   // Longitude in degrees
    nav_sat_fix_extended_msg.altitude = llh.height;   // Altitude in meters
    nav_sat_fix_extended_msg.heading = coordinateTransforms::yawENUtoNED(gnssPos.getYaw());   // degrees

    nav_sat_fix_extended_msg.is_fused_on_chip = gnssFixStatus.isFusedOnChip;
    nav_sat_fix_extended_msg.fix_type = static_cast<uint8_t>(gnssFixStatus.fixType);
    nav_sat_fix_extended_msg.horizontal_accuracy = gnssFixStatus.horizontalAccuracy;
    nav_sat_fix_extended_msg.vertical_accuracy = gnssFixStatus.verticalAccuracy;
    nav_sat_fix_extended_msg.heading_accuracy = gnssFixStatus.headingAccuracy;
    nav_sat_fix_extended_msg.last_rtcm_correction_age = gnssFixStatus.lastRtcmCorrectionAge;
    nav_sat_fix_extended_msg.num_satellites = gnssFixStatus.numSatellites;

    fused_nav_sat_fix_extended_pub_->publish(nav_sat_fix_extended_msg);
  }

  void publish_world_to_fused_tf()
  {
    PosPoint fusedPosition = mObjectState->getPosition(PosType::fused);

    double x_ = fusedPosition.getX();
    double y_ = fusedPosition.getY();
    double yawRad_ = fusedPosition.getYaw() * M_PI / 180.0;

    // -- Prepare Transform
    auto world_to_fused_tfs = geometry_msgs::msg::TransformStamped();
    world_to_fused_tfs.header.frame_id = world_frame_;
    world_to_fused_tfs.child_frame_id = gnss_reference_frame_;
    world_to_fused_tfs.header.stamp = now();
    world_to_fused_tfs.transform.translation.x = x_;
    world_to_fused_tfs.transform.translation.y = y_;
    world_to_fused_tfs.transform.translation.z = fusedPosition.getHeight();
    world_to_fused_tfs.transform.rotation.x = 0.0;
    world_to_fused_tfs.transform.rotation.y = 0.0;
    world_to_fused_tfs.transform.rotation.z = sin(yawRad_ / 2.0);
    world_to_fused_tfs.transform.rotation.w = cos(yawRad_ / 2.0);

    // -- Publish Transform
    tf_pub_->sendTransform(world_to_fused_tfs);
  }

  void publish_rtcm_frequency()
  {
    static std::deque<double> frequency_samples_;
    static std::chrono::steady_clock::time_point last_call_time_;

    auto current_time = std::chrono::steady_clock::now();
    double current_frequency = 0.0;
    auto time_diff = current_time - last_call_time_;
    double time_diff_seconds = std::chrono::duration<double>(time_diff).count();
    if (time_diff_seconds > 0.0) {
      current_frequency = 1.0 / time_diff_seconds;
    }
    last_call_time_ = current_time;

    frequency_samples_.push_back(current_frequency);
    if (frequency_samples_.size() > 3) {
      frequency_samples_.pop_front();
    }

    double averaged_frequency = std::accumulate(
      frequency_samples_.begin(), frequency_samples_.end(),
      0.0) / frequency_samples_.size();

    std_msgs::msg::Float32 rtcm_frequency_msg;
    rtcm_frequency_msg.data = averaged_frequency;
    rtcm_frequency_pub_->publish(rtcm_frequency_msg);
  }

  static void qtMessageHandler(QtMsgType type, const QMessageLogContext &, const QString & msg)
  {
    qtMessageToLogger(node_logger_, type, msg);
  }

// Parameters
  std::string urdf_file_;
  std::string world_frame_;
  std::string gnss_reference_frame_;
  std::string gnss_chip_frame_;
  std::string gnss_antenna_frame_;
  std::string nav_sat_fix_extended_topic_;
  std::string fused_nav_sat_fix_extended_topic_;
  std::string rtcm_frequency_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  bool publish_world_to_fused_tf_;
  double ext_startup_timeout_;

  RECEIVER_VARIANT gnss_variant_;
  bool gnss_print_verbose_;
  bool gnss_sensor_fusion_on_chip_; // only used for Ublox F9R
  bool gnss_sensor_fusion_imu_autoalign_; // only used for Ublox F9R
  bool gnss_sensor_fusion_force_recalibrate_; // only used for Ublox F9R
  int gnss_message_rate_;   // [Hz]
  DynamicModel gnss_dynamic_model_;
  bool use_sdvp_position_fusion_;
  int position_fusion_input_timer_rate_;   // [Hz]
  xyz_t gnss_antenna_to_gnss_chip_offset_;
  xyz_t gnss_chip_to_reference_point_offset_;
  xyz_t gnss_chip_orientation_offset_;
  llh_t enuref_;   // [lat, lon, height]

// Publishers
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Publisher<waywiser_core::msg::NavSatFixExtended>::SharedPtr
    fused_nav_sat_fix_extended_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rtcm_frequency_pub_;

// Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<waywiser_core::msg::NavSatFixExtended>::SharedPtr nav_sat_fix_extended_sub_;

// Transform buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// Timers
  rclcpp::TimerBase::SharedPtr ext_startup_watchdog_timer_;

// WayWise & WayWiseR components
  QSharedPointer<ObjectState> mObjectState;
  QSharedPointer<LocalizationComponent> mLocalizationComponent;

// Internal variables
  static rclcpp::Logger node_logger_;
  QSharedPointer<urdf::Model> mUrdfModel;
  rclcpp::Time startup_time_;
};

rclcpp::Logger WaywiserLocalization::node_logger_ =
  rclcpp::get_logger("waywiser_localization_node");

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QCoreApplication app(argc, argv);

  app.processEvents();

  auto node = std::make_shared<WaywiserLocalization>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  while (rclcpp::ok()) {
    exec.spin_some();
    app.processEvents();
  }

  exec.remove_node(node);
  rclcpp::shutdown();

  return 0;
}
#include "waywiser_localization_node.moc"
