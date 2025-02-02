#ifndef WAYWISE_TRUCK_HPP_
#define WAYWISE_TRUCK_HPP_

#include <map>

#include "waywise_car.hpp"
#include "WayWise/sensors/angle/as5600updater.h"
#include "WayWise/sensors/tof/vl53l0xtofsensor.h"
#include "WayWise/vehicles/truckstate.h"
#include "WayWise/vehicles/trailerstate.h"

#include "std_msgs/msg/float32.hpp"

class WayWiseTruck : public WayWiseCar
{
  Q_OBJECT

public:
  WayWiseTruck(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "waywise_truck_node")
  : WayWiseCar(options, node_name) {}

  virtual ~WayWiseTruck() = default;

  // virtual void initialize_node() override;

protected:
  virtual void setup_parameters() override;
  virtual void setup_publishers() override;
  virtual void setup_subscribers() override;
  virtual void setup_timers() override;
  virtual void setup_hardware() override;

  // Callback methods
  virtual void publish_odom_and_tfs(double timePassed_ms) override;
  void publish_trailer_angle();
  void updated_tof_distance_callback(const std::string & tof_sensor_name, double distance_m);

  // Utility methods

  // ROS parameters
  float trailer_length_, trailer_width_, trailer_wheelbase_;
  float angle_sensor_offset_;
  std::string angle_sensor_topic_;

  std::vector<std::string> tof_sensor_names;

  bool has_trailer_;
  std::string trailer_base_frame_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;

  // Subscribers

  // WayWise components
  QSharedPointer<TruckState> mTruckState;
  QSharedPointer<TrailerState> mTrailerState;
  QSharedPointer<AngleSensorUpdater> mAngleSensorUpdater;
  QSharedPointer<ToFSensor> mToFSensor;

  // Internal variables
  struct ToFSensorInfo
  {
    int i2c_addr;
    std::string topic_name;
    QSharedPointer<ToFSensor> sensor;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher;
  };
  std::map<std::string, ToFSensorInfo> tof_sensors_;

// private:
};

#endif  // WAYWISE_TRUCK_HPP_
