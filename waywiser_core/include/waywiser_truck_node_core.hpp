#ifndef WAYWISER_TRUCK_NODE_CORE_HPP_
#define WAYWISER_TRUCK_NODE_CORE_HPP_

#include "WayWise/vehicles/truckstate.h"

#include "truck_autopilot_component.hpp"
#include "truck_interface_component.hpp"
#include "waywiser_car_node_core.hpp"

using namespace std::placeholders;

class WaywiserTruck : public WaywiserCar
{
  Q_OBJECT

public:
  WaywiserTruck(
    const std::string & node_name = "waywiser_truck_node",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : WaywiserCar(node_name, options) {}

  virtual ~WaywiserTruck() = default;

  virtual void initialize_node() override;

protected:
  virtual void setup_parameters() override;
  virtual void setup_publishers() override;
  virtual void setup_subscribers() override;
  virtual void setup_timers() override;

  // Callback methods
  virtual void node_management_timer_callback() override;

  // Publish helper methods
  virtual void publish_tfs() override;
  virtual void publish_world_pose() override;
  void publish_trailer_angle();

  // Utility methods
  virtual double update_joint_states_msg(
    sensor_msgs::msg::JointState & joint_state_msg, double timePassedSinceLastCall_ms) override;

  // ROS parameters
  std::vector<std::string> trailer_wheel_joint_names_;
  std::string truck_trailer_link_joint_name_;

  std::string hitch_frame_;
  std::string trailer_base_frame_;
  std::string trailer_rear_axle_frame_;
  std::string trailer_center_frame_;
  std::string trailer_rear_end_frame_;
  std::string trailer_hitch_frame_;

  std::string angle_sensor_topic_;
  std::string trailer_pose_topic_;

  bool has_trailer_;
  bool invert_trailer_joint_state_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr trailer_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;

  // Timers

  // WayWise components
  QSharedPointer<TruckState> mTruckState;

  // Internal variables
  QSharedPointer<TruckInterfaceComponent> mTruckInterfaceComponent;
  QSharedPointer<TruckAutopilotComponent> mTruckAutopilotComponent;
};

#endif  // WAYWISER_TRUCK_NODE_CORE_HPP_
