#include "waywise_truck_autopilot.hpp"
#include "moc_waywise_truck_autopilot.cpp"

using namespace std::placeholders;

void WaywiseTruckAutopilot::setup_parameters()
{
  // Call base class setup_parameters
  WaywiseCarAutopilot::setup_parameters();

  // Additional parameters for truck

  has_trailer_ = this->declare_parameter("has_trailer", false);
  if (has_trailer_) {
    trailer_length_ = this->declare_parameter("trailer_length", 10.0);
    trailer_width_ = this->declare_parameter("trailer_width", 6.0);
    trailer_wheelbase_ = this->declare_parameter("trailer_wheelbase", 8.0);
    purepursuit_forward_gain_ = this->declare_parameter("purepursuit_forward_gain", 1.0);
    purepursuit_reverse_gain_ = this->declare_parameter("purepursuit_reverse_gain", -1.0);
    angle_sensor_topic_ = this->declare_parameter("angle_sensor_topic", "/sensors/angle");
  }
}

void WaywiseTruckAutopilot::setup_publishers()
{
  // Call base class setup_publishers
  WaywiseCarAutopilot::setup_publishers();
}

void WaywiseTruckAutopilot::setup_subscribers()
{
  // Call base class setup_subscribers
  WaywiseCarAutopilot::setup_subscribers();

  angle_sensor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    angle_sensor_topic_, 10,
    std::bind(&WaywiseTruckAutopilot::angle_sensor_callback, this, _1));
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

    mTruckState->setTrailingVehicle(mTrailerState);
  }

  // Call base class setup_hardware with mTruckState
  WaywiseCarAutopilot::setup_autopilot(mTruckState);

}

void WaywiseTruckAutopilot::angle_sensor_callback(const std_msgs::msg::Float32::SharedPtr angle_msg)
{
  mTruckState->setTrailerAngle(angle_msg->data);
}
