#ifndef COPTER_INTERFACE_COMPONENT_HPP_
#define COPTER_INTERFACE_COMPONENT_HPP_

#include <array>
#include <cstdint>
#include <string>

#include <QObject>

#include "WayWise/vehicles/controller/movementcontroller.h"
#include "WayWise/vehicles/copterstate.h"
#include "qobject_node.hpp"
#include "waywiser_core_utils.hpp"

class CopterInterfaceComponent : public QObject
{
  Q_OBJECT

public:
  CopterInterfaceComponent(
    QObjectNode * parentQObjectNode, const QSharedPointer<CopterState> & copterState);
  virtual ~CopterInterfaceComponent() = default;

  virtual void reset();
  virtual void setup_vehicle_interface();

  void activate_emergency_stop(const std::string & sender_id = "", const std::string & reason = "");
  void clear_emergency_stop(const std::string & sender_id = "");

  void setLength(float value) {mLength = value;}
  void setWidth(float value) {mWidth = value;}
  void setVehicleInterfaceType(VehicleInterfaceType value) {mVehicleInterfaceType = value;}
  void setVehicleStatePollRate(int value) {mVehicleStatePollRate = value;}

  float getLength() const {return mLength;}
  float getWidth() const {return mWidth;}
  VehicleInterfaceType getVehicleInterfaceType() const {return mVehicleInterfaceType;}
  int getVehicleStatePollRate() const {return mVehicleStatePollRate;}

  QSharedPointer<EmergencyStopState> getEmergencyStopState() const {return mEmergencyStopState;}
  QSharedPointer<MovementController> getMovementController() const {return mMovementController;}

protected:
  float mLength = 0.52F;
  float mWidth = 0.52F;
  VehicleInterfaceType mVehicleInterfaceType = VehicleInterfaceType::EXT_SIMULATED;
  int mVehicleStatePollRate = 30;

  QSharedPointer<CopterState> mCopterState;
  QSharedPointer<EmergencyStopState> mEmergencyStopState;
  QSharedPointer<MovementController> mMovementController;
  QObjectNode * mParentQObjectNode = nullptr;
};

// ── PX4 preflight helpers ────────────────────────────────────────────────────

struct Px4HealthFlagDefinition
{
  uint64_t bit;
  const char * name;
  const char * description;
};

inline constexpr std::array<Px4HealthFlagDefinition, 31> kPx4HealthFlagDefinitions{{
  {uint64_t{1} << 0, "none", "unassigned / no component"},
  {uint64_t{1} << 1, "absolute_pressure", "barometer / absolute pressure sensor"},
  {uint64_t{1} << 2, "differential_pressure", "airspeed / differential pressure sensor"},
  {uint64_t{1} << 3, "gps", "GPS"},
  {uint64_t{1} << 4, "optical_flow", "optical flow"},
  {uint64_t{1} << 5, "vision_position", "vision position estimate"},
  {uint64_t{1} << 6, "distance_sensor", "distance sensor / rangefinder"},
  {uint64_t{1} << 7, "remote_control", "remote control / joystick"},
  {uint64_t{1} << 8, "motors_escs", "motors / ESCs"},
  {uint64_t{1} << 9, "utm", "UTM"},
  {uint64_t{1} << 10, "logging", "logging"},
  {uint64_t{1} << 11, "battery", "battery"},
  {uint64_t{1} << 12, "communication_links", "communication links"},
  {uint64_t{1} << 13, "rate_controller", "rate controller"},
  {uint64_t{1} << 14, "attitude_controller", "attitude controller"},
  {uint64_t{1} << 15, "position_controller", "position controller"},
  {uint64_t{1} << 16, "attitude_estimate", "attitude estimate"},
  {uint64_t{1} << 17, "local_position_estimate", "local position estimate"},
  {uint64_t{1} << 18, "mission", "mission"},
  {uint64_t{1} << 19, "avoidance", "avoidance"},
  {uint64_t{1} << 20, "system", "system resources / core system"},
  {uint64_t{1} << 21, "camera", "camera"},
  {uint64_t{1} << 22, "gimbal", "gimbal"},
  {uint64_t{1} << 23, "payload", "payload"},
  {uint64_t{1} << 24, "global_position_estimate", "global position estimate"},
  {uint64_t{1} << 25, "storage", "storage such as SD card / FRAM"},
  {uint64_t{1} << 26, "parachute", "parachute"},
  {uint64_t{1} << 27, "magnetometer", "magnetometer"},
  {uint64_t{1} << 28, "accel", "accelerometer"},
  {uint64_t{1} << 29, "gyro", "gyroscope"},
  {uint64_t{1} << 30, "open_drone_id", "Open Drone ID system"},
}};

std::string format_px4_health_flags(uint64_t flags);

std::string format_px4_preflight_summary(
  bool ready_for_takeoff,
  bool ready_for_offboard,
  bool ready_to_arm,
  uint64_t health_error_flags,
  uint64_t health_warning_flags,
  uint64_t arming_check_error_flags,
  uint64_t arming_check_warning_flags,
  uint64_t can_arm_mode_flags = 0);

bool px4_ready_for_arm_command(
  bool ready_for_takeoff, bool ready_for_offboard, bool ready_to_arm);

VehicleInterfaceType parse_vehicle_interface_type(const std::string & value);

#endif  // COPTER_INTERFACE_COMPONENT_HPP_
