#include "copter_interface_component.hpp"
#include "moc_copter_interface_component.cpp"

#include <QDebug>

#include <algorithm>
#include <sstream>

std::string format_px4_health_flags(uint64_t flags)
{
  if (flags == 0) {
    return "none";
  }

  std::ostringstream stream;
  bool first = true;
  uint64_t known_bits = 0;

  for (const auto & definition : kPx4HealthFlagDefinitions) {
    known_bits |= definition.bit;
    if ((flags & definition.bit) == 0) {
      continue;
    }
    if (!first) {
      stream << ", ";
    }
    first = false;
    stream << definition.name << " (" << definition.description << ")";
  }

  const uint64_t unknown_bits = flags & ~known_bits;
  if (unknown_bits != 0) {
    if (!first) {
      stream << ", ";
    }
    stream << "unknown_bits=0x" << std::hex << unknown_bits;
  }

  return stream.str();
}

std::string format_px4_preflight_summary(
  bool ready_for_takeoff,
  bool ready_for_offboard,
  bool ready_to_arm,
  uint64_t health_error_flags,
  uint64_t health_warning_flags,
  uint64_t arming_check_error_flags,
  uint64_t arming_check_warning_flags,
  uint64_t can_arm_mode_flags)
{
  std::ostringstream stream;
  stream
    << "PX4 preflight summary:\n"
    << "  readiness:\n"
    << "    ready_for_takeoff=" << (ready_for_takeoff ? "true" : "false") << '\n'
    << "    ready_for_offboard=" << (ready_for_offboard ? "true" : "false") << '\n'
    << "    ready_to_arm=" << (ready_to_arm ? "true" : "false") << '\n'
    << "    can_arm_mode_flags=0x" << std::hex << can_arm_mode_flags << std::dec << '\n'
    << "  health_error_flags=0x" << std::hex << health_error_flags << std::dec
    << '\n'
    << "    " << format_px4_health_flags(health_error_flags) << '\n'
    << "  health_warning_flags=0x" << std::hex << health_warning_flags << std::dec
    << '\n'
    << "    " << format_px4_health_flags(health_warning_flags) << '\n'
    << "  arming_check_error_flags=0x" << std::hex << arming_check_error_flags << std::dec
    << '\n'
    << "    " << format_px4_health_flags(arming_check_error_flags) << '\n'
    << "  arming_check_warning_flags=0x" << std::hex << arming_check_warning_flags << std::dec
    << '\n'
    << "    " << format_px4_health_flags(arming_check_warning_flags);
  return stream.str();
}

bool px4_ready_for_arm_command(
  bool ready_for_takeoff, bool ready_for_offboard, bool ready_to_arm)
{
  (void)ready_to_arm;
  (void)ready_for_offboard;
  // Only require AUTO_TAKEOFF readiness (matching original Python node behavior).
  // OFFBOARD readiness depends on receiving the offboard_control_mode signal, which
  // may not be valid yet at startup even though we are streaming it.
  return ready_for_takeoff;
}

VehicleInterfaceType parse_vehicle_interface_type(const std::string & value)
{
  std::string normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), ::tolower);
  if (normalized == "waywise_simulated") {
    return VehicleInterfaceType::WAYWISE_SIMULATED;
  }
  return VehicleInterfaceType::EXT_SIMULATED;
}

CopterInterfaceComponent::CopterInterfaceComponent(
  QObjectNode * parentQObjectNode, const QSharedPointer<CopterState> & copterState)
: QObject(parentQObjectNode), mCopterState(copterState), mParentQObjectNode(parentQObjectNode)
{
}

void CopterInterfaceComponent::reset()
{
  if (!mEmergencyStopState) {
    mEmergencyStopState.reset(new EmergencyStopState());
  }
  if (!mMovementController) {
    mMovementController.reset(new MovementController(mCopterState));
  }

  mEmergencyStopState->set_clear();
  mMovementController->setDesiredSpeed(0.0);
  mMovementController->setDesiredSteering(0.0);
  mCopterState->setVelocity({0.0, 0.0, 0.0});
  mCopterState->setSteering(0.0);
}

void CopterInterfaceComponent::setup_vehicle_interface()
{
  mCopterState->setLength(mLength);
  mCopterState->setWidth(mWidth);

  mEmergencyStopState.reset(new EmergencyStopState());
  mMovementController.reset(new MovementController(mCopterState));

  if (mVehicleInterfaceType == VehicleInterfaceType::WAYWISE_SIMULATED) {
    qWarning() <<
      "waywiser_copter_node does not implement an internal multicopter simulator yet; "
      "set vehicle_interface_type='ext_simulated' for PX4/Gazebo inputs.";
  }
}

void CopterInterfaceComponent::activate_emergency_stop(
  const std::string & sender_id,
  const std::string & reason)
{
  if (!mEmergencyStopState || !mMovementController || mEmergencyStopState->is_active()) {
    return;
  }

  mMovementController->setDesiredSpeed(0.0);
  mMovementController->setDesiredSteering(0.0);
  mEmergencyStopState->set_active();
  qWarning() << QString("Emergency stop ACTIVATED%1.%2")
    .arg(sender_id.empty() ? "" : " by " + QString::fromStdString(sender_id))
    .arg(reason.empty() ? "" : " Reason: " + QString::fromStdString(reason));
}

void CopterInterfaceComponent::clear_emergency_stop(const std::string & sender_id)
{
  if (!mEmergencyStopState || mEmergencyStopState->is_clear()) {
    return;
  }

  mEmergencyStopState->set_clear();
  qWarning() << QString("Emergency stop CLEARED%1")
    .arg(sender_id.empty() ? "" : " by " + QString::fromStdString(sender_id));
}
