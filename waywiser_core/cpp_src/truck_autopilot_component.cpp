#include "truck_autopilot_component.hpp"
#include "moc_truck_autopilot_component.cpp"


void TruckAutopilotComponent::setupAutopilot(
  QSharedPointer<GNSSReceiver> gNSSReceiver,
  QSharedPointer<EmergencyStopState> emergencyStopState)
{
  CarAutopilotComponent::setupAutopilot(gNSSReceiver, emergencyStopState);

  mTruckState->setPurePursuitForwardGain(mPurePursuitForwardGain);
  mTruckState->setPurePursuitReverseGain(mPurePursuitReverseGain);
}
