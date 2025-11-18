#ifndef TRUCK_AUTOPILOT_COMPONENT_HPP_
#define TRUCK_AUTOPILOT_COMPONENT_HPP_

#include "WayWise/vehicles/truckstate.h"
#include "car_autopilot_component.hpp"

class TruckAutopilotComponent : public CarAutopilotComponent
{
  Q_OBJECT

public:
  // Constructor and destructor
  TruckAutopilotComponent(QObject * parent, const QSharedPointer<TruckState> truckState)
  : CarAutopilotComponent(parent, truckState) {mTruckState = truckState;}
  virtual ~TruckAutopilotComponent() {}

  virtual void setupAutopilot(
    QSharedPointer<GNSSReceiver> gNSSReceiver,
    QSharedPointer<EmergencyStopState> emergencyStopState) override;

  // Setters
  void setPurePursuitForwardGain(float gain) {mPurePursuitForwardGain = gain;}
  void setPurePursuitReverseGain(float gain) {mPurePursuitReverseGain = gain;}

  // Getters
  float getPurePursuitForwardGain() {return mPurePursuitForwardGain;}
  float getPurePursuitReverseGain() {return mPurePursuitReverseGain;}

  // Utility methods

protected:
  // Parameters
  float mPurePursuitForwardGain = 1.0;
  float mPurePursuitReverseGain = -1.0;

  // WayWise components
  QSharedPointer<TruckState> mTruckState;

  // Internal variables
};

#endif  // TRUCK_AUTOPILOT_COMPONENT_HPP_
