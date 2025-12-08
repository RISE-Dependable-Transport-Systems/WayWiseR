#ifndef CAR_AUTOPILOT_COMPONENT_HPP_
#define CAR_AUTOPILOT_COMPONENT_HPP_

#include <memory>
#include <string>
#include <QObject>
#include <QString>

#include "WayWise/autopilot/purepursuitwaypointfollower.h"
#include "WayWise/autopilot/waypointfollower.h"
#include "WayWise/communication/mavsdkvehicleserver.h"
#include "WayWise/communication/parameterserver.h"
#include "WayWise/core/coordinatetransforms.h"
#include "WayWise/logger/logger.h"
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/autopilot/followpoint.h"

#include "mavsdk/mavsdk.h"

#include "waywiser_core_utils.hpp"

class CarAutopilotComponent : public QObject
{
  Q_OBJECT

public:
  // Constructor and destructor
  CarAutopilotComponent(QObject * parentQObject, const QSharedPointer<CarState> carState)
  : QObject(parentQObject), mCarState(carState), mParentQObject(parentQObject) {}
  virtual ~CarAutopilotComponent() {}

  virtual void setupAutopilot(QSharedPointer<EmergencyStopState> emergencyStopState);
  void reset();

  // Setters
  void setAutopilotTimerRate(int value) {mAutopilotTimerRate = value;}
  void setEnableMavlinkInterface(bool value) {mEnableMavlinkInterface = value;}
  void setWaywiseControlTowerAddress(std::string value) {mWaywiseControlTowerAddress = value;}
  void setWaywiseControlTowerPort(int value) {mWaywiseControlTowerPort = value;}
  void setPurePursuitRadius(float value) {mPurePursuitRadius = value;}
  void setEndGoalAlignmentType(AutopilotEndGoalAlignmentType value) {mEndGoalAlignmentType = value;}
  void setEndGoalAlignmentThreshold(float value) {mEndGoalAlignmentThreshold = value;}
  void setPositionAccuracyThresholdForMission(float value)
  {
    mPositionAccuracyThresholdForMission = value;
  }
  void setYawAccuracyThresholdForMission(float value) {mYawAccuracyThresholdForMission = value;}
  void setAdaptiveApproachSpeedEnabled(bool adaptive) {mAdaptiveApproachSpeedEnabled = adaptive;}
  void setMinApproachSpeed(float minApproachSpeed) {mMinApproachSpeed = minApproachSpeed;}
  void setGnssFixStatus(GnssFixStatus gnssFixStatus) {mGnssFixStatus = gnssFixStatus;}

  // Getters
  int getAutopilotTimerRate() const {return mAutopilotTimerRate;}
  bool getEnableMavlinkInterface() const {return mEnableMavlinkInterface;}
  std::string getWaywiseControlTowerAddress() const {return mWaywiseControlTowerAddress;}
  int getWaywiseControlTowerPort() const {return mWaywiseControlTowerPort;}
  float getPurePursuitRadius() const {return mPurePursuitRadius;}
  AutopilotEndGoalAlignmentType getEndGoalAlignmentType() const {return mEndGoalAlignmentType;}
  float getEndGoalAlignmentThreshold() const {return mEndGoalAlignmentThreshold;}
  float getPositionAccuracyThresholdForMission() const
  {
    return mPositionAccuracyThresholdForMission;
  }
  float getYawAccuracyThresholdForMission() const {return mYawAccuracyThresholdForMission;}
  bool getAdaptiveApproachSpeedEnabled() const {return mAdaptiveApproachSpeedEnabled;}
  float getMinApproachSpeed() const {return mMinApproachSpeed;}

  QList<PosPoint> getWaypointList() const {return mWaypointList;}
  MissionState getCurrentMissionState() const {return currentMissionState;}
  QSharedPointer<MovementController> getAutopilotMovementController() const
  {
    return mAutopilotMovementController;
  }

  std::optional<PosPoint> getVehicleAlignmentReferencePosPoint();
  QSharedPointer<VehicleState> getReferenceVehicleState();

  // Utility methods
  virtual void provideParametersToParameterServer();
  void switchAutopilot(bool enable);
  void updateWaypointFollowerRoute(QList<PosPoint> & waypointList);
  void processMissionStateMachine();
  bool isActive();
  virtual void stopWaypointFollower();

signals:
  void updatedMissionState(MissionState state);
  void gnssFixAccuracyAssertionFailed(GnssFixStatus gnssFixStatus);

protected:
  virtual void updateMissionState(MissionState state);
  virtual void startWaypointFollower(QList<PosPoint> & waypointList);
  bool assertGnssFixAccuracy();

  // Parameters
  int mAutopilotTimerRate = 10; // [hz]
  bool mEnableMavlinkInterface = true;
  std::string mWaywiseControlTowerAddress = "127.0.0.1";
  int mWaywiseControlTowerPort = 14540;
  float mPurePursuitRadius = 1.0; // [m]
  AutopilotEndGoalAlignmentType mEndGoalAlignmentType = AutopilotEndGoalAlignmentType::REAR_AXLE;
  float mEndGoalAlignmentThreshold = 0.1; // [m]
  float mPositionAccuracyThresholdForMission = 0.05; // [m]
  float mYawAccuracyThresholdForMission = 1.0; // [deg]
  bool mAdaptiveApproachSpeedEnabled = true;
  float mMinApproachSpeed = 0.0;

  // WayWise components
  QSharedPointer<CarState> mCarState;
  QSharedPointer<EmergencyStopState> mEmergencyStopState;

  QSharedPointer<MovementController> mAutopilotMovementController;
  QSharedPointer<PurepursuitWaypointFollower> mWaypointFollower;
  QSharedPointer<MavsdkVehicleServer> mMavsdkVehicleServer;
  QSharedPointer<FollowPoint> mFollowPoint;

  // Internal variables
  QObject * mParentQObject;
  QList<PosPoint> mWaypointList;
  MissionState currentMissionState = MissionState::WaitingForVehicleInit;
  GnssFixStatus mGnssFixStatus;
};

#endif  // CAR_AUTOPILOT_COMPONENT_HPP_
