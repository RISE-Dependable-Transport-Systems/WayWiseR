#ifndef COPTER_AUTOPILOT_COMPONENT_HPP_
#define COPTER_AUTOPILOT_COMPONENT_HPP_

#include <string>
#include <QObject>
#include <QString>

#include "geometry_msgs/msg/twist.hpp"

#include "WayWise/autopilot/copterwaypointfollower.h"
#include "WayWise/communication/mavsdkvehicleserver.h"
#include "WayWise/vehicles/copterstate.h"
#include "WayWise/autopilot/followpoint.h"

#include "qobject_node.hpp"
#include "waywiser_core_utils.hpp"

class CopterAutopilotComponent : public QObject
{
  Q_OBJECT

public:
  CopterAutopilotComponent(QObjectNode * parentQObjectNode, const QSharedPointer<CopterState> copterState)
  : QObject(parentQObjectNode), mCopterState(copterState), mParentQObjectNode(parentQObjectNode) {}
  virtual ~CopterAutopilotComponent() {}

  virtual void setupAutopilot(QSharedPointer<EmergencyStopState> emergencyStopState);
  void reset();

  // Setters
  void setAutopilotTimerRate(int value) {mAutopilotTimerRate = value;}
  void setEnableMavlinkInterface(bool value) {mEnableMavlinkInterface = value;}
  void setWaywiseControlTowerAddress(std::string value) {mWaywiseControlTowerAddress = value;}
  void setWaywiseControlTowerPort(int value) {mWaywiseControlTowerPort = value;}
  void setMissionPosTypeUsed(PosType value) {mMissionPosTypeUsed = value;}
  void setRequireGnssForMission(bool value) {mRequireGnssForMission = value;}
  void setVehicleInitialized(bool value) {mVehicleInitialized = value;}
  void setGnssFixStatus(GnssFixStatus gnssFixStatus) {mGnssFixStatus = gnssFixStatus;}
  void setWaypointProximityXY(double value) {mWaypointProximityXY = value;}
  void setWaypointProximityZ(double value) {mWaypointProximityZ = value;}
  void setEndGoalAlignmentThresholdXY(double value) {mEndGoalAlignmentThresholdXY = value;}
  void setEndGoalAlignmentThresholdZ(double value) {mEndGoalAlignmentThresholdZ = value;}
  void setStopSpeedThreshold(double value) {mStopSpeedThreshold = value;}
  void setCruiseSpeed(double value) {mCruiseSpeed = value;}
  void setMaxMissionSpeed(double value) {mMaxMissionSpeed = value;}
  void setDescentSpeed(double value)
  {
    mDescentSpeed = value;
    if (mWaypointFollower) {
      mWaypointFollower->setDescentSpeed(value);
    }
  }
  void setMinApproachSpeed(double value) {mMinApproachSpeed = value;}
  void setApproachSlowdownRadius(double value) {mApproachSlowdownRadius = value;}
  void setFaceTravelDirection(bool value) {mFaceTravelDirection = value;}
  void setYawGain(double value) {mYawGain = value;}
  void setMaxYawRate(double value) {mMaxYawRate = value;}
  void setVerticalHeightTolerance(double value) {mVerticalHeightTolerance = value;}
  void setVerticalProportionalGain(double value) {mVerticalProportionalGain = value;}
  void setVerticalIntegralGain(double value) {mVerticalIntegralGain = value;}
  void setVerticalDerivativeGain(double value) {mVerticalDerivativeGain = value;}
  void setVerticalIntegralLimit(double value) {mVerticalIntegralLimit = value;}
  void setPositionAccuracyThresholdForMission(float value)
  {
    mPositionAccuracyThresholdForMission = value;
  }
  void setYawAccuracyThresholdForMission(float value) {mYawAccuracyThresholdForMission = value;}
  void setAutoClimbEnabled(bool value);
  bool setAutoClimbActive(bool value);
  void setAutoClimbHeight(double value) {mAutoClimbHeight = value;}

  // Getters
  int getAutopilotTimerRate() const {return mAutopilotTimerRate;}
  bool getEnableMavlinkInterface() const {return mEnableMavlinkInterface;}
  std::string getWaywiseControlTowerAddress() const {return mWaywiseControlTowerAddress;}
  int getWaywiseControlTowerPort() const {return mWaywiseControlTowerPort;}
  PosType getMissionPosTypeUsed() const {return mMissionPosTypeUsed;}
  bool getRequireGnssForMission() const {return mRequireGnssForMission;}
  QList<PosPoint> getWaypointList() const {return mWaypointList;}
  MissionState getCurrentMissionState() const {return currentMissionState;}
  PosPoint getCurrentGoal() const;
  QSharedPointer<MovementController> getAutopilotMovementController() const
  {
    return mAutopilotMovementController;
  }
  geometry_msgs::msg::Twist getAutopilotTwistCommand() const;
  double getWaypointProximityXY() const {return mWaypointProximityXY;}
  double getWaypointProximityZ() const {return mWaypointProximityZ;}
  double getEndGoalAlignmentThresholdXY() const {return mEndGoalAlignmentThresholdXY;}
  double getEndGoalAlignmentThresholdZ() const {return mEndGoalAlignmentThresholdZ;}
  double getStopSpeedThreshold() const {return mStopSpeedThreshold;}
  double getCruiseSpeed() const {return mCruiseSpeed;}
  double getMaxMissionSpeed() const {return mMaxMissionSpeed;}
  double getDescentSpeed() const {return mDescentSpeed;}
  double getMinApproachSpeed() const {return mMinApproachSpeed;}
  double getApproachSlowdownRadius() const {return mApproachSlowdownRadius;}
  bool getFaceTravelDirection() const {return mFaceTravelDirection;}
  double getYawGain() const {return mYawGain;}
  double getMaxYawRate() const {return mMaxYawRate;}
  double getVerticalHeightTolerance() const {return mVerticalHeightTolerance;}
  double getVerticalProportionalGain() const {return mVerticalProportionalGain;}
  double getVerticalIntegralGain() const {return mVerticalIntegralGain;}
  double getVerticalDerivativeGain() const {return mVerticalDerivativeGain;}
  double getVerticalIntegralLimit() const {return mVerticalIntegralLimit;}
  bool getAutoClimbEnabled() const {return mAutoClimbEnabled;}
  bool getAutoClimbActive() const {return mAutoClimbActive;}
  double getAutoClimbHeight() const {return mAutoClimbHeight;}
  bool getRouteClimbActive() const;

  // Utility methods
  virtual void provideParametersToParameterServer();
  void switchAutopilot(bool enable);
  void updateWaypointFollowerRoute(QList<PosPoint> & waypointList);
  void startWaypointFollowerRouteFromBeginning(QList<PosPoint> & waypointList);
  void processMissionStateMachine();
  bool isActive();
  virtual void stopWaypointFollower();
  void clearWaypointFollowerRoute();
  void cancelAutoClimb();
  bool startAutoClimbWithWaypointFollower();
  void applyMissionWaypointFollowerTuning();
  void applyAutoClimbWaypointFollowerTuning();

signals:
  void updatedMissionState(MissionState state);
  void gnssFixAccuracyAssertionFailed(GnssFixStatus gnssFixStatus);

protected:
  virtual void updateMissionState(MissionState state);
  virtual void startWaypointFollower(QList<PosPoint> & waypointList);
  MissionState deriveWaypointFollowerMissionState() const;
  bool assertGnssFixAccuracy();

  int mAutopilotTimerRate = 10; // [hz]
  bool mEnableMavlinkInterface = true;
  std::string mWaywiseControlTowerAddress = "127.0.0.1";
  int mWaywiseControlTowerPort = 14540;
  PosType mMissionPosTypeUsed = PosType::odom;
  bool mRequireGnssForMission = false;
  double mWaypointProximityXY = 0.5;
  double mWaypointProximityZ = 1.0;
  double mEndGoalAlignmentThresholdXY = 0.25;
  double mEndGoalAlignmentThresholdZ = 0.25;
  double mStopSpeedThreshold = 0.2;
  double mCruiseSpeed = 1.0;
  double mMaxMissionSpeed = 2.0;
  double mDescentSpeed = 0.3;
  double mMinApproachSpeed = 0.1;
  double mApproachSlowdownRadius = 1.5;
  bool mFaceTravelDirection = true;
  double mYawGain = 1.5;
  double mMaxYawRate = 1.0;
  double mVerticalHeightTolerance = 0.10;
  double mVerticalProportionalGain = 0.8;
  double mVerticalIntegralGain = 0.04;
  double mVerticalDerivativeGain = 0.5;
  double mVerticalIntegralLimit = 2.0;
  float mPositionAccuracyThresholdForMission = 0.5; // [m]
  float mYawAccuracyThresholdForMission = 5.0; // [deg]
  bool mAutoClimbEnabled = false;
  bool mAutoClimbActive = false;
  double mAutoClimbHeight = 2.0;

  QSharedPointer<CopterState> mCopterState;
  QSharedPointer<EmergencyStopState> mEmergencyStopState;
  QSharedPointer<MovementController> mAutopilotMovementController;
  QSharedPointer<CopterWaypointFollower> mWaypointFollower;
  QSharedPointer<MavsdkVehicleServer> mMavsdkVehicleServer;

  QObjectNode * mParentQObjectNode;
  QList<PosPoint> mWaypointList;
  MissionState currentMissionState = MissionState::WaitingForVehicleInit;
  GnssFixStatus mGnssFixStatus;
  bool mVehicleInitialized = false;
};

#endif  // COPTER_AUTOPILOT_COMPONENT_HPP_
